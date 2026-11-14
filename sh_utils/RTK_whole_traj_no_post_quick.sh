#!/usr/bin/env bash
# HAM 室外实飞（quick 版：收紧失败路径等待，正常路径不受影响）
#              确定性预检 -> 录包 -> 人工切 OFFBOARD -> 起飞
#              -> FSM READY -> 轨迹墙钟窗口 -> 降落 -> 确认落地
#
# quick 与 no_post 的唯一差异是等待常数（依据 2026-08-26 两个 bag 实测富余收紧）：
#   ARM 20->10   (实测命令->armed 0.32/0.37 s, 25 倍富余)
#   READY 45->25 (实测 armed->READY 5.31/5.49 s, 4 倍富余; 超时=起飞退化, 早脱困)
#   LAND 120->60 (预计 AUTO.LAND 10~20 s, 失败时少旋 1 分钟桨)
#   ON_GROUND 20->10 / 降落确认 12->6 (上锁即已落地)

#
# 重要安全约束：
#   1. 本脚本不切换 OFFBOARD，模式只能由连接 Pixhawk 6C 的外部遥控器切换。
#   2. READY 前根据 ROS/PX4 状态推进；轨迹触发后按运行时总时长使用墙钟等待，
#      不再因临时 rostopic 读取失败中止已在执行的轨迹和后续降落。
#   3. 空中异常或 Ctrl-C 时，不关闭 MAVROS、odomRemap、规划器或控制器，便于遥控器接管。

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
readonly UTILS_DIR="${SCRIPT_DIR}"
readonly RECORD_SCRIPT="${UTILS_DIR}/record.sh"

readonly STATE_TOPIC="/mavros/state"
readonly EXTENDED_STATE_TOPIC="/mavros/extended_state"
readonly ESTIMATOR_TOPIC="/mavros/estimator_status"
readonly RTK_STATUS_TOPIC="/mavros/gpsstatus/gps1/raw"
readonly ODOM_TOPIC="/odom/remap"
readonly SETPOINT_TOPIC="/mavros/setpoint_raw/attitude"
readonly DEBUG_TOPIC="/debugPx4ctrl"
readonly TAKEOFF_LAND_TOPIC="/drone0/ctrl_node/takeoff_land"
readonly CTRL_TRIGGER_TOPIC="/traj_start_trigger"
readonly TRAJECTORY_TRIGGER_TOPIC="/landing_trigger"
readonly TRAJECTORY_COMMAND_TOPIC="/drone0/planning/cmd"
readonly TRAJECTORY_TIME_TOPIC="/drone0/sim_odom/time_diff"
readonly TRAJECTORY_PARAM_NS="/drone0/sim_odom"
readonly CONTROL_PARAM_NS="/drone0/ctrl_node"

readonly RTK_FIXED_MIN_TYPE=6
# OFFBOARD 等待是人工操作, 保持 60 s 不收紧。
readonly OFFBOARD_WAIT_TIMEOUT=60
# 以下四个失败路径超时按 2026-08-26 bag 实测富余收紧(quick 版), 见文件头注释。
readonly ARM_WAIT_TIMEOUT=10
readonly STARTUP_RETRY_TIMEOUT=10
readonly CTRL_READY_WAIT_TIMEOUT=25
readonly TRAJECTORY_WALL_TIME_MARGIN=2
readonly POST_TRAJECTORY_SETTLE_DELAY=3
readonly LAND_WAIT_TIMEOUT=60
readonly MIN_FREE_GB="${MIN_FREE_GB:-5}"
readonly PREFLIGHT_ON_GROUND_CACHE_MAX_AGE=300

# 与 ctrl_fsm.cc 的 TAKEOFF -> HOVER 高度门槛一致。
readonly TAKEOFF_REACH_TOLERANCE="0.10"
readonly CTRL_TRIGGER_MONITOR_NODE="/ham_whole_traj_ctrl_trigger_monitor"

# PX4 AUTO.LAND 参数。悬停推力取 ctrl_node 已加载且经 debug 验证的固定映射。
PX4_LAND_SPEED="${PX4_LAND_SPEED:-0.60}"
PX4_DISARM_LAND_DELAY="${PX4_DISARM_LAND_DELAY:-0.50}"

AIRBORNE=0
TAKEOFF_PENDING=0
CTRL_TRIGGER_MONITOR_PID=""
CTRL_TRIGGER_MONITOR_FILE=""
CTRL_READY_ID=""
RECORDING_WARNING_EMITTED=0
LAST_ON_GROUND_CONFIRM_SECONDS=-1
EXTENDED_STATE_FALLBACK_ACTIVE=0
EXTENDED_STATE_FALLBACK_WARNED=0

# flock 在进入 main 前首次使用，须在使用点前检查（main 的 require_command 到不了这里）。
if ! command -v flock >/dev/null 2>&1; then
    echo "[FATAL] 缺少命令：flock" >&2
    exit 1
fi

exec 9>/tmp/ham_whole_traj.lock
if ! flock -n 9; then
    echo "[FATAL] 另一个 whole_traj.sh 正在运行，拒绝重复执行。" >&2
    exit 1
fi

# 公共 ROS 工具(log/read_state/state_value/wait_topic_messages 等)统一取自 _ros_common.sh,
# 避免与本脚本出现两份逐渐分叉的实现。
# The path is computed at runtime; _ros_common.sh is checked separately.
# shellcheck source=/dev/null
source "${UTILS_DIR}/_ros_common.sh"

fatal()
{
    log FATAL "$1" >&2
    if (( AIRBORNE || TAKEOFF_PENDING )); then
        log SAFETY "飞行器可能仍在空中：停止自动流程，但保留 MAVROS、控制器、里程计和录包。" >&2
        log SAFETY "请立即通过外部遥控器退出 OFFBOARD，并按现场预案接管/降落。" >&2
    fi
    exit 1
}

on_signal()
{
    log WARN "收到中断信号，停止任务自动推进。" >&2
    if (( AIRBORNE || TAKEOFF_PENDING )); then
        log SAFETY "不会自动降落或上锁；请使用外部遥控器接管。" >&2
    fi
    exit 130
}

cleanup()
{
    local exit_code=$?
    trap - EXIT

    if [[ -n "${CTRL_TRIGGER_MONITOR_PID:-}" ]]; then
        kill "$CTRL_TRIGGER_MONITOR_PID" >/dev/null 2>&1 || true
        wait "$CTRL_TRIGGER_MONITOR_PID" 2>/dev/null || true
    fi
    if [[ -n "${CTRL_TRIGGER_MONITOR_FILE:-}" ]]; then
        rm -f "$CTRL_TRIGGER_MONITOR_FILE" 2>/dev/null || true
    fi

    exit "$exit_code"
}

trap on_signal INT TERM
trap cleanup EXIT

require_command()
{
    command -v "$1" >/dev/null 2>&1 || fatal "缺少命令：$1"
}

is_number()
{
    [[ "$1" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$ ]]
}

read_vector()
{
    local topic="$1"
    local msg x y z

    msg=$(timeout 3 rostopic echo -n 1 "$topic" 2>/dev/null) || return 1
    x=$(awk -F': ' '$1 == "x" {print $2; exit}' <<<"$msg")
    y=$(awk -F': ' '$1 == "y" {print $2; exit}' <<<"$msg")
    z=$(awk -F': ' '$1 == "z" {print $2; exit}' <<<"$msg")

    is_number "$x" && is_number "$y" && is_number "$z" || return 1
    printf '%s %s %s\n' "$x" "$y" "$z"
}

read_topic_message_with_retry()
{
    local topic="$1"
    local timeout_s="$2"
    local start_time=$SECONDS
    local message

    # rosbag 启动时会集中建立多个订阅连接，允许短暂通信抖动；每次仍通过
    # rostopic 获取新消息，绝不以缓存状态放行安全门禁。
    while (( SECONDS - start_time < timeout_s )); do
        if message=$(timeout 3 rostopic echo -n 1 "$topic" 2>/dev/null); then
            printf '%s\n' "$message"
            return 0
        fi
        sleep 0.2
    done
    return 1
}

read_state_with_retry()
{
    local topic="$1"
    local timeout_s="${2:-$STARTUP_RETRY_TIMEOUT}"

    read_topic_message_with_retry "$topic" "$timeout_s"
}

read_vector_with_retry()
{
    local topic="$1"
    local timeout_s="${2:-$STARTUP_RETRY_TIMEOUT}"
    local start_time=$SECONDS
    local vector

    while (( SECONDS - start_time < timeout_s )); do
        if vector=$(read_vector "$topic"); then
            printf '%s\n' "$vector"
            return 0
        fi
        sleep 0.2
    done
    return 1
}

read_rosparam_with_retry()
{
    local name="$1"
    local timeout_s="${2:-$STARTUP_RETRY_TIMEOUT}"
    local start_time=$SECONDS
    local value

    while (( SECONDS - start_time < timeout_s )); do
        if value=$(rosparam get "$name" 2>/dev/null); then
            printf '%s\n' "$value"
            return 0
        fi
        sleep 0.2
    done
    return 1
}

read_optional_rosparam_with_retry()
{
    local name="$1"
    local default_value="$2"
    local timeout_s="${3:-$STARTUP_RETRY_TIMEOUT}"
    local start_time=$SECONDS
    local params remaining

    while (( SECONDS - start_time < timeout_s )); do
        if params=$(rosparam list 2>/dev/null); then
            if grep -Fxq "$name" <<<"$params"; then
                remaining=$(( timeout_s - (SECONDS - start_time) ))
                (( remaining > 0 )) || return 1
                read_rosparam_with_retry "$name" "$remaining"
            else
                printf '%s\n' "$default_value"
            fi
            return $?
        fi
        sleep 0.2
    done
    return 1
}

topic_has_endpoint()
{
    local topic="$1"
    local section="$2"
    local endpoint="$3"
    local info

    info=$(timeout 5 rostopic info "$topic" 2>/dev/null) || return 1
    awk -v wanted_section="$section" -v wanted_endpoint="$endpoint" '
        {
            # rostopic 版本之间的标题空白略有差异，终端输出也可能带 CR；
            # 先规范化再解析，避免把已连接的端点误报为缺失。
            line=$0
            sub(/\r$/, "", line)

            if (line ~ "^[[:space:]]*" wanted_section ":[[:space:]]*$") {
                inside=1
                next
            }
            if (line ~ "^[[:space:]]*(Publishers|Subscribers|Services):") {
                inside=0
            }
        }
        inside {
            sub(/^[[:space:]]*\*[[:space:]]*/, "", line)
            split(line, fields, /[[:space:]]+/)
            if (fields[1] == wanted_endpoint) found=1
        }
        END {exit !found}
    ' <<<"$info"
}

topic_has_any_endpoint()
{
    local topic="$1"
    local section="$2"
    shift 2
    local endpoint

    for endpoint in "$@"; do
        if topic_has_endpoint "$topic" "$section" "$endpoint"; then
            return 0
        fi
    done
    return 1
}

assert_topic_endpoint_any()
{
    local topic="$1"
    local section="$2"
    local component="$3"
    shift 3
    local expected

    topic_has_any_endpoint "$topic" "$section" "$@" && return 0
    printf -v expected ' %s' "$@"
    fatal "${topic} 缺少 ${component} 的 ${section} 端点；期望之一：${expected}。"
}

assert_startup_topic_endpoint_any()
{
    local topic="$1"
    local section="$2"
    local component="$3"
    shift 3
    local start_time=$SECONDS
    local expected

    while (( SECONDS - start_time < STARTUP_RETRY_TIMEOUT )); do
        if topic_has_any_endpoint "$topic" "$section" "$@"; then
            return 0
        fi
        sleep 0.2
    done
    printf -v expected ' %s' "$@"
    fatal "${STARTUP_RETRY_TIMEOUT} s 内 ${topic} 未建立 ${component} 的 ${section} 端点；期望之一：${expected}。"
}

assert_pre_record_control_graph()
{
    # 下列连接都由已启动的 MAVROS/规划器/控制器决定，不依赖录包。
    # 必须在创建 bag 前全部就绪；真正发布命令时仍会做瞬时复核。
    assert_startup_topic_endpoint_any "$TRAJECTORY_COMMAND_TOPIC" Publishers planner \
        /manager /drone0/sim_odom
    assert_startup_topic_endpoint_any "$TRAJECTORY_COMMAND_TOPIC" Subscribers controller \
        /drone0/manager /drone0/ctrl_node
    assert_startup_topic_endpoint_any "$TRAJECTORY_TIME_TOPIC" Publishers planner \
        /manager /drone0/sim_odom
    assert_startup_topic_endpoint_any "$SETPOINT_TOPIC" Publishers controller \
        /drone0/manager /drone0/ctrl_node
    assert_startup_topic_endpoint_any "$SETPOINT_TOPIC" Subscribers mavros /mavros
    assert_startup_topic_endpoint_any "$TAKEOFF_LAND_TOPIC" Subscribers controller \
        /drone0/manager /drone0/ctrl_node
    assert_startup_topic_endpoint_any "$CTRL_TRIGGER_TOPIC" Publishers controller \
        /drone0/manager /drone0/ctrl_node
    assert_startup_topic_endpoint_any "$CTRL_TRIGGER_TOPIC" Subscribers planner \
        /manager /drone0/sim_odom
    assert_startup_topic_endpoint_any "$TRAJECTORY_TRIGGER_TOPIC" Subscribers planner \
        /manager /drone0/sim_odom
    assert_startup_topic_endpoint_any "$TRAJECTORY_TRIGGER_TOPIC" Subscribers controller \
        /drone0/manager /drone0/ctrl_node

    log OK "录包前控制链路端点全部就绪。"
}

assert_recording_active()
{
    rosnode list 2>/dev/null | grep -Fxq /record ||
        fatal "录包节点 /record 已退出；停止自动任务推进。"
}

warn_if_recording_inactive()
{
    if rosnode list 2>/dev/null | grep -Fxq /record; then
        return 0
    fi

    if (( ! RECORDING_WARNING_EMITTED )); then
        log WARN "起飞命令已发出，当前未发现录包节点 /record；为保证飞行和降落流程连续，不再以录包故障中止自动流程。"
        RECORDING_WARNING_EMITTED=1
    fi
    return 0
}

recording_endpoints_ready()
{
    local topic

    # 这些话题覆盖飞行状态、控制输入/输出、轨迹握手和实际里程计；只有 rosbag
    # 全部注册为订阅者后，才允许任务继续推进。
    for topic in \
        "$STATE_TOPIC" \
        "$EXTENDED_STATE_TOPIC" \
        "$RTK_STATUS_TOPIC" \
        "$ESTIMATOR_TOPIC" \
        "$ODOM_TOPIC" \
        "$SETPOINT_TOPIC" \
        "$DEBUG_TOPIC" \
        "$TAKEOFF_LAND_TOPIC" \
        "$CTRL_TRIGGER_TOPIC" \
        "$TRAJECTORY_TRIGGER_TOPIC" \
        "$TRAJECTORY_COMMAND_TOPIC" \
        "$TRAJECTORY_TIME_TOPIC"; do
        topic_has_endpoint "$topic" Subscribers /record || return 1
    done
}

assert_controller_parameter_consistency()
{
    local accurate hover gravity debug thr2acc expected

    accurate=$(read_rosparam_with_retry "${CONTROL_PARAM_NS}/thrust_model/accurate_thrust_model") ||
        fatal "缺少控制器参数 thrust_model/accurate_thrust_model。"
    [[ "$accurate" == "false" || "$accurate" == "False" ]] ||
        fatal "实飞验证要求 accurate_thrust_model=false；当前值=${accurate}。"

    hover=$(read_rosparam_with_retry "${CONTROL_PARAM_NS}/thrust_model/hover_percentage") ||
        fatal "缺少控制器参数 thrust_model/hover_percentage。"
    gravity=$(read_rosparam_with_retry "${CONTROL_PARAM_NS}/gra") ||
        fatal "缺少控制器参数 gra。"
    is_number "$hover" || fatal "控制器悬停推力参数不是数字。"
    is_number "$gravity" || fatal "控制器重力参数不是数字。"
    awk -v h="$hover" -v g="$gravity" 'BEGIN {exit !(h>=0.1 && h<=0.8 && g>0)}' ||
        fatal "控制器悬停推力/重力参数越界：hover=${hover}, gra=${gravity}。"

    debug=$(read_topic_message_with_retry "$DEBUG_TOPIC" "$STARTUP_RETRY_TIMEOUT") ||
        fatal "${STARTUP_RETRY_TIMEOUT} s 内无法读取新鲜的 ${DEBUG_TOPIC}，不能确认控制器内部推力映射。"
    thr2acc=$(awk -F': ' '/^[[:space:]]*thr2acc:/ {print $2; exit}' <<<"$debug")
    is_number "$thr2acc" || fatal "${DEBUG_TOPIC} 缺少有效 thr2acc。"
    expected=$(awk -v g="$gravity" -v h="$hover" 'BEGIN {printf "%.9f", g/h}')

    awk -v actual="$thr2acc" -v expected="$expected" '
        BEGIN {
            d=actual-expected; if(d<0)d=-d
            tolerance=0.02+0.01*expected
            exit !(d<=tolerance)
        }' ||
        fatal "ROS 参数与 ctrl_node 内部推力映射不一致：hover=${hover}, 期望 thr2acc=${expected}, debug thr2acc=${thr2acc}。请重启 ctrl_node，不能只 rosparam set。"

    log OK "控制器推力映射一致：hover=${hover}, thr2acc=${thr2acc}。"
}

assert_safe_preflight_state()
{
    local quiet="${1:-}"
    local state mode armed connected
    state=$(read_state_with_retry "$STATE_TOPIC") ||
        fatal "${STARTUP_RETRY_TIMEOUT} s 内无法读取新鲜的 ${STATE_TOPIC}"
    connected=$(state_value "$state" connected)
    armed=$(state_value "$state" armed)
    mode=$(state_value "$state" mode)

    [[ "$connected" == "True" ]] || fatal "MAVROS 尚未连接 Pixhawk 6C。"
    if [[ "$armed" != "False" ]]; then
        [[ "$armed" == "True" ]] && AIRBORNE=1
        fatal "飞行器已经解锁或解锁状态不可用（armed=${armed:-unknown}），拒绝启动任务。"
    fi
    [[ -n "$mode" ]] || fatal "无法解析 PX4 飞行模式，拒绝启动任务。"
    [[ "$mode" != "OFFBOARD" ]] || fatal "任务启动前已处于 OFFBOARD；请先退出 OFFBOARD 并保持未解锁。"

    local extended landed_state ground_confirm_age
    if (( ! EXTENDED_STATE_FALLBACK_ACTIVE )); then
        if extended=$(read_topic_message_with_retry "$EXTENDED_STATE_TOPIC" "$STARTUP_RETRY_TIMEOUT"); then
            landed_state=$(awk -F': ' '/^[[:space:]]*landed_state:/ {print $2; exit}' <<<"$extended")
            [[ "$landed_state" == "1" ]] || fatal "PX4 未报告 ON_GROUND（landed_state=${landed_state:-unknown}）。"
            LAST_ON_GROUND_CONFIRM_SECONDS=$SECONDS
        else
            EXTENDED_STATE_FALLBACK_ACTIVE=1
        fi
    fi

    if (( EXTENDED_STATE_FALLBACK_ACTIVE )); then
        (( LAST_ON_GROUND_CONFIRM_SECONDS >= 0 )) ||
            fatal "${STARTUP_RETRY_TIMEOUT} s 内无法读取新鲜的 ${EXTENDED_STATE_TOPIC}，且本次运行尚未确认 ON_GROUND。"
        ground_confirm_age=$((SECONDS - LAST_ON_GROUND_CONFIRM_SECONDS))
        (( ground_confirm_age <= PREFLIGHT_ON_GROUND_CACHE_MAX_AGE )) ||
            fatal "${EXTENDED_STATE_TOPIC} 实时读取失败，且 ON_GROUND 确认已超过 ${PREFLIGHT_ON_GROUND_CACHE_MAX_AGE} s；请重新运行任务。"
        if (( ! EXTENDED_STATE_FALLBACK_WARNED )); then
            log WARN "${EXTENDED_STATE_TOPIC} 实时读取失败；复用本次运行 ${ground_confirm_age} s 前已确认的 ON_GROUND。未解锁和非 OFFBOARD 状态仍实时检查。"
            EXTENDED_STATE_FALLBACK_WARNED=1
        fi
    fi

    [[ "$quiet" == "quiet" ]] ||
        log OK "PX4 初始状态安全：connected=True, armed=False, mode=${mode:-unknown}, ON_GROUND。"
}

assert_takeoff_ready_state()
{
    local state mode armed connected extended landed_state

    state=$(read_state_with_retry "$STATE_TOPIC") ||
        fatal "起飞命令前 ${STARTUP_RETRY_TIMEOUT} s 内无法读取新鲜的 ${STATE_TOPIC}"
    connected=$(state_value "$state" connected)
    armed=$(state_value "$state" armed)
    mode=$(state_value "$state" mode)

    [[ "$connected" == "True" ]] || fatal "起飞命令前 MAVROS 与 Pixhawk 6C 连接断开。"
    if [[ "$armed" != "False" ]]; then
        [[ "$armed" == "True" ]] && AIRBORNE=1
        fatal "起飞命令前解锁状态不安全（armed=${armed:-unknown}）。"
    fi
    [[ "$mode" == "OFFBOARD" ]] ||
        fatal "起飞命令前 PX4 已退出 OFFBOARD（mode=${mode:-unknown}）。"

    extended=$(read_topic_message_with_retry "$EXTENDED_STATE_TOPIC" "$STARTUP_RETRY_TIMEOUT") ||
        fatal "起飞命令前 ${STARTUP_RETRY_TIMEOUT} s 内无法读取新鲜的 ${EXTENDED_STATE_TOPIC}"
    landed_state=$(awk -F': ' '/^[[:space:]]*landed_state:/ {print $2; exit}' <<<"$extended")
    [[ "$landed_state" == "1" ]] ||
        fatal "起飞命令前 PX4 未报告 ON_GROUND（landed_state=${landed_state:-unknown}）。"

    log OK "起飞最终状态复核通过：connected=True, armed=False, mode=OFFBOARD, ON_GROUND。"
}

assert_rtk_fixed()
{
    local timeout_s="$STARTUP_RETRY_TIMEOUT"
    local msg fix_type
    msg=$(read_topic_message_with_retry "$RTK_STATUS_TOPIC" "$timeout_s") ||
        fatal "${timeout_s} s 内无法读取新鲜的 RTK 状态：${RTK_STATUS_TOPIC}"
    fix_type=$(awk -F': ' '/^[[:space:]]*fix_type:/ {print $2; exit}' <<<"$msg")

    [[ "$fix_type" =~ ^[0-9]+$ ]] || fatal "RTK fix_type 无效：${fix_type:-missing}"
    (( fix_type >= RTK_FIXED_MIN_TYPE )) || fatal "RTK 不是 Fixed：fix_type=${fix_type}，要求 >=${RTK_FIXED_MIN_TYPE}。"
    log OK "RTK Fixed：fix_type=${fix_type}。"
}

assert_ekf_ready()
{
    local timeout_s="$STARTUP_RETRY_TIMEOUT"
    local estimator
    estimator=$(read_topic_message_with_retry "$ESTIMATOR_TOPIC" "$timeout_s") ||
        fatal "${timeout_s} s 内无法读取新鲜的 PX4 EKF 状态：${ESTIMATOR_TOPIC}"

    grep -Eq 'pos_horiz_abs_status_flag: (True|true)' <<<"$estimator" ||
        fatal "PX4 EKF 水平绝对位置未就绪。"
    grep -Eq 'pos_vert_abs_status_flag: (True|true)' <<<"$estimator" ||
        fatal "PX4 EKF 垂直绝对位置未就绪。"
    if grep -Eq 'gps_glitch_status_flag: (True|true)' <<<"$estimator"; then
        fatal "PX4 EKF 报告 GPS glitch。"
    fi

    log OK "PX4 EKF 绝对位置状态正常。"
}

check_disk_space()
{
    local free_kb required_kb
    [[ "$MIN_FREE_GB" =~ ^[0-9]+$ ]] || fatal "MIN_FREE_GB 必须是正整数。"
    (( MIN_FREE_GB > 0 )) || fatal "MIN_FREE_GB 必须大于 0。"
    free_kb=$(df -Pk "$UTILS_DIR" | awk 'NR == 2 {print $4}')
    required_kb=$(( MIN_FREE_GB * 1024 * 1024 ))

    [[ "$free_kb" =~ ^[0-9]+$ ]] || fatal "无法读取录包磁盘剩余空间。"
    (( free_kb >= required_kb )) ||
        fatal "录包磁盘空间不足：至少需要 ${MIN_FREE_GB} GiB。"

    log OK "录包磁盘可用空间满足最低要求 ${MIN_FREE_GB} GiB。"
}

read_px4_param()
{
    local name="$1"
    local start_time=$SECONDS
    local reply value

    while (( SECONDS - start_time < STARTUP_RETRY_TIMEOUT )); do
        reply=$(timeout 5 rosservice call /mavros/param/get "param_id: '${name}'" 2>/dev/null) || {
            sleep 0.2
            continue
        }
        if grep -Eq '^[[:space:]]*success: (True|true)' <<<"$reply"; then
            value=$(awk -F': ' '/^[[:space:]]*real:/ {print $2; exit}' <<<"$reply")
            if is_number "$value"; then
                printf '%s\n' "$value"
                return 0
            fi
        fi
        sleep 0.2
    done
    return 1
}

verify_px4_param()
{
    local name="$1"
    local expected="$2"
    local actual

    actual=$(read_px4_param "$name") || fatal "无法回读 PX4 参数 ${name}。"
    awk -v a="$actual" -v e="$expected" '
        BEGIN {d=a-e; if(d<0)d=-d; exit !(d<=0.002)}' ||
        fatal "PX4 参数 ${name} 回读不一致：期望=${expected}, 实际=${actual}。"
    log OK "PX4 参数回读通过：${name}=${actual}。"
}

set_px4_param_with_retry()
{
    local name="$1"
    local value="$2"
    local start_time=$SECONDS

    # 每次尝试前都取得新鲜状态，确保参数重试也不会跨出
    # 未解锁、非 OFFBOARD、ON_GROUND 的允许窗口。
    while (( SECONDS - start_time < STARTUP_RETRY_TIMEOUT )); do
        assert_safe_preflight_state quiet
        if timeout 5 rosrun mavros mavparam set "$name" "$value" >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.2
    done
    return 1
}

configure_px4_auto_land()
{
    local hover_thrust

    hover_thrust=$(read_rosparam_with_retry "${CONTROL_PARAM_NS}/thrust_model/hover_percentage") ||
        fatal "缺少悬停推力参数 ${CONTROL_PARAM_NS}/thrust_model/hover_percentage"
    is_number "$hover_thrust" || fatal "PX4 悬停推力参数必须是数字。"
    is_number "$PX4_LAND_SPEED" || fatal "PX4 降落速度参数必须是数字。"
    is_number "$PX4_DISARM_LAND_DELAY" || fatal "PX4 落地上锁延时参数必须是数字。"
    awk -v h="$hover_thrust" -v s="$PX4_LAND_SPEED" \
        -v d="$PX4_DISARM_LAND_DELAY" \
        'BEGIN {exit !(h>=0.1 && h<=0.8 && s>=0.6 && d>0)}' ||
        fatal "PX4 降落参数越界：MPC_THR_HOVER=${hover_thrust}, MPC_LAND_SPEED=${PX4_LAND_SPEED}, COM_DISARM_LAND=${PX4_DISARM_LAND_DELAY}"

    log ACTION "在切入 OFFBOARD 前对齐 PX4 AUTO.LAND 参数：hover=${hover_thrust}, land_speed=${PX4_LAND_SPEED}, disarm_delay=${PX4_DISARM_LAND_DELAY}"
    set_px4_param_with_retry MPC_THR_HOVER "$hover_thrust" ||
        fatal "设置 PX4 MPC_THR_HOVER 失败。"
    set_px4_param_with_retry MPC_LAND_SPEED "$PX4_LAND_SPEED" ||
        fatal "设置 PX4 MPC_LAND_SPEED 失败。"
    set_px4_param_with_retry COM_DISARM_LAND "$PX4_DISARM_LAND_DELAY" ||
        fatal "设置 PX4 COM_DISARM_LAND 失败。"

    verify_px4_param MPC_THR_HOVER "$hover_thrust"
    verify_px4_param MPC_LAND_SPEED "$PX4_LAND_SPEED"
    verify_px4_param COM_DISARM_LAND "$PX4_DISARM_LAND_DELAY"
}

start_recording()
{
    local record_dir runner start_time

    if rosnode list 2>/dev/null | grep -Fxq /record; then
        fatal "已存在 /record 节点；请先确认并在原录包窗口手动按 Ctrl+C。"
    fi

    record_dir=$(cd "$(dirname "$RECORD_SCRIPT")" && pwd) ||
        fatal "无法解析录包脚本目录：${RECORD_SCRIPT}"
    runner=$(mktemp /tmp/ham_rosbag_window.XXXXXX.sh) ||
        fatal "无法创建录包窗口临时脚本。"
    {
        echo '#!/usr/bin/env bash'
        echo 'source ~/.bashrc 2>/dev/null || true'
        echo 'set +e'
        echo 'unset RECORD_NODE_NAME'
        printf 'cd %q\n' "$record_dir"
        echo 'echo "[INFO] 独立录包窗口：bash record.sh"'
        echo 'echo "[INFO] 需要结束录包时，请在本窗口按 Ctrl+C。whole_traj 不会自动停止录包。"'
        echo 'bash record.sh'
        echo 'exit_code=$?'
        # 此行生成的子脚本需要在运行时展开 exit_code。
        # shellcheck disable=SC2016
        echo 'echo "[INFO] record.sh 已退出，状态码=${exit_code}。请确认 rosbag 已完成索引写入。"'
        echo 'exec bash'
    } > "$runner"
    chmod +x "$runner"

    log INFO "将在独立窗口通过 bash record.sh 启动录包：${RECORD_SCRIPT}"
    if ! gnome-terminal --window --title="HAM RTK rosbag" -- "$runner"; then
        rm -f "$runner"
        fatal "无法创建独立录包窗口。"
    fi

    start_time=$SECONDS
    while ! rosnode list 2>/dev/null | grep -Fxq /record; do
        if (( SECONDS - start_time >= 10 )); then
            rm -f "$runner"
            fatal "10 s 内未发现 /record 节点；请检查独立录包窗口。录包不会被自动停止。"
        fi
        sleep 0.2
    done

    log WAIT "等待 /record 完成关键任务话题订阅..."
    start_time=$SECONDS
    while ! recording_endpoints_ready; do
        if (( SECONDS - start_time >= 10 )); then
            rm -f "$runner"
            fatal "/record 已启动，但 10 s 内未完成关键任务话题订阅；请检查录包窗口。录包不会被自动停止。"
        fi
        sleep 0.2
    done

    rm -f "$runner"
    assert_recording_active
    log OK "独立录包窗口已就绪：bash record.sh（节点 /record）。"
    log INFO "本脚本不会自动停止录包；完成后请在录包窗口手动按 Ctrl+C。"
}

wait_for_external_offboard()
{
    local start_time=$SECONDS
    local state mode connected

    log ACTION "请确认起飞区域和遥控器接管开关安全，然后用外部遥控器切入 OFFBOARD。"
    log WAIT "等待 PX4 进入 OFFBOARD（最长 ${OFFBOARD_WAIT_TIMEOUT} s）..."

    while (( SECONDS - start_time < OFFBOARD_WAIT_TIMEOUT )); do
        assert_controller_not_cancelled "等待 OFFBOARD 期间"
        state=$(read_state "$STATE_TOPIC") || { sleep 1; continue; }
        connected=$(state_value "$state" connected)
        mode=$(state_value "$state" mode)

        [[ "$connected" == "True" ]] || fatal "等待 OFFBOARD 时 MAVROS 连接断开。"

        if [[ "$mode" == "OFFBOARD" ]]; then
            log OK "外部遥控器已切入 OFFBOARD。"
            return 0
        fi
        sleep 1
    done

    fatal "等待外部遥控器切入 OFFBOARD 超时。"
}

controller_handshake_status()
{
    local frame_id=""

    if [[ -n "${CTRL_TRIGGER_MONITOR_FILE:-}" && -f "$CTRL_TRIGGER_MONITOR_FILE" ]]; then
        frame_id=$(awk '
            /^[[:space:]]*frame_id:/ {
                value=$0
                sub(/^[^:]*:[[:space:]]*/, "", value)
                gsub(/["\047]/, "", value)
                if (value == "cancel") cancelled=1
                if (value ~ /^world:/) last_ready=value
                if (value != "cancel" && value !~ /^world:/) invalid=value
            }
            END {
                if (cancelled) print "cancel"
                else if (invalid != "") print invalid
                else print last_ready
            }
        ' "$CTRL_TRIGGER_MONITOR_FILE")
    fi

    case "$frame_id" in
        world:*) printf 'READY:%s\n' "$frame_id" ;;
        cancel)  printf 'CANCELLED\n' ;;
        "")      printf 'WAITING\n' ;;
        *)       printf 'INVALID:%s\n' "$frame_id" ;;
    esac
}

find_controller_handshake_monitor_node()
{
    local prefix="${CTRL_TRIGGER_MONITOR_NODE}_"

    # rostopic echo 以 anonymous=true 初始化节点；即使传入 __name，运行时名称
    # 仍可能追加 _<pid>_<timestamp>。返回固定名或此前缀下的实际节点名。
    rosnode list 2>/dev/null | awk -v exact="$CTRL_TRIGGER_MONITOR_NODE" -v prefix="$prefix" '
        $0 == exact || index($0, prefix) == 1 {print; exit}
    '
}

start_controller_handshake_monitor()
{
    local start_time
    local monitor_node=""

    assert_startup_topic_endpoint_any "$CTRL_TRIGGER_TOPIC" Publishers controller \
        /drone0/manager /drone0/ctrl_node
    assert_startup_topic_endpoint_any "$CTRL_TRIGGER_TOPIC" Subscribers planner \
        /manager /drone0/sim_odom
    monitor_node=$(find_controller_handshake_monitor_node)
    [[ -z "$monitor_node" ]] ||
        fatal "发现残留握手监听节点 ${monitor_node}；请先检查并关闭旧任务。"

    CTRL_TRIGGER_MONITOR_FILE=$(mktemp /tmp/ham_ctrl_trigger.XXXXXX.log) ||
        fatal "无法创建控制器握手监听文件。"
    PYTHONUNBUFFERED=1 rostopic echo "$CTRL_TRIGGER_TOPIC" \
        __name:="${CTRL_TRIGGER_MONITOR_NODE#/}" __ns:=/ \
        >"$CTRL_TRIGGER_MONITOR_FILE" 2>/dev/null &
    CTRL_TRIGGER_MONITOR_PID=$!
    start_time=$SECONDS

    while (( SECONDS - start_time < 5 )); do
        monitor_node=$(find_controller_handshake_monitor_node)
        if [[ -n "$monitor_node" ]] &&
           topic_has_endpoint "$CTRL_TRIGGER_TOPIC" Subscribers "$monitor_node"; then
            log OK "已提前监听 ${CTRL_TRIGGER_TOPIC}（node=${monitor_node}），不会漏掉一次性 READY/CANCEL 消息。"
            return 0
        fi
        kill -0 "$CTRL_TRIGGER_MONITOR_PID" 2>/dev/null ||
            fatal "控制器握手监听进程启动失败。"
        sleep 0.1
    done

    if [[ -n "$monitor_node" ]]; then
        fatal "握手监听节点 ${monitor_node} 已启动，但 5 s 内未订阅 ${CTRL_TRIGGER_TOPIC}。"
    fi
    fatal "5 s 内未发现 ${CTRL_TRIGGER_TOPIC} 握手监听节点。"
}

assert_controller_not_cancelled()
{
    local phase="$1"
    local status
    status=$(controller_handshake_status)

    case "$status" in
        CANCELLED)
            fatal "ctrl_node 在${phase}发布了 CANCEL：起飞 FSM 未进入 READY，禁止继续触发轨迹。"
            ;;
        INVALID:*)
            fatal "${CTRL_TRIGGER_TOPIC} 收到非法 frame_id=${status#INVALID:}。"
            ;;
    esac
}

wait_for_controller_ready()
{
    local start_time=$SECONDS
    local status state connected mode armed

    log WAIT "等待 ctrl_node 起飞 FSM 发布 READY（${CTRL_TRIGGER_TOPIC} 的 world:<id>）..."
    while (( SECONDS - start_time < CTRL_READY_WAIT_TIMEOUT )); do
        status=$(controller_handshake_status)
        case "$status" in
            READY:*)
                CTRL_READY_ID="${status#READY:world:}"
                [[ "$CTRL_READY_ID" =~ ^[1-9][0-9]*$ ]] ||
                    fatal "ctrl_node READY 中的 trajectory_id 无效：${status#READY:}。"
                log OK "ctrl_node 起飞 FSM 已 READY：${status#READY:}。"
                return 0
                ;;
            CANCELLED)
                fatal "ctrl_node 发布了 CANCEL：可能是 TAKEOFF_TIMEOUT、退出 OFFBOARD 或导航异常，禁止继续触发轨迹。"
                ;;
            INVALID:*)
                fatal "${CTRL_TRIGGER_TOPIC} 收到非法 frame_id=${status#INVALID:}。"
                ;;
        esac

        kill -0 "$CTRL_TRIGGER_MONITOR_PID" 2>/dev/null ||
            fatal "等待 READY 时控制器握手监听进程意外退出。"
        warn_if_recording_inactive
        state=$(read_state "$STATE_TOPIC") || fatal "等待 READY 时无法读取 PX4 状态。"
        connected=$(state_value "$state" connected)
        mode=$(state_value "$state" mode)
        armed=$(state_value "$state" armed)
        [[ "$connected" == "True" && "$mode" == "OFFBOARD" && "$armed" == "True" ]] ||
            fatal "等待 READY 时飞行状态异常：connected=${connected:-unknown}, mode=${mode:-unknown}, armed=${armed:-unknown}。"
        sleep 0.1
    done

    fatal "起飞后 ${CTRL_READY_WAIT_TIMEOUT} s 内未收到 ctrl_node READY；禁止仅凭脚本高度判断进入机动。"
}

publish_takeoff()
{
    assert_startup_topic_endpoint_any "$TAKEOFF_LAND_TOPIC" Subscribers controller \
        /drone0/manager /drone0/ctrl_node

    log ACTION "发送一次起飞命令。"
    TAKEOFF_PENDING=1
    rostopic pub -1 "$TAKEOFF_LAND_TOPIC" quadrotor_msgs/TakeoffLand \
        "{takeoff_land_cmd: 1}" >/dev/null || fatal "起飞命令发布失败。"
}

wait_for_armed()
{
    local start_time=$SECONDS
    local state mode armed connected

    log WAIT "等待 PX4 解锁（最长 ${ARM_WAIT_TIMEOUT} s）..."
    while (( SECONDS - start_time < ARM_WAIT_TIMEOUT )); do
        assert_controller_not_cancelled "等待解锁期间"
        warn_if_recording_inactive
        state=$(read_state "$STATE_TOPIC") || { sleep 1; continue; }
        connected=$(state_value "$state" connected)
        armed=$(state_value "$state" armed)
        mode=$(state_value "$state" mode)

        [[ "$connected" == "True" ]] || fatal "等待解锁时 MAVROS 连接断开。"
        [[ "$mode" == "OFFBOARD" ]] || fatal "等待解锁时已经退出 OFFBOARD。"

        if [[ "$armed" == "True" ]]; then
            AIRBORNE=1
            TAKEOFF_PENDING=0
            log OK "PX4 已解锁，起飞阶段开始。"
            return 0
        fi
        sleep 1
    done

    fatal "PX4 未在 ${ARM_WAIT_TIMEOUT} s 内解锁；起飞命令可能被拒绝。"
}

trajectory_total_time()
{
    local traj_type rise duration hold descend scale start_hold forward end_hold return_time
    traj_type=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/traj_type") || return 1
    scale=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/speed_scale") || return 1
    is_number "$traj_type" && is_number "$scale" || return 1

    if [[ "$traj_type" == "3" ]]; then
        start_hold=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/line_start_hold_time") || return 1
        forward=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/line_forward_time") || return 1
        end_hold=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/line_end_hold_time") || return 1
        return_time=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/line_return_time") || return 1
        is_number "$start_hold" && is_number "$forward" &&
            is_number "$end_hold" && is_number "$return_time" || return 1
        awk -v a="$start_hold" -v b="$forward" -v c="$end_hold" -v d="$return_time" -v s="$scale" \
            'BEGIN {if(s<=0) exit 1; printf "%.6f", (a+b+c+d)/s}'
        return
    fi

    rise=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/traj_rise_time") || return 1
    duration=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/traj_duration") || return 1
    hold=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/traj_hold_time") || return 1
    descend=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/traj_descend_time") || return 1

    is_number "$rise" && is_number "$duration" && is_number "$hold" &&
        is_number "$descend" && is_number "$scale" || return 1
    awk -v a="$rise" -v b="$duration" -v c="$hold" -v d="$descend" -v s="$scale" \
        'BEGIN {if(s<=0) exit 1; printf "%.6f", (a+b+c+d)/s}'
}

publish_trajectory_trigger()
{
    local status expected_status
    status=$(controller_handshake_status)
    expected_status="READY:world:${CTRL_READY_ID}"
    case "$status" in
        CANCELLED)
            fatal "发布轨迹触发前收到 ctrl_node CANCEL。"
            ;;
    esac
    [[ -n "$CTRL_READY_ID" && "$status" == "$expected_status" ]] ||
        fatal "发布轨迹触发前控制器 READY 状态无效：${status}。"
    # 规划器与控制器端点已在录包前确定性确认；空中不再重复执行易受
    # ROS XML-RPC 抖动影响的端点/录包诊断，直接发布飞行关键命令。

    log ACTION "发布一次轨迹触发。"
    rostopic pub -1 "$TRAJECTORY_TRIGGER_TOPIC" geometry_msgs/PoseStamped \
        "{header: {stamp: now, frame_id: ''}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" \
        >/dev/null || fatal "轨迹触发发布失败。"
}

wait_for_trajectory_wall_time()
{
    local total="$1"
    local wait_s elapsed_s remaining_s
    local start_time=$SECONDS
    wait_s=$(awk -v t="$total" -v m="$TRAJECTORY_WALL_TIME_MARGIN" \
        'BEGIN {printf "%d", t + m + 0.999999}')

    log WAIT "轨迹触发已发送；按运行时总时长 ${total} s 加 ${TRAJECTORY_WALL_TIME_MARGIN} s 余量等待，不再用临时 rostopic 读取判定启动/完成。"

    while (( SECONDS - start_time < wait_s )); do
        assert_controller_not_cancelled "轨迹墙钟等待期间"
        elapsed_s=$((SECONDS - start_time))
        remaining_s=$((wait_s - elapsed_s))
        if (( elapsed_s == 0 || elapsed_s % 10 == 0 )); then
            log INFO "轨迹墙钟进度：${elapsed_s}/${wait_s} s，剩余约 ${remaining_s} s。"
        fi
        sleep 1
    done

    log OK "轨迹墙钟窗口结束；再等待 ${POST_TRAJECTORY_SETTLE_DELAY} s 后发送降落命令。"
    sleep "$POST_TRAJECTORY_SETTLE_DELAY"
}

publish_land()
{
    log ACTION "发送一次降落命令；控制器将立即请求 PX4 AUTO.LAND。"
    rostopic pub -1 "$TAKEOFF_LAND_TOPIC" quadrotor_msgs/TakeoffLand \
        "{takeoff_land_cmd: 2}" >/dev/null || fatal "降落命令发布失败。"
}

confirm_landing_started()
{
    local start_time=$SECONDS

    log WAIT "确认控制器已接受降落命令..."
    while (( SECONDS - start_time < 6 )); do
        local state mode armed
        state=$(read_state "$STATE_TOPIC") || { sleep 1; continue; }
        mode=$(state_value "$state" mode)
        armed=$(state_value "$state" armed)

        if [[ "$armed" == "False" || "$mode" == "AUTO.LAND" ]]; then
            log OK "PX4 已进入降落/上锁阶段：mode=${mode:-unknown}, armed=${armed:-unknown}。"
            return 0
        fi

        if [[ "$mode" != "OFFBOARD" ]]; then
            fatal "确认降落期间进入非预期模式 ${mode:-unknown}；视为遥控器接管。"
        fi

        sleep 1
    done

    return 1
}

wait_for_disarmed()
{
    local start_time=$SECONDS
    local last_mode=""

    log WAIT "等待控制器降落、PX4 AUTO.LAND 和自动上锁（最长 ${LAND_WAIT_TIMEOUT} s）..."
    while (( SECONDS - start_time < LAND_WAIT_TIMEOUT )); do
        local state connected mode armed
        state=$(read_state "$STATE_TOPIC") || { sleep 1; continue; }
        connected=$(state_value "$state" connected)
        mode=$(state_value "$state" mode)
        armed=$(state_value "$state" armed)

        [[ "$connected" == "True" ]] || fatal "降落期间 MAVROS 连接断开。"

        if [[ "$mode" != "$last_mode" ]]; then
            log INFO "降落阶段 PX4 mode=${mode:-unknown}, armed=${armed:-unknown}。"
            last_mode="$mode"
        fi

        if [[ "$armed" == "False" ]]; then
            log OK "PX4 已上锁；继续等待 ON_GROUND，暂不判定任务完成。"
            return 0
        fi

        if [[ "$mode" != "OFFBOARD" && "$mode" != "AUTO.LAND" ]]; then
            fatal "降落期间进入非预期模式 ${mode:-unknown}；视为遥控器接管，停止自动流程。"
        fi
        sleep 1
    done

    fatal "降落后 ${LAND_WAIT_TIMEOUT} s 内 PX4 仍未上锁。"
}

wait_until_on_ground()
{
    local start_time=$SECONDS
    while (( SECONDS - start_time < 10 )); do
        local extended landed_state
        extended=$(timeout 3 rostopic echo -n 1 "$EXTENDED_STATE_TOPIC" 2>/dev/null || true)
        landed_state=$(awk -F': ' '/^[[:space:]]*landed_state:/ {print $2; exit}' <<<"$extended")
        if [[ "$landed_state" == "1" ]]; then
            AIRBORNE=0
            TAKEOFF_PENDING=0
            log OK "PX4 确认 LANDED_STATE_ON_GROUND。"
            return 0
        fi
        sleep 1
    done
    fatal "PX4 已上锁，但 10 s 内未确认 ON_GROUND；禁止报告任务完成，请人工检查 landed_state。"
}

main()
{
    local use_sim_time

    echo "================ HAM 实飞轨迹任务 ================"

    # ---------- 操作 01：检查脚本依赖命令与录包脚本 ----------
    for command in timeout rostopic rosnode rosservice rosparam rosrun awk grep df mktemp gnome-terminal; do
        require_command "$command"
    done

    [[ -f "$RECORD_SCRIPT" ]] || fatal "录包脚本不存在：${RECORD_SCRIPT}"

    # ---------- 操作 02：等待 ROS master，并确认使用实时时钟 ----------
    wait_roscore "$STARTUP_RETRY_TIMEOUT" ||
        fatal "ROS master 不可用；请先运行修复后的 RTK.sh。"
    # /use_sim_time 未设置时 ROS 默认 false；先可靠取得参数清单，只有确认缺失
    # 才使用默认值，避免把 master 瞬时错误误当成 false。
    use_sim_time=$(read_optional_rosparam_with_retry /use_sim_time false) ||
        fatal "${STARTUP_RETRY_TIMEOUT} s 内无法读取 /use_sim_time。"
    case "$use_sim_time" in
        false|False) ;;
        true|True) fatal "实飞禁止 /use_sim_time=true。" ;;
        *) fatal "/use_sim_time 值无效：${use_sim_time:-missing}。" ;;
    esac

    # ---------- 操作 03：检查录包前控制链路的 ROS 图端点 ----------
    # 图端点是确定性门禁：必须先于录包通过。
    assert_pre_record_control_graph

    # ---------- 操作 04：检查初始 PX4、RTK、EKF 与录包磁盘状态 ----------
    assert_safe_preflight_state
    assert_rtk_fixed
    assert_ekf_ready
    check_disk_space

    # ---------- 操作 05：读取并校验本次起飞与轨迹运行时参数 ----------
    local initial_position initial_x initial_y initial_z takeoff_height takeoff_height_now
    local target_z lower_z total_time confirmation
    local traj_type traj_radius traj_amp_y traj_height path_scale speed_scale line_forward_distance
    total_time=$(trajectory_total_time) || fatal "无法从运行时 ROS 参数计算轨迹总时长。"
    traj_type=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/traj_type") ||
        fatal "缺少运行时轨迹参数 traj_type。"
    [[ "$traj_type" != "0" ]] ||
        fatal "traj_type=0 是无限垂直驻留任务，不能由 whole_traj 自动等待结束；请使用 pub_trigger.sh，并在试验完成后显式执行 takeland.sh。"
    traj_radius=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/traj_radius") ||
        fatal "缺少运行时轨迹参数 traj_radius。"
    traj_amp_y=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/traj_amp_y") ||
        fatal "缺少运行时轨迹参数 traj_amp_y。"
    traj_height=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/traj_height") ||
        fatal "缺少运行时轨迹参数 traj_height。"
    path_scale=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/path_scale") ||
        fatal "缺少运行时轨迹参数 path_scale。"
    speed_scale=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/speed_scale") ||
        fatal "缺少运行时轨迹参数 speed_scale。"
    if [[ "$traj_type" == "3" ]]; then
        line_forward_distance=$(read_rosparam_with_retry "${TRAJECTORY_PARAM_NS}/line_forward_distance") ||
            fatal "缺少运行时直线参数 line_forward_distance。"
    fi
    local navigation_monitor_enabled
    navigation_monitor_enabled=$(read_rosparam_with_retry "${CONTROL_PARAM_NS}/navigation_monitor/enabled") ||
        fatal "缺少 RTK/EKF 持续监测参数 navigation_monitor/enabled。"
    [[ "$navigation_monitor_enabled" == "true" || "$navigation_monitor_enabled" == "True" ]] ||
        fatal "RTK 实飞禁止关闭 navigation_monitor/enabled；请使用 RTK 启动脚本加载控制器。"

    takeoff_height=$(read_rosparam_with_retry "${CONTROL_PARAM_NS}/takeoff_state/height") ||
        fatal "缺少起飞高度参数 ${CONTROL_PARAM_NS}/takeoff_state/height"
    is_number "$takeoff_height" || fatal "起飞高度参数不是数字：${takeoff_height}"
    awk -v h="$takeoff_height" 'BEGIN {exit !(h>0)}' ||
        fatal "起飞高度参数必须大于 0：${takeoff_height}"

    # ---------- 操作 06：显示任务参数并等待人工输入 TAKEOFF ----------
    echo "---------------------------------------------------"
    echo "起飞高度：${takeoff_height} m；高度容差：${TAKEOFF_REACH_TOLERANCE} m（与 ctrl_fsm.cc 一致）"
    echo "起飞基准位置将在切入 OFFBOARD 后、发送起飞命令前重新读取。"
    if [[ "$traj_type" == "3" ]]; then
        echo "轨迹空间参数：type=3, 沿触发时机头方向前飞 ${line_forward_distance} m 后原路返回"
    else
        echo "轨迹空间参数：type=${traj_type}, radius=${traj_radius} m, amp_y=${traj_amp_y} m, height=${traj_height} m"
    fi
    echo "轨迹缩放参数：path_scale=${path_scale}, speed_scale=${speed_scale}"
    echo "轨迹总时长：${total_time} s（读取运行时参数，不使用固定 55 s）"
    echo "轨迹后续策略：触发后墙钟等待总时长+${TRAJECTORY_WALL_TIME_MARGIN} s，再静置 ${POST_TRAJECTORY_SETTLE_DELAY} s 后降落。"
    echo "脚本不会切换 OFFBOARD；进入 OFFBOARD 后将发送起飞命令并由 FSM 请求解锁。"
    echo "确认桨区清空、遥控器接管开关有效、RTK/EKF 正常后，输入 TAKEOFF。"
    read -r -p "> " confirmation || fatal "未获得起飞确认。"
    [[ "$confirmation" == "TAKEOFF" ]] || fatal "未输入 TAKEOFF，任务取消。"

    # ---------- 操作 07：在安全窗口写入 PX4 降落参数并核对推力映射 ----------
    # 以下均为不依赖录包的确定性门禁。任何一项失败都不会创建 bag。
    # PX4 参数写入必须在未解锁、非 OFFBOARD 且已确认落地的窗口内进行。
    configure_px4_auto_land
    assert_controller_parameter_consistency

    # ---------- 操作 08：复核录包前 ODOM、setpoint、RTK 与 EKF ----------
    wait_topic_messages "$ODOM_TOPIC" 3 "$STARTUP_RETRY_TIMEOUT" ||
        fatal "录包前 ${ODOM_TOPIC} 未连续发布。"
    wait_topic_messages "$SETPOINT_TOPIC" 3 "$STARTUP_RETRY_TIMEOUT" ||
        fatal "录包前 ${SETPOINT_TOPIC} 未连续发布。"
    assert_rtk_fixed
    assert_ekf_ready

    # ---------- 操作 09：冻结并复核人工确认后的起飞高度 ----------
    # 人工确认后冻结起飞高度；禁止把这个静态一致性检查留到录包后。
    takeoff_height_now=$(read_rosparam_with_retry "${CONTROL_PARAM_NS}/takeoff_state/height") ||
        fatal "录包前无法重新读取 takeoff_state/height。"
    is_number "$takeoff_height_now" || fatal "录包前 takeoff_state/height 不是数字。"
    awk -v before="$takeoff_height" -v now="$takeoff_height_now" '
        BEGIN {d=before-now; if(d<0)d=-d; exit !(d<=0.000001)}' ||
        fatal "任务确认后 takeoff_state/height 发生变化：${takeoff_height} -> ${takeoff_height_now}。请重启并重新确认。"

    # ---------- 操作 10：提前建立 READY/CANCEL 持久握手监听 ----------
    # 在人工切换 OFFBOARD 及录包之前建立监听，保留第一个 world:<id>/cancel。
    start_controller_handshake_monitor
    assert_controller_not_cancelled "启动录包前"

    # ---------- 操作 11：封闭地面安全窗口并启动独立 rosbag ----------
    # 参数写入和所有确定性门禁完成后，以新鲜 state 和本次运行已确认的
    # ON_GROUND（实时读取失败时最多缓存 300 s）封闭安全窗口。
    assert_safe_preflight_state
    log OK "录包前确定性门禁全部通过；现在启动任务录包。"
    start_recording

    # ---------- 操作 12：等待人工切入 OFFBOARD ----------
    wait_for_external_offboard

    # ---------- 操作 13：读取 OFFBOARD 后最新起飞基准并计算目标高度 ----------
    # 检测到 OFFBOARD 后立即读取一条最新位置，但不恢复 ODOM 连续发布复核。
    initial_position=$(read_vector_with_retry "${ODOM_TOPIC}/pose/pose/position") ||
        fatal "${STARTUP_RETRY_TIMEOUT} s 内无法读取 OFFBOARD 后 ${ODOM_TOPIC} 最新起飞位置。"
    read -r initial_x initial_y initial_z <<<"$initial_position"
    target_z=$(awk -v z="$initial_z" -v h="$takeoff_height" 'BEGIN {printf "%.6f", z+h}')
    lower_z=$(awk -v z="$target_z" -v e="$TAKEOFF_REACH_TOLERANCE" 'BEGIN {printf "%.6f", z-e}')
    log OK "OFFBOARD 后最新起飞基准：x=${initial_x}, y=${initial_y}, z=${initial_z} m；目标 z=${target_z} m，完成下限=${lower_z} m。"
    log WARN "本版本仍跳过 OFFBOARD 后的未解锁/落地、ODOM 连续性、setpoint、RTK 和 EKF 复核。"
    assert_controller_not_cancelled "发送起飞命令前"

    # ---------- 操作 14：发布一次起飞命令 ----------
    publish_takeoff

    # ---------- 操作 15：等待 PX4 解锁 ----------
    wait_for_armed

    # ---------- 操作 16：等待 ctrl_node 发布带轨迹 ID 的 READY ----------
    wait_for_controller_ready

    # ---------- 操作 17：复核 READY/CANCEL 与轨迹 ID，并发布一次轨迹触发 ----------
    publish_trajectory_trigger

    # ---------- 操作 18：按运行时轨迹总时长进行墙钟等待并静置 ----------
    wait_for_trajectory_wall_time "$total_time"

    # ---------- 操作 19：发布降落命令；未确认时最多重试一次 ----------
    publish_land
    if ! confirm_landing_started; then
        log WARN "第一次降落命令在 6 s 内未确认 AUTO.LAND/上锁；仅重试一次，不再执行 ODOM 静止检查。"
        publish_land
        confirm_landing_started ||
            log WARN "第二次降落命令后仍无法即时确认；停止重试，继续等待 PX4 自动上锁。"
    fi
    # ---------- 操作 20：等待 PX4 自动上锁并确认 ON_GROUND ----------
    wait_for_disarmed
    wait_until_on_ground

    # ---------- 操作 21：报告任务完成，rosbag 保持运行等待人工停止 ----------
    echo "================ TASK COMPLETE ===================="
    log OK "轨迹、降落和自动上锁全部完成。"
    log INFO "rosbag 仍在独立窗口记录；请切换到录包窗口按 Ctrl+C，等待索引写入完成后再关闭窗口。"
    log INFO "MAVROS、规划器、odomRemap 和控制器保持运行。"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    main "$@"
fi
