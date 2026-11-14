#!/usr/bin/env bash

# One-terminal launcher/operator for the RTK + AprilTag pickup mission.
# It reuses the exact same launch files, preflight, action scripts and params as
# the documented seven-terminal method.  `start` never arms or takes off.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HAM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
STATE_DIR="/tmp/ham_apriltag_easy_${UID}"
LOG_ROOT="${HAM_ROOT}/easy_flight_logs"
ACTIVE_FILE="${STATE_DIR}/active"
META_FILE="${STATE_DIR}/metadata"
READY_LOG="${STATE_DIR}/ready.log"
LOCK_DIR="${STATE_DIR}.lock"
FCU_URL_DEFAULT="/dev/ttyACM0:57600"

# shellcheck source=sh_utils/_ros_common.sh
source "${SCRIPT_DIR}/_ros_common.sh"

source_ros()
{
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
    # shellcheck disable=SC1090
    source "${HAM_ROOT}/devel/setup.bash"
}

usage()
{
    cat <<'EOF'
用法：
  bash sh_utils/apriltag_rtk_easy_flight.sh start --dry-run [--record-images]
  bash sh_utils/apriltag_rtk_easy_flight.sh start --live [--record-images]
  bash sh_utils/apriltag_rtk_easy_flight.sh run   --dry-run|--live [--record-images]
  bash sh_utils/apriltag_rtk_easy_flight.sh status
  bash sh_utils/apriltag_rtk_easy_flight.sh prepare
  bash sh_utils/apriltag_rtk_easy_flight.sh takeoff
  bash sh_utils/apriltag_rtk_easy_flight.sh mission
  bash sh_utils/apriltag_rtk_easy_flight.sh abort
  bash sh_utils/apriltag_rtk_easy_flight.sh land
  bash sh_utils/apriltag_rtk_easy_flight.sh stop [--force]
  bash sh_utils/apriltag_rtk_easy_flight.sh logs

说明：
  start   一键启动全部基础节点、完整预检和录包；绝不解锁。
  prepare 交互完成实码可见性检查、将码移回航线、张爪和启用任务。
  run     单终端交互向导；OFFBOARD、TAKEOFF、MISSION、LAND 都需飞手逐项确认。
  --dry-run  不输出真实舵机命令；首次装桨验证必须用此模式。
  --live     允许真实 MAIN1 舵机输出；只在拆桨实物动作和 dry-run 均通过后使用。
  stop    只允许确认落地、下锁、非 OFFBOARD 后关闭；--force 仅供断电后的残留进程。

可选环境变量：FCU_URL=/dev/ttyACM0:57600（默认值同此）。
EOF
}

require_build()
{
    local command
    for command in setsid stdbuf timeout grep awk roslaunch rosbag; do
        command -v "$command" >/dev/null 2>&1 || die "缺少命令：${command}"
    done
    [ -r /opt/ros/noetic/setup.bash ] || die "缺少 ROS Noetic setup.bash。"
    [ -r "${HAM_ROOT}/devel/setup.bash" ] || die "工作区尚未构建；先在 ${HAM_ROOT} 执行 catkin_make。"
}

pid_file()
{
    printf '%s/%s.pid\n' "$STATE_DIR" "$1"
}

pid_alive()
{
    local file pid
    file="$(pid_file "$1")"
    [ -s "$file" ] || return 1
    read -r pid < "$file"
    [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 1
    kill -0 "$pid" 2>/dev/null
}

start_bg()
{
    local name="$1"
    shift
    local log_file="${CURRENT_LOG_DIR}/${name}.log"

    # The single quotes intentionally defer $1/$@ expansion to the child.
    # shellcheck disable=SC2016
    setsid bash -c '
        source /opt/ros/noetic/setup.bash
        source "$1"
        shift
        exec "$@"
    ' _ "${HAM_ROOT}/devel/setup.bash" "$@" >>"$log_file" 2>&1 < /dev/null &
    local pid=$!
    printf '%s\n' "$pid" > "$(pid_file "$name")"
    log OK "已启动 ${name} (pid=${pid}, log=${log_file})。"
}

signal_group()
{
    local name="$1"
    local signal="$2"
    local file pid pgid
    file="$(pid_file "$name")"
    [ -s "$file" ] || return 0
    read -r pid < "$file"
    [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 0
    kill -0 "$pid" 2>/dev/null || return 0
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
    if [ "$pgid" = "$pid" ]; then
        kill "-${signal}" -- "-${pid}" 2>/dev/null || true
    else
        kill "-${signal}" "$pid" 2>/dev/null || true
    fi
}

wait_process_exit()
{
    local name="$1"
    local timeout_s="${2:-10}"
    local deadline=$((SECONDS + timeout_s))
    while pid_alive "$name" && (( SECONDS < deadline )); do
        sleep 0.2
    done
}

stop_owned_processes()
{
    # rosbag must receive SIGINT first so it writes the index cleanly.
    signal_group bag INT
    wait_process_exit bag 15
    signal_group ready INT
    signal_group mission INT
    wait_process_exit mission 12
    signal_group mavros INT
    wait_process_exit mavros 8
    signal_group roscore INT
    wait_process_exit roscore 5

    for name in bag ready mission mavros roscore; do
        if pid_alive "$name"; then
            log WARN "${name} 未及时退出，发送 SIGTERM。"
            signal_group "$name" TERM
        fi
    done
}

load_metadata()
{
    [ -r "$META_FILE" ] || die "找不到便捷任务状态；请先执行 start。"
    # metadata is generated locally using shell-escaped values.
    # shellcheck disable=SC1090
    source "$META_FILE"
    : "${MISSION_MODE:?}" "${CURRENT_LOG_DIR:?}" "${RECORD_IMAGES:?}"
}

require_session()
{
    [ -f "$ACTIVE_FILE" ] || die "便捷任务未启动；请先执行 start。"
    load_metadata
    pid_alive roscore || die "便捷任务的 roscore 已退出；查看 logs，确认落地下锁后 stop。"
    pid_alive mavros || die "便捷任务的 MAVROS 已退出；禁止继续发送飞行命令。"
    pid_alive mission || die "便捷任务组合 launch 已退出；禁止继续发送飞行命令。"
}

state_field()
{
    local text="$1"
    local key="$2"
    awk -F': ' -v wanted="$key" '$1 == wanted {gsub(/[\047\042\r]/, "", $2); print $2; exit}' <<<"$text"
}

topic_data()
{
    local topic="$1"
    local text
    text="$(timeout 4 rostopic echo -n 1 "$topic" 2>/dev/null)" || return 1
    state_field "$text" data
}

show_log_tail()
{
    local name="$1"
    local file="${CURRENT_LOG_DIR}/${name}.log"
    [ -r "$file" ] && tail -n 30 "$file" >&2
}

on_start_error()
{
    local status=$?
    trap - EXIT ERR
    rmdir "$LOCK_DIR" 2>/dev/null || true
    log ERROR "一键启动失败；正在关闭本脚本已启动的地面进程。"
    if [ "${START_PROCESSES:-0}" = "1" ]; then
        stop_owned_processes
    fi
    exit "$status"
}

start_command()
{
    local mission_mode=""
    local record_images=0
    local arg
    for arg in "$@"; do
        case "$arg" in
            --dry-run) mission_mode="dry-run" ;;
            --live) mission_mode="live" ;;
            --record-images) record_images=1 ;;
            *) die "start/run 不支持参数：${arg}" ;;
        esac
    done
    [ -n "$mission_mode" ] || die "必须明确选择 --dry-run 或 --live。"

    require_build
    source_ros

    if ! mkdir "$LOCK_DIR" 2>/dev/null; then
        die "另一个便捷脚本正在操作；稍后重试。"
    fi
    START_PROCESSES=0
    trap on_start_error EXIT

    if [ -f "$ACTIVE_FILE" ]; then
        die "便捷任务已经启动；先运行 status，落地下锁后运行 stop。"
    fi
    if rostopic list >/dev/null 2>&1; then
        die "检测到已有 ROS Master。为避免与旧七终端方法重复启动，请先按原方法安全关闭全部节点。"
    fi

    local fcu_url="${FCU_URL:-$FCU_URL_DEFAULT}"
    [[ "$fcu_url" =~ ^/dev/[A-Za-z0-9._/-]+:[0-9]+$ ]] || die "FCU_URL 格式非法：${fcu_url}"
    local fcu_device="${fcu_url%%:*}"
    [ -e "$fcu_device" ] || die "飞控串口不存在：${fcu_device}"
    if [ ! -r "$fcu_device" ] || [ ! -w "$fcu_device" ]; then
        die "当前用户无权读写飞控串口：${fcu_device}"
    fi

    mkdir -p "$STATE_DIR" "$LOG_ROOT"
    CURRENT_LOG_DIR="${LOG_ROOT}/$(date +%Y-%m-%d-%H-%M-%S)"
    mkdir -p "$CURRENT_LOG_DIR"
    MISSION_MODE="$mission_mode"
    RECORD_IMAGES="$record_images"
    {
        printf 'MISSION_MODE=%q\n' "$MISSION_MODE"
        printf 'RECORD_IMAGES=%q\n' "$RECORD_IMAGES"
        printf 'CURRENT_LOG_DIR=%q\n' "$CURRENT_LOG_DIR"
        printf 'FCU_URL_USED=%q\n' "$fcu_url"
    } > "$META_FILE"
    : > "$READY_LOG"

    START_PROCESSES=1
    start_bg roscore roscore
    wait_roscore 30

    start_bg mavros roslaunch mavros px4.launch "fcu_url:=${fcu_url}"
    wait_mavros_connected 60 "$fcu_url"

    local initial_state
    initial_state="$(read_state /mavros/state)"
    [ "$(state_field "$initial_state" armed)" = "False" ] || die "飞控已经解锁；一键启动拒绝接管。"
    [ "$(state_field "$initial_state" mode)" != "OFFBOARD" ] || die "启动前飞控不得处于 OFFBOARD。"

    log CHECK "配置 MAVLink 数据频率..."
    if ! ENABLE_NAV_STATUS_RATES=1 ID31=10000 ID32=10000 ID331=20000 \
        bash "${SCRIPT_DIR}/mavlink_set_rates.sh" >"${CURRENT_LOG_DIR}/rates.log" 2>&1; then
        show_log_tail rates
        die "MAVLink 数据频率配置失败。"
    fi

    local dry_value=true
    if [ "$MISSION_MODE" = "live" ]; then
        dry_value=false
        log WARN "LIVE 模式：预检通过后会在未解锁状态真实张开 MAIN1 夹爪。"
    fi

    start_bg mission roslaunch vision_gripper apriltag_pick_return.launch \
        start_mavros:=false \
        start_realsense:=true \
        start_trajectory:=true \
        start_odom_remap:=true \
        start_controller:=true \
        "dry_run:=${dry_value}" \
        navigation_monitor_enabled:=true \
        mission_enabled_on_start:=false

    wait_node /apriltag_mission 60
    wait_node /apriltag_gripper 60
    wait_node /manager 30
    wait_node /drone0/manager 30
    wait_nodelet /manager sim_odom 30
    wait_nodelet /drone0/manager odomRemap 30
    wait_nodelet /drone0/manager ctrl_node 30

    # Listen before takeoff so the one-shot READY handshake cannot be missed.
    start_bg ready bash -c \
        'rostopic echo /traj_start_trigger | stdbuf -oL awk '\''/frame_id:.*world:[1-9][0-9]*/ {print; fflush(); exit}'\'''
    # start_bg redirects the watcher output to ready.log by default; mirror its
    # dedicated log path for later checks.
    READY_LOG="${CURRENT_LOG_DIR}/ready.log"

    log CHECK "执行与七终端方法完全相同的只读 RTK/EKF/相机/拓扑预检..."
    rosrun vision_gripper apriltag_rtk_preflight.py 2>&1 | tee "${CURRENT_LOG_DIR}/preflight.log"

    start_bg bag env "RECORD_CAMERA_IMAGES=${RECORD_IMAGES}" \
        "${SCRIPT_DIR}/record.sh"
    wait_node /record 20

    touch "$ACTIVE_FILE"
    START_PROCESSES=0
    trap - EXIT ERR
    rmdir "$LOCK_DIR" 2>/dev/null || true

    log OK "一键地面启动完成；所有节点、完整预检和录包已就绪。"
    log INFO "日志目录：${CURRENT_LOG_DIR}"
    log INFO "rosbag 目录：${SCRIPT_DIR}/Bag"
    cat <<EOF

下一步（不会由 start 自动执行）：
  1. 执行实码检查/张爪/任务准备：bash ${SCRIPT_DIR}/apriltag_rtk_easy_flight.sh prepare
  2. 飞手用遥控器切到 OFFBOARD。
  3. 执行：bash ${SCRIPT_DIR}/apriltag_rtk_easy_flight.sh takeoff
  4. READY 成功后执行：bash ${SCRIPT_DIR}/apriltag_rtk_easy_flight.sh mission
  5. 返回 HOLD_HOME 后执行：bash ${SCRIPT_DIR}/apriltag_rtk_easy_flight.sh land
  6. 确认落地下锁后执行：bash ${SCRIPT_DIR}/apriltag_rtk_easy_flight.sh stop
EOF
}

prepare_command()
{
    require_session
    source_ros
    [ -t 0 ] || die "prepare 需要飞手交互确认，必须在可输入终端中运行。"
    [ ! -f "${STATE_DIR}/prepared" ] || die "本次任务已完成 prepare，禁止重复张爪/启用。"

    local state extended mission_status answer
    state="$(timeout 4 rostopic echo -n 1 /mavros/state 2>/dev/null)" || die "无法读取 MAVROS state。"
    extended="$(timeout 4 rostopic echo -n 1 /mavros/extended_state 2>/dev/null)" || die "无法读取落地状态。"
    [ "$(state_field "$state" armed)" = "False" ] || die "prepare 仅允许在未解锁时执行。"
    [ "$(state_field "$state" mode)" != "OFFBOARD" ] || die "prepare 前必须退出 OFFBOARD。"
    [ "$(state_field "$extended" landed_state)" = "1" ] || die "prepare 仅允许在 ON_GROUND 时执行。"
    mission_status="$(topic_data /apriltag_mission/status)" || die "无法读取 mission status。"
    grep -Eqi 'disabled' <<<"$mission_status" || die "mission 不是 disabled，拒绝重复准备。"

    printf '\n确认飞行器未解锁、电机不会转，将 tag36h11 / ID 0 纸张放到彩色相机正下方 0.7～1.0 m。\n'
    read -r -p "黑色外边完整进入画面后输入 CHECK：" answer
    [ "$answer" = "CHECK" ] || die "未确认实码检查；任务保持 disabled。"

    log CHECK "等待连续 3 条真实 AprilTag 位置..."
    if ! timeout 8 rostopic echo -n 3 /apriltag_gripper/tag_position \
        >"${CURRENT_LOG_DIR}/tag_visibility.log" 2>&1; then
        show_log_tail tag_visibility
        die "8 s 内未连续看到 3 条标签位置；禁止起飞。"
    fi
    log OK "实码可见性检查通过。"

    read -r -p "现在将标签/物品放回 3 m 航线中的目标位置，固定好后输入 READY：" answer
    [ "$answer" = "READY" ] || die "标签未确认移回航线；任务保持 disabled。"

    local response
    response="$(rosservice call /apriltag_gripper/reset)"
    printf '%s\n' "$response" >"${CURRENT_LOG_DIR}/tag_reset.log"
    grep -Eq 'success: True' <<<"$response" || die "AprilTag 滤波复位失败；禁止起飞。"

    # start 时的预检可能与真正起飞相隔较久。在飞机最终摆放位置、
    # 任务仍 disabled 时再做一次完整 5 s 预检，特别拒绝 PX4 原始
    # local pose 上下跳变、/odom/remap 零点偏移或 RTK/EKF 不连续。
    log CHECK "在最终起飞点再次执行 5 s 定位连续性预检..."
    rosrun vision_gripper apriltag_rtk_preflight.py 2>&1 \
        | tee "${CURRENT_LOG_DIR}/preflight_final_position.log"

    response="$(rosservice call /apriltag_gripper/set_gripper "data: false")"
    printf '%s\n' "$response" | tee "${CURRENT_LOG_DIR}/gripper_open.log"
    grep -Eq 'success: True' <<<"$response" || die "夹爪张开服务失败；禁止起飞。"

    response="$(rosservice call /apriltag_mission/enable "data: true")"
    printf '%s\n' "$response" | tee "${CURRENT_LOG_DIR}/mission_enable.log"
    grep -Eq 'success: True' <<<"$response" || die "任务接管启用失败；禁止起飞。"

    touch "${STATE_DIR}/prepared"
    log OK "标签已检查并移回航线，夹爪已张开，单次任务已启用。"
}

quick_takeoff_gate()
{
    local state extended gps estimator mission_state
    state="$(timeout 4 rostopic echo -n 1 /mavros/state 2>/dev/null)" || die "无法读取 MAVROS state。"
    extended="$(timeout 4 rostopic echo -n 1 /mavros/extended_state 2>/dev/null)" || die "无法读取落地状态。"
    gps="$(timeout 4 rostopic echo -n 1 /mavros/gpsstatus/gps1/raw 2>/dev/null)" || die "无法读取 RTK 状态。"
    estimator="$(timeout 4 rostopic echo -n 1 /mavros/estimator_status 2>/dev/null)" || die "无法读取 EKF 状态。"
    mission_state="$(topic_data /apriltag_mission/state)" || die "无法读取任务状态。"

    [ "$(state_field "$state" connected)" = "True" ] || die "MAVROS 未连接。"
    [ "$(state_field "$state" armed)" = "False" ] || die "起飞前应为未解锁。"
    [ "$(state_field "$state" mode)" = "OFFBOARD" ] || die "请先由飞手使用遥控器切入 OFFBOARD。"
    [ "$(state_field "$state" system_status)" = "3" ] || die "PX4 不是 STANDBY(3)。"
    [ "$(state_field "$extended" landed_state)" = "1" ] || die "PX4 未确认 ON_GROUND。"
    [ "$(state_field "$gps" fix_type)" -ge 6 ] || die "RTK 不是 Fixed(fix_type>=6)。"
    [ "$(state_field "$gps" satellites_visible)" -ge 8 ] || die "可见卫星少于 8。"
    [ "$(state_field "$estimator" pos_horiz_abs_status_flag)" = "True" ] || die "EKF 水平绝对位置无效。"
    [ "$(state_field "$estimator" pos_vert_abs_status_flag)" = "True" ] || die "EKF 垂直绝对位置无效。"
    [ "$(state_field "$estimator" const_pos_mode_status_flag)" = "False" ] || die "EKF 处于 constant-position。"
    [ "$(state_field "$estimator" gps_glitch_status_flag)" = "False" ] || die "EKF 报告 GPS glitch。"
    [ "$mission_state" = "FOLLOW_PRESET" ] || die "任务状态不是 FOLLOW_PRESET：${mission_state}"
    [ -f "${STATE_DIR}/prepared" ] || die "尚未执行 prepare 实码检查/张爪/任务启用。"
    pid_alive bag || die "rosbag 录包进程不在运行；禁止起飞。"
}

takeoff_command()
{
    require_session
    source_ros
    [ ! -f "${STATE_DIR}/takeoff_sent" ] || die "本次任务已经发送过 TAKEOFF；禁止重复发送。"
    quick_takeoff_gate
    log OK "起飞前快速门禁通过；发布一次 TAKEOFF。"
    bash "${SCRIPT_DIR}/takeoff.sh"
    touch "${STATE_DIR}/takeoff_sent"

    local deadline=$((SECONDS + 25))
    log WAIT "等待精确 1.0 m 起飞、稳定和带 ID 的控制器 READY..."
    while (( SECONDS < deadline )); do
        if grep -Eq 'frame_id:.*world:[1-9][0-9]*' "$READY_LOG" 2>/dev/null; then
            log OK "控制器 READY；初始机头 yaw 已锁定。"
            return 0
        fi
        local state
        state="$(timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null || true)"
        if [ "$(state_field "$state" armed)" = "False" ] && (( SECONDS + 5 < deadline )); then
            sleep 0.5
            continue
        fi
        sleep 0.5
    done
    die "25 s 内未收到 READY；禁止触发轨迹。起飞超时会请求 PX4 AUTO.LAND；飞手立即确认模式/接管并保留 bag 和 ctrl_node 日志。"
}

mission_command()
{
    require_session
    source_ros
    [ ! -f "${STATE_DIR}/mission_sent" ] || die "本次任务已经发送过 MISSION；禁止重复发送。"
    grep -Eq 'frame_id:.*world:[1-9][0-9]*' "$READY_LOG" || die "没有带非零 ID 的控制器 READY。"

    local state mission_state z_text v_text z speed
    state="$(timeout 4 rostopic echo -n 1 /mavros/state 2>/dev/null)" || die "无法读取 MAVROS state。"
    [ "$(state_field "$state" connected)" = "True" ] || die "MAVROS 未连接。"
    [ "$(state_field "$state" armed)" = "True" ] || die "飞机尚未解锁起飞。"
    [ "$(state_field "$state" mode)" = "OFFBOARD" ] || die "飞机不在 OFFBOARD。"
    mission_state="$(topic_data /apriltag_mission/state)" || die "无法读取任务状态。"
    [ "$mission_state" = "FOLLOW_PRESET" ] || die "任务状态不是 FOLLOW_PRESET：${mission_state}"
    [ -f "${STATE_DIR}/prepared" ] || die "本次任务未完成 prepare。"

    z_text="$(timeout 4 rostopic echo -n 1 /odom/remap/pose/pose/position 2>/dev/null)" || die "无法读取悬停高度。"
    v_text="$(timeout 4 rostopic echo -n 1 /odom/remap/twist/twist/linear 2>/dev/null)" || die "无法读取悬停速度。"
    z="$(state_field "$z_text" z)"
    speed="$(awk -F': ' '
        $1 == "x" {x=$2} $1 == "y" {y=$2} $1 == "z" {z=$2}
        END {printf "%.6f", sqrt(x*x+y*y+z*z)}
    ' <<<"$v_text")"
    awk -v z="$z" 'BEGIN {exit !(z >= 0.85 && z <= 1.15)}' || die "高度 ${z} m 不在 1.0 m 安全触发窗口。"
    awk -v v="$speed" 'BEGIN {exit !(v <= 0.20)}' || die "悬停速度 ${speed} m/s 过大。"
    pid_alive bag || die "rosbag 已退出；禁止触发任务。"

    bash "${SCRIPT_DIR}/pub_trigger.sh"
    touch "${STATE_DIR}/mission_sent"
    log OK "轨迹已触发一次；禁止重复执行 mission。"
}

status_command()
{
    require_session
    source_ros
    local state ext mission_state mission_status complete pickup
    state="$(timeout 3 rostopic echo -n 1 /mavros/state 2>/dev/null || true)"
    ext="$(timeout 3 rostopic echo -n 1 /mavros/extended_state 2>/dev/null || true)"
    mission_state="$(topic_data /apriltag_mission/state 2>/dev/null || printf unavailable)"
    mission_status="$(topic_data /apriltag_mission/status 2>/dev/null || printf unavailable)"
    complete="$(topic_data /apriltag_mission/mission_complete 2>/dev/null || printf unavailable)"
    pickup="$(topic_data /apriltag_mission/pickup_succeeded 2>/dev/null || printf unavailable)"
    cat <<EOF
session_mode:      ${MISSION_MODE}
log_dir:           ${CURRENT_LOG_DIR}
mavros_connected:  $(state_field "$state" connected)
armed:             $(state_field "$state" armed)
mode:              $(state_field "$state" mode)
system_status:     $(state_field "$state" system_status)
landed_state:      $(state_field "$ext" landed_state)
mission_state:     ${mission_state}
mission_status:    ${mission_status}
mission_complete:  ${complete}
pickup_succeeded:  ${pickup}
rosbag_running:    $(pid_alive bag && printf yes || printf no)
prepared:          $([ -f "${STATE_DIR}/prepared" ] && printf yes || printf no)
ready_received:    $(grep -Eq 'frame_id:.*world:[1-9][0-9]*' "$READY_LOG" 2>/dev/null && printf yes || printf no)
EOF
}

abort_command()
{
    require_session
    source_ros
    rosservice call /apriltag_mission/abort
    log WARN "自动抓取已取消并保持当前位置；由飞手接管或执行 land。"
}

land_command()
{
    require_session
    source_ros
    bash "${SCRIPT_DIR}/takeland.sh"
}

stop_command()
{
    local force=0
    [ "${1:-}" != "--force" ] || force=1
    load_metadata
    source_ros

    if (( force == 0 )); then
        local state ext
        state="$(timeout 3 rostopic echo -n 1 /mavros/state 2>/dev/null)" || die "无法确认下锁；拒绝 stop。断电后清残留才可显式使用 --force。"
        ext="$(timeout 3 rostopic echo -n 1 /mavros/extended_state 2>/dev/null)" || die "无法确认落地；拒绝 stop。"
        [ "$(state_field "$state" armed)" = "False" ] || die "飞机仍已解锁；严禁关闭控制链。"
        [ "$(state_field "$state" mode)" != "OFFBOARD" ] || die "飞机仍在 OFFBOARD；严禁关闭控制链。"
        [ "$(state_field "$ext" landed_state)" = "1" ] || die "PX4 未确认 ON_GROUND；严禁关闭控制链。"
    else
        log WARN "使用 --force；仅应在飞行器已断电或人工确认绝不在空中时使用。"
    fi

    stop_owned_processes
    rm -f "${STATE_DIR}"/*.pid "${STATE_DIR}/active" \
        "${STATE_DIR}/prepared" "${STATE_DIR}/takeoff_sent" \
        "${STATE_DIR}/mission_sent" 2>/dev/null || true
    log OK "便捷任务进程已按 rosbag -> mission -> MAVROS -> roscore 顺序关闭。"
    log INFO "日志保留在：${CURRENT_LOG_DIR}"
}

logs_command()
{
    load_metadata
    printf '%s\n' "$CURRENT_LOG_DIR"
    find "$CURRENT_LOG_DIR" -maxdepth 1 -type f -printf '%f\n' | sort
}

wait_for_terminal_state()
{
    local deadline=$((SECONDS + 180))
    while (( SECONDS < deadline )); do
        local state status
        state="$(topic_data /apriltag_mission/state 2>/dev/null || printf unavailable)"
        status="$(topic_data /apriltag_mission/status 2>/dev/null || printf unavailable)"
        printf '[MONITOR] state=%s | %s\n' "$state" "$status"
        case "$state" in
            HOLD_HOME|FAILSAFE_HOLD|LANDED) return 0 ;;
        esac
        sleep 2
    done
    log WARN "180 s 内未到达 HOLD_HOME；可能未发现标签或仍在恢复。飞手应判断后接管/降落。"
    return 0
}

run_command()
{
    [ -t 0 ] || die "run 是交互向导，必须在可输入的终端中运行。"
    start_command "$@"

    prepare_command

    printf '\n请飞手确认遥控接管开关有效，并用遥控器切到 OFFBOARD。\n'
    read -r -p "确认状态正确后输入 TAKEOFF：" answer
    [ "$answer" = "TAKEOFF" ] || die "未确认 TAKEOFF；基础节点和录包保持运行，可稍后使用子命令继续。"
    takeoff_command

    read -r -p "确认 1.0 m 悬停稳定、航线净空后输入 MISSION：" answer
    [ "$answer" = "MISSION" ] || die "未确认 MISSION；飞机保持悬停，请执行 land。"
    mission_command
    wait_for_terminal_state
    status_command

    read -r -p "飞手确认可以降落后输入 LAND：" answer
    [ "$answer" = "LAND" ] || die "未确认 LAND；控制链和录包继续运行，严禁直接 stop。"
    land_command
    log INFO "等待落地和下锁；确认后执行 stop。"
}

main()
{
    local command="${1:-help}"
    shift || true
    case "$command" in
        start) start_command "$@" ;;
        run) run_command "$@" ;;
        status) status_command ;;
        prepare) prepare_command ;;
        takeoff) takeoff_command ;;
        mission) mission_command ;;
        abort) abort_command ;;
        land) land_command ;;
        stop) stop_command "$@" ;;
        logs) logs_command ;;
        help|-h|--help) usage ;;
        *) usage; die "未知子命令：${command}" ;;
    esac
}

main "$@"
