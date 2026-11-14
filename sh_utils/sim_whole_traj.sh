#!/usr/bin/env bash
# HAM 仿真轨迹任务：
# 启动后执行：独立窗口录包 -> 当前窗口 OFFBOARD -> 起飞 -> 稳定悬停
#          -> 轨迹 -> 静止 -> 降落。录包由操作者在独立窗口按 Ctrl+C 结束。
#
# 与 RTK_whole_traj.sh 的区别：
# 1. 不检查 RTK Fixed 和 EKF GPS absolute 状态；
# 2. 使用 Gazebo/PX4 SITL 仿真状态；
# 3. 保留相同的轨迹触发、悬停检测、降落流程。

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "${SCRIPT_DIR}/_ros_common.sh"

STATE_TOPIC="/mavros/state"
ODOM_TOPIC="/odom/remap"
SETPOINT_TOPIC="/mavros/setpoint_raw/attitude"

TAKEOFF_LAND_TOPIC="/drone0/ctrl_node/takeoff_land"
TRAJECTORY_TRIGGER_TOPIC="/landing_trigger"
TRAJECTORY_TIME_TOPIC="/drone0/sim_odom/time_diff"
CONTROLLER_READY_TOPIC="/traj_start_trigger"

TRAJECTORY_PARAM_NS="/drone0/sim_odom"
CONTROL_PARAM_NS="/drone0/ctrl_node"

OFFBOARD_WAIT_TIMEOUT=60
ARM_WAIT_TIMEOUT=20
CONTROLLER_READY_TIMEOUT=15
TRAJECTORY_START_TIMEOUT=8
POST_MISSION_HOVER_TIME=2
LAND_VELOCITY_THRESHOLD=0.50

# PX4 AUTO.LAND 仿真参数。MPC_THR_HOVER 从 ctrl_node 已验证的
# thrust_model/hover_percentage 读取，避免 ROS 控制器与 PX4 接管后的
# 悬停推力模型不一致导致落地检测“收油→再加油”弹跳。
PX4_LAND_SPEED="${PX4_LAND_SPEED:-0.60}"
PX4_DISARM_LAND_DELAY="${PX4_DISARM_LAND_DELAY:-0.50}"

AIRBORNE=0

fatal()
{
    log FATAL "$1" >&2
    if (( AIRBORNE )); then
        log SAFETY "飞行器可能仍在空中，请停止自动流程并人工检查仿真状态。"
    fi
    exit 1
}

# =====================================================
# 判断 ROS topic 是否存在 subscriber
# 兼容:
# Subscribers:
#  * /manager
#  * /drone0/manager
# =====================================================
topic_has_subscriber()
{
    local topic="$1"
    local info

    info=$(rostopic info "$topic" 2>/dev/null) || return 1

    awk '
        /^Subscribers:/ {inside=1; next}
        /^(Publishers|Services):/ {inside=0}
        inside && /\// {found=1}
        END {exit !found}
    ' <<<"$info"
}

require_command()
{
    command -v "$1" >/dev/null 2>&1 ||
        fatal "缺少命令: $1"
}


is_number()
{
    [[ "$1" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$ ]]
}


# 读取一帧 /odom/remap 世界系线速度。
read_velocity()
{
    local msg vx vy vz

    msg=$(timeout 3 rostopic echo -n 1 \
        "${ODOM_TOPIC}/twist/twist/linear" 2>/dev/null) || return 1
    vx=$(awk -F': ' '/^[[:space:]]*x:/ {print $2; exit}' <<<"$msg")
    vy=$(awk -F': ' '/^[[:space:]]*y:/ {print $2; exit}' <<<"$msg")
    vz=$(awk -F': ' '/^[[:space:]]*z:/ {print $2; exit}' <<<"$msg")

    is_number "$vx" && is_number "$vy" && is_number "$vz" || return 1
    printf '%s %s %s\n' "$vx" "$vy" "$vz"
}
wait_offboard()
{
    log ACTION "等待进入 OFFBOARD..."

    local start=$SECONDS

    while (( SECONDS-start < OFFBOARD_WAIT_TIMEOUT )); do
        local state mode
        state=$(read_state "$STATE_TOPIC") || continue

        mode=$(state_value "$state" mode)

        if [[ "$mode" == "OFFBOARD" ]]; then
            log OK "OFFBOARD 已激活"
            return 0
        fi

        sleep 1
    done

    fatal "OFFBOARD 超时"
}


publish_takeoff()
{
    topic_has_subscriber "$TAKEOFF_LAND_TOPIC" ||
        fatal "ctrl_node 未订阅起飞话题"

    log ACTION "发送起飞命令"

    rostopic pub -1 "$TAKEOFF_LAND_TOPIC" \
        quadrotor_msgs/TakeoffLand \
        "{takeoff_land_cmd: 1}" >/dev/null
}


wait_arm()
{
    log WAIT "等待 PX4 解锁"

    local start=$SECONDS

    while (( SECONDS-start < ARM_WAIT_TIMEOUT )); do
        local state armed

        state=$(read_state "$STATE_TOPIC") || continue
        armed=$(state_value "$state" armed)

        if [[ "$armed" == "True" ]]; then
            AIRBORNE=1
            log OK "无人机已解锁"
            return 0
        fi

        sleep 1
    done

    fatal "解锁失败"
}


wait_controller_ready()
{
    local msg frame_id state mode armed

    log WAIT "等待 FSM 完成 TAKEOFF --> HOVER 并发布规划器就绪握手"
    msg=$(timeout "$CONTROLLER_READY_TIMEOUT" \
        rostopic echo -n 1 "$CONTROLLER_READY_TOPIC" 2>/dev/null) ||
        fatal "${CONTROLLER_READY_TIMEOUT} s 内未收到 ${CONTROLLER_READY_TOPIC}"
    frame_id=$(awk -F': ' '/^[[:space:]]*frame_id:/ {gsub(/[\047\042\r]/, "", $2); print $2; exit}' <<<"$msg")
    [[ "$frame_id" =~ ^world:[1-9][0-9]*$ ]] ||
        fatal "收到无效的规划器就绪握手：frame_id=${frame_id:-missing}"

    state=$(read_state "$STATE_TOPIC") ||
        fatal "收到规划器就绪握手后无法读取 PX4 状态"
    mode=$(state_value "$state" mode)
    armed=$(state_value "$state" armed)
    [[ "$mode" == "OFFBOARD" && "$armed" == "True" ]] ||
        fatal "规划器就绪时飞行状态异常：mode=${mode:-unknown}, armed=${armed:-unknown}"

    log OK "FSM HOVER 与规划器握手已就绪（${frame_id}）"
}


configure_px4_auto_land()
{
    local hover_thrust
    hover_thrust=$(rosparam get "${CONTROL_PARAM_NS}/thrust_model/hover_percentage" 2>/dev/null) ||
        fatal "缺少悬停推力参数 ${CONTROL_PARAM_NS}/thrust_model/hover_percentage"

    is_number "$hover_thrust" && is_number "$PX4_LAND_SPEED" &&
        is_number "$PX4_DISARM_LAND_DELAY" ||
        fatal "PX4 降落参数必须是数字"
    awk -v h="$hover_thrust" -v s="$PX4_LAND_SPEED" \
        -v d="$PX4_DISARM_LAND_DELAY" \
        'BEGIN {exit !(h>=0.1 && h<=0.8 && s>=0.6 && d>0)}' ||
        fatal "PX4 降落参数越界：MPC_THR_HOVER=${hover_thrust}, MPC_LAND_SPEED=${PX4_LAND_SPEED}, COM_DISARM_LAND=${PX4_DISARM_LAND_DELAY}"

    log ACTION "对齐 PX4 AUTO.LAND 参数：hover=${hover_thrust}, land_speed=${PX4_LAND_SPEED}, disarm_delay=${PX4_DISARM_LAND_DELAY}"
    rosrun mavros mavparam set MPC_THR_HOVER "$hover_thrust" >/dev/null ||
        fatal "设置 PX4 MPC_THR_HOVER 失败"
    rosrun mavros mavparam set MPC_LAND_SPEED "$PX4_LAND_SPEED" >/dev/null ||
        fatal "设置 PX4 MPC_LAND_SPEED 失败"
    rosrun mavros mavparam set COM_DISARM_LAND "$PX4_DISARM_LAND_DELAY" >/dev/null ||
        fatal "设置 PX4 COM_DISARM_LAND 失败"
    log OK "PX4 AUTO.LAND 参数已应用"
}


publish_trigger()
{
    topic_has_subscriber "$TRAJECTORY_TRIGGER_TOPIC" ||
        fatal "轨迹规划器未启动"

    log ACTION "触发轨迹"

    rostopic pub -1 "$TRAJECTORY_TRIGGER_TOPIC" \
    geometry_msgs/PoseStamped \
    "{header: {stamp: now, frame_id: ''}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" \
    >/dev/null || fatal "轨迹触发发布失败"
}


wait_trajectory_started()
{
    log WAIT "确认 HOVER --> MISSION 且轨迹时钟开始推进"
    wait_topic_messages "$TRAJECTORY_TIME_TOPIC" 2 "$TRAJECTORY_START_TIMEOUT" ||
        fatal "触发后 ${TRAJECTORY_START_TIMEOUT} s 内轨迹未启动；请检查 controller/planner 握手日志"
}


trajectory_total_time()
{
    local traj_type rise duration hold descend scale start_hold forward end_hold return_time
    traj_type=$(rosparam get "${TRAJECTORY_PARAM_NS}/traj_type" 2>/dev/null) || return 1
    scale=$(rosparam get "${TRAJECTORY_PARAM_NS}/speed_scale" 2>/dev/null) || return 1
    is_number "$traj_type" && is_number "$scale" || return 1

    if [[ "$traj_type" == "3" ]]; then
        start_hold=$(rosparam get "${TRAJECTORY_PARAM_NS}/line_start_hold_time" 2>/dev/null) || return 1
        forward=$(rosparam get "${TRAJECTORY_PARAM_NS}/line_forward_time" 2>/dev/null) || return 1
        end_hold=$(rosparam get "${TRAJECTORY_PARAM_NS}/line_end_hold_time" 2>/dev/null) || return 1
        return_time=$(rosparam get "${TRAJECTORY_PARAM_NS}/line_return_time" 2>/dev/null) || return 1
        is_number "$start_hold" && is_number "$forward" &&
            is_number "$end_hold" && is_number "$return_time" || return 1
        awk -v a="$start_hold" -v b="$forward" -v c="$end_hold" -v d="$return_time" -v s="$scale" \
            'BEGIN {if(s<=0) exit 1; printf "%.6f", (a+b+c+d)/s}'
        return
    fi

    rise=$(rosparam get "${TRAJECTORY_PARAM_NS}/traj_rise_time" 2>/dev/null) || return 1
    duration=$(rosparam get "${TRAJECTORY_PARAM_NS}/traj_duration" 2>/dev/null) || return 1
    hold=$(rosparam get "${TRAJECTORY_PARAM_NS}/traj_hold_time" 2>/dev/null) || return 1
    descend=$(rosparam get "${TRAJECTORY_PARAM_NS}/traj_descend_time" 2>/dev/null) || return 1

    is_number "$rise" && is_number "$duration" && is_number "$hold" &&
        is_number "$descend" && is_number "$scale" || return 1
    awk -v a="$rise" -v b="$duration" -v c="$hold" -v d="$descend" -v s="$scale" \
        'BEGIN {if(s<=0) exit 1; printf "%.6f", (a+b+c+d)/s}'
}


# 把非负十进制秒数转成整数毫秒，供 Bash 直接比较，避免在
# 200 Hz 轨迹时间流中为每一帧启动 awk 子进程。科学计数法的早期小值
# 可直接跳过；接近终点时 rostopic CSV 会输出普通十进制数。
seconds_to_millis()
{
    local value="${1//$'\r'/}"
    local whole fraction

    [[ "$value" =~ ^([0-9]+)([.]([0-9]+))?$ ]] || return 1
    whole="${BASH_REMATCH[1]}"
    fraction="${BASH_REMATCH[3]:-0}000"
    fraction="${fraction:0:3}"
    printf '%d\n' "$((10#$whole * 1000 + 10#$fraction))"
}


wait_trajectory()
{
    local total="$1"
    local timeout_s total_ms
    timeout_s=$(awk -v t="$total" 'BEGIN {printf "%d", t + 45}')
    total_ms=$(seconds_to_millis "$total") ||
        fatal "轨迹总时长无法转换：${total}"

    log WAIT "等待轨迹完成：总时长=${total} s，超时=${timeout_s} s"

    # 使用 coproc 保留 rostopic 进程 PID。旧的“rostopic | awk”在 awk
    # 读到最终帧退出后，rostopic 因为话题已停发而不会再写入断开的
    # 管道，Bash 会继续等它直到 timeout，平白多等约 45 s。
    coproc TRAJECTORY_ECHO {
        # rostopic 是 Python 程序；通过管道读取时必须禁用 stdout 块缓冲，
        # 否则最后一帧可能留在 Python 缓冲区内而无法被父进程看到。
        PYTHONUNBUFFERED=1 exec rostopic echo -p "$TRAJECTORY_TIME_TOPIC" 2>/dev/null
    }

    local stream_pid="$TRAJECTORY_ECHO_PID"
    local stream_fd="${TRAJECTORY_ECHO[0]}"
    local stream_write_fd="${TRAJECTORY_ECHO[1]}"
    local start_time=$SECONDS
    local last_print_ms=-1000
    local complete=0
    local line elapsed elapsed_ms

    # 父进程不会向 coproc 写数据，立即关闭无用的写端。
    exec {stream_write_fd}>&-

    while (( SECONDS - start_time < timeout_s )); do
        if IFS= read -r -t 1 line <&"$stream_fd"; then
            [[ "$line" == %* || "$line" != *,* ]] && continue
            elapsed="${line##*,}"
            elapsed_ms=$(seconds_to_millis "$elapsed" 2>/dev/null) || continue

            if (( elapsed_ms >= last_print_ms + 1000 ||
                  elapsed_ms >= total_ms )); then
                printf 'trajectory time: %s\n' "$elapsed"
                last_print_ms=$elapsed_ms
            fi

            if (( elapsed_ms >= total_ms )); then
                complete=1
                break
            fi
        elif ! kill -0 "$stream_pid" 2>/dev/null; then
            break
        fi
    done

    # 无论完成或超时都主动结束订阅者，不再等待话题下一帧。
    kill "$stream_pid" 2>/dev/null || true
    wait "$stream_pid" 2>/dev/null || true
    exec {stream_fd}<&-

    if (( complete )); then
        log OK "轨迹完整时间线已完成：${total} s"
        return 0
    fi

    fatal "轨迹未在 ${timeout_s} s 内完成"
}


wait_post_mission_land_ready()
{
    local state connected mode armed velocity vx vy vz speed

    log WAIT "轨迹结束；保持 HOVER ${POST_MISSION_HOVER_TIME} s，等待降落交接"
    sleep "$POST_MISSION_HOVER_TIME"

    state=$(read_state "$STATE_TOPIC") ||
        fatal "降落交接时无法读取 PX4 状态"
    connected=$(state_value "$state" connected)
    mode=$(state_value "$state" mode)
    armed=$(state_value "$state" armed)
    [[ "$connected" == "True" && "$mode" == "OFFBOARD" && "$armed" == "True" ]] ||
        fatal "降落交接时飞行状态异常：connected=${connected:-unknown}, mode=${mode:-unknown}, armed=${armed:-unknown}"

    velocity=$(read_velocity) ||
        fatal "降落交接时 ${ODOM_TOPIC} 速度数据中断"
    read -r vx vy vz <<<"$velocity"
    speed=$(awk -v x="$vx" -v y="$vy" -v z="$vz" \
        'BEGIN {printf "%.6f", sqrt(x*x+y*y+z*z)}')

    awk -v speed="$speed" -v limit="$LAND_VELOCITY_THRESHOLD" \
        'BEGIN {exit !(speed <= limit)}' ||
        fatal "降落交接被拒绝：总速度 ${speed} m/s > ${LAND_VELOCITY_THRESHOLD} m/s"

    log OK "降落交接就绪：固定等待 ${POST_MISSION_HOVER_TIME} s，总速度=${speed} m/s <= ${LAND_VELOCITY_THRESHOLD} m/s"
}


publish_land()
{
    topic_has_subscriber "$TAKEOFF_LAND_TOPIC" ||
        fatal "ctrl_node 未订阅降落话题"

    log ACTION "发送降落命令"

    rostopic pub -1 "$TAKEOFF_LAND_TOPIC" \
        quadrotor_msgs/TakeoffLand \
        "{takeoff_land_cmd: 2}" >/dev/null || fatal "降落命令发布失败"
}


wait_landed_and_disarmed()
{
    local timeout_s=120
    local start=$SECONDS
    local landing_accepted=0
    local on_ground=0
    local disarmed=0
    local last_mode=""

    log WAIT "等待 PX4 降落并自动上锁（最长 ${timeout_s} s）"

    while (( SECONDS-start < timeout_s )); do
        local state connected mode armed extended landed_state
        state=$(read_state "$STATE_TOPIC") || { sleep 1; continue; }
        connected=$(state_value "$state" connected)
        mode=$(state_value "$state" mode)
        armed=$(state_value "$state" armed)

        [[ "$connected" == "True" ]] || fatal "降落期间 MAVROS 连接断开"

        extended=$(timeout 3 rostopic echo -n 1 /mavros/extended_state 2>/dev/null || true)
        landed_state=$(awk -F': ' '/^[[:space:]]*landed_state:/ {print $2; exit}' <<<"$extended")

        if [[ "$mode" != "$last_mode" ]]; then
            log INFO "降落阶段 PX4 mode=${mode:-unknown}, armed=${armed:-unknown}, landed_state=${landed_state:-unknown}"
            last_mode="$mode"
        fi

        if [[ "$mode" == "AUTO.LAND" || "$landed_state" == "4" ||
              "$landed_state" == "1" || "$armed" == "False" ]]; then
            landing_accepted=1
        fi

        if [[ "$landed_state" == "1" && "$on_ground" == "0" ]]; then
            on_ground=1
            log OK "PX4 确认 LANDED_STATE_ON_GROUND"
        fi

        if [[ "$armed" == "False" && "$disarmed" == "0" ]]; then
            disarmed=1
            log OK "PX4 已上锁"
        fi

        if (( on_ground && disarmed )); then
            AIRBORNE=0
            return 0
        fi

        if (( ! landing_accepted && SECONDS-start >= 12 )); then
            fatal "降落命令发出后 12 s 内 PX4 未进入 AUTO.LAND/降落状态"
        fi

        if [[ "$armed" == "True" && "$mode" != "OFFBOARD" && "$mode" != "AUTO.LAND" ]]; then
            fatal "降落期间进入非预期模式 ${mode:-unknown}"
        fi

        sleep 1
    done

    fatal "降落后 ${timeout_s} s 内未同时确认落地和上锁"
}


start_record()
{
    local record_script="${SCRIPT_DIR}/record.sh"
    local record_dir runner start_time
    [[ -f "$record_script" ]] || fatal "录包脚本不存在：${record_script}"

    if rosnode list 2>/dev/null | grep -Fxq /record; then
        fatal "已存在 /record 节点；请先确认并在原录包窗口手动按 Ctrl+C。"
    fi

    record_dir=$(cd "$(dirname "$record_script")" && pwd) ||
        fatal "无法解析录包脚本目录：${record_script}"
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
        echo 'echo "[INFO] record.sh 已退出，状态码=${exit_code}。请确认 rosbag 已完成索引写入。"'
        echo 'exec bash'
    } > "$runner"
    chmod +x "$runner"

    log INFO "将在独立窗口通过 bash record.sh 启动录包：${record_script}"
    if ! gnome-terminal --window --title="HAM simulation rosbag" -- "$runner"; then
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

    rm -f "$runner"
    log OK "独立录包窗口已就绪：bash record.sh（节点 /record）。"
    log INFO "本脚本不会自动停止录包；完成后请在录包窗口手动按 Ctrl+C。"
}


main()
{
    echo "============= HAM 仿真轨迹任务 ============="

    for c in rostopic rosnode rosparam rosrun awk grep timeout mktemp gnome-terminal; do
        require_command "$c"
    done

    rostopic list >/dev/null ||
        fatal "ROS master 不存在"


    log INFO "检查仿真节点"

    wait_topic_messages "$ODOM_TOPIC" 5 5 ||
        fatal "/odom/remap 无数据"

    wait_topic_messages "$SETPOINT_TOPIC" 5 5 ||
        fatal "控制器无输出"

    topic_has_subscriber "$TRAJECTORY_TRIGGER_TOPIC" ||
        fatal "sim_odom 未启动"

    local traj_type total_time
    traj_type=$(rosparam get "${TRAJECTORY_PARAM_NS}/traj_type" 2>/dev/null) ||
        fatal "缺少运行时轨迹参数 traj_type"
    [[ "$traj_type" != "0" ]] ||
        fatal "traj_type=0 是无限垂直驻留任务，不能由 sim_whole_traj 自动等待结束；请使用 pub_trigger.sh，并在试验完成后显式执行 takeland.sh。"
    total_time=$(trajectory_total_time) ||
        fatal "无法从运行时 ROS 参数计算轨迹总时长"
    log INFO "轨迹总时长=${total_time} s（按 traj_type 读取各阶段并应用 speed_scale）"


    echo ""
    echo "输入 TAKEOFF 开始仿真任务"
    read -r confirm

    [[ "$confirm" == "TAKEOFF" ]] ||
        fatal "任务取消"


    start_record

    wait_offboard

    configure_px4_auto_land

    publish_takeoff

    wait_arm

    wait_controller_ready


    publish_trigger

    wait_trajectory_started

    wait_trajectory "$total_time"


    wait_post_mission_land_ready


    publish_land

    wait_landed_and_disarmed

    log OK "仿真轨迹、降落和自动上锁全部完成。"
    log INFO "rosbag 仍在独立窗口记录；请切换到录包窗口按 Ctrl+C，等待索引写入完成后再关闭窗口。"
}


main "$@"
