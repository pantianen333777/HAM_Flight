#!/bin/bash

# HAM real-flight one-key startup.
# This script deliberately stops at READY:
# it does not arm the vehicle, switch to OFFBOARD, take off, or start a mission.

FCU_DEVICE="/dev/ttyACM0"
FCU_BAUD="57600"
RTK_STATUS_TOPIC="/mavros/gpsstatus/gps1/raw"
RTK_FIXED_MIN_TYPE=6
MAX_OFFBOARD_LOSS_TIMEOUT="${MAX_OFFBOARD_LOSS_TIMEOUT:-1.0}"

# MAVLink 数据流频率由 mavlink_set_rates.sh 统一下发,实飞频率在下方调用处以
# 环境变量(ID31/ID32/ID331)覆盖,不再在此声明本地常量。

# Prevent two startup scripts from running at the same time.
# flock 在文件顶部首次使用，须在使用点前检查（本脚本无 require_command 列表兜底）。
if ! command -v flock >/dev/null 2>&1; then
    echo "[FATAL] Missing command: flock."
    exit 1
fi

exec 9>/tmp/ham_rtk_start.lock
if ! flock -n 9; then
    echo "[FATAL] Another RTK.sh is already running."
    exit 1
fi

# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash ||
    {
        echo "[FATAL] Failed to source /opt/ros/noetic/setup.bash."
        exit 1
    }
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HAM_FLIGHT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_SETUP="${HAM_FLIGHT_ROOT}/devel/setup.bash"
if [ ! -r "$WORKSPACE_SETUP" ]; then
    echo "[FATAL] Workspace is not built or setup is missing: $WORKSPACE_SETUP"
    echo "[FATAL] Run catkin_make in $HAM_FLIGHT_ROOT before real flight."
    exit 1
fi
# shellcheck source=/dev/null
source "$WORKSPACE_SETUP" ||
    {
        echo "[FATAL] Failed to source $WORKSPACE_SETUP."
        exit 1
    }

# 公共 ROS 编排工具(log / wait_node / wait_nodelet / wait_topic_messages /
# wait_roscore / ensure_roscore / wait_mavros_connected / ensure_node_absent /
# read_state / state_value / mk_tab)统一取自 _ros_common.sh,避免两份实现分叉。
# shellcheck source=./_ros_common.sh
source "$SCRIPT_DIR/_ros_common.sh" ||
    {
        echo "[FATAL] Failed to source $SCRIPT_DIR/_ros_common.sh."
        exit 1
    }
trap cleanup_tabs EXIT

fatal()
{
    echo "[FATAL] $1"
    echo "[FATAL] Startup stopped. Already opened nodes are left running for manual inspection."
    exit 1
}

check_duplicate_nodes()
{
    local node

    for node in \
        /mavros \
        /manager \
        /drone0/manager \
        /drone0/odomRemap \
        /drone0/ctrl_node
    do
        ensure_node_absent "$node" || return 1
    done

    echo "[OK] No duplicate MAVROS, manager, odomRemap or controller nodes."
}

check_real_time()
{
    local use_sim_time
    use_sim_time=$(rosparam get /use_sim_time 2>/dev/null || echo "false")

    if [ "$use_sim_time" = "true" ]; then
        echo "[ERROR] /use_sim_time=true. Real flight must not use simulation time."
        return 1
    fi

    echo "[OK] Real-flight wall clock is active."
}

check_px4_safe_state()
{
    local state
    local mode
    local armed

    state=$(timeout 3 rostopic echo -n 1 /mavros/state 2>/dev/null) ||
        return 1

    mode=$(echo "$state" |
        awk -F': ' '/^mode:/ {print $2}' |
        tr -d "'\"")
    armed=$(echo "$state" |
        awk -F': ' '/^armed:/ {print $2}')

    # 严格 fail-closed: 字段缺失/解析失败必须拒绝, 不得以 armed=unknown 放行。
    if [ "$armed" != "False" ]; then
        echo "[ERROR] PX4 is armed or armed state unavailable (armed=${armed:-unknown}). Refusing to start."
        return 1
    fi

    if [ -z "$mode" ]; then
        echo "[ERROR] Cannot parse the PX4 flight mode from /mavros/state."
        return 1
    fi

    if [ "$mode" = "OFFBOARD" ]; then
        echo "[ERROR] PX4 is already in OFFBOARD before the chain is ready."
        return 1
    fi

    echo "[OK] PX4 safe state: mode=$mode, armed=$armed."
}

check_vehicle_on_ground()
{
    local extended_state
    local landed_state

    extended_state=$(
        timeout 10 rostopic echo -n 1 /mavros/extended_state 2>/dev/null
    ) || {
        echo "[ERROR] Cannot read /mavros/extended_state."
        return 1
    }

    landed_state=$(echo "$extended_state" |
        awk -F': ' '/^[[:space:]]*landed_state:/ {print $2}')

    if [ "$landed_state" != "1" ]; then
        echo "[ERROR] PX4 does not report LANDED_STATE_ON_GROUND (landed_state=$landed_state)."
        return 1
    fi

    echo "[OK] PX4 reports that the vehicle is on the ground."
}


read_px4_param_field_once()
{
    local param_id="$1"
    local field="$2"
    local response
    local value

    response=$(
        timeout 4 rosservice call /mavros/param/get \
            "param_id: '${param_id}'" 2>/dev/null
    ) || return 1

    echo "$response" |
        grep -Eq '^[[:space:]]*success: (True|true)$' || return 1

    value=$(echo "$response" |
        awk -F': ' -v key="$field" '$1 ~ key "$" {print $2; exit}' |
        tr -d '[:space:]')
    [ -n "$value" ] || return 1
    printf '%s\n' "$value"
}

wait_px4_param_field()
{
    local param_id="$1"
    local field="$2"
    local timeout_s="${3:-20}"
    local start_time=$SECONDS
    local value

    while (( SECONDS - start_time < timeout_s )); do
        value=$(read_px4_param_field_once "$param_id" "$field") && {
            printf '%s\n' "$value"
            return 0
        }
        sleep 1
    done
    return 1
}

check_offboard_loss_failsafe()
{
    local loss_timeout
    local rc_action

    # PX4 must remain independently capable of landing if this process or its
    # setpoint stream disappears before the ROS AUTO.LAND request is delivered.
    # 现役固件用 COM_OF_LOSS_T + COM_OBL_RC_ACT 两参数; 本检查只读不写。
    # (旧固件"无遥控链路动作"独立参数的兼容检查已随旧固件淘汰移除。)
    [[ "$MAX_OFFBOARD_LOSS_TIMEOUT" =~ ^[0-9]+([.][0-9]+)?$ ]] || {
        echo "[ERROR] Invalid MAX_OFFBOARD_LOSS_TIMEOUT=$MAX_OFFBOARD_LOSS_TIMEOUT."
        return 1
    }

    loss_timeout=$(wait_px4_param_field COM_OF_LOSS_T real 20) || {
        echo "[ERROR] Cannot read PX4 COM_OF_LOSS_T through MAVROS."
        return 1
    }
    [[ "$loss_timeout" =~ ^-?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$ ]] || {
        echo "[ERROR] PX4 COM_OF_LOSS_T is not numeric: $loss_timeout"
        return 1
    }
    awk -v value="$loss_timeout" -v maximum="$MAX_OFFBOARD_LOSS_TIMEOUT" \
        'BEGIN { exit !(value >= 0.0 && value <= maximum) }' || {
        echo "[ERROR] PX4 COM_OF_LOSS_T=$loss_timeout s exceeds the allowed ${MAX_OFFBOARD_LOSS_TIMEOUT} s."
        return 1
    }

    rc_action=$(wait_px4_param_field COM_OBL_RC_ACT integer 20) || {
        echo "[ERROR] Cannot read PX4 COM_OBL_RC_ACT through MAVROS."
        return 1
    }
    if [ "$rc_action" != "4" ]; then
        echo "[ERROR] PX4 COM_OBL_RC_ACT=$rc_action; HAM requires 4 (Land) for Offboard loss."
        return 1
    fi

    echo "[OK] PX4 Offboard-loss failsafe: timeout=${loss_timeout}s, action=Land (COM_OBL_RC_ACT=${rc_action})."
}

wait_rtk_fixed()
{
    local timeout_s="${1:-90}"
    local start_time=$SECONDS
    local gps_msg
    local fix_type

    echo "[WAIT] Waiting for RTK Fixed on $RTK_STATUS_TOPIC..."

    while (( SECONDS - start_time < timeout_s )); do
        gps_msg=$(
            timeout 2 rostopic echo -n 1 "$RTK_STATUS_TOPIC" 2>/dev/null
        )
        fix_type=$(echo "$gps_msg" |
            awk -F': ' '/^[[:space:]]*fix_type:/ {print $2; exit}')

        case "$fix_type" in
            ''|*[!0-9]*)
                ;;
            *)
                if [ "$fix_type" -ge "$RTK_FIXED_MIN_TYPE" ]; then
                    echo "[OK] RTK Fixed: fix_type=$fix_type."
                    return 0
                fi
                echo "[WAIT] Current GPS fix_type=$fix_type; RTK Fixed requires >=$RTK_FIXED_MIN_TYPE."
                ;;
        esac

        sleep 1
    done

    echo "[ERROR] RTK did not reach Fixed within ${timeout_s} s."
    echo "[ERROR] Verify that $RTK_STATUS_TOPIC is the GPSRAW topic used by this MAVROS installation."
    return 1
}

wait_ekf_ready()
{
    local timeout_s="${1:-90}"
    local start_time=$SECONDS
    local estimator

    echo "[WAIT] Waiting for PX4 EKF absolute-position readiness..."

    while (( SECONDS - start_time < timeout_s )); do
        estimator=$(
            timeout 2 rostopic echo -n 1 /mavros/estimator_status 2>/dev/null
        )

        if echo "$estimator" |
            grep -Eq "pos_horiz_abs_status_flag: (True|true)" &&
           echo "$estimator" |
            grep -Eq "pos_vert_abs_status_flag: (True|true)" &&
           ! echo "$estimator" |
            grep -Eq "gps_glitch_status_flag: (True|true)"; then
            echo "[OK] PX4 EKF absolute position is ready."
            return 0
        fi

        sleep 1
    done

    echo "[ERROR] PX4 EKF was not ready within ${timeout_s} s."
    echo "[ERROR] Check /mavros/estimator_status and the PX4 estimator configuration."
    return 1
}

check_fcu_device_access()
{
    local mode
    local other_digit

    [ -e "$FCU_DEVICE" ] || {
        echo "[ERROR] FCU device does not exist: $FCU_DEVICE"
        return 1
    }

    [ -r "$FCU_DEVICE" ] && [ -w "$FCU_DEVICE" ] || {
        echo "[ERROR] Current user cannot read/write $FCU_DEVICE."
        echo "[ERROR] Add the user to the dialout group and install a udev rule with MODE=\"0660\"."
        return 1
    }

    mode=$(stat -c '%a' "$FCU_DEVICE" 2>/dev/null) || return 1
    other_digit=${mode: -1}
    if (( (other_digit & 2) != 0 )); then
        echo "[WARN] $FCU_DEVICE is world-writable (mode=$mode); use group dialout with mode 0660."
    else
        echo "[OK] FCU device access is available without world-write permission (mode=$mode)."
    fi
}

topic_has_endpoint()
{
    local topic="$1"
    local role="$2"
    local node="$3"
    local info

    info=$(timeout 5 rostopic info "$topic" 2>/dev/null) || return 1
    awk -v role="$role" -v wanted="$node" '
        {
            # rostopic versions differ slightly in whitespace and some terminal
            # environments leave a trailing CR. Normalize both before parsing.
            line = $0
            sub(/\r$/, "", line)

            if (line ~ "^[[:space:]]*" role ":[[:space:]]*$") {
                inside = 1
                next
            }
            if (line ~ "^[[:space:]]*(Publishers|Subscribers):") {
                inside = 0
            }
            if (inside) {
                sub(/^[[:space:]]*\*[[:space:]]*/, "", line)
                split(line, fields, /[[:space:]]+/)
                if (fields[1] == wanted) {
                    found = 1
                }
            }
        }
        END { exit !found }
    ' <<<"$info"
}

topic_has_any_endpoint()
{
    local topic="$1"
    local role="$2"
    shift 2
    local node

    for node in "$@"; do
        if topic_has_endpoint "$topic" "$role" "$node"; then
            return 0
        fi
    done
    return 1
}

require_topic_endpoint_any()
{
    local topic="$1"
    local role="$2"
    local component="$3"
    shift 3
    local expected

    if topic_has_any_endpoint "$topic" "$role" "$@"; then
        return 0
    fi

    printf -v expected ' %s' "$@"
    echo "[ERROR] $topic is missing its $component $role endpoint; expected one of:${expected}."
    return 1
}

require_topic_type()
{
    local topic="$1"
    local expected="$2"
    local actual

    actual=$(timeout 5 rostopic type "$topic" 2>/dev/null) || {
        echo "[ERROR] Cannot determine the message type of $topic."
        return 1
    }
    if [ "$actual" != "$expected" ]; then
        echo "[ERROR] $topic has type $actual; expected $expected."
        return 1
    fi
}

trajectory_control_graph_ready()
{
    # ROS1 normally attributes nodelet endpoints to the hosting manager. Some
    # nodelet/roslaunch combinations expose the loader name instead. Both names
    # identify the same already-verified component, so accept either spelling
    # while still requiring every publisher/subscriber direction below.
    topic_has_any_endpoint /landing_trigger Subscribers \
        /manager /drone0/sim_odom || return 1
    topic_has_any_endpoint /landing_trigger Subscribers \
        /drone0/manager /drone0/ctrl_node || return 1
    topic_has_any_endpoint /traj_start_trigger Publishers \
        /drone0/manager /drone0/ctrl_node || return 1
    topic_has_any_endpoint /traj_start_trigger Subscribers \
        /manager /drone0/sim_odom || return 1
    topic_has_any_endpoint /drone0/planning/cmd Publishers \
        /manager /drone0/sim_odom || return 1
    topic_has_any_endpoint /drone0/planning/cmd Subscribers \
        /drone0/manager /drone0/ctrl_node || return 1
}

# 可选参数 timeout_s 有意默认 15, 调用点不传参。
# shellcheck disable=SC2120
check_trajectory_control_graph()
{
    local timeout_s="${1:-15}"
    local start_time=$SECONDS
    local failed=0

    # Give the ROS master a bounded interval to expose all nodelet topic
    # registrations. A single immediate snapshot can race nodelet startup.
    while (( SECONDS - start_time < timeout_s )); do
        if trajectory_control_graph_ready; then
            require_topic_type /landing_trigger geometry_msgs/PoseStamped || return 1
            require_topic_type /traj_start_trigger geometry_msgs/PoseStamped || return 1
            require_topic_type /drone0/planning/cmd quadrotor_msgs/PositionCommand || return 1
            echo "[OK] Trajectory trigger/command topic graph is complete."
            return 0
        fi
        sleep 0.2
    done

    # Print component-specific diagnostics only after the bounded wait expires.
    require_topic_endpoint_any /landing_trigger Subscribers planner \
        /manager /drone0/sim_odom || failed=1
    require_topic_endpoint_any /landing_trigger Subscribers controller \
        /drone0/manager /drone0/ctrl_node || failed=1
    require_topic_endpoint_any /traj_start_trigger Publishers controller \
        /drone0/manager /drone0/ctrl_node || failed=1
    require_topic_endpoint_any /traj_start_trigger Subscribers planner \
        /manager /drone0/sim_odom || failed=1
    require_topic_endpoint_any /drone0/planning/cmd Publishers planner \
        /manager /drone0/sim_odom || failed=1
    require_topic_endpoint_any /drone0/planning/cmd Subscribers controller \
        /drone0/manager /drone0/ctrl_node || failed=1

    if (( failed != 0 )); then
        echo "[INFO] Current ROS graph for trajectory topics:"
        timeout 5 rostopic info /landing_trigger 2>/dev/null || true
        timeout 5 rostopic info /traj_start_trigger 2>/dev/null || true
        timeout 5 rostopic info /drone0/planning/cmd 2>/dev/null || true
        return 1
    fi

    require_topic_type /landing_trigger geometry_msgs/PoseStamped || return 1
    require_topic_type /traj_start_trigger geometry_msgs/PoseStamped || return 1
    require_topic_type /drone0/planning/cmd quadrotor_msgs/PositionCommand || return 1
    echo "[OK] Trajectory trigger/command topic graph is complete."
}

echo "================ HAM real-flight startup ================"

# 1. Read/write access is required. World-write is reported as a warning so an
# existing field setup is not blocked, but dialout/udev mode 0660 is preferred.
check_fcu_device_access ||
    fatal "Secure access to $FCU_DEVICE is not configured"

# 2. Start or reuse ROS master.
ensure_roscore ||
    fatal "ROS master is unavailable"

check_real_time ||
    fatal "Simulation time is enabled"

check_duplicate_nodes ||
    fatal "A previous HAM flight-control chain is still running"

# fuser 缺失时 if 条件静默为 false, 会无声跳过占用检查, 须先确认命令存在。
if ! command -v fuser >/dev/null 2>&1; then
    fatal "Missing command: fuser (psmisc); cannot verify $FCU_DEVICE is unused"
fi

if fuser "$FCU_DEVICE" >/dev/null 2>&1; then
    fatal "$FCU_DEVICE is already in use, possibly by another MAVROS process"
fi

# 3. Connect MAVROS to Pixhawk.
T_MAVROS=$(mk_tab "roslaunch mavros px4.launch fcu_url:=serial://$FCU_DEVICE:$FCU_BAUD")
gnome-terminal --tab --title="mavros" -- "$T_MAVROS" ||
    fatal "Failed to open the mavros terminal tab"

wait_mavros_connected 120 "Check $FCU_DEVICE, baud $FCU_BAUD, Pixhawk power and firmware." ||
    fatal "MAVROS did not connect to Pixhawk"

# One combined preflight-state check replaces separate armed/mode checks.
check_px4_safe_state ||
    fatal "PX4 is not in a safe startup state"

check_vehicle_on_ground ||
    fatal "PX4 does not confirm that the vehicle is on the ground"

# 4. Configure required MAVLink data before odomRemap and controller startup.
# 实飞频率: ID105=200Hz(默认), ID31/ID32=100Hz, ID331=50Hz,
# ID245=2Hz, GPS_RAW_INT/ESTIMATOR_STATUS=5Hz(默认)。
ENABLE_NAV_STATUS_RATES=1 ID31=10000 ID32=10000 ID331=20000 bash "$SCRIPT_DIR/mavlink_set_rates.sh" ||
    fatal "MAVLink message interval configuration failed"

check_offboard_loss_failsafe ||
    fatal "PX4 Offboard-loss failsafe is not configured for timely landing"

wait_topic_messages /mavros/imu/data 2 30 ||
    fatal "MAVROS IMU data is unavailable"

wait_rtk_fixed 90 ||
    fatal "RTK Fixed is unavailable"

wait_ekf_ready 90 ||
    fatal "PX4 EKF is not ready"

# 5. Start trajectory node and create /manager plus /drone0/manager.
T_TRAJ=$(mk_tab "roslaunch trajectory_utils traj_gen.launch")
gnome-terminal --tab --title="traj_gen" -- "$T_TRAJ" ||
    fatal "Failed to open the traj_gen terminal tab"

wait_nodelet /manager sim_odom 60 ||
    fatal "trajectory nodelet sim_odom failed to load"

# 6. Start odomRemap and wait for calibration plus actual output data.
T_REMAP=$(mk_tab "roslaunch realflight_utils odom_remap.launch")
gnome-terminal --tab --title="odom_remap" -- "$T_REMAP" ||
    fatal "Failed to open the odom_remap terminal tab"

wait_nodelet /drone0/manager odomRemap 60 ||
    fatal "odomRemap failed to load"

wait_topic_messages /odom/remap 2 30 ||
    fatal "odomRemap loaded, but calibration/output did not become ready"

# Recheck state immediately before loading the controller.
check_px4_safe_state ||
    fatal "PX4 state changed before controller startup"

# 7. Load ctrl_node only after /odom/remap is valid.
T_CTRL=$(mk_tab "roslaunch ctrl_node run_ctrl.launch navigation_monitor_enabled:=true")
gnome-terminal --tab --title="controller" -- "$T_CTRL" ||
    fatal "Failed to open the controller terminal tab"

wait_nodelet /drone0/manager ctrl_node 60 ||
    fatal "ctrl_node failed to load"

wait_topic_messages /mavros/setpoint_raw/attitude 2 15 ||
    fatal "ctrl_node loaded but is not publishing attitude setpoints"

# 发布侧只能证明 ctrl_node 在输出; 仍须确认 MAVROS 端订阅在位, 链路才算闭合。
require_topic_endpoint_any /mavros/setpoint_raw/attitude Subscribers mavros /mavros ||
    fatal "MAVROS is not subscribed to /mavros/setpoint_raw/attitude"

check_trajectory_control_graph ||
    fatal "trajectory trigger/command topic graph is incomplete"

check_px4_safe_state ||
    fatal "PX4 unexpectedly armed or entered OFFBOARD during startup"

echo "======================= READY ============================"
echo "[READY] MAVROS connected to Pixhawk."
echo "[READY] RTK Fixed and PX4 EKF absolute position are valid."
echo "[READY] /odom/remap is calibrated and publishing."
echo "[READY] ctrl_node is publishing attitude setpoints."
echo "[READY] Trajectory trigger and command endpoints are connected."
echo "[SAFE]  Vehicle remains disarmed and is not in OFFBOARD."
echo "[NEXT]  Enter OFFBOARD only with the external RC transmitter; do NOT use set_offboard.sh."
echo "[NEXT]  With OFFBOARD engaged, run takeoff.sh to arm and take off, or RTK_whole_traj.sh for the full mission."
echo "==========================================================="
