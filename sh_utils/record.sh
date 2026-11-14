#!/usr/bin/env bash

set -Eeuo pipefail

# 将 bag 固定保存到脚本同级目录下的 Bag 文件夹
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BAG_DIR="${SCRIPT_DIR}/Bag"
mkdir -p "${BAG_DIR}"
cd "${BAG_DIR}" || exit 1

RECORD_NODE_NAME="${RECORD_NODE_NAME:-record}"
RECORD_CAMERA_IMAGES="${RECORD_CAMERA_IMAGES:-0}"
[[ "$RECORD_NODE_NAME" =~ ^[A-Za-z][A-Za-z0-9_]*$ ]] || {
    echo "[ERROR] 非法 RECORD_NODE_NAME: ${RECORD_NODE_NAME}" >&2
    exit 2
}

topics=( \
/debugPx4ctrl \
/drone0/ctrl_node/takeoff_land \
/landing_trigger \
/traj_start_trigger \
/mavros/state \
/mavros/setpoint_raw/attitude \
/mavros/local_position/odom \
/mavros/local_position/velocity_local \
/mavros/global_position/raw/fix \
/drone0/planning/preset_cmd \
/drone0/planning/cmd \
/apriltag_gripper/tag_position \
/apriltag_gripper/distance_m \
/apriltag_gripper/detected \
/apriltag_gripper/detection_latency_s \
/apriltag_gripper/measurement_age_s \
/apriltag_gripper/detection_interval_s \
/apriltag_gripper/at_target \
/apriltag_gripper/gripper_alignment_error \
/apriltag_gripper/desired_velocity_body_ned \
/apriltag_gripper/status \
/apriltag_gripper/commanded_value \
/apriltag_gripper/command_action \
/apriltag_gripper/command_accepted \
/tag_detections \
/apriltag_mission/state \
/apriltag_mission/status \
/apriltag_mission/alignment_error \
/apriltag_mission/raw_alignment_error \
/apriltag_mission/motion_compensation \
/apriltag_mission/tag_measurement_age_s \
/apriltag_mission/horizontal_error_m \
/apriltag_mission/descent_scale \
/apriltag_mission/mission_complete \
/apriltag_mission/pickup_succeeded \
/desire_pose_current_traj \
/mavros/imu/data \
/odom/remap \
/mavros/extended_state \
/mavros/setpoint_raw/target_attitude \
/mavros/gpsstatus/gps1/raw \
/mavros/estimator_status \
/drone0/sim_odom/time_diff \
/mavros/rc/in \
/mavros/battery \
/mavros/statustext \
/diagnostics \
/tf \
/tf_static
)

if [[ "${RECORD_CAMERA_IMAGES}" == "1" ]]; then
    topics+=(
        /camera/color/camera_info
        /camera/color/image_raw/compressed
    )
    echo "[WARN] 相机压缩图像录包已启用；起飞前确认CPU、磁盘写入和AprilTag频率仍正常。"
else
    echo "[INFO] 默认不录相机图像以避免增加识别延迟；诊断飞行可用 RECORD_CAMERA_IMAGES=1 bash record.sh。"
fi

# 板载时钟可能在重启后回到旧日期；只精确到分钟会重用文件名，
# 导致新实验覆盖/混淆历史包。加入秒和当前 PID，并显式检查冲突。
BAG_STEM="ham_flight_$(date +%Y-%m-%d-%H-%M-%S)_pid$$"
BAG_SUFFIX=0
while [[ -e "${BAG_STEM}.bag" || -e "${BAG_STEM}.bag.active" ]]; do
    BAG_SUFFIX=$((BAG_SUFFIX + 1))
    BAG_STEM="${BAG_STEM%_r*}_r${BAG_SUFFIX}"
done

echo "[INFO] rosbag output: ${BAG_DIR}/${BAG_STEM}.bag"
exec rosbag record __name:="${RECORD_NODE_NAME}" --tcpnodelay \
-O "${BAG_STEM}.bag" \
"${topics[@]}"
