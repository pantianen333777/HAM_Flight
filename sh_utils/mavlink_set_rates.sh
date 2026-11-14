#!/bin/bash
# =============================================================================
# mavlink_set_rates.sh —— MAV_CMD_SET_MESSAGE_INTERVAL(511)批量下发
# 配置 PX4 经 MAVROS 下行的关键数据流频率:
#   ID105 HIGHRES_IMU / ID31 ATTITUDE_QUATERNION / ID32 LOCAL_POSITION_NED /
#   ID331 ODOMETRY / ID245 EXTENDED_SYS_STATE /
#   ID24 GPS_RAW_INT / ID230 ESTIMATOR_STATUS
#
# 姿态/位置/里程计默认 5000us=200Hz；EXTENDED_SYS_STATE 默认 500000us=2Hz。
# 实飞频率不同,可用环境变量覆盖,例如:
#   ID31=10000 ID32=10000 ID331=20000 bash mavlink_set_rates.sh
#
# 可靠性: master/MAVROS 不在时 rosrun mavcmd 会无限阻塞且零输出, 故
#   1) 顶部预检 master/MAVROS, 缺失立即失败;
#   2) 每条 mavcmd 带 timeout 硬上界 —— 预检通过后 mavros 中途死亡(串口拔出/
#      进程崩溃)也不会无限等待。超时可用 MAVCMD_TIMEOUT_S 覆盖。
# =============================================================================
# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash 2>/dev/null
# shellcheck source=/dev/null
source ~/.bashrc 2>/dev/null

# 预检: /mavros/state 由 mavros 发布, 不存在即 master 或 mavros 缺位。
timeout 5 rostopic info /mavros/state >/dev/null 2>&1 ||
    { echo "[ERROR] ROS master 不可用或 MAVROS 未运行(/mavros/state 不存在), 无法下发 MAVLink 频率。"; exit 1; }

MAVCMD_TIMEOUT_S="${MAVCMD_TIMEOUT_S:-10}"

ID105="${ID105:-5000}"
ID31="${ID31:-5000}"
ID32="${ID32:-5000}"
ID331="${ID331:-5000}"
ID245="${ID245:-500000}"
ID24="${ID24:-200000}"
ID230="${ID230:-200000}"
ENABLE_NAV_STATUS_RATES="${ENABLE_NAV_STATUS_RATES:-0}"

echo "[CONFIG] MAVLink 数据流频率: ID105=${ID105}us ID31=${ID31}us ID32=${ID32}us ID331=${ID331}us ID245=${ID245}us"

timeout -k 3 "$MAVCMD_TIMEOUT_S" rosrun mavros mavcmd long 511 105 "$ID105" 0 0 0 0 0 || { echo "[ERROR] 配置 HIGHRES_IMU(105)失败(超时 ${MAVCMD_TIMEOUT_S}s 或被拒绝)"; exit 1; }
timeout -k 3 "$MAVCMD_TIMEOUT_S" rosrun mavros mavcmd long 511 31  "$ID31"  0 0 0 0 0 || { echo "[ERROR] 配置 ATTITUDE_QUATERNION(31)失败(超时 ${MAVCMD_TIMEOUT_S}s 或被拒绝)"; exit 1; }
timeout -k 3 "$MAVCMD_TIMEOUT_S" rosrun mavros mavcmd long 511 32  "$ID32"  0 0 0 0 0 || { echo "[ERROR] 配置 LOCAL_POSITION_NED(32)失败(超时 ${MAVCMD_TIMEOUT_S}s 或被拒绝)"; exit 1; }
timeout -k 3 "$MAVCMD_TIMEOUT_S" rosrun mavros mavcmd long 511 331 "$ID331" 0 0 0 0 0 || { echo "[ERROR] 配置 ODOMETRY(331)失败(超时 ${MAVCMD_TIMEOUT_S}s 或被拒绝)"; exit 1; }
timeout -k 3 "$MAVCMD_TIMEOUT_S" rosrun mavros mavcmd long 511 245 "$ID245" 0 0 0 0 0 || { echo "[ERROR] 配置 EXTENDED_SYS_STATE(245)失败(超时 ${MAVCMD_TIMEOUT_S}s 或被拒绝)"; exit 1; }
if [ "$ENABLE_NAV_STATUS_RATES" = "1" ]; then
    echo "[CONFIG] RTK/EKF 监测频率: ID24=${ID24}us ID230=${ID230}us"
    timeout -k 3 "$MAVCMD_TIMEOUT_S" rosrun mavros mavcmd long 511 24  "$ID24"  0 0 0 0 0 || { echo "[ERROR] 配置 GPS_RAW_INT(24)失败(超时 ${MAVCMD_TIMEOUT_S}s 或被拒绝)"; exit 1; }
    timeout -k 3 "$MAVCMD_TIMEOUT_S" rosrun mavros mavcmd long 511 230 "$ID230" 0 0 0 0 0 || { echo "[ERROR] 配置 ESTIMATOR_STATUS(230)失败(超时 ${MAVCMD_TIMEOUT_S}s 或被拒绝)"; exit 1; }
fi

echo "[OK] MAVLink 频率配置完成。"
