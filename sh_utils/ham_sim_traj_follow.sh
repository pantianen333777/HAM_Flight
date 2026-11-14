#!/usr/bin/env bash
# =============================================================================
# ham_sim_traj_follow.sh  ——  HAM 升力翼仿真【一键启动】
# 把"启动前需手动做的"全部自动化:检查残留仿真 -> 确保 PX4 在路径 ->
# 确保 iris_liftwing 模型/机架存在 -> 启动全链路(Gazebo+PX4+MAVROS+规划器+控制器)。
#
# 用法:
#   cd sh_utils
#   ./ham_sim_traj_follow.sh          # 保留其他任务，发现冲突时退出
#   ./ham_sim_traj_follow.sh --clean  # 先清理残留仿真，再启动
#
# 该工程的仿真 odom 配置固定使用 iris_liftwing_0，因此不再暴露无效的 iris 参数。
# 前提:请在【交互式终端】运行，并在 ~/.bashrc 中配置好 ROS、PX4 与 HAM_Flight。
# =============================================================================
set -Eeuo pipefail

VEHICLE="iris_liftwing"
HAM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
COMMON="$HAM/sh_utils/_ros_common.sh"
RATES="$HAM/sh_utils/mavlink_set_rates.sh"
CLEAN=0

case "${1:-}" in
  "") ;;
  --clean) CLEAN=1 ;;
  -h|--help)
    echo "用法: ./ham_sim_traj_follow.sh [--clean]"
    exit 0
    ;;
  *)
    echo "不支持的参数: $1" >&2
    echo "用法: ./ham_sim_traj_follow.sh [--clean]" >&2
    exit 2
    ;;
esac
(( $# <= 1 )) || {
  echo "参数过多；用法: ./ham_sim_traj_follow.sh [--clean]" >&2
  exit 2
}

source "$COMMON"
trap cleanup_tabs EXIT

sim_processes_running()
{
  pgrep -x gzserver >/dev/null 2>&1 ||
    pgrep -x gzclient >/dev/null 2>&1 ||
    pgrep -x px4 >/dev/null 2>&1 ||
    pgrep -f '[r]osmaster' >/dev/null 2>&1 ||
    pgrep -f '[r]oslaunch' >/dev/null 2>&1 ||
    pgrep -f '[m]avros' >/dev/null 2>&1
}

clean_sim_processes()
{
  log ACTION "清理残留 ROS/PX4/Gazebo 仿真进程..."
  pkill -TERM -x gzserver 2>/dev/null || true
  pkill -TERM -x gzclient 2>/dev/null || true
  pkill -TERM -x px4 2>/dev/null || true
  pkill -TERM -f '[m]avros' 2>/dev/null || true
  pkill -TERM -f '[r]oslaunch' 2>/dev/null || true
  pkill -TERM -f '[r]osmaster' 2>/dev/null || true

  local deadline=$((SECONDS + 10))
  while sim_processes_running && (( SECONDS < deadline )); do
    sleep 0.2
  done
  sim_processes_running &&
    die "残留仿真进程在 10s 内未退出；请人工检查后重试。"
  log OK "残留仿真进程已清理。"
}

echo "==================== HAM 仿真一键启动 ===================="
echo "vehicle = ${VEHICLE}"

# ---------- [1/4] 检查残留仿真(仅 --clean 会主动结束进程) ----------
echo "[1/4] 检查残留仿真进程..."
if sim_processes_running; then
  (( CLEAN == 1 )) ||
    die "检测到正在运行的 ROS/PX4/Gazebo 进程；确认可以清理后使用 --clean，或先人工结束冲突任务。"
  clean_sim_processes
else
  log OK "未发现残留仿真进程。"
fi

# ---------- [2/4] 确保 PX4 在 ROS 路径 ----------
echo "[2/4] 检查 PX4 是否在 ROS 路径..."
set +u
source ~/.bashrc 2>/dev/null || true
set -Eeuo pipefail
if [ -z "$(rospack find mavlink_sitl_gazebo 2>/dev/null)" ]; then
  echo "  ⚠️  rospack 找不到 mavlink_sitl_gazebo —— PX4 未 source。"
  echo "      请确认 ~/.bashrc 里有:"
  echo "        source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default"
  echo "        export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/PX4-Autopilot"
  echo "      然后开一个新终端再跑本脚本。"
  exit 1
fi
echo "  ✓ mavlink_sitl_gazebo 已就绪"

# ---------- [3/4] 确保 iris_liftwing 模型 + PX4 机架存在 ----------
echo "[3/4] 检查 iris_liftwing 模型/机架..."
SDF="$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_liftwing/iris_liftwing.sdf"
if [ ! -f "$SDF" ]; then
  echo "  ⚠️  iris_liftwing 模型缺失:$SDF"
  echo "      请从 PX4-Autopilot 侧补齐该 SDF(含翼面 + 气动插件 + PX4 机架)后重跑。"
  exit 1
fi
echo "  ✓ iris_liftwing 模型/机架就绪"

# ---------- [4/4] 启动全链路(单窗口多 tab,tab 内自等待上游就绪) ----------
#   所有节点跑在【同一个新 gnome-terminal 窗口】的各 tab 里;当前终端不跑 roslaunch,
#   只等待全链路就绪后交还用户控制。每个 tab 先 wait 自己的上游再 roslaunch,
#   以"等待就绪"取代固定 sleep,保证启动顺序与可靠性。
#   必须在同一次 gnome-terminal 调用中创建 window 和所有 tab；否则多次
#   "gnome-terminal --tab" 会按当时的 last-opened window 选择容器，不能保证
#   加入本脚本刚创建的新窗口。--command 虽已标记 deprecated，但它是
#   gnome-terminal 单次调用时为每个 tab 分配不同命令的接口。
echo "[4/4] 启动全链路(vehicle=${VEHICLE}  ->  Gazebo 模型名 ${VEHICLE}_0)"
echo "      长运行节点开在另一个 GNOME Terminal 窗口的 4 个 tab 中。"
echo "      当前终端只负责编排并等待全链路就绪。"

# 每个 tab:先 wait 上游就绪(失败则 exec bash 保留终端排查),再 roslaunch。
T_BASE=$(mk_tab "roslaunch uav_utils base_sim_single_vehicle_ground_truth.launch vehicle:=${VEHICLE}")
T_TRAJ=$(mk_tab  "wait_roscore 90 || exec bash; roslaunch trajectory_utils traj_gen.launch")
T_CTRL=$(mk_tab  "wait_node /drone0/manager 90 || exec bash; roslaunch ctrl_node run_ctrl.launch")
T_ODOM=$(mk_tab   "wait_node /manager 90 || exec bash; roslaunch realflight_utils odom_remap_sim.launch")

gnome-terminal --quiet \
  --window --title="base_sim"  --command="$T_BASE" \
  --tab    --title="traj_gen"  --command="$T_TRAJ" \
  --tab    --title="controller" --command="$T_CTRL" \
  --tab    --title="odom_remap" --command="$T_ODOM" ||
  die "无法创建包含 4 个 tab 的仿真终端窗口。"

# 当前终端统一完成一次性速率配置和最终链路检查，不再为一次性命令保留空 tab。
wait_mavros_connected 120 "请检查 base_sim tab 中的 PX4/MAVROS。" ||
  die "MAVROS 未在 120s 内连接 PX4。"
log ACTION "配置仿真 MAVLink 数据流频率..."
bash "$RATES" || die "MAVLink 数据流频率配置失败。"

wait_nodelet /manager sim_odom 90 || die "sim_odom 未加载。"
wait_nodelet /manager odomRemap 90 || die "odomRemap 未加载。"
wait_nodelet /drone0/manager ctrl_node 90 || die "ctrl_node 未加载。"
wait_topic_messages /odom/remap 3 120 ||
  die "/odom/remap 未就绪(请检查 odom_remap / base_sim tab)。"

echo ""
echo "==================== 启动命令已发出 ===================="
echo "新终端窗口中的 4 个 tab: base_sim / traj_gen / controller / odom_remap"
echo "当前终端已完成启动检查，可在此继续执行分阶段命令:"
echo "  切OFFBOARD: bash sh_utils/set_offboard.sh"
echo "  起飞:       bash sh_utils/takeoff.sh"
echo "  机动:       bash sh_utils/pub_trigger.sh"
echo "  降落:       bash sh_utils/takeland.sh   # 驻留试验结束后显式执行"
echo "========================================================="
