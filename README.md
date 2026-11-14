codex resume 01a1bf75-6e76-7750-bddf-be23389e8404


# HAM_Flight

HAM_Flight 是一个基于 ROS Noetic、PX4/MAVROS、Gazebo Classic 和 catkin 的升力翼四旋翼控制与轨迹飞行工作空间。仓库包含控制器、轨迹生成、里程计重映射、RTK 相关工具，以及仿真和真机启动脚本。

D435i 视觉识别与 MAIN 通道夹爪控制见
[`src/vision_gripper/README.md`](src/vision_gripper/README.md)。

2026-11-12 AprilTag 实飞现象、录包诊断、软件修复、控制架构和后续优化记录见
[`EXPERIMENT_RECORD_2026-11-12.md`](EXPERIMENT_RECORD_2026-11-12.md)。最新实飞操作只使用
[`APRILTAG_RTK_SAFE_MISSION_README.md`](APRILTAG_RTK_SAFE_MISSION_README.md)。

> **安全结论：当前代码可以完成 Release 构建，但不能据此直接真机起飞。** 先完成仿真、无桨台架和低风险系留验收，再由具备资质的操作者在合规场地试飞。任何时候都应以外部遥控器接管和 PX4 独立 failsafe 为最终安全手段。

## 当前状态（2026-11-13 更新）

测试环境：Ubuntu 20.04.6 LTS、arm64、ROS Noetic。

| 检查项 | 结果 | 说明 |
|---|---:|---|
| `catkin_make -DCMAKE_BUILD_TYPE=Release` | ✅ 通过 | 14 个 catkin 包构建完成，退出码 0 |
| Shell 语法检查 `bash -n sh_utils/*.sh` | ✅ 通过 | 所有脚本通过语法检查 |
| 4 个核心 launch 文件解析 | ✅ 通过 | 轨迹、控制器、真机/仿真 odom remap 可解析 |
| `catkin_make run_tests` | ❌ 未通过 | 测试目标本身存在构建问题，见“已知问题” |
| MAVROS、pyproj、地理数据 | ✅ 已安装 | MAVROS 1.20.1、pyproj 2.5.0，GeographicLib 三类数据集已安装 |
| PX4-Autopilot | ✅ AprilTag 仿真可用 | PX4 SITL、Gazebo Classic、MAVROS 闭环已运行 |
| 自定义 `iris_liftwing` | ❌ 缺失 | 当前机器未找到模型，必须从旧电脑迁移 |
| AprilTag 仿真闭环 | ✅ 已完成 | 正常抓取、短时丢码重捕获、永久丢码安全返航三类验证通过 |
| 真机飞行验收 | ⛔ 未证明 | 编译成功不等于飞行安全或参数适配完成 |

因此，准确状态是：**AprilTag 软件闭环和故障状态机已经通过标准 Iris 仿真；真实舵机、
相机角外参、重心/振动、RTK 现场状态和装桨真机验收仍未由软件证明。**

### 仿真依赖和实机依赖不是一回事

- PX4 SITL、Gazebo Classic 和 `iris_liftwing.sdf` 只用于仿真；
- 实机运行使用 Pixhawk 中已经刷入的 PX4 固件，不读取电脑上的 SDF 模型；
- 因此模型缺失不直接阻止实机连接，但在未完成仿真和台架验收前，不应直接执行轨迹实飞；
- 实机需要 MAVROS、遥控器、Pixhawk、RTK、正确的机体/控制参数和可靠的人工接管方案。

## 为什么不能直接飞

当前默认参数具有明确的机型和场地假设：

- 飞机质量为 `4.6 kg`，悬停油门为 `0.58`；
- 当前 AprilTag 任务起飞高度为 `1.5 m`；
- 当前默认轨迹为 `traj_type=3`：原点悬停 3 s 后沿触发时机头方向平飞 3 m；
- 真机串口固定为 `/dev/ttyACM0:57600`；
- 依赖 RTK Fixed、PX4 EKF 绝对位置和 `/odom/remap`；
- 要求 `COM_OF_LOSS_T <= 1.0 s`、`COM_OBL_RC_ACT=4 (Land)`；
- 整段任务脚本会写入 PX4 的 `MPC_THR_HOVER`、`MPC_LAND_SPEED` 和 `COM_DISARM_LAND`。

这些值必须与实际重量、桨叶、电机、电池、重心、推力模型、PX4 固件、定位源和飞行场地逐项核对。3 m 直线任务也只能在分级验收完成后执行。

关键参数文件：

- 控制器与机体参数：[`src/ctrl_node/config/ctrl_param.yaml`](src/ctrl_node/config/ctrl_param.yaml)
- 轨迹参数：[`src/utils/trajectory_utils/config/odom_param.yaml`](src/utils/trajectory_utils/config/odom_param.yaml)
- 真机里程计配置：[`src/utils/realflight_utils/launch/odom_remap.launch`](src/utils/realflight_utils/launch/odom_remap.launch)
- 仿真里程计配置：[`src/utils/realflight_utils/launch/odom_remap_sim.launch`](src/utils/realflight_utils/launch/odom_remap_sim.launch)

## 环境与依赖

建议使用 Ubuntu 20.04 + ROS Noetic + Gazebo Classic。先安装基础依赖：

```bash
sudo apt update
sudo apt install \
  ros-noetic-mavros \
  ros-noetic-geodesy \
  python3-pyproj \
  libarmadillo-dev \
  psmisc \
  gnome-terminal
```

本仓库自带 `mavros_msgs` 和 `geodesy` 源码包，但不包含 MAVROS 节点本体；`roslaunch mavros px4.launch` 仍要求安装 `ros-noetic-mavros`。若 MAVROS 提示 GeographicLib 数据缺失，按安装包提供的脚本安装数据集：

```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

当前机器已将官方 PX4-Autopilot v1.14.4 下载到：

```text
/home/cat/PX4-Autopilot
```

选择 v1.14.4 是因为本项目使用 Ubuntu 20.04、ROS Noetic、Gazebo Classic，以及 `Tools/simulation/gazebo-classic` 目录结构。不要直接切换到新版 PX4/Gazebo 后继续使用现有 launch 文件。

仿真还需要：

1. 与 ROS Noetic/Gazebo Classic 兼容的 PX4-Autopilot；
2. PX4 内存在自定义机型 `iris_liftwing`；
3. 以下文件存在：

```text
~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_liftwing/iris_liftwing.sdf
```

4. ROS 能找到 `mavlink_sitl_gazebo`。可在 `~/.bashrc` 配置：

```bash
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
  ~/PX4-Autopilot \
  ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$HOME/PX4-Autopilot"
export PX4_ROOT="$HOME/PX4-Autopilot"
```

### 完成当前 PX4 SITL 构建

当前已补齐 GStreamer 开发库，但浅克隆的 NuttX 子模块还缺少版本标签。联网后执行：

```bash
cd ~/PX4-Autopilot/platforms/nuttx/NuttX/nuttx
git fetch --tags --depth=1 origin

cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo-classic
```

验证构建产物：

```bash
test -x ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 \
  && echo "PX4 SITL build OK"
```

### 在旧电脑查找 `iris_liftwing`

先在旧电脑执行：

```bash
find "$HOME" /opt /mnt /media \
  -type f \
  \( -iname 'iris_liftwing.sdf' \
     -o -iname 'iris_liftwing.sdf.jinja' \
     -o -iname '*liftwing*.urdf' \
     -o -iname '*liftwing*.xacro' \) \
  -print 2>/dev/null
```

如果知道旧电脑的 PX4 路径，再执行：

```bash
cd ~/PX4-Autopilot

# 查模型、机架编号、启动脚本和所有文本引用
find Tools ROMFS -iname '*liftwing*' -print
rg -n 'iris_liftwing|liftwing' Tools ROMFS boards CMakeLists.txt

# 检查是否还有未提交的自定义改动
git status --short
git diff -- Tools ROMFS boards > ~/iris_liftwing.patch
git describe --tags --always
git submodule status > ~/px4-submodules.txt
```

至少要找齐：

1. `models/iris_liftwing/` 整个目录，而不只是一个 SDF；
2. SDF 引用的 `meshes/`、`materials/`、贴图和插件配置；
3. `ROMFS/px4fmu_common/init.d-posix/airframes/` 下对应的自定义机架文件；
4. airframe 列表、CMake 或启动脚本中的注册改动；
5. 如果修改过 Gazebo 插件，还要保存插件源码和 `git diff`；
6. 旧电脑使用的 PX4 tag/commit 和各子模块 commit。

最稳妥的迁移方法是直接保存旧电脑整个 PX4 工作树：

```bash
cd ~
tar --exclude='PX4-Autopilot/build' \
    --exclude='PX4-Autopilot/.git/modules/*/objects' \
    -czf PX4-Autopilot-with-iris_liftwing.tgz PX4-Autopilot
```

传到新电脑后先解压到单独目录进行比对，不要立即覆盖当前 v1.14.4：

```bash
mkdir -p ~/px4_old_compare
tar -xzf PX4-Autopilot-with-iris_liftwing.tgz \
    -C ~/px4_old_compare

diff -ruN \
  ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models \
  ~/px4_old_compare/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
```

如果旧电脑不在身边，可以让对方执行上述 `find`、`git status`、`git describe` 和打包命令，再通过 U 盘、局域网 `scp` 或网盘传输压缩包与两个文本文件。

## 编译

在工作空间根目录执行：

```bash
cd ~/HAM_Flight
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

每个新终端都要重新加载环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/HAM_Flight/devel/setup.bash
```

可以用以下命令检查关键包：

```bash
rospack find trajectory_utils
rospack find ctrl_node
rospack find realflight_utils
rospack find mavros
```

## 推荐流程：先跑完整仿真

### 1. 启动仿真链路

在交互式 GNOME 桌面终端运行：

```bash
cd ~/HAM_Flight
bash sh_utils/ham_sim_traj_follow.sh
```

脚本会打开 `base_sim`、`traj_gen`、`controller`、`odom_remap` 四个标签页，并检查 MAVROS、nodelet 和 `/odom/remap` 是否就绪。

若检测到残留的 ROS/PX4/Gazebo 进程，先确认这些进程可以结束，再使用：

```bash
bash sh_utils/ham_sim_traj_follow.sh --clean
```

`--clean` 会终止当前用户的 Gazebo、PX4、MAVROS、roslaunch 和 rosmaster 相关进程，不要在其他 ROS 任务仍运行时使用。

### 2. 执行带检查的完整仿真任务

推荐使用完整任务脚本，而不是逐条手工发命令。

终端 A：

```bash
cd ~/HAM_Flight
source devel/setup.bash
bash sh_utils/sim_whole_traj.sh
```

确认仿真场景、参数和桨区安全后，按提示输入大写 `TAKEOFF`。脚本开始等待 OFFBOARD 时，在终端 B 执行：

```bash
cd ~/HAM_Flight
source devel/setup.bash
bash sh_utils/set_offboard.sh
```

之后终端 A 会自动完成起飞、等待控制器 READY、触发轨迹、降落和上锁。rosbag 会在独立窗口继续记录；任务完成后在录包窗口按 `Ctrl+C`，等待 bag 索引写入完成。

### 3. 分阶段仿真（仅用于调试）

完整链路 READY 后可依次执行：

```bash
bash sh_utils/set_offboard.sh
bash sh_utils/takeoff.sh
bash sh_utils/pub_trigger.sh
bash sh_utils/takeland.sh
```

每一步都要先观察 PX4/MAVROS 状态。`takeoff.sh` 本身没有完整任务脚本的全部门禁，不建议作为首次验证入口。

## 真机操作

> **AprilTag RTK 抓取任务不要执行本节的通用 RTK 脚本。** 该任务只能使用
> [APRILTAG_RTK_SAFE_MISSION_README.md](APRILTAG_RTK_SAFE_MISSION_README.md) 的唯一拓扑；
> 混用下面的 `RTK_realflight_traj_follow.sh` 会重复启动规划器和控制器。

### 真机前必须完成

- 仿真完整任务连续成功，且检查过 rosbag；
- 拆桨完成串口、MAVROS、RTK、EKF、坐标系、姿态方向和 setpoint 台架测试；
- 校准实际质量、重心、悬停油门和控制增益；
- 确认当前 3 m 直线轨迹与现场净空、机头方向和标签位置一致；
- 验证遥控器模式开关、人工接管、急停/上锁和失联降落；
- 确认 PX4 failsafe、地理围栏、电池保护和返航/降落策略；
- 使用防护装备、观察员和隔离场地，并遵守当地法规；
- 首次有桨测试采用系留、低高度、无人员进入桨区的方案。

### 1. 启动真机控制链路（不会解锁）

连接 Pixhawk 和 RTK，保持飞行器在地面、未解锁、非 OFFBOARD，然后运行：

```bash
cd ~/HAM_Flight
bash sh_utils/RTK_realflight_traj_follow.sh
```

该脚本使用 `/dev/ttyACM0:57600`，会依次检查串口、ROS master、MAVROS、落地状态、PX4 Offboard 丢失保护、RTK Fixed、EKF、里程计、控制器输出和 ROS 话题链路。它只在全部检查通过后显示 `READY`，**不会解锁、不会切 OFFBOARD、不会起飞**。

如果设备名或波特率不同，应先修改脚本顶部的 `FCU_DEVICE` 和 `FCU_BAUD`，不要用临时软链接掩盖设备识别问题。

### 2. 首次有桨测试：只做低高度悬停

首次有桨测试不要直接触发完整 3 m 抓取轨迹。应在隔离场地、最好系留，并安排一名遥控器飞手随时接管。

先运行真机链路并等待 `READY`：

```bash
cd ~/HAM_Flight
bash sh_utils/RTK_realflight_traj_follow.sh
```

看到全部 `READY` 且没有 `FATAL` 后：

1. 清空桨区，飞手保持遥控器接管开关可用；
2. 由外部遥控器切入 OFFBOARD；
3. 立即在终端发送起飞命令：

```bash
bash sh_utils/takeoff.sh
```

4. 只观察约 1 m 高度悬停，不执行 `pub_trigger.sh`；
5. 确认悬停后执行：

```bash
bash sh_utils/takeland.sh
```

6. 必须确认 `armed: False` 且 `landed_state: 1` 后才能接近飞机。

任何漂移、振荡、方向错误、RTK/EKF 异常或高度异常，都应立即通过外部遥控器退出 OFFBOARD，并按现场预案接管/降落。空中按 `Ctrl+C` 不等于安全降落。

### 3. 完整轨迹任务（首次测试禁止使用）

只有仿真、无桨台架、低高度悬停、人工接管和 AUTO.LAND 均验证通过，并缩小默认轨迹后，才可以执行完整任务。

保持飞行器未解锁且非 OFFBOARD，在另一个终端运行：

```bash
cd ~/HAM_Flight
source devel/setup.bash
bash sh_utils/RTK_whole_traj_no_post_quick.sh
```

正确顺序是：

1. 脚本复核控制链路、RTK/EKF、PX4 状态、磁盘空间和任务参数；
2. 人工确认参数和现场安全后，输入大写 `TAKEOFF`；
3. 脚本启动录包，并提示等待 OFFBOARD；
4. **此时才由外部遥控器切入 OFFBOARD**；
5. 脚本发送起飞命令，等待控制器 READY，执行轨迹并请求 PX4 AUTO.LAND；
6. 确认自动上锁且 `ON_GROUND` 后，任务才报告完成；
7. 在独立录包窗口按 `Ctrl+C`，等待 bag 索引完成。

真机严禁使用 `sh_utils/set_offboard.sh` 切模式。该脚本只用于仿真；真机必须保留外部遥控器的人工模式切换和接管能力。

### 中断与接管

- 空中出现任何异常，立即使用外部遥控器退出 OFFBOARD 并按现场预案接管/降落；
- 空中按 `Ctrl+C` 只会停止自动流程，脚本不会保证自动降落或上锁；
- `takeland.sh` 是辅助命令，不可替代遥控器接管和 PX4 独立 failsafe；
- 不要在空中关闭 MAVROS、控制器、里程计或规划器终端；
- 不要重复启动两套 MAVROS、manager 或控制器。

## 运行状态快速检查

```bash
# PX4 是否连接、模式和解锁状态
rostopic echo -n 1 /mavros/state

# 是否在地面
rostopic echo -n 1 /mavros/extended_state

# RTK 状态
rostopic echo -n 1 /mavros/gpsstatus/gps1/raw

# EKF 状态
rostopic echo -n 1 /mavros/estimator_status

# 重映射里程计频率
rostopic hz /odom/remap

# 控制器姿态设定值频率
rostopic hz /mavros/setpoint_raw/attitude

# 节点和 nodelet
rosnode list
rosservice call /manager/list
rosservice call /drone0/manager/list
```

## 录包

完整任务脚本会自动打开录包窗口。也可手动运行：

```bash
bash sh_utils/record.sh
```

bag 保存到 `sh_utils/Bag/`。结束时使用 `Ctrl+C`，不要直接关闭终端，否则 bag 可能没有完成索引。

## 已知问题

`catkin_make run_tests` 当前失败，不能宣称“测试全部通过”：

1. `hc_msg_parser_test`、`figure_eight_test`、`mission_trigger_test` 链接时报 `undefined reference to main`；
2. `uav_utils-test` 仍按 5 参数调用 `extract_odometry`，而当前接口只提供 4 参数或 6 参数版本；
3. `python3-pyproj` 已安装；`pose_utils/package.xml` 使用的 `libarmadillo-dev` 仍不是可解析的 rosdep key，但系统 Debian 包已经安装；
4. Gazebo Classic 已进入上游停止维护阶段，继续使用时应固定已验证的软件版本；
5. 当前目录的 `.git` 不含可用仓库元数据，因此无法从 Git 状态判断本地修改或版本来源；
6. PX4 v1.14.4 已下载，首次 Gazebo 构建曾因缺少 GStreamer 开发库和浅克隆缺少 NuttX tags 中止；GStreamer 已补齐，NuttX tags 和最终重编译仍待完成；
7. 当前机器没有 `iris_liftwing` 模型，仿真启动脚本会在模型检查阶段退出。

测试修复后重新执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make run_tests
catkin_test_results --all build/test_results
```

## 脚本索引

| 脚本 | 用途 | 真机可用 |
|---|---|---:|
| `ham_sim_traj_follow.sh` | 启动完整 PX4/Gazebo 仿真链路 | 否 |
| `sim_whole_traj.sh` | 仿真起飞、轨迹、降落完整任务 | 否 |
| `RTK_realflight_traj_follow.sh` | 真机链路预检并停在 READY | 是 |
| `RTK_whole_traj_no_post_quick.sh` | 真机完整任务与录包 | 是，需严格验收 |
| `record.sh` | 记录关键 ROS 话题 | 是 |
| `pub_trigger.sh` | 单次轨迹触发 | 调试用 |
| `takeoff.sh` / `takeland.sh` | 分阶段起降 | 调试用 |
| `set_offboard.sh` | 通过 MAVROS 切 OFFBOARD | **仅仿真** |
| `oldRTK_realflight_traj_follow.sh` | 旧版真机启动脚本 | 不建议使用 |

## AprilTag 抓取返航扩展

`vision_gripper` 现已提供固定任务：起飞至 1.5 m，沿机头平飞 3 m，途中识别
AprilTag，在 45 cm 对准闭爪，恢复 1.5 m 巡航高度，直线返回原点并自动请求
PX4 `AUTO.LAND`。完整参数、启动顺序、状态话题和故障处理见
[vision_gripper/README.md](src/vision_gripper/README.md)。该扩展不会自动切换 OFFBOARD；
首次验证必须拆桨或使用 `dry_run:=true`。

带桨实飞前必须阅读并逐项执行唯一推荐的新流程：
[APRILTAG_RTK_SAFE_MISSION_README.md](APRILTAG_RTK_SAFE_MISSION_README.md)。该文档只描述
当前 1.5 m/3 m/45 cm RTK 任务，包含自动定位门禁、唯一节点拓扑、实际起飞/触发/
抓取/返航/降落顺序和接管方式。
