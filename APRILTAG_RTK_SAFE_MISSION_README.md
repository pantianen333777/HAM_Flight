# HAM_Flight RTK + AprilTag 最安全实飞任务

这是当前 **唯一推荐** 的 AprilTag 抓取实飞流程。它适用于以下固定任务：

1. 无人机从地面起飞到原点上方 `1.0 m`；
2. 原点稳定后，沿发布 TAKEOFF 瞬间锁定的初始机头方向平飞 `3.0 m`；
3. 途中发现 ID 0 的 AprilTag 后，停止后续预设直线并进入相对跟踪；
4. 用完整姿态补偿和实测外参对准物品，在 D435i 光轴距离 `0.45 m` 处闭爪一次；
5. 恢复到 `1.0 m` 巡航世界高度，以 `0.18 m/s` 直线返回任务原点；
6. 原点稳定 `2 s` 后进入 `HOLD_HOME`，持续悬停；
7. 飞手确认位置和状态后，手动发布 `takeland.sh`。

本文的目标是让 RTK 定位链路、规划器、仲裁器和控制器形成一条唯一、可检查、失败关闭的
控制链。不要把本文命令与旧的 RTK 整机启动脚本混用。

## 1. 唯一允许的节点与命令拓扑

本任务只允许以下结构：

```text
Pixhawk/PX4
  <-> /mavros                         （只启动一次）

/manager
  └─ sim_odom                         （只生成预设轨迹）
       └─ /drone0/planning/preset_cmd
             └─ /apriltag_mission     （唯一命令仲裁器）
                   └─ /drone0/planning/cmd
                         └─ /drone0/manager/ctrl_node
                               └─ /mavros/setpoint_raw/attitude

/drone0/manager
  ├─ odomRemap -> /odom/remap
  └─ ctrl_node

D435i -> apriltag_detector -> apriltag_gripper/tag_position
                              └─ apriltag_mission
```

关键约束：

- `/manager` 中只能有一个 `sim_odom`；
- `/drone0/manager` 中只能有一个 `odomRemap` 和一个 `ctrl_node`；
- `/drone0/planning/preset_cmd` 只能由 `/manager` 发布；
- `/drone0/planning/cmd` 只能由 `/apriltag_mission` 发布；
- 规划器不能直接发布最终 `/drone0/planning/cmd`；
- MAVROS、规划器、里程计重映射、控制器和 AprilTag 任务都不能重复启动。

配套预检程序会从 ROS Master 读取真实发布/订阅图，任何一项不符都拒绝给出 READY。
它也会核对检测器实际加载的是 `tag36h11 / ID 0 / 0.08746 m`，从而阻止旧标签尺寸
残留在运行时参数中。

## 2. 本任务中禁止运行的命令

执行本文任务时，禁止同时运行：

```text
sh_utils/RTK_realflight_traj_follow.sh
sh_utils/RTK_whole_traj_no_post_quick.sh
sh_utils/ham_sim_traj_follow.sh
sh_utils/sim_whole_traj.sh
sh_utils/set_offboard.sh
```

前两个旧 RTK 脚本会自行创建规划器、manager 或控制器，与 AprilTag 组合 launch 重复。
`set_offboard.sh` 只供仿真使用；实机必须使用遥控器切换模式并保留人工接管能力。

如果已经运行过任何旧脚本，应先在飞行器 **未解锁、非 OFFBOARD、确认在地面** 的情况下
正常关闭旧节点，再重新开始本文。不要在空中重启或关闭 MAVROS、manager、控制器、相机。

## 3. 当前固定参数

### 飞行轨迹

```yaml
takeoff_height: 1.0 m
origin_hold: 3.0 s
forward_distance: 3.0 m
forward_time: 27.0 s（3 m七阶平滑轨迹峰值约0.243 m/s）
endpoint_hold_without_tag: 2.0 s
fallback_return_time: 27.0 s
```

轨迹参数文件：
[`src/utils/trajectory_utils/config/odom_param.yaml`](src/utils/trajectory_utils/config/odom_param.yaml)

起飞参数文件：
[`src/ctrl_node/config/ctrl_param.yaml`](src/ctrl_node/config/ctrl_param.yaml)

### 本任务使用的控制算法

这套任务是“任务状态机 + 平滑位置/速度外环 + 现有姿态控制器”的分层结构，没有在视觉节点中
再直接控制电机，因此不会与 `ctrl_node` 或 PX4 的姿态环争用。

1. **起飞高度闭环**：以 `0.25 m/s` 生成到 `1.0 m` 的垂向参考。只有实际高度误差小于
   `0.08 m`、垂向速度小于 `0.15 m/s` 并连续稳定 `0.5 s`，才认为起飞完成。不再用
   “快到目标就把当前位置当悬停点”的旧逻辑，因此不会把掉高固化成新高度。
2. **初始航向锁存**：发布 TAKEOFF 时记录机头 yaw，通过带任务 ID 的 READY 握手传给规划器。
   规划器和视觉任务都使用这个 yaw，不采用悬停稳定后可能已偏移的机头方向。
3. **预设直线轨迹**：3 m 前飞/返回使用七阶时间缩放多项式，起点和终点的速度、加速度、
   jerk 都连续归零。27 s 前飞的理论峰值速度约 `0.243 m/s`，避免阶跃轨迹引起俯仰摇摆。
4. **全姿态视觉转换**：先将 D435i 光学坐标转到机体 FLU，加上相机—夹爪平移，再使用飞机实时
   roll/pitch/yaw 转到重力水平世界系；同时用图像时间戳和 odom 历史消除图像延迟期间的机体位移。
5. **比例速度外环**：视觉误差通过 `v*=sat(Kp*e)` 变成世界系速度参考。水平/垂向比例增益为
   `0.55/0.40`，远处限速 `0.12/0.07 m/s`，近处限速 `0.05/0.03 m/s`。速度参考再经过
   `0.10 m/s²` 斜率限制，并限制位置参考超前量，防止标签抖动变成左右急促指令。
6. **水平优先与垂向迟滞**：水平误差超过 `0.16 m` 时禁止下降，`0.07～0.16 m` 内连续缩放下降
   速度，对准后才全速下降。相机距离进入 `0.45±0.03 m` 后垂向速度置零，消除深度噪声引起的
   反复上下。
7. **有限状态机安全逻辑**：`FOLLOW_PRESET -> TRACK_TAG -> CLOSING -> ASCEND -> RETURN_HOME -> HOLD_HOME`。
   丢码不会立即失败，而是减速保持、小幅上升重获；只有超时才中止抓取并返航。闭爪还需同时通过
   连续对准和三个速度门限，且单次任务只允许调用一次闭爪服务。
8. **返航梯形速度规划**：返航最大速度 `0.18 m/s`、加/减速度 `0.10 m/s²`；每个周期用
   `sqrt(2*a*remaining_distance)` 计算可在原点前停下的制动速度，因此返航启动和到点都不会产生速度阶跃。

`ctrl_node` 仍是唯一低层飞行控制器：它把上述位置/速度参考转成推力和目标姿态，PX4 内环跟踪姿态。
上位机不同时运行第二套电机或姿态控制器。

### AprilTag 与夹爪

```yaml
family: tag36h11
id: 0
configured_tag_size: 0.08746 m
camera_target_z: 0.45 m
distance_tolerance: 0.03 m
horizontal_tolerance: 0.03 m
camera_to_gripper_forward: -0.058 m
camera_to_gripper_right: 0.000 m
camera_to_gripper_down: 0.110 m
object_forward_from_tag: 0.030 m
object_height_above_tag: 0.340 m
max_tracking_xy_speed: 0.12 m/s
max_tracking_z_speed: 0.07 m/s
near_tracking_xy_speed: 0.05 m/s
near_tracking_z_speed: 0.03 m/s
tracking_acceleration_limit: 0.10 m/s^2
pose_filter_window: 3 frames
apriltag_decimate: 1.0
controller_command_timeout: 0.30 s
return_speed: 0.18 m/s
auto_land_on_return: false
```

> 2026-11-10 已更换为打印标签。120 组 AprilTag 位姿与 D435i 对齐深度同步样本得到：
> 深度中位数 `0.916 m`、旧尺寸位姿 z 中位数 `0.462344 m`，因此当前有效边长为
> `0.08746 m`。如果这里仍显示旧尺寸，必须重新构建并重新启动终端 3，禁止实飞。

`0.45 m` 是相机光轴方向的 AprilTag 位姿 z，不是无人机离地高度。待抓物固定在标签
上方 `0.340 m`、朝机头前方 `0.030 m`；相机位于夹爪前方 `0.058 m`、高 `0.110 m`，
因此名义抓取图像坐标为 `(0, 0.088, 0.45) m`。任务使用完整 roll/pitch/yaw 转换到
世界系后计算误差，不再把机身倾斜产生的图像位移当成水平位置误差。

这里的 `0.340 m` 指物品被两个爪真正夹住的接触中心到标签平面的垂直高度，不是物品
顶面高度。当前布置必须满足：标签水平固定、标签一条边与机头方向平行，夹持中心位于
标签中心朝机头 `30 mm`、高 `340 mm` 处。标签必须保持可见。若实际接触中心高度为
`h`，相机目标距离应改为 `0.110 + h`，不能仍使用 0.45 m。当前任务没有最终缓降段，
所以放置尺寸错误不会由下降动作补偿。

更换标签、打印尺寸、手机显示比例或缩放后，必须重新测量真实有效边长并修改
[`src/vision_gripper/config/apriltag_tags.yaml`](src/vision_gripper/config/apriltag_tags.yaml)。

新标签或重新打印后，在飞控未解锁、任务与自动闭爪均关闭时，可用 D435i 对齐深度复核：

```bash
rosrun vision_gripper calibrate_apriltag_depth.py \
  _tag_id:=0 \
  _configured_tag_size_m:=0.08746 \
  _known_distance_m:=0.89 \
  _sample_count:=120
```

该工具只订阅相机和检测话题，不发布控制命令。`known_distance_m` 只作为人工尺量对照，
最终应优先检查 `D435 aligned depth` 与 `AprilTag pose z` 是否一致。标签有效边长与
相机—夹爪刚体外参是两个独立参数；修改打印尺寸不会改变刚体外参。

### 2026-11-14 软件闭环验收结果

- Python 语法、视觉跟踪单测、控制器单测和相关包编译均通过；
- 几何测试中人为加入 10° 横滚，完整姿态补偿后的世界系目标误差仍接近零；
- 当前版 PX4 SITL + Gazebo 已验证：精确 1.0 m 起飞、锁定 TAKEOFF 航向、发现标签、闭爪、
  0.10 m/s 回升、加减速返航和 `HOLD_HOME`；最终位置约
  `(0.002, -0.0002, 1.013) m`，`pickup_succeeded=true`、`mission_complete=true`；
- 随后人工 `takeland.sh` 已让 PX4 进入 `AUTO.LAND` 并落地下锁。仿真暴露的
  `HOLD_HOME -> AUTO.LAND` 被误认为退出 OFFBOARD 故障的状态文本问题已修复，并新增单测锁定此转移；
- 已修复视觉接管时把规划器高度参考重置为实际高度造成的推力阶跃。接管现在继承规划器
  的位置和速度参考；实测仿真接管前后 Z 参考保持 `1.4447 m`，没有阶跃；
- 已修复参考超前限制跟随跑偏飞机一起移动的问题：参考超过限制后保持不动，只阻止继续
  远离，不再因飞机向错误方向运动而卸掉恢复位置误差；
- 已修正 Gazebo 真值节点中相机到夹爪刚体平移的符号；
- 一轮诊断仿真中，50 Hz Python 任务流出现过 `0.116 s` 调度间隙，旧 `0.10 s` 控制器
  看门狗会不可恢复地从 `MISSION` 退到 `HOVER`。控制命令看门狗现为 `0.30 s`；最终完整
  仿真没有再中途退出任务；
- 跟踪中人为断开检测 4 s，状态按
  `TRACK_TAG -> REACQUIRE_TAG -> TRACK_TAG` 恢复，随后仍完整抓取返航；
- 持续断开检测时，任务会先保持、小幅上升重获，超时才中止抓取、回升、返回原点并
  进入 `HOLD_HOME`，最终降落仍由飞手发布 `takeland.sh`；
- 2026-11-14 诊断仿真的跟踪阶段实际横滚峰峰值约 `0.68°`、俯仰峰峰值约 `2.74°`；
  闭爪前最终世界系水平误差约 `5.3 mm`、垂直误差约 `25.6 mm`，满足 30 mm 门限。

仿真验证的是任务仲裁、坐标变换、控制参考和故障状态机，不包含真实 D435i 光照/模糊、
RTK 多路径、风、重心、桨振或无反馈舵机是否真实运动。因此它不能替代第 4、7、15 节的
真机分级验收。

## 4. 装桨前不可跳过的硬件验收

以下项目没有自动化替代方案：

- 电机编号、旋向、桨叶 CW/CCW 和安装方向逐个核对；
- 机架、夹爪、D435i、电池和 RTK 天线紧固，重心合理；
- 夹爪 MAIN1 映射、张开方向、闭合方向和最大行程已在拆桨状态验证；
- 遥控器模式开关、人工接管、急停/上锁和 PX4 `AUTO.LAND` 已分别验证；
- 飞行区有足够的 3 m 前向净空、返航净空和降落区，人员全部退出桨区；
- 驾驶员全程握持遥控器，观察员能看到飞机、标签和降落区；
- 电池、天气、法规、地理围栏和现场应急预案满足实飞要求。

最近一次拆桨检查曾观测到 `fix_type=0`、卫星数 0、EKF 水平位置无效和
`/odom/remap` 发散。在重新获得 RTK Fixed 并通过本文连续采样预检前，禁止装桨起飞。

`PWM_MAIN_FUNC1=301` 只证明 PX4 把 MAIN1 配成 Servo 1，不证明线束插在 MAIN1，也不
证明舵机供电正常。拆桨状态分别发送一次张开和闭合命令，同时检查：舵机插头确在 MAIN1、
信号/5V/GND 顺序正确、舵机电源与飞控共地、机械行程没有顶死，并观察以下话题：

```bash
rostopic echo /apriltag_gripper/command_action
rostopic echo /apriltag_gripper/commanded_value
rostopic echo /apriltag_gripper/command_accepted
```

`command_accepted=true` 只表示 PX4 接受命令；仍必须肉眼确认夹爪真实运动。没有位置或电流
反馈时，软件无法证明已经夹住物品。

## 5. 构建

```bash
cd ~/HAM_Flight
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

构建失败不得继续。每个新终端都必须执行两条 `source`。

## 6. 两套实飞操作方法（二选一）

两套方法调用的是同一个 `apriltag_pick_return.launch`、同一个飞前预检、同一组配置文件以及同一组
`takeoff.sh / pub_trigger.sh / takeland.sh`。新方法只负责编排进程和增加操作门禁，没有复制第二套
控制算法。

**每次实验只能选一套。** 一键脚本检测到已有 ROS Master 会直接拒绝启动，防止与原来的
7 终端方法重复启动 MAVROS、planner、controller 或 mission。

### 6.1 新方法：单终端一键编排（推荐）

脚本：
[`sh_utils/apriltag_rtk_easy_flight.sh`](sh_utils/apriltag_rtk_easy_flight.sh)

它的 `start` 会一次完成：

1. 启动独立 roscore 和唯一 MAVROS；
2. 配置 MAVLink/RTK/EKF 数据频率；
3. 启动 D435i、AprilTag、planner、`odomRemap`、`ctrl_node` 和 mission；
4. 运行与第 7 节完全相同的 5 s 连续飞前预检；
5. 预先监听一次性 READY，不会错过起飞完成握手；
6. 预检通过后启动 `record.sh`。

`start` **不会切换 OFFBOARD、不会解锁、不会起飞、不会发布轨迹**。

首次装桨分级验证使用 dry-run：

```bash
cd ~/HAM_Flight
bash sh_utils/apriltag_rtk_easy_flight.sh start --dry-run
```

只有 dry-run、拆桨真实舵机、人工接管和降落都通过后，才启动真实夹爪：

```bash
cd ~/HAM_Flight
bash sh_utils/apriltag_rtk_easy_flight.sh start --live
```

需要同时录制压缩彩色图像时，在上述命令末尾加 `--record-images`。该选项会增加 CPU 和磁盘负载，
只用于识别漏检诊断。

看到脚本最后的“一键地面启动完成”后，先用 `prepare` 完成原方法中不可省略的实码可见性检查。
`prepare` 会要求飞手先把码放到相机下方，连续收到 3 条有效位置后，再要求把码和物品放回航线；
随后它会在飞机的最终起飞位置再做一次 5 s 完整预检，只有原始 PX4 local pose、
`/odom/remap`、RTK 和 EKF 都持续稳定，才会张开夹爪并启用单次任务。

在同一终端按以下步骤执行：

```bash
# 随时可查看当前状态
bash sh_utils/apriltag_rtk_easy_flight.sh status

# 1. 未解锁/非 OFFBOARD 时进行实码检查，将码移回航线，张爪并启用任务
bash sh_utils/apriltag_rtk_easy_flight.sh prepare

# 2. 飞手先用遥控器切入 OFFBOARD，然后发布一次起飞
bash sh_utils/apriltag_rtk_easy_flight.sh takeoff

# 3. 脚本确认 1.0 m 稳定和带 ID 的 READY 后，发布一次轨迹
bash sh_utils/apriltag_rtk_easy_flight.sh mission

# 4. 看到 HOLD_HOME 且现场确认返回原点后，人工降落
bash sh_utils/apriltag_rtk_easy_flight.sh land

# 5. 确认落地、下锁、非 OFFBOARD 后才停止节点并完成 bag 索引
bash sh_utils/apriltag_rtk_easy_flight.sh stop
```

`takeoff` 会再检查 `prepare` 已通过、MAVROS、PX4 STANDBY、ON_GROUND、RTK Fixed、卫星数、EKF、任务状态和 rosbag；
`mission` 会检查带 ID 的 READY、已解锁 OFFBOARD、0.85～1.15 m 高度、低速悬停和 rosbag。两个命令
都在状态目录中记录“已发布”，同一次实验拒绝重复 TAKEOFF 和 MISSION。
如果飞机未到 1.0 m 并稳定，控制器不会发 READY；起飞超时后会请求 PX4
`AUTO.LAND`，不再把当时的贴地高度锁成悬停点。

如果飞行稳定但只想取消自动抓取：

```bash
bash sh_utils/apriltag_rtk_easy_flight.sh abort
```

脚本日志和 bag 位置：

```bash
bash sh_utils/apriltag_rtk_easy_flight.sh logs
```

- 启动日志：`~/HAM_Flight/easy_flight_logs/<时间>/`；
- rosbag：`~/HAM_Flight/sh_utils/Bag/`；
- `stop` 先向 rosbag 发 `SIGINT` 并等待写完索引，再按 mission、MAVROS、roscore 逆序关闭；
- `stop --force` 只能用于飞行器已断电后清理残留进程，严禁在空中使用。

如果希望一个命令进入全程交互向导，可使用：

```bash
cd ~/HAM_Flight
bash sh_utils/apriltag_rtk_easy_flight.sh run --dry-run
# 或者，完成所有分级验证后：
bash sh_utils/apriltag_rtk_easy_flight.sh run --live
```

`run` 仍要求飞手依次手动输入大写 `TAKEOFF`、`MISSION`、`LAND`；它不能操作遥控器，也不会跳过任何门禁。

### 6.2 原方法：7 终端手动启动（完整保留）

以下原始操作步骤仍然有效，可以用于逐窗口调试和人工检查每个节点。选择本方法时不得运行
`apriltag_rtk_easy_flight.sh start/run`。

全程保持：未解锁、非 OFFBOARD、飞行器放在最终起飞点且静止，标签先不要放进相机视野。

本任务固定使用 **7 个终端窗口或标签页**。不要在运行中的 `roscore`、`roslaunch`、
`rostopic echo` 或 `rosbag record` 窗口里继续输入其他命令。

| 终端 | 固定用途 | 是否持续占用 |
|---|---|---|
| 1 | ROS Master | 是 |
| 2 | 唯一 MAVROS | 是 |
| 3 | 唯一 AprilTag 任务组合 launch | 是 |
| 4 | 监听一次性控制器 READY | 是 |
| 5 | 监视任务状态 | 是 |
| 6 | 操作终端：频率配置、预检、夹爪、起飞、轨迹触发、备用 LAND | 否，可连续输入命令 |
| 7 | rosbag 录包 | 是，落地下锁后才按 `Ctrl+C` |

每个新终端首先运行：

```bash
source /opt/ros/noetic/setup.bash
source ~/HAM_Flight/devel/setup.bash
```

#### 终端 1：ROS Master

```bash
source /opt/ros/noetic/setup.bash
source ~/HAM_Flight/devel/setup.bash
roscore
```

看到 `started core service [/rosout]` 后保持窗口运行。若已经有确认属于本次任务的 ROS
Master，可复用；不得同时运行第二个 roscore。

#### 终端 2：唯一 MAVROS

```bash
source /opt/ros/noetic/setup.bash
source ~/HAM_Flight/devel/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
```

保持窗口运行。不要在这个终端继续输入频率配置命令，也不要让后续组合 launch 再启动
MAVROS。

#### 终端 6：确认 MAVROS 并配置数据频率

终端 2 启动后，在终端 6 运行：

```bash
source /opt/ros/noetic/setup.bash
source ~/HAM_Flight/devel/setup.bash

rostopic echo -n 1 /mavros/state

cd ~/HAM_Flight
ENABLE_NAV_STATUS_RATES=1 \
ID31=10000 ID32=10000 ID331=20000 \
bash sh_utils/mavlink_set_rates.sh
```

第一条命令必须显示 `connected: True`。频率脚本必须成功结束；失败时检查 Pixhawk 串口、
波特率和 MAVROS，不能继续。这里的 MAVROS 只启动一次。

#### 终端 3：唯一 AprilTag 任务链

第一次有桨验证先使用 `dry_run:=true`：

```bash
source /opt/ros/noetic/setup.bash
source ~/HAM_Flight/devel/setup.bash
roslaunch vision_gripper apriltag_pick_return.launch \
  start_mavros:=false \
  start_realsense:=true \
  start_trajectory:=true \
  start_odom_remap:=true \
  start_controller:=true \
  dry_run:=true \
  navigation_monitor_enabled:=true \
  mission_enabled_on_start:=false
```

只有 dry-run、人工接管和降落均验证通过后，才能完全关闭上述 launch，再重新启动真实夹爪：

```bash
roslaunch vision_gripper apriltag_pick_return.launch \
  start_mavros:=false \
  start_realsense:=true \
  start_trajectory:=true \
  start_odom_remap:=true \
  start_controller:=true \
  dry_run:=false \
  navigation_monitor_enabled:=true \
  mission_enabled_on_start:=false
```

不要先启动 `traj_gen.launch`、`odom_remap.launch` 或 `run_ctrl.launch`，再运行这个组合
launch；组合 launch 已经负责这三个组件。

保持终端 3 持续运行。若出现 nodelet 加载失败、重复节点、相机启动失败或 `FATAL`，立即
停止，不要继续预检。

#### 终端 4：提前监听控制器 READY

READY 是一次性消息，必须在起飞前开始监听：

```bash
source /opt/ros/noetic/setup.bash
source ~/HAM_Flight/devel/setup.bash
rostopic echo /traj_start_trigger
```

起飞前这个终端没有输出是正常的。保持运行，不要关闭，也不要在这里输入其他命令。

#### 终端 5：持续监视任务状态

```bash
source /opt/ros/noetic/setup.bash
source ~/HAM_Flight/devel/setup.bash
rostopic echo /apriltag_mission/state
```

正常初始状态应为：

```text
data: "FOLLOW_PRESET"
```

需要诊断时，可以暂时停止本命令，再查看具体状态；飞行前应恢复任务状态监视：

```bash
rostopic echo -n 1 /apriltag_mission/status
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/extended_state
rostopic echo /apriltag_mission/state
```

#### 终端 6：保持为空，等待执行预检与飞行操作

终端 6 是唯一操作终端。后续第 7～9 节的所有一次性命令都只在这里执行。不要另开旧
RTK 脚本，也不要在终端 1～5 中混入操作命令。

#### 终端 7：暂不启动录包

先保留为空。只有终端 6 的自动预检出现 `[READY]` 和 `[SAFE]` 后，才按第 8 节启动
`record.sh`。录包命令会一直占用终端 7。

#### 启动阶段完成时应看到

```text
终端 1：roscore 正常运行
终端 2：MAVROS 正常运行，connected=True
终端 3：相机、检测器、planner、odomRemap、ctrl_node、mission 均已启动
终端 4：等待 READY，目前可无输出
终端 5：FOLLOW_PRESET
终端 6：可继续输入命令
终端 7：尚未录包
```

## 7. 自动 RTK/控制链预检

本节同时是两套方法的预检标准。使用 6.1 一键方法时，`start` 已自动运行本节程序；不要在
`prepare` 启用任务后重复手动运行，因为预检会正确拒绝“mission 已 enabled”的状态。使用 6.2
7 终端方法时，按下述命令手动执行。

飞机保持地面静止至少 5 秒，在 **终端 6** 运行：

```bash
source /opt/ros/noetic/setup.bash
source ~/HAM_Flight/devel/setup.bash
rosrun vision_gripper apriltag_rtk_preflight.py
```

该程序只读，不解锁、不切换模式、不移动夹爪、不发布飞行指令。它会连续检查：

- MAVROS 已连接，飞机未解锁、非 OFFBOARD、PX4 报告 ON_GROUND；
- PX4 `system_status=3 (STANDBY)`；`0 (UNINIT)` 或 `8 (FLIGHT_TERMINATION)` 必须清除终止/急停条件并重启飞控，重复发布 TAKEOFF 无效；
- RTK 在整个采样窗口持续 `fix_type >= 6`，可见卫星不少于 8；
- PX4 EKF 水平/垂直绝对位置持续有效，无 constant-position 和 GPS glitch；
- `/mavros/local_position/odom` 频率不低于 30 Hz，无 NaN/Inf，地面静止 5 s 的水平/高度跳动不过限；
- `/odom/remap` 频率不低于 30 Hz，无 NaN/Inf，地面静止漂移和速度不过限，且最终起飞点零点接近 `(0,0,0)`；
- IMU 不低于 50 Hz，D435i 彩色图像不低于 10 Hz，camera_info 存在；
- PX4 `COM_OF_LOSS_T <= 1 s`、`COM_OBL_RC_ACT=4 (Land)`；
- `PWM_MAIN_FUNC1=301`；
- `navigation_monitor/enabled=true`；
- 运行时 1.0 m 起飞/巡航、轨迹、45 cm、回升、返航和人工 LAND 参数正确；
- planner、arbiter、controller 的节点、nodelet 和发布者拓扑完全唯一；
- 任务仍为 `FOLLOW_PRESET` 且接管明确关闭。

只有最后出现以下两行才允许继续：

```text
[READY] RTK、EKF、里程计、相机、规划器、仲裁器、控制器及 PX4 保护全部通过。
[SAFE] 当前仍未解锁、非 OFFBOARD、任务接管关闭。
```

出现任意 `[FAIL]` 或 `[BLOCKED]` 必须停止，不得通过修改阈值、伪造话题或关闭
`navigation_monitor` 绕过。

`/mavros/local_position/pose` 是 PX4 自己的局部 ENU 坐标，它的 x/y/z **不要求为 0**，换电、重启、
EKF 重置或在原点建立后搬动飞机都可能让绝对数值不同。本项目的控制器读取的是
`/odom/remap`，它在最终起飞点应接近零。判断故障的关键是：飞机静止时原始 local pose 是否
连续、`/odom/remap` 是否连续且近零，而不是原始 x/y/z 的数字是否刚好为零。

`satellites_visible=32` 也只是卫星数，不等于垂直位置必然稳定。必须同时满足整个 5 s 窗口内
`fix_type>=6`、EKF 水平/垂直有效、无 glitch，以及上述两路 odom 的连续性门禁。

## 8. 地面任务准备

使用 6.1 新方法时，下面的实码可见性、reset、张爪和 enable 由
`bash sh_utils/apriltag_rtk_easy_flight.sh prepare` 交互执行；不要再手动调用同样的服务。
使用 6.2 原 7 终端方法时，按本节原命令执行。

预检通过后，仍保持未解锁、非 OFFBOARD。先做一次标签可见性检查：在飞行器确认下锁、
电机不会转动时，临时把打印纸水平放到彩色相机正下方约 `0.7～1.0 m`，保证完整黑边进入
画面，然后在 **终端 6** 运行：

```bash
timeout 8 rostopic echo /apriltag_gripper/tag_position
```

必须连续输出 ID 0 对应的位置（`z` 应接近实际相机到标签距离）。如果完全无输出，禁止起飞：
确认打印码是 `tag36h11 / ID 0`、黑色外边完整、纸面无强反光，并用
`rqt_image_view /camera/color/image_raw` 确认标签确实在彩色相机画面内。检查后把标签移回航线，
并清掉地面检测滤波状态：

```bash
rosservice call /apriltag_gripper/reset
```

使用 6.2 原 7 终端方法时，在标签移回航线、飞机仍保持最终起飞点且 mission 仍 disabled 时，
再运行一次：

```bash
rosrun vision_gripper apriltag_rtk_preflight.py
```

这是临近张爪/启用任务的最终 5 s 定位连续性门禁；必须再次看到 `[READY]` 和 `[SAFE]`。
6.1 一键方法已由 `prepare` 自动完成这一步，不要额外手动重复。

注意D435i彩色镜头垂直向下：标签放在“机头正前方”的地面上时，静止在起飞点本来就看不到；
只有无人机沿航线接近到相机地面视场内才会出现。随后在 **终端 6** 运行：

```bash
# 张开夹爪
rosservice call /apriltag_gripper/set_gripper "data: false"

# 启用单次 AprilTag 接管；此操作本身不会起飞
rosservice call /apriltag_mission/enable "data: true"
```

确认两次服务都返回 `success: True`。把 AprilTag 放在机头前方 3 m 航线中、相机能在
途中看到的位置，并按第 3 节固定物品与标签的相对位置。起飞点不要直接看到标签，否则
预设轨迹刚开始就可能立即接管。

然后在 **终端 7** 开始录包：

```bash
source /opt/ros/noetic/setup.bash
source ~/HAM_Flight/devel/setup.bash
cd ~/HAM_Flight
bash sh_utils/record.sh
```

看到 rosbag 开始订阅话题后保持终端 7 运行。此时 7 个终端的用途已经全部固定。
默认包内包含 RTK/EKF、实际与目标姿态、odom、轨迹命令、原始 Tag 检测、滤波位置、任务
状态和舵机命令/接受结果。默认不录图像以免增加延迟；仅诊断识别漏检时，在起飞前确认
CPU、磁盘和识别频率有余量后改用：

```bash
RECORD_CAMERA_IMAGES=1 bash sh_utils/record.sh
```

## 9. 起飞与轨迹触发

使用 6.1 新方法时，飞手仍需先用遥控器切换 OFFBOARD，但只运行
`apriltag_rtk_easy_flight.sh takeoff` 和 `apriltag_rtk_easy_flight.sh mission`；不要再另外手动执行本节的
`takeoff.sh` 或 `pub_trigger.sh`，否则等于绕过新脚本的单次发布记录。使用 6.2 原方法时，以下命令保持有效。

### 9.1 使用遥控器切入 OFFBOARD

只能由现场驾驶员使用外部遥控器切入 OFFBOARD。随后在 **终端 6** 运行：

```bash
rostopic echo -n 1 /mavros/state
```

必须确认：

```text
/mavros/state.connected = true
/mavros/state.armed = false
/mavros/state.mode = OFFBOARD
/mavros/state.system_status = 3
```

终端 2 的 MAVROS 日志和终端 5 的任务状态不能有异常。任何一项不符都先退出 OFFBOARD，
不得发布起飞命令。

### 9.2 在终端 6 发布一次起飞命令

```bash
cd ~/HAM_Flight
bash sh_utils/takeoff.sh
```

控制器现在还有第二道硬门禁：如果 RTK/GPS、EKF 状态不健康/不新鲜，或者PX4不是
`STANDBY(3)`，它会在请求 ARM 之前拒绝 TAKEOFF，不会采用“先解锁、再降落”的策略。
被拒绝后必须退出 OFFBOARD、保持未解锁，修复故障并从第 7 节重新预检。

发布 TAKEOFF 的这一刻会同时锁定“初始机头方向”。此后即使悬停稳定期间有少量偏航，
直线轨迹、视觉跟踪和返航航向仍使用这个初始方向。因此发布前必须把机头对准实际航线，
飞控箭头朝机头，并保持飞机静止。

正常情况下，控制器请求解锁，以 `0.25 m/s` 上升到原点上方 `1.0 m`。等待飞机稳定，
**终端 4** 必须看到：

```text
header.frame_id: "world:<非零ID>"
```

没有带非零 ID 的 READY，禁止发布轨迹。

### 9.3 在终端 6 只触发一次轨迹

```bash
cd ~/HAM_Flight
bash sh_utils/pub_trigger.sh
```

该脚本会检查 `/landing_trigger` 同时连接规划器和控制器。重复触发会被规划器拒绝，但仍
禁止人工连续执行多次。

执行后不要再在终端 6 输入其他飞行命令，除非需要按故障流程执行 `takeland.sh`。

## 10. 飞行中会怎样执行

成功状态顺序：

```text
FOLLOW_PRESET
  -> TRACK_TAG
  -> CLOSING
  -> ASCEND
  -> RETURN_HOME
  -> HOLD_HOME
  -> 飞手发布 takeland.sh
  -> LANDING / LANDED
```

- `FOLLOW_PRESET`：原点悬停 3 s，随后保持 1.0 m，沿 TAKEOFF 时锁定的初始机头方向前飞；
- `TRACK_TAG`：连续识别到 3 帧标签后，仲裁器接管三轴位置参考；
- `CLOSING`：夹爪中心对准且相机光轴距离达到 `0.45 ± 0.03 m` 后闭爪一次；
- `ASCEND`：以 0.10 m/s 恢复到接管前保存的 1.0 m 巡航高度；
- `RETURN_HOME`：以 0.18 m/s 沿水平直线返回轨迹原点；
- `HOLD_HOME`：原点稳定 2 s 后持续发布原点 1.0 m 悬停参考，`mission_complete=true`；
- `LANDING/LANDED`：飞手检查返航位置后才在终端 6 发布 `takeland.sh`。
  PX4 确认 `AUTO.LAND` 后，mission 转入 `LANDING` 并停止位置指令，不会把这次有意退出 OFFBOARD 误报为故障。

任务成功必须同时看到：

```text
/apriltag_mission/state = HOLD_HOME
/apriltag_mission/mission_complete = true
/apriltag_mission/pickup_succeeded = true
/mavros/state.armed = true
```

这只表示已返回原点，飞机仍在空中且已解锁，严禁靠近。`mission_complete=true` 也可能表示“丢码后未闭爪、
安全返航完成”；只有 `pickup_succeeded=true` 才表示软件已接受闭爪命令并走抓取返航分支。
它仍不等于有传感器确认物品已经夹住。

终端 5 看到 `HOLD_HOME` 后，在 **终端 6** 先确认：

```bash
rostopic echo -n 1 /apriltag_mission/mission_complete
rostopic echo -n 1 /apriltag_mission/pickup_succeeded
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /apriltag_mission/state
```

必须看到 `mission_complete=true`、`pickup_succeeded=true`、`state=HOLD_HOME`，并从现场确认飞机在原点上方稳定。
然后才发布一次：

```bash
bash ~/HAM_Flight/sh_utils/takeland.sh
```

落地后再确认 `armed=false` 且 `landed_state=1`。

如果全程没有发现标签，预设轨迹会在 3 m 端点悬停 2 s，再用 27 s 原路返回原点；此时
状态仍为 `FOLLOW_PRESET`，不会伪装成抓取成功，也不会自动 LAND。确认已回到原点并稳定后：

```bash
bash ~/HAM_Flight/sh_utils/takeland.sh
```

## 11. Dry-run 与真实夹爪的差异

`dry_run:=true` 到达抓取条件后只模拟舵机输出，并故意进入 `FAILSAFE_HOLD`；它不会继续
回升或返航。驾驶员应确认飞行稳定后执行人工 LAND。这是防止把“虚拟抓取成功”
误认为已经携带物体。

只有 dry-run 完整通过、rosbag 无异常、遥控器接管有效后，才能使用 `dry_run:=false`。

## 12. RTK/EKF 运行中保护

起飞后 `ctrl_node` 持续订阅：

```text
/mavros/gpsstatus/gps1/raw
/mavros/estimator_status
/odom/remap
/mavros/imu/data
/mavros/state
```

RTK/EKF 异常超过宽限时间时：

1. `TAKEOFF` 或 `MISSION` 先退出到当前位置 HOVER；
2. 异常持续则进入 LAND；
3. 控制器请求 PX4 `AUTO.LAND`；
4. 若上位机 setpoint 消失，PX4 还会按 `COM_OF_LOSS_T` 和 `COM_OBL_RC_ACT=4` 独立降落。

这些保护不能纠正已经发散的错误坐标。驾驶员看到明显漂移、姿态异常或路线错误时，应
立即用遥控器退出 OFFBOARD并接管，不能等待自动保护。

## 13. 丢码与防摇晃策略

短暂漏检不是任务失败：漏检超过 `0.25 s` 后跟踪速度按加速度限制降到零；连续丢码
`1.0 s` 进入 `REACQUIRE_TAG`，先保持 `0.40 s`，随后最多以 `0.05 m/s` 上升 `0.15 m`
扩大视场，并在总计 `6 s` 的恢复窗口内等待。期间连续恢复 5 帧就继续跟踪；
仍未恢复则不闭爪，进入 `ABORT_ASCEND`，先上升到 1.0 m，再返航并在原点等待人工降落。

跟踪不再直接把相机光学 x/y 当成机体水平误差。程序现在使用相机—夹爪完整平移、飞机
实时 roll/pitch/yaw、锁定的起飞初始航向，把标签位置转换到重力水平世界系；同时采用
3 帧带采样时间戳的中值滤波、0.10 m/s² 速度变化限制、横向/垂向 0.10/0.06 m 参考
前置限制，远处横向/垂向速度分别限制为 0.12/0.07 m/s，接近目标后降为 0.05/0.03 m/s。下降速度
在水平误差 0.07 m 内为全速，在 0.07～0.16 m 间连续衰减，超过 0.16 m 停止下降，
避免旧二值门限造成反复停走。

高度误差进入 `±0.03 m` 抓取容差带后，垂向指令被强制为零，不再追逐深度噪声上下往复。
闭爪前必须连续 5 帧对准，且水平速度不超过 0.10 m/s、垂向速度不超过 0.08 m/s、
任务指令速度不超过 0.04 m/s；满足后只调用一次真实舵机服务。

这些修改消除了原代码中“机身倾斜—图像偏移—反向追逐”的软件耦合，但不能修复真实
重心偏移、机架松动、桨/电机振动、姿态估计噪声或控制器参数不匹配。低空分级试飞中若
出现幅度逐次增大的左右摆动、快速反复横滚指令或明显偏航，应立即退出 OFFBOARD 接管；
不要靠增大视觉速度来压制摆动。落地后用 bag 对照 `/mavros/imu/data`、
`/mavros/setpoint_raw/target_attitude`、`/debugPx4ctrl`、`/odom/remap`、原始 Tag 检测与
`/apriltag_mission/alignment_error`：目标姿态先振荡是上游参考/视觉问题，目标平稳而实际
姿态振荡才优先排查重心、振动和姿态控制器。

## 14. 中止与故障处理

### 飞行稳定、定位正常，只取消自动抓取

```bash
rosservice call /apriltag_mission/abort
```

它会冻结当前位置参考。随后由驾驶员决定继续人工接管或 LAND。

### 已返回原点（正常就不会自动降落）

```bash
bash ~/HAM_Flight/sh_utils/takeland.sh
```

### 任意空中异常

第一动作始终是遥控器退出 OFFBOARD并按现场预案接管/降落。不要在空中按 `Ctrl+C`、拔掉
USB、关闭 MAVROS、关闭相机或重启节点。`Ctrl+C` 不是降落命令。

### 落地后复位任务

确认 `armed=false`、`landed_state=1`、非 OFFBOARD 后：

```bash
rosservice call /apriltag_mission/reset
```

## 15. 推荐验收阶梯

必须逐级完成，不能第一次装桨就执行完整任务：

1. 拆桨运行本文第 6～7 节，确保预检稳定重复通过；
2. 人工 POSCTL 低空悬停、移动、降落；
3. OFFBOARD 只起飞到 1.0 m、原点悬停、人工 LAND，不触发轨迹；
4. 任务接管保持关闭，只验证 3 m 前飞、原路返回、人工 LAND；
5. `dry_run=true` 验证途中发现、对准 45 cm、进入安全保持、人工 LAND；
6. `dry_run=false` 在起点附近做短距离真实夹取、回升、返航、原点悬停、人工 LAND；
7. 最后执行完整的“前飞途中发现—抓取—返航悬停—人工降落”。

每一级都检查 rosbag 中的 RTK、EKF、odom、轨迹参考、控制输出、标签位置和状态切换，
确认没有漂移、振荡、跳变、重复发布者或状态异常后再进入下一级。

## 16. 正常落地后的关闭顺序

使用 6.1 一键方法时，确认落地下锁后只执行：

```bash
bash ~/HAM_Flight/sh_utils/apriltag_rtk_easy_flight.sh stop
```

它内部按下述同样的逆序关闭，并保证 rosbag 先收到 SIGINT 完成索引。使用 6.2 原 7 终端方法时，
手动按以下顺序关闭。

只有终端 6 已确认 `mission_complete=true`、`armed=false`、`landed_state=1` 后，才按以下
顺序关闭：

1. **终端 7**：按一次 `Ctrl+C` 停止 rosbag，等待终端返回提示符，确保 bag 索引写完；
2. **终端 5**：按 `Ctrl+C` 停止任务状态监视；
3. **终端 4**：按 `Ctrl+C` 停止 READY 监听；
4. **终端 3**：按 `Ctrl+C` 正常关闭相机、检测器、planner、odomRemap、ctrl_node 和 mission；
5. **终端 2**：按 `Ctrl+C` 关闭 MAVROS；
6. **终端 1**：最后按 `Ctrl+C` 关闭 roscore；
7. 确认桨叶完全停止后，按现场规程断开飞行器电源。

终端 3～1 必须按上述逆序关闭。空中或仍解锁时严禁执行这套关闭操作。

## 17. 起飞不到 1 m 或轨迹无响应时

两者通常是同一条状态链的结果：只有起飞到目标高度、垂直速度进入门限并稳定保持后，
`ctrl_node` 才会发带任务 ID 的 READY。没有 READY 时，`mission`/planner 拒绝轨迹是正确的安全行为，
不应重复发布 `pub_trigger.sh` 强行绕过。

当场处理：

1. 立即由飞手退出 OFFBOARD 并接管；不得在贴地上下波动时继续触发轨迹。
2. 落地下锁后用 `stop` 正常完成 bag 索引，不要直接关终端或断电破坏包。
3. 下一次必须从 `start` 和 `prepare` 重新开始；最终 5 s 预检有任意 `[FAIL]` 都不起飞。
4. 如果起飞超时，新版控制器请求 `AUTO.LAND`；它不会再低空 HOVER，也不会发 READY。

无 bag 时无法区分当次究竟是 PX4 高度估计跳变、推力/质量模型不匹配、OFFBOARD/ARM 状态变化、
原始 odom 不连续，还是其他机械与飞控问题。一键方法现在把 rosbag 存活作为 `takeoff` 和 `mission`
的强制门禁；包内已包含 `/mavros/local_position/odom`、`/odom/remap`、RTK/EKF、目标/实际姿态、
控制调试、任务状态和舵机命令，足以对下一次进行定量定位。

2026-11-14 最新 `04-41` 真机包证明：飞机实际到达约 `1.20 m`，但在高度已下降时，
同步 `velocity_local.z` 仍持续报告约 `+0.2～+0.4 m/s`，相对高度差分约有 `+0.232 m/s` 偏置，
使 TAKEOFF 无法满足 `|vz|≤0.15 m/s` 的稳定门限，约 10 s 后正确触发超时 `AUTO.LAND`。
同一包中水平同步速度仍与位置差分基本一致。因此当前使用混合反馈：
水平 x/y 使用时间同步并经初始姿态旋转的 `velocity_local`，垂直 z 使用同一条重映射高度差分的低通速度。
按本包离线回算，新 z 速度能在超时前连续满足高度/速度稳定判据。

下一次不先做完整抓取，只做“1 m 起飞—原点悬停—人工降落”的分级验证：

```bash
cd ~/HAM_Flight
bash sh_utils/apriltag_rtk_easy_flight.sh start --dry-run
bash sh_utils/apriltag_rtk_easy_flight.sh prepare
# 飞手用遥控器切到 OFFBOARD
bash sh_utils/apriltag_rtk_easy_flight.sh takeoff
# 到 1 m 稳定后不执行 mission，现场观察后降落
bash sh_utils/apriltag_rtk_easy_flight.sh land
# 落地、下锁、退出 OFFBOARD 后
bash sh_utils/apriltag_rtk_easy_flight.sh stop
```

如果 `prepare` 报定位连续性失败，不起飞。如果 `takeoff` 没有返回 READY 或飞机再次上下波动，
飞手立即接管，不执行 `mission`；等安全落地下锁后用 `stop` 完成 bag 索引，然后优先分析该包。
