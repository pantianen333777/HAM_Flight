# D435i 视觉夹爪

该包提供两种目标识别方式：黑色胶带圆环和 AprilTag。两种方式都通过 MAVROS 的
`MAV_CMD_DO_SET_ACTUATOR` 控制 PX4 的夹爪舵机。AprilTag 模式还可以发布受限的
BODY_NED 速度设定值，使无人机移动到标签上方 0.45 m 并对准夹爪后自动闭合。

## 本机已经确认的硬件状态

- D435i 已连接并被 USB 识别；
- PX4/MAVROS 串口为 `/dev/ttyACM0:57600`；
- 夹爪输出为 `MAIN1`；
- `PWM_MAIN_FUNC1=301`，即 `Offboard Actuator Set 1`；
- 当前实机舵机值为 `+0.85` 张开、`-0.60` 闭合；
- 飞控报告的版本为 PX4 1.15.x。

本机 Ubuntu 的旧版 `uvcdynctrl` 会在 D435i 重连时异常写入巨量日志。仓库内的
`config/80-uvcdynctrl.rules` 是同名空规则覆盖，用来关闭这个已经过时的 udev
辅助程序；它不影响 librealsense。非常老、依赖 vendor-extension XML 的普通 UVC
摄像头可能不再有额外控制项。系统副本安装在
`/etc/udev/rules.d/80-uvcdynctrl.rules`。

## 启动与标定

务必先拆下螺旋桨。首次启动保持干运行：

```bash
source /opt/ros/noetic/setup.bash
source ~/HAM_Flight/devel/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
roslaunch vision_gripper vision_gripper.launch dry_run:=true
```

在另一个终端查看检测图：

```bash
rqt_image_view /vision_gripper/debug_image
```

当前配置已针对夹爪在 `+0.85` 张开状态下的黑胶带卷完成 50 帧标定：圆心约
`(853, 604) px`，表面深度约 `0.233 m`。由于胶带卷中空，
程序从可见上半圆环取深度的第 30 百分位，避免把孔内地板距离当成目标距离。
调试图会显示圆、抓取框、像素误差、深度有效率以及相机光学坐标系下的 XYZ。

深度与三维信息也分别发布到：

```text
/vision_gripper/target_depth_m      std_msgs/Float32（米）
/vision_gripper/depth_valid_ratio   std_msgs/Float32
/vision_gripper/target              geometry_msgs/PointStamped（米，光学坐标系）
/vision_gripper/alignment_error_m   geometry_msgs/PointStamped（目标减标定抓取点）
```

若改用实心彩色物体，可将 `detector_mode` 改为 `hsv_color` 并调整 HSV 阈值。

干运行下测试开合命令（`true` 为夹紧，`false` 为松开）：

```bash
rosservice call /vision_gripper/set_gripper "data: true"
rosservice call /vision_gripper/set_gripper "data: false"
```

确认识别区域、MAIN1 接线和舵机方向后，拆桨状态下启用真实输出：

```bash
roslaunch vision_gripper vision_gripper.launch dry_run:=false
```

当前实机已确认 MAIN1 舵机方向：`open_value=0.85` 为松开，
`close_value=-0.60` 为夹紧。确认机械行程没有顶死之后才开启视觉自动夹紧：

```bash
rosservice call /vision_gripper/enable_auto_grab "data: true"
```

自动夹紧只触发一次。松开夹爪或调用以下服务可重新允许触发：

```bash
rosservice call /vision_gripper/reset_auto_grab
```

## 安全逻辑

- 默认 `dry_run=true`、自动抓取关闭；
- 每次真实输出前检查 MAVROS 已连接；
- 每次真实输出前检查 `PWM_MAIN_FUNC1` 仍为 `301`；
- 目标必须连续多帧位于抓取区域且深度有效；
- 胶带环面深度使用 7 帧中值滤波，抑制偶发的近点和远点跳变；
- 自动动作带单次锁存与命令冷却；
- 黑胶带节点不会写飞控参数，也不会解锁、起飞或发送飞行控制指令。

## AprilTag 45 cm 自动抓取

当前标定使用 `tag36h11`、ID `0`。2026-11-10 更换打印标签后，使用 120 组同步
AprilTag 位姿和 D435i 对齐深度重新标定：对齐深度中位数 `0.916 m`，旧尺寸位姿 z
中位数 `0.462344 m`，得到标签有效边长 `0.08746 m`。改变打印比例或换标签后，需要
重新运行 `calibrate_apriltag_depth.py` 并同步修改 `config/apriltag_tags.yaml` 中的 `size`。

实测相机位于夹爪前方 `0.058 m`、夹爪闭合中心低于相机 `0.110 m`，左右偏移为零。
待抓物放在标签上方 `0.340 m`、朝机头方向前方 `0.030 m`，因此不会遮挡标签；正确
抓取姿态下标签在相机光学坐标系的名义位置为 `(x=0, y=0.088, z=0.45) m`。任务节点
使用 `/odom/remap` 完整四元数把测量转换到世界系，补偿实时滚转、俯仰和偏航后再控制。
`0.340 m` 是夹爪接触中心到标签平面的高度；若真实物品夹持中心高度为 `h`，应把目标
距离改为 `0.110 + h`，不能继续套用 0.45 m。本任务按用户要求不包含最终缓降段。

先以干运行启动：

```bash
source /opt/ros/noetic/setup.bash
source ~/HAM_Flight/devel/setup.bash
roslaunch vision_gripper apriltag_gripper.launch start_mavros:=true dry_run:=true
```

主要状态话题：

```text
/apriltag_gripper/tag_position               标签在相机光学坐标系中的 XYZ
/apriltag_gripper/distance_m                 相机光轴方向距离
/apriltag_gripper/slant_range_m              相机到标签的斜距
/apriltag_gripper/gripper_alignment_error    标签相对夹爪投影的 XYZ 误差
/apriltag_gripper/desired_velocity_body_ned  建议速度：前、右、下
/apriltag_gripper/at_target                  距离和水平对齐均满足要求
/apriltag_gripper/status                     当前状态
/apriltag_gripper/command_action             最近请求的 open/close
/apriltag_gripper/commanded_value            最近请求的舵机归一化值
/apriltag_gripper/command_accepted           PX4 是否接受最近一次真实命令
```

自动闭合和飞行跟踪默认都关闭。夹爪真实输出需要用 `dry_run:=false` 重启，再单独启用：

```bash
rosservice call /apriltag_gripper/enable_auto_close "data: true"
```

独立调试节点使用 `/mavros/setpoint_raw/local` 的 BODY_NED 速度，最大水平速度
`0.15 m/s`、垂直速度 `0.10 m/s`。正式组合任务不使用该直接速度输出，而由任务仲裁器
发布世界系 `PositionCommand`。节点不会解锁，也不会切换飞行模式。飞控已经进入
`OFFBOARD` 后，以下服务才会允许发布非零速度：

```bash
rosservice call /apriltag_gripper/enable_flight "data: true"
```

距离目标为 `0.45 ± 0.03 m`，水平对齐误差不超过 `0.03 m`，连续 5 次检测满足条件
后只闭合一次。标签丢失超过 0.25 秒时，速度设定自动归零。复位单次触发锁存：

```bash
rosservice call /apriltag_gripper/reset
```

当前低延迟配置使用 4 个检测线程、原分辨率检测（`decimate=1`）、3 帧带采样时间戳的
中值滤波，并关闭检测图发布。若修改启动文件后终端提示找不到 launch 文件，重新加载
工作区并刷新包缓存：

```bash
source /opt/ros/noetic/setup.bash
source ~/HAM_Flight/devel/setup.bash
rospack profile
```

## 预设轨迹中发现标签、抓取并返航

`apriltag_trajectory_mission_node.py` 是 `PositionCommand` 仲裁器。正常阶段把规划器的
`/drone0/planning/preset_cmd` 原样转发到控制器的 `/drone0/planning/cmd`；连续识别到
标签后停止透传并接管同一个轨迹 ID，依次执行：

1. 轨迹开始时锁定起飞悬停航向，后续跟踪、上升和返航始终使用该航向；
2. 用完整 roll/pitch/yaw 和实测相机—夹爪外参计算世界系误差，先水平对齐再下降；
3. 相机距离达到 `0.45 ± 0.03 m`、世界系水平误差不超过 `0.03 m` 后只闭爪一次；
4. 闭爪确认后等待 `1.0 s`，再以 `0.12 m/s` 恢复 `1.5 m` 巡航世界高度；
5. 以 `0.25 m/s` 沿水平直线返回该次预设轨迹的起点 x/y；
6. 原点稳定 `2.0 s` 后请求 `ctrl_node` 进入 PX4 `AUTO.LAND`，确认落地并下锁后才报告完成。

轨迹生成器、仲裁器和控制器必须按下面的话题链连接，禁止再有第二个节点直接发布
`/drone0/planning/cmd`：

```text
sim_odom -> /drone0/planning/preset_cmd
         -> apriltag_mission
         -> /drone0/planning/cmd
         -> ctrl_node
```

已经单独启动 MAVROS 后，可用组合 launch 启动相机、检测、轨迹、里程计、控制器和仲裁器：

```bash
source /opt/ros/noetic/setup.bash
source ~/HAM_Flight/devel/setup.bash
roslaunch vision_gripper apriltag_pick_return.launch \
  start_mavros:=false dry_run:=true navigation_monitor_enabled:=true
```

不要在已经运行 `RTK_realflight_traj_follow.sh` 的控制链上再启动这个组合 launch；该脚本
已经创建规划器、manager 和控制器，会造成重复节点和双命令源。组合 launch 应在只有
MAVROS（以及所需 RTK 驱动）已启动时使用。启动后先确认命令图只有一个最终发布者：

```bash
rostopic info /drone0/planning/preset_cmd
rostopic info /drone0/planning/cmd
```

前者应由 `/manager`（`sim_odom` nodelet）发布并由 `/apriltag_mission` 订阅；后者应只由
`/apriltag_mission` 发布并由 `/drone0/manager`（`ctrl_node` nodelet）订阅。

首次必须保持 `dry_run:=true` 完成无桨台架检查。此模式即使收到虚拟闭爪成功，也会进入
`FAILSAFE_HOLD` 并禁止上升/返航，避免把未实际抓取误判为成功。真机夹爪输出使用
`dry_run:=false`，但任务接管仍默认关闭；只有检查相机、RTK/EKF、遥控器接管、里程计
和控制话题都正常后才显式启用：

```bash
rosservice call /apriltag_mission/enable "data: true"
```

随后仍按 HAM_Flight 原流程通过外部遥控器进入 OFFBOARD、起飞并发布一次轨迹触发。
可实时观察：

```bash
rostopic echo /apriltag_mission/state
rostopic echo /apriltag_mission/status
rostopic echo /apriltag_mission/alignment_error
rostopic echo /apriltag_mission/mission_complete
rostopic echo /apriltag_mission/pickup_succeeded
```

状态顺序应为：

```text
FOLLOW_PRESET -> TRACK_TAG -> CLOSING -> ASCEND -> RETURN_HOME -> LANDING -> LANDED
```

短暂丢码时会先平滑停止；连续丢码 `0.75 s` 后进入可恢复的 `REACQUIRE_TAG`，先保持
`0.40 s`，再以不超过 `0.05 m/s` 上升最多 `0.15 m` 扩大视场。在总计 `5 s` 的恢复
窗口内连续重新识别 5 帧会回到 `TRACK_TAG`；仍未恢复则进入
`ABORT_ASCEND -> RETURN_HOME -> LANDING`，不闭爪，先安全上升再返航。

紧急停止自动任务并保持当前位置：

```bash
rosservice call /apriltag_mission/abort
```

标签丢失 0.25 秒时先把跟踪速度平滑降为零；丢码采用上述可恢复/安全返航逻辑。
夹爪命令失败、退出 OFFBOARD 或里程计失效时仍进入 `FAILSAFE_HOLD`。实际空中异常应立即使用遥控器
退出 OFFBOARD 并按现场预案接管。复位仅允许在非 OFFBOARD/未解锁状态进行：

```bash
rosservice call /apriltag_mission/reset
```
