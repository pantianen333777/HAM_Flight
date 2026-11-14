# AprilTag 抓取任务 Gazebo 闭环仿真

本仿真只连接 PX4 SITL 的 UDP 端口，不访问真实飞控串口，也不会驱动真实夹爪。
仿真复用实飞任务节点和控制器；夹爪使用 `dry_run` 应答，AprilTag 检测由 Gazebo
模型真值生成，因此验证的是飞行控制、任务状态机、几何标定和抓取时序，不是 D435i
图像质量或 AprilTag 解码鲁棒性。

## 仿真任务

1. Iris 起飞并在约 1.0 m 高度稳定。
2. 沿机头方向执行 3 m 预设直线轨迹。
3. 途中发现位于前方 1.5 m 的 AprilTag 后接管轨迹。
4. 将夹爪中心对准标签中心，并将相机到标签的垂直距离控制到 `0.45 ± 0.03 m`。
5. 连续 5 帧满足位置条件且飞行器速度稳定后，执行一次夹爪闭合。
6. 回升至 1.0 m，沿直线加减速返回原点，进入 `HOLD_HOME`。
7. 人工发布 `takeland.sh`，然后落地并上锁。

## 启动

终端 1：

```bash
cd /home/cat/HAM_Flight
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source /home/cat/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
  /home/cat/PX4-Autopilot \
  /home/cat/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/home/cat/PX4-Autopilot:/home/cat/PX4-Autopilot/Tools/simulation/gazebo-classic"
roslaunch vision_gripper apriltag_gazebo_sim.launch gui:=true
```

无桌面或只做自动检查时使用 `gui:=false`。

终端 2：等待 `/mavros/state` 中 `connected: True` 后执行：

```bash
cd /home/cat/HAM_Flight
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"

rostopic pub -1 /drone0/ctrl_node/takeoff_land \
  quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
```

确认 `/odom/remap` 的 `pose.pose.position.z` 已稳定在约 1.0 m 后触发轨迹：

```bash
bash sh_utils/pub_trigger.sh
```

终端 3：观察任务：

```bash
source /opt/ros/noetic/setup.bash
source /home/cat/HAM_Flight/devel/setup.bash

rostopic echo /apriltag_mission/state
rostopic echo /apriltag_mission/status
rostopic echo /apriltag_mission/mission_complete
```

正常状态顺序为：

```text
FOLLOW_PRESET -> TRACK_TAG -> CLOSING -> ASCEND
-> RETURN_HOME -> HOLD_HOME
```

## 验收

任务结束后应同时满足：

```bash
rostopic echo -n 1 /apriltag_mission/mission_complete
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/extended_state
rostopic echo -n 1 /odom/remap
```

- `mission_complete.data: True`
- `/apriltag_mission/state`: `HOLD_HOME`
- `/odom/remap` 约为原点上方 1.0 m

然后人工执行：

```bash
bash sh_utils/takeland.sh
```

落地后应看到 `/mavros/state.armed=false` 和
`/mavros/extended_state.landed_state=1`。

2026-11-14 当前版本机闭环结果：标签被正常接管、闭爪一次、回升和返航；
`HOLD_HOME` 时位置为 `x=0.002 m, y=-0.0002 m, z=1.013 m`，
`pickup_succeeded=True`、`mission_complete=True`。随后人工 `takeland.sh`
已使 PX4 进入 `AUTO.LAND` 并落地下锁。

## 参数说明

- 标签模型初始位置：`tag_x=1.5`, `tag_y=0.0`
- 目标距离：`0.45 m`
- 水平与距离容差：`0.03 m`
- 相机到夹爪中心标定偏移：`x=0.0533 m`, `y=0.0556 m`
- Gazebo Iris 悬停归一化推力：`0.81`

可在启动时改变标签位置，例如：

```bash
roslaunch vision_gripper apriltag_gazebo_sim.launch \
  gui:=true tag_x:=2.0 tag_y:=0.2
```

如果终端持续打印 `IMU , which is too low!`，这是当前控制器按 100 Hz 阈值检查、
而 PX4 SITL/MAVROS IMU 为约 50 Hz 造成的频率提示。本次仿真中 IMU 数据仍持续有效，
未触发输入超时或任务失效。

停止仿真时在终端 1 按 `Ctrl+C`，等待所有 ROS、PX4 和 Gazebo 子进程退出。
