接下来只做“1 m 起飞—悬停—降落”验证，不发布轨迹、不抓取。先确认旧 ROS 进程全部关闭，因为必须重新加载刚编译的 odom_remap。

注意：--dry-run 只是不驱动真实夹爪，无人机仍会真实解锁、转桨、起飞。

## 1. 场地和硬件准备

- 安装、检查桨叶方向和紧固情况。
- 无人机放在最终起飞点，之后不要再搬动。
- RTK 基站、移动端正常，QGC 无 Preflight Fail。
- 遥控器接管、模式开关、急停均已验证。
- 周围至少留出数米净空，人员退出桨区。
- 飞手全程握住遥控器，出现倾斜、上下振荡或漂移立即退出 OFFBOARD。
- 本次不要携带待抓物，只放 AprilTag 用于地面检查。

## 2. 关闭旧控制链

如果旧终端仍在运行，按原来的逆序全部 Ctrl+C 关闭。确认：

rosnode list

如果提示无法连接 ROS Master，说明已关闭，可以继续。不要让新旧 MAVROS、控制器同时运行。

## 3. 一键启动

打开一个终端：

cd ~/HAM_Flight
bash sh_utils/apriltag_rtk_easy_flight.sh start --dry-run

必须看到“一键地面启动完成”。出现任何 [FAIL] 或 [BLOCKED] 就停止，不得起飞。

## 4. 地面准备

bash sh_utils/apriltag_rtk_easy_flight.sh prepare

按提示操作：

1. 飞机未解锁、非 OFFBOARD。
2. 将 tag36h11 / ID 0 放到相机正下方约 0.7～1 m。
3. 输入：

CHECK

4. 检测通过后，把标签放回航线目标位置并固定。
5. 输入：

READY

脚本随后会重新执行最终位置的 5 秒预检。只有再次通过才能继续。

## 5. 起飞前确认

bash sh_utils/apriltag_rtk_easy_flight.sh status

必须看到类似：

armed:             False
system_status:     3
landed_state:      1
mission_state:     FOLLOW_PRESET
rosbag_running:    yes
prepared:          yes

任一项不符合都不要起飞。
## 6. 只测试 1 m 起飞

飞手使用遥控器切入 OFFBOARD，然后马上执行：

bash sh_utils/apriltag_rtk_easy_flight.sh takeoff

正常表现：

- 前约 1 秒电机升转速；
- 随后以约 0.25 m/s 上升；
- 约 5 秒到达 1 m；
- 稳定后终端显示收到控制器 READY。

本次绝对不要执行：

bash sh_utils/apriltag_rtk_easy_flight.sh mission

如果出现以下任一情况，立即用遥控器退出 OFFBOARD并接管：

- 贴地反复上下；
- 明显左右摇晃；
- 快速掉高；
- 航向或位置快速漂移；
- 终端未返回 READY；
- PX4模式异常。

不要在空中按 Ctrl+C。

## 7. 成功后悬停并降落

达到 1 m 后只观察约 10 秒。稳定则执行：

bash sh_utils/apriltag_rtk_easy_flight.sh status
bash sh_utils/apriltag_rtk_easy_flight.sh land

等待 PX4 完成降落和自动下锁。

如果 takeoff 超时报错，新控制器会请求 AUTO.LAND。此时不要发布轨迹，也不要反复发送起飞命令；飞手观察模式并随时接管。

## 8. 落地后保存数据

确认已经落地、下锁并退出 OFFBOARD：

bash sh_utils/apriltag_rtk_easy_flight.sh status

必须满足：

armed: False
landed_state: 1
mode: 不是 OFFBOARD

然后：

bash sh_utils/apriltag_rtk_easy_flight.sh stop

查看新包：

ls -lhtr ~/HAM_Flight/sh_utils/Bag | tail

把最新 bag 文件名告诉我。无论这次成功还是失败，都先分析这个 1 m 悬停包，不要紧接着执行完整抓取任务。
