#!/usr/bin/env python3
"""Read-only RTK/PX4/ROS topology gate for the AprilTag pickup mission."""

import math
import sys
import time

import rosgraph
import rosnode
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from mavros_msgs.msg import EstimatorStatus, ExtendedState, GPSRAW, State
from mavros_msgs.srv import ParamGet
from nav_msgs.msg import Odometry
from nodelet.srv import NodeletList
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String


class Preflight:
    def __init__(self):
        # These are hard safety gates, deliberately not ROS parameters.  A launch
        # argument must not be able to weaken real-flight preflight criteria.
        self.duration = 5.0
        self.min_fix = 6
        self.min_satellites = 8
        self.min_odom_hz = 30.0
        self.min_imu_hz = 50.0
        self.min_image_hz = 10.0
        self.min_detector_hz = 10.0
        self.max_detector_latency_p95 = 0.20
        self.max_detector_latency = 0.35
        self.max_horizontal_span = 0.15
        self.max_vertical_span = 0.10
        self.max_ground_speed = 0.25
        self.max_remap_origin_horizontal = 0.20
        self.max_remap_origin_vertical = 0.15
        self.errors = []
        self.notes = []
        self.state = None
        self.state_time = None
        self.extended = None
        self.extended_time = None
        self.mission_state = None
        self.mission_status = None
        self.gps = []
        self.estimator = []
        self.odom = []
        self.local_odom = []
        self.imu_times = []
        self.image_times = []
        self.detector_times = []
        self.detector_latencies = []
        self.camera_info_count = 0

        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=5)
        rospy.Subscriber(
            "/mavros/extended_state", ExtendedState, self._extended_cb, queue_size=5
        )
        rospy.Subscriber(
            "/mavros/gpsstatus/gps1/raw", GPSRAW, self._gps_cb, queue_size=20
        )
        rospy.Subscriber(
            "/mavros/estimator_status", EstimatorStatus,
            self._estimator_cb, queue_size=20
        )
        rospy.Subscriber("/odom/remap", Odometry, self._odom_cb, queue_size=100)
        rospy.Subscriber(
            "/mavros/local_position/odom", Odometry,
            self._local_odom_cb, queue_size=100
        )
        rospy.Subscriber("/mavros/imu/data", Imu, self._imu_cb, queue_size=100)
        rospy.Subscriber(
            "/camera/color/image_raw", Image, self._image_cb,
            queue_size=5, buff_size=2 ** 22
        )
        rospy.Subscriber(
            "/camera/color/camera_info", CameraInfo,
            self._camera_info_cb, queue_size=5
        )
        rospy.Subscriber(
            "/tag_detections", AprilTagDetectionArray,
            self._tag_detections_cb, queue_size=5
        )
        rospy.Subscriber(
            "/apriltag_mission/state", String, self._mission_state_cb, queue_size=5
        )
        rospy.Subscriber(
            "/apriltag_mission/status", String, self._mission_status_cb, queue_size=5
        )

    def _now(self):
        return time.monotonic()

    def _state_cb(self, msg):
        self.state = msg
        self.state_time = self._now()

    def _extended_cb(self, msg):
        self.extended = msg
        self.extended_time = self._now()

    def _gps_cb(self, msg):
        self.gps.append((self._now(), msg))

    def _estimator_cb(self, msg):
        self.estimator.append((self._now(), msg))

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.odom.append((self._now(), p.x, p.y, p.z, v.x, v.y, v.z))

    def _local_odom_cb(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.local_odom.append((self._now(), p.x, p.y, p.z, v.x, v.y, v.z))

    def _imu_cb(self, _msg):
        self.imu_times.append(self._now())

    def _image_cb(self, _msg):
        self.image_times.append(self._now())

    def _camera_info_cb(self, _msg):
        self.camera_info_count += 1

    def _tag_detections_cb(self, msg):
        self.detector_times.append(self._now())
        if msg.header.stamp.is_zero():
            self.detector_latencies.append(float("inf"))
            return
        self.detector_latencies.append(
            (rospy.Time.now() - msg.header.stamp).to_sec()
        )

    def _mission_state_cb(self, msg):
        self.mission_state = msg.data

    def _mission_status_cb(self, msg):
        self.mission_status = msg.data

    def fail(self, text):
        self.errors.append(text)

    def note(self, text):
        self.notes.append(text)

    @staticmethod
    def _rate(times):
        if len(times) < 2:
            return 0.0
        span = times[-1] - times[0]
        return (len(times) - 1) / span if span > 0.0 else 0.0

    @staticmethod
    def _percentile(values, fraction):
        ordered = sorted(values)
        if not ordered:
            return float("inf")
        index = int(round((len(ordered) - 1) * fraction))
        return ordered[max(0, min(len(ordered) - 1, index))]

    @staticmethod
    def _nodes_for(entries, topic):
        for name, nodes in entries:
            if name == topic:
                return set(nodes)
        return set()

    def check_static_configuration(self):
        if rospy.get_param("/use_sim_time", False):
            self.fail("/use_sim_time=true；实飞必须使用墙钟")

        expected = {
            "/drone0/ctrl_node/navigation_monitor/enabled": True,
            "/drone0/ctrl_node/navigation_monitor/rtk_fixed_min_type": 6,
            "/drone0/ctrl_node/navigation_monitor/status_timeout": 1.0,
            "/drone0/ctrl_node/navigation_monitor/failure_grace": 1.0,
            "/drone0/ctrl_node/navigation_monitor/auto_land_timeout": 3.0,
            "/drone0/ctrl_node/takeoff_state/height": 1.0,
            "/drone0/ctrl_node/takeoff_state/speed": 0.25,
            "/drone0/ctrl_node/msg_timeout/cmd": 0.3,
            "/drone0/sim_odom/traj_type": 3,
            "/drone0/sim_odom/line_start_hold_time": 3.0,
            "/drone0/sim_odom/line_forward_distance": 3.0,
            "/drone0/sim_odom/line_forward_time": 27.0,
            "/drone0/sim_odom/line_end_hold_time": 2.0,
            "/drone0/sim_odom/line_return_time": 27.0,
            "/drone0/sim_odom/path_scale": 1.0,
            "/drone0/sim_odom/speed_scale": 1.0,
            "/apriltag_detector/tag_family": "tag36h11",
            "/apriltag_detector/tag_decimate": 1.0,
            "/apriltag_detector/standalone_tags": [
                {"id": 0, "size": 0.08746, "name": "tag_0"}
            ],
            "/apriltag_gripper/pose_filter_window": 3,
            "/apriltag_gripper/detection_timeout_s": 0.25,
            "/apriltag_gripper/gripper_offset_x_m": 0.0,
            "/apriltag_gripper/gripper_offset_y_m": 0.088,
            "/apriltag_mission/target_distance_m": 0.45,
            "/apriltag_mission/camera_to_gripper_forward_m": -0.058,
            "/apriltag_mission/camera_to_gripper_right_m": 0.0,
            "/apriltag_mission/camera_to_gripper_down_m": 0.110,
            "/apriltag_mission/object_forward_from_tag_m": 0.030,
            "/apriltag_mission/object_right_from_tag_m": 0.0,
            "/apriltag_mission/object_height_above_tag_m": 0.340,
            "/apriltag_mission/camera_mount_roll_correction_deg": 0.0,
            "/apriltag_mission/camera_mount_pitch_correction_deg": 0.0,
            "/apriltag_mission/camera_mount_yaw_correction_deg": 0.0,
            "/apriltag_mission/tag_timeout_s": 0.25,
            "/apriltag_mission/tag_abort_timeout_s": 1.0,
            "/apriltag_mission/reacquire_hold_s": 6.0,
            "/apriltag_mission/reacquire_stable_detections": 5,
            "/apriltag_mission/reacquire_initial_hold_s": 0.40,
            "/apriltag_mission/reacquire_climb_m": 0.15,
            "/apriltag_mission/reacquire_climb_speed_mps": 0.05,
            "/apriltag_mission/max_tag_measurement_age_s": 0.25,
            "/apriltag_mission/tag_odom_sync_tolerance_s": 0.05,
            "/apriltag_mission/lateral_kp": 0.55,
            "/apriltag_mission/vertical_kp": 0.40,
            "/apriltag_mission/max_lateral_speed_mps": 0.12,
            "/apriltag_mission/max_vertical_speed_mps": 0.07,
            "/apriltag_mission/near_lateral_speed_mps": 0.05,
            "/apriltag_mission/near_vertical_speed_mps": 0.03,
            "/apriltag_mission/descent_full_alignment_m": 0.07,
            "/apriltag_mission/descent_stop_alignment_m": 0.16,
            "/apriltag_mission/max_tracking_accel_mps2": 0.10,
            "/apriltag_mission/max_xy_reference_lead_m": 0.10,
            "/apriltag_mission/max_z_reference_lead_m": 0.06,
            "/apriltag_mission/close_max_horizontal_speed_mps": 0.10,
            "/apriltag_mission/close_max_vertical_speed_mps": 0.08,
            "/apriltag_mission/close_max_command_speed_mps": 0.04,
            "/apriltag_mission/lift_speed_mps": 0.10,
            "/apriltag_mission/return_speed_mps": 0.18,
            "/apriltag_mission/return_accel_mps2": 0.10,
            "/apriltag_mission/return_settle_s": 2.0,
            "/apriltag_mission/return_to_preset_altitude": True,
            "/apriltag_mission/auto_land_on_return": False,
        }
        for name, wanted in expected.items():
            if not rospy.has_param(name):
                self.fail("缺少运行时参数 {}".format(name))
                continue
            actual = rospy.get_param(name)
            if isinstance(wanted, float):
                try:
                    matches = abs(float(actual) - wanted) <= 1e-6
                except (TypeError, ValueError):
                    matches = False
            else:
                matches = actual == wanted
            if not matches:
                self.fail("参数 {}={}，要求 {}".format(name, actual, wanted))

    def check_nodes_and_graph(self):
        required_nodes = {
            "/mavros", "/manager", "/drone0/manager", "/apriltag_detector",
            "/apriltag_gripper", "/apriltag_mission",
        }
        try:
            running = set(rosnode.get_node_names())
        except rosnode.ROSNodeIOException as exc:
            self.fail("无法读取 ROS 节点图：{}".format(exc))
            return
        missing = sorted(required_nodes - running)
        if missing:
            self.fail("缺少节点：{}".format(", ".join(missing)))

        for service, suffixes in (
            ("/manager/list", ("sim_odom",)),
            ("/drone0/manager/list", ("odomRemap", "ctrl_node")),
        ):
            try:
                rospy.wait_for_service(service, timeout=3.0)
                loaded = rospy.ServiceProxy(service, NodeletList)().nodelets
            except (rospy.ROSException, rospy.ServiceException) as exc:
                self.fail("无法读取 nodelet manager {}：{}".format(service, exc))
                continue
            for suffix in suffixes:
                if not any(name.endswith(suffix) for name in loaded):
                    self.fail("{} 未加载 nodelet {}".format(service, suffix))

        try:
            publishers, subscribers, _services = rosgraph.Master(
                rospy.get_name()
            ).getSystemState()
        except Exception as exc:  # rosgraph exposes multiple XML-RPC exception types
            self.fail("无法读取 ROS 连接图：{}".format(exc))
            return

        exact_publishers = {
            "/drone0/planning/preset_cmd": {"/manager"},
            "/drone0/planning/cmd": {"/apriltag_mission"},
            "/apriltag_gripper/tag_position": {"/apriltag_gripper"},
            "/mavros/setpoint_raw/attitude": {"/drone0/manager"},
        }
        required_subscribers = {
            "/drone0/planning/preset_cmd": "/apriltag_mission",
            "/drone0/planning/cmd": "/drone0/manager",
            "/apriltag_gripper/tag_position": "/apriltag_mission",
            "/mavros/setpoint_raw/attitude": "/mavros",
            "/drone0/ctrl_node/takeoff_land": "/drone0/manager",
        }
        for topic, wanted in exact_publishers.items():
            actual = self._nodes_for(publishers, topic)
            if actual != wanted:
                self.fail(
                    "{} 发布者为 {}，必须且只能为 {}".format(
                        topic, sorted(actual), sorted(wanted)
                    )
                )
        for topic, wanted in required_subscribers.items():
            actual = self._nodes_for(subscribers, topic)
            if wanted not in actual:
                self.fail("{} 缺少订阅者 {}".format(topic, wanted))

    def check_px4_parameters(self):
        try:
            rospy.wait_for_service("/mavros/param/get", timeout=5.0)
            get_param = rospy.ServiceProxy("/mavros/param/get", ParamGet)
        except rospy.ROSException as exc:
            self.fail("MAVROS 参数服务不可用：{}".format(exc))
            return

        def read(name):
            try:
                reply = get_param(param_id=name)
            except rospy.ServiceException as exc:
                self.fail("读取 PX4 参数 {} 失败：{}".format(name, exc))
                return None
            if not reply.success:
                self.fail("PX4 参数 {} 不可用".format(name))
                return None
            return reply.value

        loss = read("COM_OF_LOSS_T")
        action = read("COM_OBL_RC_ACT")
        mapping = read("PWM_MAIN_FUNC1")
        if loss is not None and not (0.0 <= loss.real <= 1.0):
            self.fail("COM_OF_LOSS_T={} s，要求 0..1 s".format(loss.real))
        if action is not None and action.integer != 4:
            self.fail("COM_OBL_RC_ACT={}，要求 4 (Land)".format(action.integer))
        if mapping is not None and mapping.integer != 301:
            self.fail("PWM_MAIN_FUNC1={}，要求 301".format(mapping.integer))

    def check_samples(self):
        if self.state is None:
            self.fail("未收到 /mavros/state")
        else:
            if self._now() - self.state_time > 1.0:
                self.fail("/mavros/state 已超过 1 s 未更新")
            if not self.state.connected:
                self.fail("MAVROS 未连接 PX4")
            if self.state.armed:
                self.fail("飞行器已经解锁；起飞前检查拒绝继续")
            if self.state.mode == "OFFBOARD":
                self.fail("检查时已经处于 OFFBOARD；必须先退出并保持未解锁")
            if self.state.system_status != 3:
                self.fail(
                    "PX4 system_status={}，要求 STANDBY(3)；0=UNINIT、8=FLIGHT_TERMINATION，必须先排除终止/预解锁故障并重启飞控".format(
                        self.state.system_status
                    )
                )

        if self.extended is None:
            self.fail("未收到 /mavros/extended_state")
        else:
            if self._now() - self.extended_time > 1.0:
                self.fail("/mavros/extended_state 已超过 1 s 未更新")
            if self.extended.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
                self.fail("PX4 未确认 ON_GROUND")

        if len(self.gps) < 5:
            self.fail("RTK 状态样本不足（{}）".format(len(self.gps)))
        else:
            bad_fix = [msg.fix_type for _, msg in self.gps if msg.fix_type < self.min_fix]
            if bad_fix:
                self.fail("RTK 在采样窗口内不是持续 Fixed，最低 fix_type={}".format(min(bad_fix)))
            min_sats = min(msg.satellites_visible for _, msg in self.gps)
            if any(msg.satellites_visible == 255 for _, msg in self.gps):
                self.fail("RTK 可见卫星数为 unknown (255)")
            elif min_sats < self.min_satellites:
                self.fail("RTK 可见卫星最低 {}，要求 >= {}".format(min_sats, self.min_satellites))

        if len(self.estimator) < 5:
            self.fail("EKF 状态样本不足（{}）".format(len(self.estimator)))
        else:
            for _, msg in self.estimator:
                if not msg.pos_horiz_abs_status_flag:
                    self.fail("EKF 水平绝对位置在采样窗口内失效")
                    break
                if not msg.pos_vert_abs_status_flag:
                    self.fail("EKF 垂直绝对位置在采样窗口内失效")
                    break
                if msg.const_pos_mode_status_flag or msg.gps_glitch_status_flag:
                    self.fail("EKF 报告 constant-position 或 GPS glitch")
                    break

        odom_times = [sample[0] for sample in self.odom]
        odom_rate = self._rate(odom_times)
        if odom_rate < self.min_odom_hz:
            self.fail("/odom/remap 频率 {:.1f} Hz，要求 >= {:.1f} Hz".format(odom_rate, self.min_odom_hz))
        if self.odom:
            if not all(all(math.isfinite(v) for v in sample[1:]) for sample in self.odom):
                self.fail("/odom/remap 含 NaN/Inf")
            xs, ys, zs = zip(*[(s[1], s[2], s[3]) for s in self.odom])
            horizontal_span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            vertical_span = max(zs) - min(zs)
            max_speed = max(
                math.sqrt(s[4] * s[4] + s[5] * s[5] + s[6] * s[6])
                for s in self.odom
            )
            if horizontal_span > self.max_horizontal_span:
                self.fail("地面静止时 odom 水平漂移 {:.3f} m".format(horizontal_span))
            if vertical_span > self.max_vertical_span:
                self.fail("地面静止时 odom 高度漂移 {:.3f} m".format(vertical_span))
            if max_speed > self.max_ground_speed:
                self.fail("地面静止时 odom 最大速度 {:.3f} m/s".format(max_speed))
            mean_x = sum(xs) / len(xs)
            mean_y = sum(ys) / len(ys)
            mean_z = sum(zs) / len(zs)
            origin_horizontal = math.hypot(mean_x, mean_y)
            if origin_horizontal > self.max_remap_origin_horizontal:
                self.fail(
                    "/odom/remap 零点水平偏移 {:.3f} m，要求 <= {:.3f} m；"
                    "飞机必须在最终起飞点静止后重启定位/控制链".format(
                        origin_horizontal, self.max_remap_origin_horizontal
                    )
                )
            if abs(mean_z) > self.max_remap_origin_vertical:
                self.fail(
                    "/odom/remap 零点 z={:.3f} m，要求 |z| <= {:.3f} m；"
                    "禁止在零点标定后搬动飞机再起飞".format(
                        mean_z, self.max_remap_origin_vertical
                    )
                )
            self.note(
                "odom {:.1f} Hz，水平跨度 {:.3f} m，高度跨度 {:.3f} m，"
                "零点=({:.3f}, {:.3f}, {:.3f}) m，最大速度 {:.3f} m/s".format(
                    odom_rate, horizontal_span, vertical_span,
                    mean_x, mean_y, mean_z, max_speed
                )
            )

        local_times = [sample[0] for sample in self.local_odom]
        local_rate = self._rate(local_times)
        if local_rate < self.min_odom_hz:
            self.fail(
                "/mavros/local_position/odom 频率 {:.1f} Hz，要求 >= {:.1f} Hz".format(
                    local_rate, self.min_odom_hz
                )
            )
        if self.local_odom:
            if not all(
                all(math.isfinite(value) for value in sample[1:])
                for sample in self.local_odom
            ):
                self.fail("/mavros/local_position/odom 含 NaN/Inf")
            local_xs, local_ys, local_zs = zip(*[
                (sample[1], sample[2], sample[3]) for sample in self.local_odom
            ])
            local_horizontal_span = math.hypot(
                max(local_xs) - min(local_xs),
                max(local_ys) - min(local_ys),
            )
            local_vertical_span = max(local_zs) - min(local_zs)
            if local_horizontal_span > self.max_horizontal_span:
                self.fail(
                    "地面静止时 PX4 local pose 水平跳动 {:.3f} m".format(
                        local_horizontal_span
                    )
                )
            if local_vertical_span > self.max_vertical_span:
                self.fail(
                    "地面静止时 PX4 local pose 高度跳动 {:.3f} m".format(
                        local_vertical_span
                    )
                )
            self.note(
                "PX4 local pose {:.1f} Hz，原始坐标均值=({:.3f}, {:.3f}, {:.3f}) m，"
                "水平/垂向跨度={:.3f}/{:.3f} m；原始坐标可非零，但必须连续".format(
                    local_rate,
                    sum(local_xs) / len(local_xs),
                    sum(local_ys) / len(local_ys),
                    sum(local_zs) / len(local_zs),
                    local_horizontal_span,
                    local_vertical_span,
                )
            )

        imu_rate = self._rate(self.imu_times)
        image_rate = self._rate(self.image_times)
        detector_rate = self._rate(self.detector_times)
        if imu_rate < self.min_imu_hz:
            self.fail("IMU 频率 {:.1f} Hz，要求 >= {:.1f} Hz".format(imu_rate, self.min_imu_hz))
        if image_rate < self.min_image_hz:
            self.fail("D435i 彩色图像频率 {:.1f} Hz，要求 >= {:.1f} Hz".format(image_rate, self.min_image_hz))
        if detector_rate < self.min_detector_hz:
            self.fail("AprilTag 检测输出频率 {:.1f} Hz，要求 >= {:.1f} Hz".format(
                detector_rate, self.min_detector_hz
            ))
        finite_latencies = [
            value for value in self.detector_latencies
            if math.isfinite(value) and value >= 0.0
        ]
        if len(finite_latencies) != len(self.detector_latencies) or not finite_latencies:
            self.fail("AprilTag 检测消息时间戳无效或与 ROS 时钟不一致")
        else:
            latency_p95 = self._percentile(finite_latencies, 0.95)
            latency_max = max(finite_latencies)
            if latency_p95 > self.max_detector_latency_p95:
                self.fail("AprilTag 检测延迟 P95 {:.3f} s，要求 <= {:.3f} s".format(
                    latency_p95, self.max_detector_latency_p95
                ))
            if latency_max > self.max_detector_latency:
                self.fail("AprilTag 检测最大延迟 {:.3f} s，要求 <= {:.3f} s".format(
                    latency_max, self.max_detector_latency
                ))
            self.note(
                "AprilTag 输出 {:.1f} Hz，延迟 P95 {:.3f} s，最大 {:.3f} s".format(
                    detector_rate, latency_p95, latency_max
                )
            )
        if self.camera_info_count < 1:
            self.fail("未收到 D435i camera_info")

        if self.mission_state != "FOLLOW_PRESET":
            self.fail("任务状态为 {}，要求 FOLLOW_PRESET".format(self.mission_state))
        if not self.mission_status or "disabled" not in self.mission_status.lower():
            self.fail("任务接管不是明确 disabled 状态：{}".format(self.mission_status))

        self.note("IMU {:.1f} Hz，D435i 彩色图像 {:.1f} Hz".format(imu_rate, image_rate))

    def run(self):
        print(
            "[CHECK] 收集 {:.1f} s 连续 RTK/EKF/odom/IMU/相机样本...".format(
                self.duration
            ),
            flush=True,
        )
        # Use wall time so a stray /use_sim_time=true cannot make preflight hang.
        time.sleep(self.duration)
        self.check_static_configuration()
        self.check_nodes_and_graph()
        self.check_px4_parameters()
        self.check_samples()
        for text in self.notes:
            print("[INFO] " + text)
        if self.errors:
            for text in self.errors:
                print("[FAIL] " + text, file=sys.stderr)
            print("[BLOCKED] 起飞前检查失败；禁止启用任务、进入 OFFBOARD 或解锁。", file=sys.stderr)
            return 1
        print("[READY] RTK、EKF、里程计、相机、规划器、仲裁器、控制器及 PX4 保护全部通过。")
        print("[SAFE] 当前仍未解锁、非 OFFBOARD、任务接管关闭。")
        return 0


def main():
    rospy.init_node("apriltag_rtk_preflight", anonymous=True)
    raise SystemExit(Preflight().run())


if __name__ == "__main__":
    main()
