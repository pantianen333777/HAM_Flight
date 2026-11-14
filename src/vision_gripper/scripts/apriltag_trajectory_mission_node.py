#!/usr/bin/env python3
"""Arbitrate a preset HAM_Flight trajectory and a one-shot AprilTag pickup.

The preset PositionCommand stream is passed through without modification until
the configured tag is detected.  The node then owns the command topic for the
rest of that trajectory: align at the calibrated camera/gripper offset, close
the gripper once, restore cruise altitude, return to the origin, and land.
"""

import copy
import math
import threading
from collections import deque

import rospy
from geometry_msgs.msg import PointStamped
from mavros_msgs.msg import ExtendedState, State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, TakeoffLand
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


FOLLOW_PRESET = "FOLLOW_PRESET"
TRACK_TAG = "TRACK_TAG"
REACQUIRE_TAG = "REACQUIRE_TAG"
CLOSING = "CLOSING"
ASCEND = "ASCEND"
ABORT_ASCEND = "ABORT_ASCEND"
RETURN_HOME = "RETURN_HOME"
HOLD_HOME = "HOLD_HOME"
LANDING = "LANDING"
LANDED = "LANDED"
FAILSAFE_HOLD = "FAILSAFE_HOLD"


def clamp(value, limit):
    return max(-limit, min(limit, value))


def finite(values):
    return all(math.isfinite(value) for value in values)


def descent_speed_scale(horizontal_error, full_speed_gate, stop_gate):
    """Continuous descent authority: full inside inner gate, zero outside outer."""
    if horizontal_error <= full_speed_gate:
        return 1.0
    if horizontal_error >= stop_gate:
        return 0.0
    return (stop_gate - horizontal_error) / (stop_gate - full_speed_gate)


def yaw_from_quaternion(q):
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError("invalid odometry quaternion")
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rotate_vector(q, vector):
    """Rotate a FLU body vector into the odometry world frame."""
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError("invalid odometry quaternion")
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    vx, vy, vz = vector
    # Quaternion rotation expanded as R(q) * v.
    return [
        (1.0 - 2.0 * (y * y + z * z)) * vx
        + 2.0 * (x * y - z * w) * vy
        + 2.0 * (x * z + y * w) * vz,
        2.0 * (x * y + z * w) * vx
        + (1.0 - 2.0 * (x * x + z * z)) * vy
        + 2.0 * (y * z - x * w) * vz,
        2.0 * (x * z - y * w) * vx
        + 2.0 * (y * z + x * w) * vy
        + (1.0 - 2.0 * (x * x + y * y)) * vz,
    ]


def rotate_rpy(vector, roll, pitch, yaw):
    """Apply a fixed Rz(yaw)*Ry(pitch)*Rx(roll) mounting correction."""
    vx, vy, vz = vector
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    after_roll = [vx, cr * vy - sr * vz, sr * vy + cr * vz]
    after_pitch = [
        cp * after_roll[0] + sp * after_roll[2],
        after_roll[1],
        -sp * after_roll[0] + cp * after_roll[2],
    ]
    return [
        cy * after_pitch[0] - sy * after_pitch[1],
        sy * after_pitch[0] + cy * after_pitch[1],
        after_pitch[2],
    ]


def yaw_rotate_flu(vector, yaw):
    """Rotate a level FLU vector into ENU using only the locked mission yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [c * vector[0] - s * vector[1],
            s * vector[0] + c * vector[1], vector[2]]


class AprilTagTrajectoryMission:
    def __init__(self):
        self.lock = threading.RLock()

        self.enabled = bool(rospy.get_param("~enabled_on_start", False))
        self.rate_hz = float(rospy.get_param("~control_rate_hz", 50.0))
        if not math.isfinite(self.rate_hz) or self.rate_hz < 20.0:
            raise ValueError("~control_rate_hz must be finite and >= 20 Hz")

        self.target_z = float(rospy.get_param("~target_distance_m", 0.45))
        self.distance_tolerance = float(rospy.get_param("~distance_tolerance_m", 0.03))
        self.horizontal_tolerance = float(rospy.get_param("~horizontal_tolerance_m", 0.03))
        # Measured rigid geometry, expressed in aircraft FRD for operator-facing
        # parameters and converted to ROS body FLU internally.  The camera is
        # 58 mm ahead of the gripper and 110 mm above its closing centre.
        camera_to_gripper_forward = float(
            rospy.get_param("~camera_to_gripper_forward_m", -0.058)
        )
        camera_to_gripper_right = float(
            rospy.get_param("~camera_to_gripper_right_m", 0.0)
        )
        camera_to_gripper_down = float(
            rospy.get_param("~camera_to_gripper_down_m", 0.110)
        )
        self.camera_to_gripper_flu = [
            camera_to_gripper_forward,
            -camera_to_gripper_right,
            -camera_to_gripper_down,
        ]
        self.object_forward = float(rospy.get_param("~object_forward_from_tag_m", 0.030))
        self.object_right = float(rospy.get_param("~object_right_from_tag_m", 0.0))
        self.object_height = float(rospy.get_param("~object_height_above_tag_m", 0.340))
        self.mount_rpy = [
            math.radians(float(rospy.get_param("~camera_mount_roll_correction_deg", 0.0))),
            math.radians(float(rospy.get_param("~camera_mount_pitch_correction_deg", 0.0))),
            math.radians(float(rospy.get_param("~camera_mount_yaw_correction_deg", 0.0))),
        ]
        self.takeover_required = max(1, int(rospy.get_param("~detection_takeover_count", 3)))
        self.target_required = max(1, int(rospy.get_param("~target_stable_detections", 5)))
        self.tag_timeout = float(rospy.get_param("~tag_timeout_s", 0.25))
        self.tag_abort_timeout = float(rospy.get_param("~tag_abort_timeout_s", 0.75))
        self.reacquire_hold = float(rospy.get_param("~reacquire_hold_s", 5.0))
        self.reacquire_required = max(
            1, int(rospy.get_param("~reacquire_stable_detections", 5))
        )
        self.lost_tag_climb = float(rospy.get_param("~lost_tag_climb_m", 0.30))
        self.max_tag_jump = float(rospy.get_param("~max_tag_jump_m", 0.20))
        self.max_tag_measurement_age = float(
            rospy.get_param("~max_tag_measurement_age_s", 0.25)
        )
        self.tag_odom_sync_tolerance = float(
            rospy.get_param("~tag_odom_sync_tolerance_s", 0.05)
        )
        self.odom_history_duration = float(
            rospy.get_param("~odom_history_duration_s", 2.0)
        )

        self.kp_xy = float(rospy.get_param("~lateral_kp", 0.8))
        self.kp_z = float(rospy.get_param("~vertical_kp", 0.8))
        self.max_xy_speed = float(rospy.get_param("~max_lateral_speed_mps", 0.15))
        self.max_z_speed = float(rospy.get_param("~max_vertical_speed_mps", 0.10))
        self.near_xy_speed = float(rospy.get_param("~near_lateral_speed_mps", 0.08))
        self.near_z_speed = float(rospy.get_param("~near_vertical_speed_mps", 0.06))
        self.near_distance = float(rospy.get_param("~near_target_error_m", 0.15))
        self.descent_full_alignment = float(
            rospy.get_param("~descent_full_alignment_m", 0.08)
        )
        self.descent_stop_alignment = float(
            rospy.get_param("~descent_stop_alignment_m", 0.20)
        )
        self.max_tracking_accel = float(rospy.get_param("~max_tracking_accel_mps2", 0.20))
        self.deadband = float(rospy.get_param("~control_deadband_m", 0.01))
        self.close_max_horizontal_speed = float(
            rospy.get_param("~close_max_horizontal_speed_mps", 0.10)
        )
        self.close_max_vertical_speed = float(
            rospy.get_param("~close_max_vertical_speed_mps", 0.08)
        )
        self.close_max_command_speed = float(
            rospy.get_param("~close_max_command_speed_mps", 0.04)
        )
        legacy_reference_lead = float(rospy.get_param("~max_reference_lead_m", 0.25))
        self.max_xy_reference_lead = float(
            rospy.get_param("~max_xy_reference_lead_m", legacy_reference_lead)
        )
        self.max_z_reference_lead = float(
            rospy.get_param("~max_z_reference_lead_m", legacy_reference_lead)
        )
        self.reacquire_initial_hold = float(
            rospy.get_param("~reacquire_initial_hold_s", 0.40)
        )
        self.reacquire_climb = float(rospy.get_param("~reacquire_climb_m", 0.15))
        self.reacquire_climb_speed = float(
            rospy.get_param("~reacquire_climb_speed_mps", 0.05)
        )

        self.lift_height = float(rospy.get_param("~lift_height_m", 0.50))
        self.gripper_settle = float(rospy.get_param("~gripper_settle_s", 1.0))
        self.lift_speed = float(rospy.get_param("~lift_speed_mps", 0.12))
        self.lift_tolerance = float(rospy.get_param("~lift_tolerance_m", 0.05))
        self.lift_settle = float(rospy.get_param("~lift_settle_s", 0.50))
        self.return_speed = float(rospy.get_param("~return_speed_mps", 0.25))
        self.return_accel = float(rospy.get_param("~return_accel_mps2", 0.10))
        self.return_tolerance = float(rospy.get_param("~return_tolerance_m", 0.10))
        self.return_settle = float(rospy.get_param("~return_settle_s", 2.0))
        self.return_to_preset_altitude = bool(
            rospy.get_param("~return_to_preset_altitude", True)
        )
        self.auto_land_on_return = bool(rospy.get_param("~auto_land_on_return", True))

        self.odom_timeout = float(rospy.get_param("~odom_timeout_s", 0.30))
        self.preset_timeout = float(rospy.get_param("~preset_timeout_s", 0.15))
        self.require_connected = bool(rospy.get_param("~require_mavros_connected", True))
        self.require_armed_offboard = bool(rospy.get_param("~require_armed_offboard", True))
        self.gripper_dry_run = bool(rospy.get_param("~gripper_dry_run", True))
        self.allow_dry_run_sequence = bool(rospy.get_param("~allow_dry_run_sequence", False))

        positive = (
            self.target_z, self.distance_tolerance, self.horizontal_tolerance,
            self.tag_timeout, self.tag_abort_timeout, self.reacquire_hold,
            self.lost_tag_climb, self.max_tag_jump, self.max_xy_speed,
            self.max_z_speed, self.near_xy_speed, self.near_z_speed,
            self.near_distance, self.descent_full_alignment,
            self.descent_stop_alignment, self.max_tracking_accel,
            self.close_max_horizontal_speed, self.close_max_vertical_speed,
            self.close_max_command_speed,
            self.max_xy_reference_lead, self.max_z_reference_lead,
            self.max_tag_measurement_age, self.tag_odom_sync_tolerance,
            self.odom_history_duration, self.reacquire_initial_hold,
            self.reacquire_climb, self.reacquire_climb_speed,
            self.lift_height, self.gripper_settle, self.lift_speed, self.lift_tolerance,
            self.lift_settle, self.return_speed, self.return_accel, self.return_tolerance,
            self.return_settle, self.odom_timeout,
            self.preset_timeout, camera_to_gripper_down, self.object_height,
        )
        if not finite(positive) or any(value <= 0.0 for value in positive):
            raise ValueError("mission distances, limits, and timeouts must be finite and > 0")
        if self.tag_abort_timeout <= self.tag_timeout:
            raise ValueError("~tag_abort_timeout_s must exceed ~tag_timeout_s")
        if self.descent_stop_alignment <= self.descent_full_alignment:
            raise ValueError("~descent_stop_alignment_m must exceed ~descent_full_alignment_m")
        geometry_values = (
            camera_to_gripper_forward, camera_to_gripper_right,
            self.object_forward, self.object_right, *self.mount_rpy,
        )
        if not finite(geometry_values):
            raise ValueError("camera/gripper/object geometry must be finite")
        expected_camera_distance = camera_to_gripper_down + self.object_height
        if abs(expected_camera_distance - self.target_z) > self.distance_tolerance:
            raise ValueError(
                "geometry requires camera/tag distance {:.3f} m, but target_distance_m={:.3f} m".format(
                    expected_camera_distance, self.target_z
                )
            )

        self.state = FOLLOW_PRESET
        self.status = "ready; mission takeover {}".format(
            "enabled" if self.enabled else "disabled"
        )
        self.latest_odom = None
        self.odom_rx_time = rospy.Time(0)
        self.odom_history = deque()
        self.latest_preset = None
        self.preset_rx_time = rospy.Time(0)
        self.tag_position = None
        self.tag_error_world = None
        self.tag_rx_time = rospy.Time(0)
        self.detection_count = 0
        self.target_count = 0
        self.connected = False
        self.armed = False
        self.mode = ""
        self.landed_state = 0

        self.trajectory_id = 0
        self.command_frame = "world"
        self.command_kx = [0.0, 0.0, 0.0]
        self.command_kv = [0.0, 0.0, 0.0]
        self.home = None
        self.home_yaw = None
        self.command_position = None
        self.command_velocity = [0.0, 0.0, 0.0]
        self.command_yaw = 0.0
        self.last_control_time = rospy.Time(0)
        self.settle_since = None
        self.lift_goal_z = None
        self.mission_cruise_z = None
        self.return_start = None
        self.return_distance = 0.0
        self.return_progress = 0.0
        self.return_current_speed = 0.0
        self.gripper_request_started = False
        self.mission_complete = False
        self.pickup_succeeded = False
        self.recovery_start = None
        self.reacquire_goal_z = None
        self.abort_reason = ""
        self.tag_measurement_age = float("inf")
        self.raw_tag_error_world = [0.0, 0.0, 0.0]
        self.tag_motion_compensation = [0.0, 0.0, 0.0]

        preset_topic = rospy.get_param("~preset_command_topic", "/drone0/planning/preset_cmd")
        output_topic = rospy.get_param("~command_output_topic", "/drone0/planning/cmd")
        odom_topic = rospy.get_param("~odom_topic", "/odom/remap")
        tag_topic = rospy.get_param("~tag_position_topic", "/apriltag_gripper/tag_position")
        state_topic = rospy.get_param("~mavros_state_topic", "/mavros/state")
        extended_state_topic = rospy.get_param(
            "~mavros_extended_state_topic", "/mavros/extended_state"
        )
        land_topic = rospy.get_param(
            "~takeoff_land_topic", "/drone0/ctrl_node/takeoff_land"
        )
        self.gripper_service_name = rospy.get_param(
            "~gripper_service", "/apriltag_gripper/set_gripper"
        )

        self.command_pub = rospy.Publisher(output_topic, PositionCommand, queue_size=10)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.complete_pub = rospy.Publisher("~mission_complete", Bool, queue_size=1, latch=True)
        self.pickup_pub = rospy.Publisher("~pickup_succeeded", Bool, queue_size=1, latch=True)
        self.error_pub = rospy.Publisher("~alignment_error", PointStamped, queue_size=5)
        self.tag_measurement_age_pub = rospy.Publisher(
            "~tag_measurement_age_s", Float32, queue_size=5
        )
        self.horizontal_error_pub = rospy.Publisher(
            "~horizontal_error_m", Float32, queue_size=5
        )
        self.raw_error_pub = rospy.Publisher(
            "~raw_alignment_error", PointStamped, queue_size=5
        )
        self.motion_compensation_pub = rospy.Publisher(
            "~motion_compensation", PointStamped, queue_size=5
        )
        self.descent_scale_pub = rospy.Publisher(
            "~descent_scale", Float32, queue_size=5
        )
        self.land_pub = rospy.Publisher(land_topic, TakeoffLand, queue_size=1)
        self.gripper_client = rospy.ServiceProxy(self.gripper_service_name, SetBool)

        rospy.Subscriber(preset_topic, PositionCommand, self._preset_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(odom_topic, Odometry, self._odom_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(tag_topic, PointStamped, self._tag_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(state_topic, State, self._mavros_state_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(
            extended_state_topic, ExtendedState, self._extended_state_cb,
            queue_size=1, tcp_nodelay=True
        )

        rospy.Service("~enable", SetBool, self._enable_service)
        rospy.Service("~abort", Trigger, self._abort_service)
        rospy.Service("~reset", Trigger, self._reset_service)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._control_timer)
        rospy.Timer(rospy.Duration(1.0), self._status_timer)

        self._publish_state_locked()
        rospy.logwarn(
            "apriltag_mission ready: camera target=%.2f m, object=(forward %.2f, height %.2f) m, takeover=%s",
            self.target_z, self.object_forward, self.object_height,
            self.enabled,
        )

    def _set_status_locked(self, text):
        self.status = text
        self.status_pub.publish(String(data=text))

    def _publish_state_locked(self):
        self.state_pub.publish(String(data=self.state))
        self.complete_pub.publish(Bool(data=self.mission_complete))
        self.pickup_pub.publish(Bool(data=self.pickup_succeeded))
        self.status_pub.publish(String(data=self.status))

    def _status_timer(self, _event):
        with self.lock:
            self._publish_state_locked()

    def _odom_fresh_locked(self, now):
        return self.latest_odom is not None and (now - self.odom_rx_time).to_sec() <= self.odom_timeout

    def _vehicle_ready_locked(self):
        if self.require_connected and not self.connected:
            return False
        if self.require_armed_offboard and (not self.armed or self.mode != "OFFBOARD"):
            return False
        return True

    def _odom_xyz_locked(self):
        p = self.latest_odom.pose.pose.position
        return [float(p.x), float(p.y), float(p.z)]

    def _odom_velocity_locked(self):
        v = self.latest_odom.twist.twist.linear
        return [float(v.x), float(v.y), float(v.z)]

    def _camera_optical_to_body_flu(self, point):
        # D435 optical: +x image-right, +y image-down/tail, +z lens-forward/down.
        # ROS body FLU: +x nose, +y left, +z up.
        nominal = [-float(point[1]), -float(point[0]), -float(point[2])]
        return rotate_rpy(nominal, *self.mount_rpy)

    def _desired_tag_from_gripper_world_locked(self):
        # Object from tag is (forward, right, up). Therefore tag from the
        # gripper/object centre is (aft, left, down), expressed here in FLU.
        desired_flu = [-self.object_forward, self.object_right, -self.object_height]
        yaw = self.home_yaw
        if yaw is None:
            yaw = yaw_from_quaternion(self.latest_odom.pose.pose.orientation)
        return yaw_rotate_flu(desired_flu, yaw)

    def _tag_error_world_locked(self, point, measurement_odom=None):
        if measurement_odom is None:
            measurement_odom = self.latest_odom
        tag_from_camera_flu = self._camera_optical_to_body_flu(point)
        tag_from_gripper_flu = [
            tag_from_camera_flu[i] - self.camera_to_gripper_flu[i]
            for i in range(3)
        ]
        measured_world = rotate_vector(
            measurement_odom.pose.pose.orientation, tag_from_gripper_flu
        )
        desired_world = self._desired_tag_from_gripper_world_locked()
        error = [measured_world[i] - desired_world[i] for i in range(3)]
        # The image can be hundreds of milliseconds old.  Reconstruct the tag
        # location at capture time, then remove aircraft translation since that
        # instant. This avoids chasing a stale image with the current odometry.
        capture = measurement_odom.pose.pose.position
        current = self.latest_odom.pose.pose.position
        translation = [
            float(current.x - capture.x),
            float(current.y - capture.y),
            float(current.z - capture.z),
        ]
        self.raw_tag_error_world = list(error)
        self.tag_motion_compensation = list(translation)
        return [error[i] - translation[i] for i in range(3)]

    def _odom_at_stamp_locked(self, stamp):
        if stamp.is_zero() or not self.odom_history:
            return self.latest_odom
        target = stamp.to_sec()
        nearest_time, nearest_msg = min(
            self.odom_history, key=lambda sample: abs(sample[0] - target)
        )
        if abs(nearest_time - target) > self.tag_odom_sync_tolerance:
            return None
        return nearest_msg

    def _mavros_state_cb(self, msg):
        with self.lock:
            was_armed = self.armed
            self.connected = bool(msg.connected)
            self.armed = bool(msg.armed)
            self.mode = str(msg.mode)
            # Normal mission completion intentionally waits in HOLD_HOME until
            # the operator runs takeland.sh.  Once PX4 acknowledges AUTO.LAND,
            # stop publishing position commands and monitor landing instead of
            # misclassifying the deliberate OFFBOARD exit as FAILSAFE_HOLD.
            if (
                self.state == HOLD_HOME
                and self.armed
                and self.mode == "AUTO.LAND"
            ):
                self.state = LANDING
                self._set_status_locked(
                    "operator LAND accepted after HOLD_HOME; waiting for PX4 disarm"
                )
                self._publish_state_locked()
                rospy.logwarn(
                    "apriltag_mission: operator AUTO.LAND confirmed after HOLD_HOME"
                )
            if was_armed and not self.armed:
                self._reset_after_landing_locked("PX4 disarmed")

    def _extended_state_cb(self, msg):
        with self.lock:
            self.landed_state = int(msg.landed_state)
            self._reset_after_landing_locked("PX4 confirmed ON_GROUND")

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        if not finite((p.x, p.y, p.z, v.x, v.y, v.z)):
            rospy.logerr_throttle(1.0, "apriltag_mission: rejected non-finite odometry")
            return
        try:
            yaw_from_quaternion(msg.pose.pose.orientation)
        except ValueError as exc:
            rospy.logerr_throttle(1.0, "apriltag_mission: %s", exc)
            return
        with self.lock:
            self.latest_odom = msg
            now = rospy.Time.now()
            self.odom_rx_time = now
            stamp = msg.header.stamp if not msg.header.stamp.is_zero() else now
            stamp_sec = stamp.to_sec()
            if self.odom_history and stamp_sec <= self.odom_history[-1][0]:
                self.odom_history.clear()
            self.odom_history.append((stamp_sec, msg))
            cutoff = stamp_sec - self.odom_history_duration
            while self.odom_history and self.odom_history[0][0] < cutoff:
                self.odom_history.popleft()

    def _preset_cb(self, msg):
        ready_flag = getattr(PositionCommand, "TRAJECTORY_STATUS_READY", 1)
        relay = None
        with self.lock:
            # An active/failsafe mission owns the command stream. Never attach
            # a new trajectory id to an old hold position: that made a later
            # flight accept the previous flight's captured FAILSAFE point.
            if self.state != FOLLOW_PRESET:
                if (
                    int(msg.trajectory_id) != 0
                    and int(msg.trajectory_id) != self.trajectory_id
                ):
                    rospy.logerr_throttle(
                        1.0,
                        "apriltag_mission: ignoring new trajectory %d while state=%s; land/reset first",
                        int(msg.trajectory_id), self.state,
                    )
                return
            now = rospy.Time.now()
            self.latest_preset = msg
            self.preset_rx_time = now
            valid = int(msg.trajectory_id) != 0 and int(msg.trajectory_flag) == ready_flag
            if valid and int(msg.trajectory_id) != self.trajectory_id:
                self.trajectory_id = int(msg.trajectory_id)
                self.command_frame = msg.header.frame_id or "world"
                self.command_kx = list(msg.kx)
                self.command_kv = list(msg.kv)
                # The first command is generated at the frozen trajectory origin.
                self.home = [msg.position.x, msg.position.y, msg.position.z]
                # The planner command carries the yaw captured when TAKEOFF was
                # requested.  Do not replace it with the possibly drifted yaw
                # measured after hover stabilisation.
                self.home_yaw = float(msg.yaw)
                self.command_yaw = self.home_yaw
                self.mission_complete = False
                self.pickup_succeeded = False
                self.detection_count = 0
                self.target_count = 0
                rospy.loginfo(
                    "apriltag_mission: recorded trajectory %d origin (%.3f, %.3f, %.3f), locked yaw %.1f deg",
                    self.trajectory_id, self.home[0], self.home[1], self.home[2],
                    math.degrees(self.home_yaw),
                )
            if self.state == FOLLOW_PRESET:
                relay = copy.deepcopy(msg)
                if self.home_yaw is not None:
                    relay.yaw = self.home_yaw
                    relay.yaw_dot = 0.0
        # Relay in the subscriber callback to retain the original 200 Hz timing
        # and to avoid extending a trajectory after its generator has stopped.
        if relay is not None:
            self.command_pub.publish(relay)

    def _tag_cb(self, msg):
        p = msg.point
        if not finite((p.x, p.y, p.z)) or p.z <= 0.05:
            return
        now = rospy.Time.now()
        error_values = None
        with self.lock:
            stamp = msg.header.stamp if not msg.header.stamp.is_zero() else now
            measurement_age = (now - stamp).to_sec()
            if (
                not math.isfinite(measurement_age)
                or measurement_age < -0.05
                or measurement_age > self.max_tag_measurement_age
            ):
                rospy.logwarn_throttle(
                    1.0,
                    "apriltag_mission: rejected stale/future tag measurement age %.3f s",
                    measurement_age,
                )
                return
            measurement_odom = self._odom_at_stamp_locked(stamp)
            if measurement_odom is None:
                rospy.logwarn_throttle(
                    1.0,
                    "apriltag_mission: no odometry within %.3f s of tag image",
                    self.tag_odom_sync_tolerance,
                )
                return
            raw = [float(p.x), float(p.y), float(p.z)]
            if self.tag_position is not None and (now - self.tag_rx_time).to_sec() <= self.tag_timeout:
                jump = math.sqrt(sum(
                    (raw[i] - self.tag_position[i]) ** 2 for i in range(3)
                ))
                if jump > self.max_tag_jump:
                    rospy.logwarn_throttle(
                        1.0, "apriltag_mission: rejected %.3f m AprilTag pose jump", jump
                    )
                    return
            if (now - self.tag_rx_time).to_sec() > self.tag_timeout:
                self.detection_count = 0
            self.tag_position = raw
            self.tag_rx_time = now
            self.tag_measurement_age = max(0.0, measurement_age)
            self.detection_count += 1
            if self.latest_odom is not None:
                error_values = self._tag_error_world_locked(raw, measurement_odom)
                self.tag_error_world = error_values
            if self.state == TRACK_TAG and error_values is not None:
                at_target = (
                    math.hypot(error_values[0], error_values[1])
                    <= self.horizontal_tolerance
                    and abs(error_values[2]) <= self.distance_tolerance
                )
                self.target_count = self.target_count + 1 if at_target else 0
        if error_values is not None:
            error = PointStamped()
            error.header.stamp = msg.header.stamp
            error.header.frame_id = self.command_frame
            error.point.x, error.point.y, error.point.z = error_values
            self.error_pub.publish(error)
            raw_error = PointStamped()
            raw_error.header = error.header
            raw_error.point.x, raw_error.point.y, raw_error.point.z = (
                self.raw_tag_error_world
            )
            self.raw_error_pub.publish(raw_error)
            compensation = PointStamped()
            compensation.header = error.header
            compensation.point.x, compensation.point.y, compensation.point.z = (
                self.tag_motion_compensation
            )
            self.motion_compensation_pub.publish(compensation)
            self.tag_measurement_age_pub.publish(
                Float32(data=self.tag_measurement_age)
            )
            self.horizontal_error_pub.publish(
                Float32(data=math.hypot(error_values[0], error_values[1]))
            )

    def _enable_service(self, req):
        with self.lock:
            if req.data and self.state == FAILSAFE_HOLD:
                return SetBoolResponse(
                    success=False,
                    message="FAILSAFE_HOLD is latched; land and reset before enabling a new mission",
                )
            if (
                req.data
                and self.state == LANDED
                and not self.armed
                and self.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
            ):
                # Preserve the previous result latch until the operator starts
                # preparing the next flight, then create a clean flight epoch.
                self._clear_mission_locked(
                    "previous landed mission cleared; AprilTag takeover enabled"
                )
                self.enabled = True
                return SetBoolResponse(success=True, message=self.status)
            self.enabled = bool(req.data)
            if self.state in (LANDING, LANDED):
                text = "landing already committed; takeover flag updated without cancelling LAND"
                self._set_status_locked(text)
            elif not self.enabled and self.state not in (FOLLOW_PRESET, HOLD_HOME, FAILSAFE_HOLD):
                self._enter_failsafe_locked("mission disabled during active pickup")
                text = self.status
            else:
                text = "AprilTag mission takeover enabled" if self.enabled else "AprilTag mission takeover disabled"
                self._set_status_locked(text)
        return SetBoolResponse(success=True, message=text)

    def _abort_service(self, _req):
        with self.lock:
            self.enabled = False
            if self.state in (LANDING, LANDED):
                text = "LAND remains active; mission abort cannot cancel PX4 landing"
                self._set_status_locked(text)
            elif self.state == FOLLOW_PRESET:
                text = "mission takeover disabled; preset relay continues"
                self._set_status_locked(text)
            else:
                self._enter_failsafe_locked("operator abort; holding current position")
                text = self.status
        return TriggerResponse(success=True, message=text)

    def _clear_mission_locked(self, status):
        self.state = FOLLOW_PRESET
        self.enabled = False
        self.trajectory_id = 0
        self.latest_preset = None
        self.preset_rx_time = rospy.Time(0)
        self.home = None
        self.home_yaw = None
        self.command_position = None
        self.command_velocity = [0.0, 0.0, 0.0]
        self.detection_count = 0
        self.target_count = 0
        self.tag_position = None
        self.tag_error_world = None
        self.tag_rx_time = rospy.Time(0)
        self.gripper_request_started = False
        self.mission_complete = False
        self.pickup_succeeded = False
        self.mission_cruise_z = None
        self.return_start = None
        self.return_distance = 0.0
        self.return_progress = 0.0
        self.return_current_speed = 0.0
        self.settle_since = None
        self.recovery_start = None
        self.reacquire_goal_z = None
        self.abort_reason = ""
        self._set_status_locked(status)
        self._publish_state_locked()

    def _reset_after_landing_locked(self, reason):
        if (
            self.state not in (FOLLOW_PRESET, LANDING, LANDED)
            and not self.armed
            and self.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
        ):
            rospy.logwarn(
                "apriltag_mission: %s; clearing the completed/failed flight epoch",
                reason,
            )
            self._clear_mission_locked(
                "landed/disarmed; previous mission cleared; takeover disabled"
            )

    def _reset_service(self, _req):
        with self.lock:
            if self.armed and self.mode == "OFFBOARD":
                return TriggerResponse(
                    success=False,
                    message="reset refused while armed in OFFBOARD; abort holds safely, reset after landing",
                )
            self._clear_mission_locked("mission reset; takeover disabled")
        return TriggerResponse(success=True, message=self.status)

    def _enter_failsafe_locked(self, reason):
        if self.latest_odom is not None:
            self.command_position = self._odom_xyz_locked()
        self.command_velocity = [0.0, 0.0, 0.0]
        self.state = FAILSAFE_HOLD
        self.enabled = False
        self.settle_since = None
        self._set_status_locked("FAILSAFE_HOLD: " + reason)
        self.state_pub.publish(String(data=self.state))
        rospy.logerr("apriltag_mission: %s", self.status)

    def _start_tracking_locked(self):
        self.state = TRACK_TAG
        # Preserve the planner reference at handover.  Resetting the reference
        # to the measured pose creates a position-command step whenever the
        # aircraft has tracking error.  On the real controller that step also
        # changes thrust abruptly and was the main source of the observed
        # pitch/roll transient and altitude excursion.
        if self.latest_preset is not None:
            self.command_position = [
                float(self.latest_preset.position.x),
                float(self.latest_preset.position.y),
                float(self.latest_preset.position.z),
            ]
            self.command_velocity = [
                float(self.latest_preset.velocity.x),
                float(self.latest_preset.velocity.y),
                float(self.latest_preset.velocity.z),
            ]
        else:
            # The takeover predicate requires a fresh preset, so this is only
            # a defensive fallback for future callers.
            self.command_position = self._odom_xyz_locked()
            self.command_velocity = self._odom_velocity_locked()
        if self.home_yaw is None:
            self.home_yaw = yaw_from_quaternion(self.latest_odom.pose.pose.orientation)
        self.command_yaw = self.home_yaw
        self.mission_cruise_z = float(self.latest_preset.position.z)
        self.target_count = 0
        self.settle_since = None
        self._set_status_locked("tag acquired; preset trajectory intercepted")
        self.state_pub.publish(String(data=self.state))
        rospy.logwarn("apriltag_mission: tag acquired, taking over trajectory %d", self.trajectory_id)

    def _start_reacquire_locked(self, now):
        self.state = REACQUIRE_TAG
        self.command_position = self._odom_xyz_locked()
        self.command_velocity = [0.0, 0.0, 0.0]
        self.target_count = 0
        self.detection_count = 0
        self.recovery_start = now
        current_z = self.command_position[2]
        upper_bound = current_z + self.reacquire_climb
        if self.mission_cruise_z is not None:
            upper_bound = min(upper_bound, max(current_z, self.mission_cruise_z))
        self.reacquire_goal_z = upper_bound
        self._set_status_locked(
            "AprilTag lost; holding then climbing up to {:.2f} m for {:.1f} s reacquisition".format(
                self.reacquire_goal_z, self.reacquire_hold
            )
        )
        self.state_pub.publish(String(data=self.state))
        rospy.logwarn("apriltag_mission: tag lost; entering recoverable hold")

    def _start_abort_ascent_locked(self, reason):
        current = self._odom_xyz_locked()
        self.state = ABORT_ASCEND
        self.abort_reason = reason
        self.pickup_succeeded = False
        self.command_position = current
        self.command_velocity = [0.0, 0.0, 0.0]
        recovery_goal = current[2] + self.lost_tag_climb
        if self.mission_cruise_z is not None:
            recovery_goal = max(recovery_goal, self.mission_cruise_z)
        self.lift_goal_z = recovery_goal
        self.settle_since = None
        self._set_status_locked(
            "pickup aborted: {}; ascending safely to world z={:.2f} m before return".format(
                reason, self.lift_goal_z
            )
        )
        self.state_pub.publish(String(data=self.state))
        rospy.logerr("apriltag_mission: %s", self.status)

    def _start_gripper_close_locked(self):
        self.state = CLOSING
        self.command_position = self._odom_xyz_locked()
        self.command_velocity = [0.0, 0.0, 0.0]
        self.gripper_request_started = True
        self._set_status_locked(
            "world-compensated target stable at camera distance {:.2f} m; closing gripper once".format(
                self.target_z
            )
        )
        self.state_pub.publish(String(data=self.state))
        threading.Thread(target=self._gripper_worker, name="mission_gripper_close", daemon=True).start()

    def _gripper_worker(self):
        try:
            rospy.wait_for_service(self.gripper_service_name, timeout=2.0)
            response = self.gripper_client(True)
            success = bool(response.success)
            message = str(response.message)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            success = False
            message = str(exc)

        with self.lock:
            if self.state != CLOSING:
                return
            if not success:
                self._enter_failsafe_locked("gripper close failed: " + message)
                return
            if self.gripper_dry_run and not self.allow_dry_run_sequence:
                self.pickup_succeeded = False
                self._enter_failsafe_locked(
                    "dry-run gripper close confirmed; ascent/return intentionally inhibited"
                )
                return
            self.pickup_succeeded = True
            self._set_status_locked(
                "gripper close accepted; holding {:.1f} s for servo travel".format(
                    self.gripper_settle
                )
            )

        # Keep publishing the captured hold point while the servo finishes its
        # mechanical travel; do not begin lifting on MAVLink ACK alone.
        rospy.sleep(self.gripper_settle)

        with self.lock:
            if self.state != CLOSING:
                return
            if self.latest_odom is None:
                self._enter_failsafe_locked("gripper closed but odometry is unavailable")
                return
            self.command_position = self._odom_xyz_locked()
            self.command_velocity = [0.0, 0.0, 0.0]
            relative_lift_goal = self.command_position[2] + self.lift_height
            if self.return_to_preset_altitude and self.mission_cruise_z is not None:
                # Never descend immediately after grasp. On the intended flat
                # task this is exactly the configured 1.0 m preset cruise altitude.
                self.lift_goal_z = max(self.command_position[2], self.mission_cruise_z)
            else:
                self.lift_goal_z = relative_lift_goal
            self.state = ASCEND
            self.settle_since = None
            self._set_status_locked(
                "gripper closed; ascending to world z={:.2f} m at {:.2f} m/s".format(
                    self.lift_goal_z, self.lift_speed
                )
            )
            self.state_pub.publish(String(data=self.state))

    def _slew_velocity_locked(self, desired, dt):
        step = self.max_tracking_accel * dt
        for index in range(3):
            delta = desired[index] - self.command_velocity[index]
            self.command_velocity[index] += clamp(delta, step)

    def _integrate_tracking_reference_locked(self, dt):
        """Integrate tracking velocity without snapping an existing reference.

        A conventional ``odom + clamp(reference - odom)`` limiter moves the
        reference when the vehicle moves in the wrong direction.  That turns a
        disturbance into a moving target and can remove the very position
        error needed to recover.  Here a reference already outside the lead
        envelope is retained, while only further motion away from odometry is
        rejected.  Once the aircraft catches up, the configured envelope
        automatically becomes active again.
        """
        odom = self._odom_xyz_locked()
        current = list(self.command_position)
        proposed = [
            current[i] + self.command_velocity[i] * dt for i in range(3)
        ]

        current_xy = math.hypot(current[0] - odom[0], current[1] - odom[1])
        proposed_xy = math.hypot(proposed[0] - odom[0], proposed[1] - odom[1])
        if (
            proposed_xy > self.max_xy_reference_lead
            and proposed_xy > current_xy
        ):
            if current_xy < self.max_xy_reference_lead and proposed_xy > 1.0e-9:
                scale = self.max_xy_reference_lead / proposed_xy
                proposed[0] = odom[0] + (proposed[0] - odom[0]) * scale
                proposed[1] = odom[1] + (proposed[1] - odom[1]) * scale
            else:
                proposed[0], proposed[1] = current[0], current[1]

        current_z = abs(current[2] - odom[2])
        proposed_z = abs(proposed[2] - odom[2])
        if proposed_z > self.max_z_reference_lead and proposed_z > current_z:
            if current_z < self.max_z_reference_lead:
                proposed[2] = odom[2] + clamp(
                    proposed[2] - odom[2], self.max_z_reference_lead
                )
            else:
                proposed[2] = current[2]

        self.command_position = proposed

    def _tracking_step_locked(self, now, dt):
        age = (now - self.tag_rx_time).to_sec()
        if age > self.tag_abort_timeout:
            self._start_reacquire_locked(now)
            return

        desired = [0.0, 0.0, 0.0]
        descent_scale = 0.0
        if age <= self.tag_timeout and self.tag_error_world is not None:
            ex, ey, ez = self.tag_error_world
            horizontal_error = math.hypot(ex, ey)
            near = horizontal_error <= self.near_distance and abs(ez) <= self.near_distance
            xy_limit = self.near_xy_speed if near else self.max_xy_speed
            z_limit = self.near_z_speed if near else self.max_z_speed
            desired[0] = 0.0 if abs(ex) <= self.deadband else clamp(self.kp_xy * ex, xy_limit)
            desired[1] = 0.0 if abs(ey) <= self.deadband else clamp(self.kp_xy * ey, xy_limit)
            # Use a continuous descent envelope instead of the old 8 cm binary
            # gate. This prevents stop/go behaviour while preserving clearance
            # when the tag is grossly misaligned.
            # Use the capture tolerance as vertical hysteresis.  Depth noise
            # inside the 45 cm band must not cause repeated up/down commands.
            if abs(ez) > self.distance_tolerance:
                if ez >= 0.0:
                    desired[2] = clamp(self.kp_z * ez, z_limit)
                    descent_scale = 1.0
                else:
                    descent_scale = descent_speed_scale(
                        horizontal_error,
                        self.descent_full_alignment,
                        self.descent_stop_alignment,
                    )
                    desired[2] = clamp(
                        self.kp_z * ez, z_limit * descent_scale
                    )
        else:
            # Target-frame stability must be consecutive. Never carry close
            # eligibility across even a short perception outage.
            self.target_count = 0

        self.descent_scale_pub.publish(Float32(data=descent_scale))

        self._slew_velocity_locked(desired, dt)
        self._integrate_tracking_reference_locked(dt)

        command_speed = math.sqrt(sum(value * value for value in self.command_velocity))
        odom_velocity = self._odom_velocity_locked()
        odom_horizontal_speed = math.hypot(odom_velocity[0], odom_velocity[1])
        odom_vertical_speed = abs(odom_velocity[2])
        if (
            age <= self.tag_timeout
            and self.target_count >= self.target_required
            and command_speed <= self.close_max_command_speed
            and odom_horizontal_speed <= self.close_max_horizontal_speed
            and odom_vertical_speed <= self.close_max_vertical_speed
            and not self.gripper_request_started
        ):
            self._start_gripper_close_locked()

    def _reacquire_step_locked(self, now, dt):
        age = (now - self.tag_rx_time).to_sec()
        if age <= self.tag_timeout and self.detection_count >= self.reacquire_required:
            self.state = TRACK_TAG
            self.command_position = self._odom_xyz_locked()
            self.command_velocity = [0.0, 0.0, 0.0]
            self.target_count = 0
            self.recovery_start = None
            self.reacquire_goal_z = None
            self._set_status_locked("AprilTag reacquired; resuming smooth world-frame tracking")
            self.state_pub.publish(String(data=self.state))
            rospy.logwarn("apriltag_mission: tag reacquired; tracking resumed")
            return
        self.command_velocity = [0.0, 0.0, 0.0]
        elapsed = 0.0 if self.recovery_start is None else (
            now - self.recovery_start
        ).to_sec()
        if (
            elapsed >= self.reacquire_initial_hold
            and self.reacquire_goal_z is not None
            and self.command_position[2] < self.reacquire_goal_z
        ):
            step = min(
                self.reacquire_goal_z - self.command_position[2],
                self.reacquire_climb_speed * dt,
            )
            self.command_position[2] += step
            self.command_velocity[2] = self.reacquire_climb_speed if step > 0.0 else 0.0
        if self.recovery_start is not None and (
            now - self.recovery_start
        ).to_sec() >= self.reacquire_hold:
            self._start_abort_ascent_locked("AprilTag reacquisition timed out")

    def _ascend_step_locked(self, now, dt):
        remaining = self.lift_goal_z - self.command_position[2]
        if remaining > 0.0:
            step = min(remaining, self.lift_speed * dt)
            self.command_position[2] += step
            self.command_velocity = [0.0, 0.0, self.lift_speed if remaining > step else 0.0]
        else:
            self.command_velocity = [0.0, 0.0, 0.0]

        odom = self._odom_xyz_locked()
        velocity = self._odom_velocity_locked()
        reached = (
            abs(odom[2] - self.lift_goal_z) <= self.lift_tolerance
            and abs(velocity[2]) <= 0.12
            and remaining <= 0.0
        )
        if reached:
            if self.settle_since is None:
                self.settle_since = now
            elif (now - self.settle_since).to_sec() >= self.lift_settle:
                self.return_start = list(self.command_position)
                dx = self.home[0] - self.return_start[0]
                dy = self.home[1] - self.return_start[1]
                self.return_distance = math.hypot(dx, dy)
                self.return_progress = 0.0
                self.return_current_speed = 0.0
                self.state = RETURN_HOME
                self.settle_since = None
                prefix = "pickup lift complete" if self.pickup_succeeded else "safe abort climb complete"
                self._set_status_locked(
                    "{}; returning {:.2f} m straight to origin x/y".format(
                        prefix, self.return_distance
                    )
                )
                self.state_pub.publish(String(data=self.state))
        else:
            self.settle_since = None

    def _return_step_locked(self, now, dt):
        if self.return_distance > 1e-6:
            ux = (self.home[0] - self.return_start[0]) / self.return_distance
            uy = (self.home[1] - self.return_start[1]) / self.return_distance
            remaining = max(0.0, self.return_distance - self.return_progress)
            # Acceleration-limited trapezoidal return profile.  The square-root
            # bound is the braking speed that can stop at home with the same
            # acceleration, avoiding a step at both departure and arrival.
            braking_speed = math.sqrt(max(0.0, 2.0 * self.return_accel * remaining))
            target_speed = min(self.return_speed, braking_speed)
            speed_step = self.return_accel * dt
            if self.return_current_speed < target_speed:
                self.return_current_speed = min(
                    target_speed, self.return_current_speed + speed_step
                )
            else:
                self.return_current_speed = max(
                    target_speed, self.return_current_speed - speed_step
                )
            self.return_progress = min(
                self.return_distance,
                self.return_progress + self.return_current_speed * dt,
            )
            self.command_position[0] = self.return_start[0] + ux * self.return_progress
            self.command_position[1] = self.return_start[1] + uy * self.return_progress
            moving = self.return_progress < self.return_distance
            self.command_velocity = [ux * self.return_current_speed if moving else 0.0,
                                     uy * self.return_current_speed if moving else 0.0, 0.0]
        else:
            self.command_position[0] = self.home[0]
            self.command_position[1] = self.home[1]
            self.command_velocity = [0.0, 0.0, 0.0]

        odom = self._odom_xyz_locked()
        velocity = self._odom_velocity_locked()
        horizontal_error = math.hypot(odom[0] - self.home[0], odom[1] - self.home[1])
        horizontal_speed = math.hypot(velocity[0], velocity[1])
        reference_done = self.return_progress >= self.return_distance
        reached = reference_done and horizontal_error <= self.return_tolerance and horizontal_speed <= 0.15
        if reached:
            if self.settle_since is None:
                self.settle_since = now
            elif (now - self.settle_since).to_sec() >= self.return_settle:
                self.command_position[0] = self.home[0]
                self.command_position[1] = self.home[1]
                self.command_velocity = [0.0, 0.0, 0.0]
                if self.auto_land_on_return:
                    self._start_landing_locked()
                else:
                    self.state = HOLD_HOME
                    self.mission_complete = True
                    result = "pickup complete" if self.pickup_succeeded else "pickup aborted"
                    self._set_status_locked(
                        "{}; holding above origin at cruise altitude".format(result)
                    )
                    self._publish_state_locked()
                    rospy.logwarn("apriltag_mission: pickup and return complete")
        else:
            self.settle_since = None

    def _start_landing_locked(self):
        if self.land_pub.get_num_connections() < 1:
            self._enter_failsafe_locked(
                "return complete but ctrl_node LAND subscriber is unavailable"
            )
            return
        msg = TakeoffLand()
        msg.takeoff_land_cmd = TakeoffLand.LAND
        self.state = LANDING
        self.command_velocity = [0.0, 0.0, 0.0]
        result = "pickup complete" if self.pickup_succeeded else "pickup aborted safely"
        self._set_status_locked("{}; return complete; PX4 AUTO.LAND requested".format(result))
        self.state_pub.publish(String(data=self.state))
        self.land_pub.publish(msg)
        rospy.logwarn("apriltag_mission: return complete, LAND requested")

    def _make_command_locked(self, now):
        msg = PositionCommand()
        msg.header.stamp = now
        msg.header.frame_id = self.command_frame
        msg.position.x, msg.position.y, msg.position.z = self.command_position
        msg.velocity.x, msg.velocity.y, msg.velocity.z = self.command_velocity
        msg.acceleration.x = msg.acceleration.y = msg.acceleration.z = 0.0
        msg.jerk.x = msg.jerk.y = msg.jerk.z = 0.0
        msg.yaw = self.command_yaw
        msg.yaw_dot = 0.0
        msg.kx = self.command_kx
        msg.kv = self.command_kv
        msg.trajectory_id = self.trajectory_id
        msg.trajectory_flag = getattr(PositionCommand, "TRAJECTORY_STATUS_READY", 1)
        return msg

    def _control_timer(self, event):
        now = rospy.Time.now()
        output = None
        with self.lock:
            if self.last_control_time.is_zero():
                dt = 1.0 / self.rate_hz
            else:
                dt = max(0.001, min(0.05, (now - self.last_control_time).to_sec()))
            self.last_control_time = now

            if self.state == LANDING:
                if not self.armed and self.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                    self.state = LANDED
                    self.mission_complete = True
                    self._set_status_locked("mission complete; PX4 disarmed on ground")
                    self._publish_state_locked()
                    rospy.logwarn("apriltag_mission: landing and disarm confirmed")
                return
            if self.state == LANDED:
                return

            if self.state == FOLLOW_PRESET:
                tag_fresh = self.tag_position is not None and (now - self.tag_rx_time).to_sec() <= self.tag_timeout
                preset_fresh = self.latest_preset is not None and (now - self.preset_rx_time).to_sec() <= self.preset_timeout
                if (
                    self.enabled and self.home is not None and self.trajectory_id != 0
                    and self.detection_count >= self.takeover_required
                    and tag_fresh and preset_fresh and self._odom_fresh_locked(now)
                    and self._vehicle_ready_locked()
                ):
                    self._start_tracking_locked()
                else:
                    return

            if not self._odom_fresh_locked(now):
                if self.state != FAILSAFE_HOLD:
                    self._enter_failsafe_locked("odometry stale; command stream stopped")
                rospy.logerr_throttle(1.0, "apriltag_mission: active mission has stale odometry; command stream stopped")
                return
            if not self._vehicle_ready_locked():
                if self.state != FAILSAFE_HOLD:
                    self._enter_failsafe_locked("vehicle left connected/armed/OFFBOARD flight state")
                return

            if self.state == TRACK_TAG:
                self._tracking_step_locked(now, dt)
            elif self.state == REACQUIRE_TAG:
                self._reacquire_step_locked(now, dt)
            elif self.state in (ASCEND, ABORT_ASCEND):
                self._ascend_step_locked(now, dt)
            elif self.state == RETURN_HOME:
                self._return_step_locked(now, dt)
            elif self.state in (CLOSING, HOLD_HOME, FAILSAFE_HOLD):
                self.command_velocity = [0.0, 0.0, 0.0]

            if self.command_position is not None and self.trajectory_id != 0:
                output = self._make_command_locked(now)

        if output is not None:
            self.command_pub.publish(output)


def main():
    rospy.init_node("apriltag_mission")
    try:
        AprilTagTrajectoryMission()
    except Exception as exc:
        rospy.logfatal("apriltag_mission initialization failed: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
