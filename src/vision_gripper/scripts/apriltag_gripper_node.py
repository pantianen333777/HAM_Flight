#!/usr/bin/env python3
"""Track one AprilTag, command guarded body velocity, and close a PX4 gripper."""

import math
import threading
import time
from collections import deque

import numpy as np
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PointStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandLong, CommandLongRequest, ParamGet, ParamGetRequest
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


MAV_CMD_DO_SET_ACTUATOR = 187
MAV_RESULT_ACCEPTED = 0


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class AprilTagGripper:
    def __init__(self):
        self.lock = threading.RLock()

        self.tag_id = int(rospy.get_param("~tag_id", 0))
        self.target_distance = float(rospy.get_param("~target_distance_m", 0.45))
        self.distance_tolerance = float(rospy.get_param("~distance_tolerance_m", 0.03))
        self.horizontal_tolerance = float(
            rospy.get_param("~horizontal_tolerance_m", 0.03)
        )
        self.gripper_offset_x = float(rospy.get_param("~gripper_offset_x_m", 0.0))
        self.gripper_offset_y = float(rospy.get_param("~gripper_offset_y_m", 0.088))
        self.stable_required = max(1, int(rospy.get_param("~stable_detections", 5)))
        self.detection_timeout = float(rospy.get_param("~detection_timeout_s", 0.25))
        filter_window = max(1, int(rospy.get_param("~pose_filter_window", 3)))
        if filter_window % 2 == 0:
            filter_window += 1
        self.x_history = deque(maxlen=filter_window)
        self.y_history = deque(maxlen=filter_window)
        self.z_history = deque(maxlen=filter_window)
        self.stamp_history = deque(maxlen=filter_window)

        self.kp_xy = float(rospy.get_param("~lateral_kp", 0.8))
        self.kp_z = float(rospy.get_param("~vertical_kp", 0.8))
        self.max_xy_speed = float(rospy.get_param("~max_lateral_speed_mps", 0.15))
        self.max_z_speed = float(rospy.get_param("~max_vertical_speed_mps", 0.10))
        self.control_deadband = float(rospy.get_param("~control_deadband_m", 0.01))
        self.flight_enabled = bool(rospy.get_param("~flight_control_on_start", False))
        self.auto_close_enabled = bool(rospy.get_param("~auto_close_on_start", False))

        self.main_output = int(rospy.get_param("~main_output", 1))
        self.actuator_index = int(rospy.get_param("~actuator_index", 1))
        if not 1 <= self.actuator_index <= 6:
            raise ValueError("~actuator_index must be in [1, 6]")
        self.expected_output_function = 300 + self.actuator_index
        self.require_mapping = bool(rospy.get_param("~require_output_mapping", True))
        self.require_connected = bool(rospy.get_param("~require_mavros_connected", True))
        self.open_value = float(rospy.get_param("~open_value", 0.85))
        self.close_value = float(rospy.get_param("~close_value", -0.60))
        self.command_cooldown = float(rospy.get_param("~command_cooldown_s", 1.0))
        self.dry_run = bool(rospy.get_param("~dry_run", True))

        self.connected = False
        self.armed = False
        self.mode = ""
        self.last_detection_time = rospy.Time(0)
        self.last_valid_detection_time = rospy.Time(0)
        self.filtered_position = None
        self.stable_count = 0
        self.close_latched = False
        self.command_in_progress = False
        self.last_command_monotonic = -1e9
        self.last_status = "starting"

        command_service = rospy.get_param(
            "~mavros_command_service", "/mavros/cmd/command"
        )
        param_service = rospy.get_param(
            "~mavros_param_get_service", "/mavros/param/get"
        )
        self.command_client = rospy.ServiceProxy(command_service, CommandLong)
        self.param_client = rospy.ServiceProxy(param_service, ParamGet)

        detection_topic = rospy.get_param("~tag_detection_topic", "/tag_detections")
        setpoint_topic = rospy.get_param(
            "~setpoint_topic", "/mavros/setpoint_raw/local"
        )
        rospy.Subscriber(detection_topic, AprilTagDetectionArray, self._detection_cb, queue_size=1)
        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=1)

        self.tag_position_pub = rospy.Publisher("~tag_position", PointStamped, queue_size=5)
        self.error_pub = rospy.Publisher("~gripper_alignment_error", PointStamped, queue_size=5)
        self.distance_pub = rospy.Publisher("~distance_m", Float32, queue_size=5)
        self.slant_range_pub = rospy.Publisher("~slant_range_m", Float32, queue_size=5)
        self.distance_error_pub = rospy.Publisher("~distance_error_m", Float32, queue_size=5)
        self.at_target_pub = rospy.Publisher("~at_target", Bool, queue_size=1)
        self.detected_pub = rospy.Publisher("~detected", Bool, queue_size=1)
        self.detection_latency_pub = rospy.Publisher(
            "~detection_latency_s", Float32, queue_size=5
        )
        self.measurement_age_pub = rospy.Publisher(
            "~measurement_age_s", Float32, queue_size=5
        )
        self.detection_interval_pub = rospy.Publisher(
            "~detection_interval_s", Float32, queue_size=5
        )
        self.velocity_debug_pub = rospy.Publisher(
            "~desired_velocity_body_ned", TwistStamped, queue_size=1
        )
        self.setpoint_pub = rospy.Publisher(setpoint_topic, PositionTarget, queue_size=10)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        # Software-side actuator telemetry.  This proves what was requested and
        # whether PX4 acknowledged it; a feedback-less hobby servo still needs
        # a props-off physical movement check before flight.
        self.commanded_value_pub = rospy.Publisher(
            "~commanded_value", Float32, queue_size=1, latch=True
        )
        self.command_action_pub = rospy.Publisher(
            "~command_action", String, queue_size=1, latch=True
        )
        self.command_accepted_pub = rospy.Publisher(
            "~command_accepted", Bool, queue_size=1, latch=True
        )

        rospy.Service("~enable_flight", SetBool, self._enable_flight_service)
        rospy.Service("~enable_auto_close", SetBool, self._enable_auto_close_service)
        rospy.Service("~set_gripper", SetBool, self._set_gripper_service)
        rospy.Service("~reset", Trigger, self._reset_service)

        rospy.Timer(rospy.Duration(0.05), self._control_timer)
        rospy.Timer(rospy.Duration(1.0), self._status_timer)
        mode = "DRY RUN" if self.dry_run else "LIVE OUTPUT"
        self._set_status(
            "ready; {}; flight={}; auto_close={}".format(
                mode, self.flight_enabled, self.auto_close_enabled
            )
        )

    def _state_cb(self, msg):
        self.connected = bool(msg.connected)
        self.armed = bool(msg.armed)
        self.mode = str(msg.mode)

    def _set_status(self, text):
        self.last_status = text
        self.status_pub.publish(String(data=text))

    def _status_timer(self, _event):
        if rospy.is_shutdown():
            return
        try:
            self.status_pub.publish(String(data=self.last_status))
        except rospy.ROSException:
            pass

    def _enable_flight_service(self, req):
        self.flight_enabled = bool(req.data)
        text = "AprilTag flight control enabled" if req.data else "AprilTag flight control disabled"
        self._set_status(text)
        return SetBoolResponse(success=True, message=text)

    def _enable_auto_close_service(self, req):
        with self.lock:
            self.auto_close_enabled = bool(req.data)
            self.stable_count = 0
            if not req.data:
                self.close_latched = False
        text = "AprilTag auto close enabled" if req.data else "AprilTag auto close disabled"
        self._set_status(text)
        return SetBoolResponse(success=True, message=text)

    def _reset_service(self, _req):
        with self.lock:
            self.stable_count = 0
            self.close_latched = False
            self.x_history.clear()
            self.y_history.clear()
            self.z_history.clear()
            self.stamp_history.clear()
        self._set_status("AprilTag latch and pose filter reset")
        return TriggerResponse(success=True, message=self.last_status)

    def _set_gripper_service(self, req):
        close = bool(req.data)
        ok, message = self._command_gripper(close, "manual service")
        if ok and not close:
            with self.lock:
                self.close_latched = False
                self.stable_count = 0
        return SetBoolResponse(success=ok, message=message)

    def _validate_mapping(self):
        if not self.require_mapping:
            return True, "mapping check disabled"
        parameter = "PWM_MAIN_FUNC{}".format(self.main_output)
        try:
            response = self.param_client(ParamGetRequest(param_id=parameter))
        except (rospy.ServiceException, rospy.ROSException) as exc:
            return False, "cannot read {}: {}".format(parameter, exc)
        if not response.success:
            return False, "PX4 parameter {} is unavailable".format(parameter)
        actual = int(response.value.integer)
        if actual != self.expected_output_function:
            return False, "{}={} but expected {}".format(
                parameter, actual, self.expected_output_function
            )
        return True, "{}={}".format(parameter, actual)

    def _command_gripper(self, close, source):
        with self.lock:
            now = time.monotonic()
            if self.command_in_progress:
                return False, "another gripper command is in progress"
            if now - self.last_command_monotonic < self.command_cooldown:
                return False, "gripper command cooldown is active"
            self.command_in_progress = True
        try:
            action = "close" if close else "open"
            value = self.close_value if close else self.open_value
            self.commanded_value_pub.publish(Float32(data=value))
            self.command_action_pub.publish(String(data=action))
            self.command_accepted_pub.publish(Bool(data=False))
            if self.require_connected and not self.connected:
                return False, "MAVROS is not connected to PX4"
            mapping_ok, mapping_message = self._validate_mapping()
            if not mapping_ok:
                return False, "refusing output: " + mapping_message
            if self.dry_run:
                message = "dry-run {} ({:+.2f}); {}".format(
                    action, value, mapping_message
                )
                rospy.logwarn("apriltag_gripper: %s requested by %s", message, source)
                self._set_status(message)
                return True, message

            values = [math.nan] * 6
            values[self.actuator_index - 1] = value
            request = CommandLongRequest(
                broadcast=False,
                command=MAV_CMD_DO_SET_ACTUATOR,
                confirmation=0,
                param1=values[0], param2=values[1], param3=values[2],
                param4=values[3], param5=values[4], param6=values[5], param7=0.0,
            )
            try:
                response = self.command_client(request)
            except (rospy.ServiceException, rospy.ROSException) as exc:
                return False, "MAVROS command failed: {}".format(exc)
            if not response.success or response.result != MAV_RESULT_ACCEPTED:
                return False, "PX4 rejected {}: MAV_RESULT={}".format(action, response.result)
            message = "gripper {} accepted ({:+.2f}); {}".format(
                action, value, mapping_message
            )
            rospy.logwarn("apriltag_gripper: %s requested by %s", message, source)
            self._set_status(message)
            self.command_accepted_pub.publish(Bool(data=True))
            return True, message
        finally:
            with self.lock:
                self.command_in_progress = False
                self.last_command_monotonic = time.monotonic()

    def _close_worker(self):
        ok, message = self._command_gripper(True, "AprilTag target trigger")
        if not ok:
            rospy.logerr("AprilTag automatic close failed: %s", message)
            self._set_status("AprilTag automatic close failed: " + message)
            with self.lock:
                self.close_latched = False
                self.stable_count = 0

    def _detection_cb(self, msg):
        receive_time = rospy.Time.now()
        raw_stamp = msg.header.stamp if not msg.header.stamp.is_zero() else receive_time
        raw_latency = (receive_time - raw_stamp).to_sec()
        self.detection_latency_pub.publish(Float32(data=raw_latency))

        detection = None
        for candidate in msg.detections:
            if self.tag_id in candidate.id:
                detection = candidate
                break
        if detection is None:
            self.detected_pub.publish(Bool(data=False))
            return

        position = detection.pose.pose.pose.position
        if not all(math.isfinite(v) for v in (position.x, position.y, position.z)):
            return
        if position.z <= 0.05:
            return

        if (
            not self.last_valid_detection_time.is_zero()
            and (receive_time - self.last_valid_detection_time).to_sec()
            > self.detection_timeout
        ):
            # Do not mix a reacquired pose with samples retained from before a
            # perception outage; that would add hidden filter delay and jumps.
            self.x_history.clear()
            self.y_history.clear()
            self.z_history.clear()
            self.stamp_history.clear()

        self.x_history.append(position.x)
        self.y_history.append(position.y)
        self.z_history.append(position.z)
        self.stamp_history.append(raw_stamp.to_sec())
        x = float(np.median(self.x_history))
        y = float(np.median(self.y_history))
        z = float(np.median(self.z_history))
        # The median pose represents the middle sample, not the newest image.
        # Preserve that time so the mission can synchronize it with historical
        # odometry instead of combining an old image with the current attitude.
        stamp = rospy.Time.from_sec(float(np.median(self.stamp_history)))
        measurement_age = (receive_time - stamp).to_sec()
        self.measurement_age_pub.publish(Float32(data=measurement_age))
        if not self.last_valid_detection_time.is_zero():
            interval = (receive_time - self.last_valid_detection_time).to_sec()
            self.detection_interval_pub.publish(Float32(data=interval))
        self.last_valid_detection_time = receive_time

        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = msg.header.frame_id or "camera_color_optical_frame"
        point.point.x, point.point.y, point.point.z = x, y, z
        self.tag_position_pub.publish(point)

        error = PointStamped()
        error.header = point.header
        error.point.x = x - self.gripper_offset_x
        error.point.y = y - self.gripper_offset_y
        error.point.z = z - self.target_distance
        self.error_pub.publish(error)

        slant_range = math.sqrt(x * x + y * y + z * z)
        aligned = math.hypot(error.point.x, error.point.y) <= self.horizontal_tolerance
        at_distance = abs(error.point.z) <= self.distance_tolerance
        at_target = aligned and at_distance

        self.distance_pub.publish(Float32(data=z))
        self.slant_range_pub.publish(Float32(data=slant_range))
        self.distance_error_pub.publish(Float32(data=error.point.z))
        self.at_target_pub.publish(Bool(data=at_target))
        self.detected_pub.publish(Bool(data=True))

        with self.lock:
            self.last_detection_time = receive_time
            self.filtered_position = (x, y, z)
            self.stable_count = self.stable_count + 1 if at_target else 0
            should_close = (
                self.auto_close_enabled
                and not self.close_latched
                and self.stable_count >= self.stable_required
            )
            if should_close:
                self.close_latched = True
        if should_close:
            threading.Thread(
                target=self._close_worker, name="apriltag_close", daemon=True
            ).start()

    def _control_timer(self, _event):
        if rospy.is_shutdown():
            return
        now = rospy.Time.now()
        with self.lock:
            position = self.filtered_position
            age = (now - self.last_detection_time).to_sec()
            fresh = position is not None and age <= self.detection_timeout
            if not fresh:
                self.stable_count = 0

        vx = vy = vz = 0.0
        if fresh:
            x, y, z = position
            error_x = x - self.gripper_offset_x
            error_y = y - self.gripper_offset_y
            error_z = z - self.target_distance
            at_target = (
                abs(error_z) <= self.distance_tolerance
                and math.hypot(error_x, error_y) <= self.horizontal_tolerance
            )
            # Hold zero velocity throughout the close-trigger zone. Outside it,
            # retain the smaller control deadband for smooth convergence.
            if not at_target:
                if abs(error_y) > self.control_deadband:
                    vx = _clamp(-self.kp_xy * error_y, self.max_xy_speed)
                if abs(error_x) > self.control_deadband:
                    vy = _clamp(self.kp_xy * error_x, self.max_xy_speed)
                if abs(error_z) > self.control_deadband:
                    vz = _clamp(self.kp_z * error_z, self.max_z_speed)

        debug = TwistStamped()
        debug.header.stamp = now
        debug.header.frame_id = "base_link_ned"
        debug.twist.linear.x = vx
        debug.twist.linear.y = vy
        debug.twist.linear.z = vz
        try:
            self.velocity_debug_pub.publish(debug)
        except rospy.ROSException:
            return

        if not self.flight_enabled:
            return
        target = PositionTarget()
        target.header.stamp = now
        target.coordinate_frame = PositionTarget.FRAME_BODY_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        )
        if self.connected and self.armed and self.mode == "OFFBOARD" and fresh:
            target.velocity.x = vx
            target.velocity.y = vy
            target.velocity.z = vz
        try:
            self.setpoint_pub.publish(target)
        except rospy.ROSException:
            pass


def main():
    rospy.init_node("apriltag_gripper")
    try:
        AprilTagGripper()
    except Exception as exc:
        rospy.logfatal("apriltag_gripper initialization failed: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
