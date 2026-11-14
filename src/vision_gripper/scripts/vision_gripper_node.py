#!/usr/bin/env python3
"""Detect a target with D435i data and safely command a PX4 gripper."""

import math
import threading
import time
from collections import deque

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong, CommandLongRequest, ParamGet, ParamGetRequest
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


MAV_CMD_DO_SET_ACTUATOR = 187
MAV_RESULT_ACCEPTED = 0


def _triplet_param(name, default):
    value = rospy.get_param("~" + name, default)
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError("~{} must contain exactly three integers".format(name))
    return np.array([int(v) for v in value], dtype=np.uint8)


class VisionGripper:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.RLock()

        self.detector_mode = str(rospy.get_param("~detector_mode", "dark_circle"))
        if self.detector_mode not in ("dark_circle", "hsv_color"):
            raise ValueError("~detector_mode must be dark_circle or hsv_color")
        self.lower_1 = _triplet_param("hsv_lower_1", [0, 90, 60])
        self.upper_1 = _triplet_param("hsv_upper_1", [12, 255, 255])
        self.lower_2 = _triplet_param("hsv_lower_2", [168, 90, 60])
        self.upper_2 = _triplet_param("hsv_upper_2", [179, 255, 255])
        self.min_area = float(rospy.get_param("~min_contour_area_px", 1200))
        kernel_px = max(1, int(rospy.get_param("~morphology_kernel_px", 5)))
        if kernel_px % 2 == 0:
            kernel_px += 1
        self.kernel = np.ones((kernel_px, kernel_px), dtype=np.uint8)
        self.depth_window_px = max(1, int(rospy.get_param("~depth_window_px", 9)))
        self.min_depth_samples = max(1, int(rospy.get_param("~min_valid_depth_samples", 8)))
        self.ignore_bottom_ratio = float(rospy.get_param("~ignore_bottom_ratio", 0.20))
        if not 0.0 <= self.ignore_bottom_ratio < 1.0:
            raise ValueError("~ignore_bottom_ratio must be in [0, 1)")

        # Calibrated black tape roll detector. Radii are normalized by image width.
        self.circle_scale = float(rospy.get_param("~circle_scale", 0.50))
        self.circle_min_radius_ratio = float(rospy.get_param("~circle_min_radius_ratio", 0.10))
        self.circle_max_radius_ratio = float(rospy.get_param("~circle_max_radius_ratio", 0.20))
        self.circle_expected_radius_ratio = float(
            rospy.get_param("~circle_expected_radius_ratio", 0.151)
        )
        self.circle_hough_param1 = float(rospy.get_param("~circle_hough_param1", 100.0))
        self.circle_hough_param2 = float(rospy.get_param("~circle_hough_param2", 35.0))
        self.circle_dark_v_max = int(rospy.get_param("~circle_dark_v_max", 90))
        self.circle_min_dark_ratio = float(rospy.get_param("~circle_min_dark_ratio", 0.40))
        self.circle_search_u_min = float(rospy.get_param("~circle_search_u_min_ratio", 0.25))
        self.circle_search_u_max = float(rospy.get_param("~circle_search_u_max_ratio", 0.96))
        self.circle_search_v_min = float(rospy.get_param("~circle_search_v_min_ratio", 0.35))
        self.circle_search_v_max = float(rospy.get_param("~circle_search_v_max_ratio", 1.05))
        self.ring_depth_inner_ratio = float(rospy.get_param("~ring_depth_inner_ratio", 0.55))
        self.ring_depth_outer_ratio = float(rospy.get_param("~ring_depth_outer_ratio", 0.85))
        self.ring_depth_upper_cut_ratio = float(
            rospy.get_param("~ring_depth_upper_cut_ratio", -0.05)
        )
        self.ring_depth_percentile = float(rospy.get_param("~ring_depth_percentile", 30.0))
        self.circle_min_depth_samples = int(rospy.get_param("~circle_min_depth_samples", 100))
        if not 0.1 <= self.circle_scale <= 1.0:
            raise ValueError("~circle_scale must be in [0.1, 1.0]")
        if not 0.0 <= self.ring_depth_percentile <= 100.0:
            raise ValueError("~ring_depth_percentile must be in [0, 100]")

        self.grasp_u_ratio = float(rospy.get_param("~grasp_u_ratio", 0.50))
        self.grasp_v_ratio = float(rospy.get_param("~grasp_v_ratio", 0.68))
        self.zone_w_ratio = float(rospy.get_param("~grasp_zone_width_ratio", 0.18))
        self.zone_h_ratio = float(rospy.get_param("~grasp_zone_height_ratio", 0.18))
        self.depth_min = float(rospy.get_param("~grab_depth_min_m", 0.16))
        self.depth_max = float(rospy.get_param("~grab_depth_max_m", 0.40))
        self.depth_filter_window = max(
            1, int(rospy.get_param("~depth_temporal_window", 7))
        )
        if self.depth_filter_window % 2 == 0:
            self.depth_filter_window += 1
        self.depth_history = deque(maxlen=self.depth_filter_window)
        self.grasp_reference_x = float(rospy.get_param("~grasp_reference_x_m", 0.0603))
        self.grasp_reference_y = float(rospy.get_param("~grasp_reference_y_m", 0.0840))
        self.grasp_reference_z = float(rospy.get_param("~grasp_reference_z_m", 0.2760))
        self.stable_frames_required = max(1, int(rospy.get_param("~stable_frames", 12)))

        self.main_output = int(rospy.get_param("~main_output", 1))
        self.actuator_index = int(rospy.get_param("~actuator_index", 1))
        if not 1 <= self.actuator_index <= 6:
            raise ValueError("~actuator_index must be in [1, 6]")
        self.expected_output_function = 300 + self.actuator_index
        self.require_mapping = bool(rospy.get_param("~require_output_mapping", True))
        self.require_connected = bool(rospy.get_param("~require_mavros_connected", True))
        self.open_value = float(rospy.get_param("~open_value", 0.60))
        self.close_value = float(rospy.get_param("~close_value", -0.60))
        if not (-1.0 <= self.open_value <= 1.0 and -1.0 <= self.close_value <= 1.0):
            raise ValueError("~open_value and ~close_value must be in [-1, 1]")
        self.cooldown = float(rospy.get_param("~command_cooldown_s", 1.0))
        self.dry_run = bool(rospy.get_param("~dry_run", True))
        self.auto_enabled = bool(rospy.get_param("~auto_grab_on_start", False))
        self.publish_debug = bool(rospy.get_param("~publish_debug_image", True))

        self.connected = False
        self.stable_frames = 0
        self.auto_latched = False
        self.command_in_progress = False
        self.last_command_monotonic = -1e9
        self.camera_info = None
        self.last_status = "starting"

        command_service = rospy.get_param("~mavros_command_service", "/mavros/cmd/command")
        param_service = rospy.get_param("~mavros_param_get_service", "/mavros/param/get")
        self.command_client = rospy.ServiceProxy(command_service, CommandLong, persistent=True)
        self.param_client = rospy.ServiceProxy(param_service, ParamGet, persistent=True)

        self.target_pub = rospy.Publisher("~target", PointStamped, queue_size=5)
        self.alignment_error_pub = rospy.Publisher(
            "~alignment_error_m", PointStamped, queue_size=5
        )
        self.depth_pub = rospy.Publisher("~target_depth_m", Float32, queue_size=5)
        self.depth_valid_pub = rospy.Publisher("~depth_valid_ratio", Float32, queue_size=5)
        self.detected_pub = rospy.Publisher("~detected", Bool, queue_size=1)
        self.ready_pub = rospy.Publisher("~ready_to_grab", Bool, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1)

        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=1)
        info_topic = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        rospy.Subscriber(info_topic, CameraInfo, self._camera_info_cb, queue_size=1)

        color_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        depth_topic = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        color_sub = message_filters.Subscriber(color_topic, Image, queue_size=1)
        depth_sub = message_filters.Subscriber(depth_topic, Image, queue_size=1)
        sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=8, slop=0.08, allow_headerless=False
        )
        sync.registerCallback(self._image_cb)
        self._sync = sync

        rospy.Service("~set_gripper", SetBool, self._set_gripper_service)
        rospy.Service("~enable_auto_grab", SetBool, self._enable_auto_service)
        rospy.Service("~reset_auto_grab", Trigger, self._reset_auto_service)
        rospy.Timer(rospy.Duration(1.0), self._status_timer)

        mode = "DRY RUN" if self.dry_run else "LIVE OUTPUT"
        self._set_status("ready; {}; auto_grab={}".format(mode, self.auto_enabled))
        rospy.logwarn(
            "vision_gripper: %s, MAIN%d must map to function %d; automatic grab=%s",
            mode, self.main_output, self.expected_output_function, self.auto_enabled,
        )

    def _state_cb(self, msg):
        self.connected = bool(msg.connected)

    def _camera_info_cb(self, msg):
        self.camera_info = msg

    def _set_status(self, text):
        self.last_status = text
        self.status_pub.publish(String(data=text))

    def _status_timer(self, _event):
        self.status_pub.publish(String(data=self.last_status))

    def _enable_auto_service(self, req):
        with self.lock:
            self.auto_enabled = bool(req.data)
            self.stable_frames = 0
            if not self.auto_enabled:
                self.auto_latched = False
        text = "automatic grab enabled" if req.data else "automatic grab disabled"
        self._set_status(text)
        return SetBoolResponse(success=True, message=text)

    def _reset_auto_service(self, _req):
        with self.lock:
            self.auto_latched = False
            self.stable_frames = 0
        self._set_status("automatic grab latch reset")
        return TriggerResponse(success=True, message="automatic grab latch reset")

    def _set_gripper_service(self, req):
        close = bool(req.data)
        ok, message = self._command_gripper(close, source="manual service")
        if ok and not close:
            with self.lock:
                self.auto_latched = False
                self.stable_frames = 0
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
            if now - self.last_command_monotonic < self.cooldown:
                return False, "gripper command cooldown is active"
            self.command_in_progress = True

        try:
            action = "close" if close else "open"
            value = self.close_value if close else self.open_value
            if self.require_connected and not self.connected:
                return False, "MAVROS is not connected to PX4"
            mapping_ok, mapping_message = self._validate_mapping()
            if not mapping_ok:
                return False, "refusing output: " + mapping_message
            if self.dry_run:
                message = "dry-run {} ({:+.2f}); {}".format(action, value, mapping_message)
                rospy.logwarn("vision_gripper: %s requested by %s", message, source)
                self._set_status(message)
                return True, message

            values = [math.nan] * 6
            values[self.actuator_index - 1] = value
            request = CommandLongRequest(
                broadcast=False,
                command=MAV_CMD_DO_SET_ACTUATOR,
                confirmation=0,
                param1=values[0],
                param2=values[1],
                param3=values[2],
                param4=values[3],
                param5=values[4],
                param6=values[5],
                param7=0.0,
            )
            try:
                response = self.command_client(request)
            except (rospy.ServiceException, rospy.ROSException) as exc:
                return False, "MAVROS command failed: {}".format(exc)
            if not response.success or response.result != MAV_RESULT_ACCEPTED:
                return False, "PX4 rejected {}: MAV_RESULT={}".format(action, response.result)
            message = "gripper {} accepted ({:+.2f}); {}".format(action, value, mapping_message)
            rospy.logwarn("vision_gripper: %s requested by %s", message, source)
            self._set_status(message)
            return True, message
        finally:
            with self.lock:
                self.command_in_progress = False
                self.last_command_monotonic = time.monotonic()

    def _depth_metres(self, depth_image, encoding, u, v):
        radius = max(1, self.depth_window_px // 2)
        y0, y1 = max(0, v - radius), min(depth_image.shape[0], v + radius + 1)
        x0, x1 = max(0, u - radius), min(depth_image.shape[1], u + radius + 1)
        samples = depth_image[y0:y1, x0:x1].astype(np.float32).reshape(-1)
        if encoding in ("16UC1", "mono16"):
            samples *= 0.001
        elif encoding != "32FC1":
            rospy.logwarn_throttle(5.0, "unsupported depth encoding: %s", encoding)
            return None
        samples = samples[np.isfinite(samples) & (samples > 0.05) & (samples < 10.0)]
        if samples.size < self.min_depth_samples:
            return None
        return float(np.median(samples))

    @staticmethod
    def _depth_image_metres(depth_image, encoding):
        depth_m = depth_image.astype(np.float32)
        if encoding in ("16UC1", "mono16"):
            depth_m *= 0.001
        elif encoding != "32FC1":
            rospy.logwarn_throttle(5.0, "unsupported depth encoding: %s", encoding)
            return None
        return depth_m

    def _ring_depth_metres(self, depth_image, encoding, u, v, radius):
        depth_m = self._depth_image_metres(depth_image, encoding)
        if depth_m is None:
            return None, 0.0, 0
        yy, xx = np.ogrid[:depth_m.shape[0], :depth_m.shape[1]]
        radial_distance = np.sqrt((xx - u) ** 2 + (yy - v) ** 2)
        ring_mask = (
            (radial_distance >= self.ring_depth_inner_ratio * radius)
            & (radial_distance <= self.ring_depth_outer_ratio * radius)
            & (yy < v + self.ring_depth_upper_cut_ratio * radius)
        )
        valid_mask = ring_mask & np.isfinite(depth_m) & (depth_m > 0.05) & (depth_m < 10.0)
        samples = depth_m[valid_mask]
        valid_ratio = float(samples.size) / float(max(1, np.count_nonzero(ring_mask)))
        if samples.size < self.circle_min_depth_samples:
            return None, valid_ratio, int(samples.size)
        # A tape roll is hollow. The lower percentile isolates its near surface
        # instead of the floor visible through the centre hole.
        value = float(np.percentile(samples, self.ring_depth_percentile))
        return value, valid_ratio, int(samples.size)

    def _detect_hsv_target(self, bgr, depth, depth_encoding):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_1, self.upper_1)
        mask |= cv2.inRange(hsv, self.lower_2, self.upper_2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        if self.ignore_bottom_ratio > 0.0:
            ignore_from = int(round(mask.shape[0] * (1.0 - self.ignore_bottom_ratio)))
            mask[ignore_from:, :] = 0
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        moments = cv2.moments(contour)
        if area < self.min_area or moments["m00"] <= 0.0:
            return None
        u = int(round(moments["m10"] / moments["m00"]))
        v = int(round(moments["m01"] / moments["m00"]))
        depth_m = self._depth_metres(depth, depth_encoding, u, v)
        return {
            "kind": "contour", "shape": contour, "u": u, "v": v,
            "depth_m": depth_m, "metric": area, "depth_valid_ratio": 1.0,
            "depth_samples": 0,
        }

    def _detect_dark_circle(self, bgr, depth, depth_encoding):
        height, width = bgr.shape[:2]
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        small = cv2.resize(
            gray, None, fx=self.circle_scale, fy=self.circle_scale,
            interpolation=cv2.INTER_AREA,
        )
        small = cv2.medianBlur(small, 7)
        min_radius = max(3, int(round(self.circle_min_radius_ratio * width * self.circle_scale)))
        max_radius = max(min_radius + 1, int(round(
            self.circle_max_radius_ratio * width * self.circle_scale
        )))
        circles = cv2.HoughCircles(
            small, cv2.HOUGH_GRADIENT, dp=1.2,
            minDist=max(10.0, 0.08 * width * self.circle_scale),
            param1=self.circle_hough_param1, param2=self.circle_hough_param2,
            minRadius=min_radius, maxRadius=max_radius,
        )
        if circles is None:
            return None

        hsv_value = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[:, :, 2]
        yy, xx = np.ogrid[:height, :width]
        candidates = []
        for small_u, small_v, small_radius in circles[0]:
            u = float(small_u / self.circle_scale)
            v = float(small_v / self.circle_scale)
            radius = float(small_radius / self.circle_scale)
            u_ratio, v_ratio = u / width, v / height
            if not (
                self.circle_search_u_min <= u_ratio <= self.circle_search_u_max
                and self.circle_search_v_min <= v_ratio <= self.circle_search_v_max
            ):
                continue
            radial_distance = np.sqrt((xx - u) ** 2 + (yy - v) ** 2)
            dark_annulus = (radial_distance >= 0.80 * radius) & (
                radial_distance <= 1.05 * radius
            )
            if not np.any(dark_annulus):
                continue
            dark_ratio = float(np.mean(hsv_value[dark_annulus] <= self.circle_dark_v_max))
            if dark_ratio < self.circle_min_dark_ratio:
                continue
            radius_error = abs(radius / width - self.circle_expected_radius_ratio)
            score = radius_error - 0.04 * dark_ratio
            candidates.append((score, u, v, radius, dark_ratio))
        if not candidates:
            return None

        _, u, v, radius, dark_ratio = min(candidates, key=lambda item: item[0])
        depth_m, valid_ratio, sample_count = self._ring_depth_metres(
            depth, depth_encoding, u, v, radius
        )
        return {
            "kind": "circle", "shape": radius, "u": int(round(u)), "v": int(round(v)),
            "depth_m": depth_m, "metric": dark_ratio,
            "depth_valid_ratio": valid_ratio, "depth_samples": sample_count,
        }

    def _publish_target(self, header, u, v, depth_m):
        info = self.camera_info
        if info is None or info.K[0] == 0.0 or info.K[4] == 0.0:
            return
        fx, fy, cx, cy = info.K[0], info.K[4], info.K[2], info.K[5]
        point = PointStamped()
        point.header = header
        point.point.z = depth_m
        point.point.x = (u - cx) * depth_m / fx
        point.point.y = (v - cy) * depth_m / fy
        self.target_pub.publish(point)
        error = PointStamped()
        error.header = header
        error.point.x = point.point.x - self.grasp_reference_x
        error.point.y = point.point.y - self.grasp_reference_y
        error.point.z = point.point.z - self.grasp_reference_z
        self.alignment_error_pub.publish(error)
        return point.point.x, point.point.y, point.point.z

    def _image_cb(self, color_msg, depth_msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            rospy.logerr_throttle(2.0, "cv_bridge conversion failed: %s", exc)
            return
        if bgr.shape[:2] != depth.shape[:2]:
            rospy.logerr_throttle(2.0, "color/depth sizes differ; enable aligned depth")
            return

        height, width = bgr.shape[:2]
        zone_u = int(round(self.grasp_u_ratio * width))
        zone_v = int(round(self.grasp_v_ratio * height))
        half_w = int(round(self.zone_w_ratio * width / 2.0))
        half_h = int(round(self.zone_h_ratio * height / 2.0))
        zone_x0, zone_x1 = zone_u - half_w, zone_u + half_w
        zone_y0, zone_y1 = zone_v - half_h, zone_v + half_h

        if self.detector_mode == "dark_circle":
            target = self._detect_dark_circle(bgr, depth, depth_msg.encoding)
        else:
            target = self._detect_hsv_target(bgr, depth, depth_msg.encoding)

        detected = target is not None
        ready = False
        target_xyz = None
        if target is not None:
            u, v, depth_m = target["u"], target["v"], target["depth_m"]
            if depth_m is not None:
                target["raw_depth_m"] = depth_m
                self.depth_history.append(depth_m)
                depth_m = float(np.median(self.depth_history))
                target["depth_m"] = depth_m
            in_zone = zone_x0 <= u <= zone_x1 and zone_y0 <= v <= zone_y1
            depth_ok = depth_m is not None and self.depth_min <= depth_m <= self.depth_max
            ready = bool(in_zone and depth_ok)
            self.depth_valid_pub.publish(Float32(data=target["depth_valid_ratio"]))
            if depth_m is not None:
                self.depth_pub.publish(Float32(data=depth_m))
                target_xyz = self._publish_target(color_msg.header, u, v, depth_m)

        self.detected_pub.publish(Bool(data=detected))
        self.ready_pub.publish(Bool(data=ready))
        with self.lock:
            self.stable_frames = self.stable_frames + 1 if ready else 0
            should_grab = (
                self.auto_enabled
                and not self.auto_latched
                and self.stable_frames >= self.stable_frames_required
            )
            if should_grab:
                self.auto_latched = True

        if should_grab:
            thread = threading.Thread(
                target=self._auto_command_worker, name="auto_grab_command", daemon=True
            )
            thread.start()

        if self.publish_debug and self.debug_pub.get_num_connections() > 0:
            debug = bgr.copy()
            zone_color = (0, 255, 0) if ready else (0, 180, 255)
            cv2.rectangle(debug, (zone_x0, zone_y0), (zone_x1, zone_y1), zone_color, 2)
            cv2.drawMarker(debug, (zone_u, zone_v), zone_color, cv2.MARKER_CROSS, 18, 2)
            if self.detector_mode == "hsv_color" and self.ignore_bottom_ratio > 0.0:
                ignore_from = int(round(height * (1.0 - self.ignore_bottom_ratio)))
                cv2.line(debug, (0, ignore_from), (width - 1, ignore_from), (80, 80, 80), 2)
                cv2.putText(
                    debug, "SELF MASK", (12, min(height - 8, ignore_from + 24)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 80, 80), 2, cv2.LINE_AA,
                )
            if target is not None:
                u, v, depth_m = target["u"], target["v"], target["depth_m"]
                if target["kind"] == "circle":
                    radius = int(round(target["shape"]))
                    display_radius = radius
                    cv2.circle(debug, (u, v), radius, (255, 0, 255), 3)
                    cv2.ellipse(
                        debug, (u, v),
                        (int(self.ring_depth_outer_ratio * radius), int(self.ring_depth_outer_ratio * radius)),
                        0, 180, 360, (255, 255, 0), 2,
                    )
                    metric_text = "dark={:.2f}".format(target["metric"])
                else:
                    display_radius = 0
                    cv2.drawContours(debug, [target["shape"]], -1, (255, 0, 255), 2)
                    metric_text = "area={:.0f}".format(target["metric"])
                cv2.circle(debug, (u, v), 5, (255, 255, 255), -1)
                depth_text = "no depth" if depth_m is None else "{:.3f} m".format(depth_m)
                dx, dy = u - zone_u, v - zone_v
                cv2.putText(
                    debug, "{}, depth={}, valid={:.1%} (n={})".format(
                        metric_text, depth_text, target["depth_valid_ratio"], target["depth_samples"]
                    ),
                    (max(0, min(width - 500, u - 210)), max(22, v - display_radius - 38)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 2, cv2.LINE_AA,
                )
                xyz_text = "error=({:+d},{:+d})px".format(dx, dy)
                if target_xyz is not None:
                    xyz_text += "  XYZ=({:+.3f},{:+.3f},{:.3f})m".format(*target_xyz)
                cv2.putText(
                    debug, xyz_text,
                    (max(0, min(width - 500, u - 210)), max(44, v - display_radius - 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA,
                )
            status = "READY {}/{}".format(
                min(self.stable_frames, self.stable_frames_required), self.stable_frames_required
            ) if ready else "SEARCH"
            cv2.putText(debug, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, zone_color, 2)
            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))
            except CvBridgeError as exc:
                rospy.logerr_throttle(2.0, "debug image conversion failed: %s", exc)

    def _auto_command_worker(self):
        ok, message = self._command_gripper(True, source="automatic vision trigger")
        if not ok:
            rospy.logerr("automatic grab failed: %s", message)
            self._set_status("automatic grab failed: " + message)
            with self.lock:
                self.auto_latched = False
                self.stable_frames = 0


def main():
    rospy.init_node("vision_gripper")
    try:
        VisionGripper()
    except (ValueError, rospy.ROSException) as exc:
        rospy.logfatal("vision_gripper initialization failed: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
