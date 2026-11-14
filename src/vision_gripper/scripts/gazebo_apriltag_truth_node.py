#!/usr/bin/env python3
"""Publish an ideal AprilTag detection from Gazebo model ground truth.

The simulator deliberately bypasses image rendering and AprilTag decoding.  It
keeps the calibrated camera optical-frame convention used by the real mission:
x is image-right, y is image-down/opposite body-forward, and z is down to the
ground target.  Visibility is limited by a configurable pinhole field of view.
"""

import math
import threading

import rospy
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse


def world_to_body_flu(quaternion, vector):
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if not math.isfinite(norm) or norm < 1.0e-9:
        return None
    x = quaternion.x / norm
    y = quaternion.y / norm
    z = quaternion.z / norm
    w = quaternion.w / norm
    # R(q)^T * v, where q maps ROS body FLU into the Gazebo world frame.
    vx, vy, vz = vector
    return [
        (1.0 - 2.0 * (y * y + z * z)) * vx
        + 2.0 * (x * y + z * w) * vy
        + 2.0 * (x * z - y * w) * vz,
        2.0 * (x * y - z * w) * vx
        + (1.0 - 2.0 * (x * x + z * z)) * vy
        + 2.0 * (y * z + x * w) * vz,
        2.0 * (x * z + y * w) * vx
        + 2.0 * (y * z - x * w) * vy
        + (1.0 - 2.0 * (x * x + y * y)) * vz,
    ]


class GazeboAprilTagTruth:
    def __init__(self):
        self.drone_model = rospy.get_param("~drone_model", "iris_0")
        self.tag_model = rospy.get_param("~tag_model", "apriltag_target")
        self.tag_id = int(rospy.get_param("~tag_id", 0))
        self.tag_size = float(rospy.get_param("~tag_size_m", 0.08746))
        # Operator-facing FRD geometry converted to ROS body FLU.
        self.camera_to_gripper_flu = [
            float(rospy.get_param("~camera_to_gripper_forward_m", -0.058)),
            -float(rospy.get_param("~camera_to_gripper_right_m", 0.0)),
            -float(rospy.get_param("~camera_to_gripper_down_m", 0.110)),
        ]
        self.min_depth = float(rospy.get_param("~min_depth_m", 0.10))
        self.max_depth = float(rospy.get_param("~max_depth_m", 3.0))
        horizontal_fov = math.radians(float(rospy.get_param("~horizontal_fov_deg", 69.0)))
        vertical_fov = math.radians(float(rospy.get_param("~vertical_fov_deg", 42.0)))
        self.tan_half_hfov = math.tan(0.5 * horizontal_fov)
        self.tan_half_vfov = math.tan(0.5 * vertical_fov)
        rate_hz = float(rospy.get_param("~publish_rate_hz", 30.0))

        positive = (self.tag_size, self.min_depth, self.max_depth, rate_hz)
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("tag size, depth limits and publish rate must be finite and positive")
        if self.max_depth <= self.min_depth:
            raise ValueError("~max_depth_m must exceed ~min_depth_m")

        self.lock = threading.Lock()
        self.model_states = None
        self.enabled = True
        self.detection_pub = rospy.Publisher(
            "/sim/tag_detections", AprilTagDetectionArray, queue_size=2
        )
        self.point_pub = rospy.Publisher("~tag_position_truth", PointStamped, queue_size=2)
        self.visible_pub = rospy.Publisher("~visible", Bool, queue_size=1, latch=True)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._states_cb, queue_size=1)
        rospy.Service("~set_enabled", SetBool, self._set_enabled)
        rospy.Timer(rospy.Duration(1.0 / rate_hz), self._timer_cb)

    def _set_enabled(self, request):
        with self.lock:
            self.enabled = bool(request.data)
        return SetBoolResponse(success=True, message="truth detections {}".format(
            "enabled" if request.data else "disabled"
        ))

    def _states_cb(self, msg):
        with self.lock:
            self.model_states = msg

    def _timer_cb(self, _event):
        with self.lock:
            msg = self.model_states
            enabled = self.enabled
        if not enabled:
            self.visible_pub.publish(Bool(data=False))
            return
        if msg is None:
            return
        try:
            drone_index = msg.name.index(self.drone_model)
            tag_index = msg.name.index(self.tag_model)
        except ValueError:
            rospy.logwarn_throttle(
                2.0,
                "Gazebo AprilTag truth waiting for models %s and %s",
                self.drone_model,
                self.tag_model,
            )
            self.visible_pub.publish(Bool(data=False))
            return

        drone = msg.pose[drone_index]
        tag = msg.pose[tag_index]
        body_tag_from_gripper = world_to_body_flu(
            drone.orientation,
            [
                tag.position.x - drone.position.x,
                tag.position.y - drone.position.y,
                tag.position.z - drone.position.z,
            ],
        )
        if body_tag_from_gripper is None:
            self.visible_pub.publish(Bool(data=False))
            return
        # C->T = C->G + G->T.  The previous subtraction applied the rigid
        # camera-to-gripper offset with the wrong sign, making the simulated
        # camera height/forward offset disagree with the real calibration.
        tag_from_camera_flu = [
            body_tag_from_gripper[i] + self.camera_to_gripper_flu[i]
            for i in range(3)
        ]
        # ROS body FLU -> D435 optical (right, image-down/tail, lens/down).
        optical_x = -tag_from_camera_flu[1]
        optical_y = -tag_from_camera_flu[0]
        optical_z = -tag_from_camera_flu[2]
        finite = all(math.isfinite(value) for value in (optical_x, optical_y, optical_z))
        visible = (
            finite
            and self.min_depth <= optical_z <= self.max_depth
            and abs(optical_x) <= optical_z * self.tan_half_hfov
            and abs(optical_y) <= optical_z * self.tan_half_vfov
        )
        self.visible_pub.publish(Bool(data=visible))
        if not visible:
            return

        stamp = rospy.Time.now()
        header_frame = "camera_color_optical_frame"
        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = header_frame
        point.point.x = optical_x
        point.point.y = optical_y
        point.point.z = optical_z
        self.point_pub.publish(point)

        detection = AprilTagDetection()
        detection.id = [self.tag_id]
        detection.size = [self.tag_size]
        detection.pose.header.stamp = stamp
        detection.pose.header.frame_id = header_frame
        detection.pose.pose.pose.position.x = optical_x
        detection.pose.pose.pose.position.y = optical_y
        detection.pose.pose.pose.position.z = optical_z
        detection.pose.pose.pose.orientation.w = 1.0

        array = AprilTagDetectionArray()
        array.header.stamp = stamp
        array.header.frame_id = header_frame
        array.detections = [detection]
        self.detection_pub.publish(array)


def main():
    rospy.init_node("gazebo_apriltag_truth")
    GazeboAprilTagTruth()
    rospy.spin()


if __name__ == "__main__":
    main()
