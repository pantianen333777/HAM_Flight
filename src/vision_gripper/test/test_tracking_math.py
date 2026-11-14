#!/usr/bin/env python3

import importlib.util
import os
import threading
import unittest
from collections import deque
from unittest import mock

import rospy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from quadrotor_msgs.msg import PositionCommand


MODULE_PATH = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    "scripts",
    "apriltag_trajectory_mission_node.py",
)
SPEC = importlib.util.spec_from_file_location("apriltag_mission_under_test", MODULE_PATH)
MISSION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MISSION)


def odom(stamp, x=0.0, y=0.0, z=0.0):
    msg = Odometry()
    msg.header.stamp = rospy.Time.from_sec(stamp)
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.w = 1.0
    return msg


class TrackingMathTest(unittest.TestCase):
    def test_operator_autoland_after_hold_home_is_not_a_failsafe(self):
        mission = MISSION.AprilTagTrajectoryMission.__new__(
            MISSION.AprilTagTrajectoryMission
        )
        mission.lock = threading.RLock()
        mission.state = MISSION.HOLD_HOME
        mission.armed = True
        mission._set_status_locked = mock.Mock()
        mission._publish_state_locked = mock.Mock()
        mission._reset_after_landing_locked = mock.Mock()

        msg = State()
        msg.connected = True
        msg.armed = True
        msg.mode = "AUTO.LAND"
        with mock.patch.object(MISSION.rospy, "logwarn"):
            mission._mavros_state_cb(msg)

        self.assertEqual(mission.state, MISSION.LANDING)
        mission._set_status_locked.assert_called_once()
        mission._publish_state_locked.assert_called_once()

    def test_failsafe_cannot_adopt_a_new_trajectory_id(self):
        mission = MISSION.AprilTagTrajectoryMission.__new__(
            MISSION.AprilTagTrajectoryMission
        )
        mission.lock = threading.RLock()
        mission.state = MISSION.FAILSAFE_HOLD
        mission.trajectory_id = 42

        msg = PositionCommand()
        msg.trajectory_id = 43
        msg.trajectory_flag = getattr(
            PositionCommand, "TRAJECTORY_STATUS_READY", 1
        )

        with mock.patch.object(MISSION.rospy, "logerr_throttle"):
            mission._preset_cb(msg)

        self.assertEqual(mission.trajectory_id, 42)
        self.assertFalse(hasattr(mission, "latest_preset"))

    def test_descent_envelope_is_continuous_and_bounded(self):
        scale = MISSION.descent_speed_scale
        self.assertEqual(scale(0.00, 0.08, 0.20), 1.0)
        self.assertEqual(scale(0.08, 0.08, 0.20), 1.0)
        self.assertAlmostEqual(scale(0.14, 0.08, 0.20), 0.5)
        self.assertEqual(scale(0.20, 0.08, 0.20), 0.0)
        self.assertEqual(scale(0.30, 0.08, 0.20), 0.0)

    def test_delayed_tag_error_removes_vehicle_translation(self):
        mission = MISSION.AprilTagTrajectoryMission.__new__(
            MISSION.AprilTagTrajectoryMission
        )
        mission.mount_rpy = [0.0, 0.0, 0.0]
        mission.camera_to_gripper_flu = [0.0, 0.0, 0.0]
        mission.object_forward = 0.0
        mission.object_right = 0.0
        mission.object_height = 0.0
        mission.home_yaw = 0.0
        capture = odom(1.0, x=0.0)
        mission.latest_odom = odom(1.2, x=0.10)

        # Optical y=-1 maps to body/world x=+1.  Since the aircraft moved
        # +0.10 m after capture, the current relative error must be +0.90 m.
        error = mission._tag_error_world_locked([0.0, -1.0, 0.0], capture)
        self.assertAlmostEqual(error[0], 0.90, places=9)
        self.assertAlmostEqual(error[1], 0.0, places=9)

    def test_odom_lookup_rejects_unsynchronized_measurement(self):
        mission = MISSION.AprilTagTrajectoryMission.__new__(
            MISSION.AprilTagTrajectoryMission
        )
        mission.latest_odom = odom(1.10)
        mission.tag_odom_sync_tolerance = 0.02
        mission.odom_history = deque([
            (1.00, odom(1.00)),
            (1.10, mission.latest_odom),
        ])
        self.assertIsNotNone(mission._odom_at_stamp_locked(rospy.Time.from_sec(1.01)))
        self.assertIsNone(mission._odom_at_stamp_locked(rospy.Time.from_sec(1.05)))

    def test_reference_limiter_does_not_follow_vehicle_moving_away(self):
        mission = MISSION.AprilTagTrajectoryMission.__new__(
            MISSION.AprilTagTrajectoryMission
        )
        mission.latest_odom = odom(1.0, z=1.70)
        mission.command_position = [0.0, 0.0, 1.44]
        mission.command_velocity = [0.0, 0.0, 0.0]
        mission.max_xy_reference_lead = 0.12
        mission.max_z_reference_lead = 0.10

        mission._integrate_tracking_reference_locked(0.02)

        # A reference that was already below the aircraft must remain fixed;
        # snapping it to odom-0.10 would remove recovery authority.
        self.assertAlmostEqual(mission.command_position[2], 1.44, places=9)

    def test_reference_limiter_blocks_only_further_lead_growth(self):
        mission = MISSION.AprilTagTrajectoryMission.__new__(
            MISSION.AprilTagTrajectoryMission
        )
        mission.latest_odom = odom(1.0)
        mission.command_position = [0.11, 0.0, -0.09]
        mission.command_velocity = [1.0, 0.0, -1.0]
        mission.max_xy_reference_lead = 0.12
        mission.max_z_reference_lead = 0.10

        mission._integrate_tracking_reference_locked(0.02)

        self.assertAlmostEqual(mission.command_position[0], 0.12, places=9)
        self.assertAlmostEqual(mission.command_position[2], -0.10, places=9)


if __name__ == "__main__":
    unittest.main()
