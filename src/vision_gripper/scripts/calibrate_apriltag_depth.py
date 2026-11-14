#!/usr/bin/env python3
"""Estimate AprilTag edge size from aligned D435 depth and tag pose.

This node is intentionally read-only: it subscribes to detections, aligned
depth and camera intrinsics, then prints robust calibration statistics.  It
does not publish setpoints or command the gripper.
"""

import math
import sys
import threading
import time

import message_filters
import numpy as np
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo, Image


def _robust_summary(values):
    data = np.asarray(values, dtype=np.float64)
    median = float(np.median(data))
    absolute_deviation = np.abs(data - median)
    mad = float(np.median(absolute_deviation))
    threshold = max(4.0 * 1.4826 * mad, abs(median) * 0.005, 1.0e-9)
    kept = data[absolute_deviation <= threshold]
    if kept.size < max(5, data.size // 2):
        kept = data
    return {
        "median": float(np.median(kept)),
        "mean": float(np.mean(kept)),
        "std": float(np.std(kept)),
        "p05": float(np.percentile(kept, 5.0)),
        "p95": float(np.percentile(kept, 95.0)),
        "kept": int(kept.size),
        "total": int(data.size),
    }


class DepthTagCalibrator:
    def __init__(self):
        self.tag_id = int(rospy.get_param("~tag_id", 0))
        self.configured_size = float(
            rospy.get_param("~configured_tag_size_m", 0.08746)
        )
        self.sample_count = max(20, int(rospy.get_param("~sample_count", 100)))
        self.timeout_s = max(3.0, float(rospy.get_param("~timeout_s", 25.0)))
        self.patch_radius = max(1, int(rospy.get_param("~patch_radius_px", 6)))
        self.known_distance = float(rospy.get_param("~known_distance_m", 0.0))

        self.lock = threading.Lock()
        self.samples = []
        self.camera_info = None
        self.rejected = 0

        camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )
        detection_topic = rospy.get_param("~detection_topic", "/tag_detections")
        depth_topic = rospy.get_param(
            "~depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        rospy.Subscriber(
            camera_info_topic, CameraInfo, self._camera_info_cb, queue_size=1
        )
        detection_sub = message_filters.Subscriber(
            detection_topic, AprilTagDetectionArray, queue_size=5
        )
        depth_sub = message_filters.Subscriber(depth_topic, Image, queue_size=5)
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            [detection_sub, depth_sub], queue_size=30, slop=0.10
        )
        synchronizer.registerCallback(self._sample_cb)
        self._synchronizer = synchronizer

    def _camera_info_cb(self, msg):
        self.camera_info = msg

    def _sample_cb(self, detections, depth_msg):
        if self.camera_info is None:
            return
        with self.lock:
            if len(self.samples) >= self.sample_count:
                return

        detection = None
        for candidate in detections.detections:
            if self.tag_id in candidate.id:
                detection = candidate
                break
        if detection is None:
            return

        position = detection.pose.pose.pose.position
        if not all(math.isfinite(v) for v in (position.x, position.y, position.z)):
            self.rejected += 1
            return
        if position.z <= 0.05:
            self.rejected += 1
            return

        info = self.camera_info
        fx, fy = float(info.K[0]), float(info.K[4])
        cx, cy = float(info.K[2]), float(info.K[5])
        u = int(round(fx * position.x / position.z + cx))
        v = int(round(fy * position.y / position.z + cy))

        if depth_msg.encoding in ("16UC1", "mono16"):
            dtype = np.dtype(np.uint16)
        elif depth_msg.encoding == "32FC1":
            dtype = np.dtype(np.float32)
        else:
            rospy.logwarn_throttle(
                5.0, "unsupported depth encoding: %s", depth_msg.encoding
            )
            self.rejected += 1
            return
        byte_order_differs = bool(depth_msg.is_bigendian) != (sys.byteorder == "big")
        if byte_order_differs:
            dtype = dtype.newbyteorder(">" if depth_msg.is_bigendian else "<")
        values_per_row = int(depth_msg.step) // dtype.itemsize
        expected_values = int(depth_msg.height) * values_per_row
        depth = np.frombuffer(depth_msg.data, dtype=dtype, count=expected_values)
        depth = depth.reshape((int(depth_msg.height), values_per_row))
        depth = depth[:, : int(depth_msg.width)]
        if depth.ndim != 2:
            self.rejected += 1
            return

        x0 = max(0, u - self.patch_radius)
        x1 = min(depth.shape[1], u + self.patch_radius + 1)
        y0 = max(0, v - self.patch_radius)
        y1 = min(depth.shape[0], v + self.patch_radius + 1)
        if x0 >= x1 or y0 >= y1:
            self.rejected += 1
            return

        patch = np.asarray(depth[y0:y1, x0:x1], dtype=np.float64)
        if depth_msg.encoding in ("16UC1", "mono16"):
            patch *= 0.001
        valid = patch[np.isfinite(patch) & (patch > 0.10) & (patch < 10.0)]
        if valid.size < 5:
            self.rejected += 1
            return

        depth_m = float(np.median(valid))
        scale = depth_m / float(position.z)
        sample = {
            "depth": depth_m,
            "pose_x": float(position.x),
            "pose_y": float(position.y),
            "pose_z": float(position.z),
            "corrected_size": self.configured_size * scale,
            "corrected_x": float(position.x) * scale,
            "corrected_y": float(position.y) * scale,
            "pixel_u": float(u),
            "pixel_v": float(v),
        }
        with self.lock:
            self.samples.append(sample)

    def collect(self):
        start = time.monotonic()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            with self.lock:
                count = len(self.samples)
            if count >= self.sample_count or time.monotonic() - start >= self.timeout_s:
                break
            rate.sleep()
        with self.lock:
            return list(self.samples)


def _print_stat(label, stat, unit="m"):
    print(
        "{}: median={:.6f} {}, std={:.6f}, p05={:.6f}, p95={:.6f} "
        "(kept {}/{})".format(
            label,
            stat["median"],
            unit,
            stat["std"],
            stat["p05"],
            stat["p95"],
            stat["kept"],
            stat["total"],
        )
    )


def main():
    rospy.init_node("calibrate_apriltag_depth", anonymous=True)
    calibrator = DepthTagCalibrator()
    samples = calibrator.collect()
    if len(samples) < 20:
        rospy.logfatal(
            "only %d valid synchronized samples (rejected %d); need at least 20",
            len(samples), calibrator.rejected,
        )
        raise SystemExit(2)

    stats = {
        key: _robust_summary([sample[key] for sample in samples])
        for key in samples[0]
    }
    print("\nAprilTag + aligned-depth calibration (read-only)")
    print("tag_id: {}".format(calibrator.tag_id))
    print("configured_tag_size_m: {:.6f}".format(calibrator.configured_size))
    print("valid_samples: {}  rejected_callbacks: {}".format(
        len(samples), calibrator.rejected
    ))
    _print_stat("D435 aligned depth", stats["depth"])
    _print_stat("AprilTag pose z (old size)", stats["pose_z"])
    _print_stat("recommended tag edge size", stats["corrected_size"])
    _print_stat("current tag center x after scale correction", stats["corrected_x"])
    _print_stat("current tag center y after scale correction", stats["corrected_y"])
    _print_stat("projected center u", stats["pixel_u"], "px")
    _print_stat("projected center v", stats["pixel_v"], "px")

    if calibrator.known_distance > 0.0:
        known_size = (
            calibrator.configured_size
            * calibrator.known_distance
            / stats["pose_z"]["median"]
        )
        difference = stats["depth"]["median"] - calibrator.known_distance
        print("known_distance_m: {:.6f}".format(calibrator.known_distance))
        print("tag size inferred only from known distance: {:.6f} m".format(known_size))
        print("D435 depth minus known distance: {:+.6f} m".format(difference))

    print("\nYAML recommendation:")
    print("  tag_size_m: {:.5f}".format(stats["corrected_size"]["median"]))
    print(
        "  # x/y below are valid gripper offsets only when the tag center is "
        "physically under the gripper center"
    )
    print("  observed_center_x_m: {:.5f}".format(stats["corrected_x"]["median"]))
    print("  observed_center_y_m: {:.5f}".format(stats["corrected_y"]["median"]))


if __name__ == "__main__":
    main()
