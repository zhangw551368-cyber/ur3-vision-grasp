#!/usr/bin/env python3

"""Read-only RGB-D check of blue objects against a locked target YAML.

This is the fallback validator used when an already placed block hides enough
checkerboard corners that the full board detector can no longer run.  It never
publishes robot, controller, or gripper commands.
"""

import argparse
import itertools
import sys
import threading

import cv2
import message_filters
import numpy as np
import rospy
import tf.transformations
import tf2_ros
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--targets-file", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--max-shift", type=float, default=0.012)
    parser.add_argument("--max-mad", type=float, default=0.006)
    parser.add_argument("--association-radius", type=float, default=0.080)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class LockedBlueValidator:
    def __init__(self, args, task_config, locked, target_frame):
        self.args = args
        self.config = task_config
        self.locked = np.asarray(locked, dtype=float)
        if self.locked.ndim != 2 or self.locked.shape[1] != 3:
            raise RuntimeError("object_top_points must be an Nx3 list")
        if len(self.locked) < 1 or len(self.locked) > 3:
            raise RuntimeError("validator supports one to three locked objects")

        self.bridge = CvBridge()
        self.samples = []
        self.last_diagnostic = None
        self.done = threading.Event()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.color_topic = task_config.get(
            "image_topic", "/camera/color/image_raw"
        )
        self.depth_topic = task_config.get(
            "depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.info_topic = task_config.get(
            "camera_info_topic", "/camera/color/camera_info"
        )
        self.camera_frame = task_config.get(
            "camera_frame", "camera_color_optical_frame"
        )
        self.target_frame = target_frame

        info = rospy.wait_for_message(self.info_topic, CameraInfo, timeout=3.0)
        if info.header.frame_id:
            self.camera_frame = info.header.frame_id
        self.camera_matrix = np.asarray(info.K, dtype=float).reshape(3, 3)
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            self.camera_frame,
            rospy.Time(0),
            rospy.Duration(3.0),
        ).transform
        quaternion = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
        self.rotation = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
        self.translation = np.asarray(
            [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ]
        )

        self.color_sub = message_filters.Subscriber(self.color_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.08
        )
        self.sync.registerCallback(self.callback)

    def blue_mask(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = np.asarray(
            [
                self.config.get("h_min", 90),
                self.config.get("s_min", 128),
                self.config.get("v_min", 135),
            ],
            dtype=np.uint8,
        )
        upper = np.asarray(
            [
                self.config.get("h_max", 108),
                self.config.get("s_max", 255),
                self.config.get("v_max", 255),
            ],
            dtype=np.uint8,
        )
        mask = cv2.inRange(hsv, lower, upper)
        close_size = int(self.config.get("close_kernel", 5))
        if close_size > 0:
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                np.ones((close_size, close_size), np.uint8),
            )
        dilate_size = int(self.config.get("dilate_kernel", 3))
        iterations = int(self.config.get("dilate_iterations", 1))
        if dilate_size > 0 and iterations > 0:
            mask = cv2.dilate(
                mask,
                np.ones((dilate_size, dilate_size), np.uint8),
                iterations=iterations,
            )
        return mask

    def contour_point(self, contour, depth):
        x, y, width, height = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        fill = area / float(max(1, width * height))
        min_area = max(700.0, float(self.config.get("min_area", 1200.0)) * 0.55)
        max_area = max(18000.0, float(self.config.get("max_area", 15000.0)))
        if not (min_area <= area <= max_area):
            return None
        if not (18 <= width <= 170 and 15 <= height <= 170 and fill >= 0.25):
            return None

        component = np.zeros(depth.shape[:2], dtype=np.uint8)
        cv2.drawContours(component, [contour], -1, 255, thickness=cv2.FILLED)
        component = cv2.erode(component, np.ones((3, 3), np.uint8), iterations=1)
        rows, cols = np.nonzero(
            (component > 0)
            & np.isfinite(depth)
            & (depth >= float(self.config.get("min_depth_m", 0.05)))
            & (depth <= float(self.config.get("max_depth_m", 3.0)))
        )
        if len(rows) < 60:
            return None
        z = depth[rows, cols]
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        points_camera = np.column_stack(
            ((cols - cx) * z / fx, (rows - cy) * z / fy, z)
        )
        point_camera = np.median(points_camera, axis=0)
        point_target = self.rotation.dot(point_camera) + self.translation
        return point_target, (x, y, width, height, area)

    def associate(self, candidates):
        if len(candidates) < len(self.locked):
            return None
        best = None
        for indices in itertools.permutations(
            range(len(candidates)), len(self.locked)
        ):
            distances = np.asarray(
                [
                    np.linalg.norm(candidates[index][0] - self.locked[row])
                    for row, index in enumerate(indices)
                ]
            )
            score = float(np.sum(distances))
            if best is None or score < best[0]:
                best = (score, distances, indices)
        if best is None or np.max(best[1]) > self.args.association_radius:
            return None
        return np.asarray([candidates[index][0] for index in best[2]])

    def callback(self, color_message, depth_message):
        if self.done.is_set():
            return
        image = self.bridge.imgmsg_to_cv2(color_message, desired_encoding="bgr8")
        depth = np.asarray(
            self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough"),
            dtype=float,
        )
        if depth_message.encoding in ("16UC1", "mono16"):
            depth *= 0.001
        if image.shape[:2] != depth.shape[:2]:
            self.last_diagnostic = "color/depth size mismatch"
            return

        contours = cv2.findContours(
            self.blue_mask(image), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[-2]
        candidates = []
        for contour in contours:
            result = self.contour_point(contour, depth)
            if result is not None:
                candidates.append(result)
        associated = self.associate(candidates)
        self.last_diagnostic = "candidates={} xyz={}".format(
            len(candidates),
            [np.round(item[0], 4).tolist() for item in candidates],
        )
        if associated is not None:
            self.samples.append(associated)
            if len(self.samples) >= self.args.samples:
                self.done.set()

    def run(self):
        if not self.done.wait(self.args.timeout):
            raise RuntimeError(
                "only received {}/{} associated samples; {}".format(
                    len(self.samples), self.args.samples, self.last_diagnostic
                )
            )
        samples = np.asarray(self.samples)
        median = np.median(samples, axis=0)
        shift = np.linalg.norm(median - self.locked, axis=1)
        mad = np.median(np.abs(samples - median), axis=0)
        for index in range(len(self.locked)):
            rospy.loginfo(
                "object%d live=%s shift_mm=%.2f MAD_mm=%s",
                index + 1,
                np.round(median[index], 6).tolist(),
                shift[index] * 1000.0,
                np.round(mad[index] * 1000.0, 2).tolist(),
            )
        if np.max(shift) > self.args.max_shift:
            raise RuntimeError(
                "locked object moved: shift_mm={}".format(
                    np.round(shift * 1000.0, 2).tolist()
                )
            )
        if np.max(mad) > self.args.max_mad:
            raise RuntimeError(
                "live detection is unstable: max_MAD_mm={:.2f}".format(
                    np.max(mad) * 1000.0
                )
            )
        rospy.loginfo(
            "LOCKED BLUE TARGET VALIDATION PASSED: samples=%d", len(samples)
        )


def main():
    args = parse_args()
    task_config = load_yaml(args.config)
    targets = load_yaml(args.targets_file)
    target_frame = str(targets.get("frame_id", "")).strip()
    if not target_frame:
        raise RuntimeError("targets file has an empty frame_id")
    rospy.init_node("validate_locked_blue_targets")
    LockedBlueValidator(
        args, task_config, targets["object_top_points"], target_frame
    ).run()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("Locked blue target validation failed: %s", exc)
        sys.exit(1)
