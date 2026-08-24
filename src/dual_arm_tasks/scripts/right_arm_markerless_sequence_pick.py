#!/usr/bin/python3

import argparse
import math
import os
import sys
import time

import cv2
import moveit_commander
import numpy as np
import rospy
import tf.transformations
import tf2_ros
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from right_arm_visual_pick import RightArmVisualPick


def parse_bool_arg(value):
    lowered = str(value).strip().lower()
    if lowered in ("1", "true", "yes", "on"):
        return True
    if lowered in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError("Expected boolean, got {!r}".format(value))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Detect markerless teaching blocks in Kinect2 RGB-D and pick them in a configured order."
    )
    parser.add_argument("--config", required=True)
    parser.add_argument(
        "--sequence",
        default="",
        help="Comma-separated order, e.g. banana,apple,clock,bird,red,black.",
    )
    parser.add_argument(
        "--execute",
        nargs="?",
        const=True,
        default=False,
        type=parse_bool_arg,
        help="Execute real robot motion.",
    )
    parser.add_argument(
        "--yes",
        nargs="?",
        const=True,
        default=False,
        type=parse_bool_arg,
        help="Skip typed EXECUTE confirmation.",
    )
    parser.add_argument(
        "--detect-only",
        nargs="?",
        const=True,
        default=False,
        type=parse_bool_arg,
        help="Detect and save debug image only.",
    )
    parser.add_argument(
        "--allow-missing",
        nargs="?",
        const=True,
        default=False,
        type=parse_bool_arg,
        help="Skip missing labels instead of failing.",
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise RuntimeError("Invalid YAML: {}".format(path))
    return data


def clamp_roi(roi, shape):
    height, width = shape[:2]
    x_min = max(0, min(width, int(roi.get("x_min", 0))))
    y_min = max(0, min(height, int(roi.get("y_min", 0))))
    x_max = int(roi.get("x_max", 0)) or width
    y_max = int(roi.get("y_max", 0)) or height
    x_max = max(x_min, min(width, x_max))
    y_max = max(y_min, min(height, y_max))
    return x_min, y_min, x_max, y_max


def apply_roi(mask, roi, shape):
    x_min, y_min, x_max, y_max = clamp_roi(roi, shape)
    limited = np.zeros_like(mask)
    limited[y_min:y_max, x_min:x_max] = mask[y_min:y_max, x_min:x_max]
    return limited


def color_name(label):
    return {
        "banana": (0, 220, 255),
        "apple": (0, 0, 255),
        "bird": (255, 130, 0),
        "clock": (255, 0, 255),
        "clock_1": (255, 0, 255),
        "clock_2": (220, 0, 220),
        "red": (0, 0, 255),
        "black": (20, 20, 20),
    }.get(label, (255, 255, 255))


class MarkerlessSceneDetector:
    def __init__(self, config):
        self.config = config
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.last_color = None
        self.last_depth = None
        self.last_info = None

    def capture(self):
        timeout = float(self.config.get("image_timeout", 6.0))
        color_msg = rospy.wait_for_message(self.config["color_topic"], Image, timeout=timeout)
        depth_msg = rospy.wait_for_message(self.config["depth_topic"], Image, timeout=timeout)
        info_msg = rospy.wait_for_message(
            self.config["camera_info_topic"], CameraInfo, timeout=timeout
        )
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth = np.asarray(
            self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough"),
            dtype=np.float64,
        )
        if depth_msg.encoding in ("16UC1", "mono16"):
            depth *= 0.001
        depth[~np.isfinite(depth)] = 0.0
        self.last_color = color
        self.last_depth = depth
        self.last_info = info_msg
        return color, depth, info_msg

    @staticmethod
    def contour_stats(contour):
        x, y, width, height = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        fill = area / float(max(1, width * height))
        aspect = width / float(max(1, height))
        return x, y, width, height, area, fill, aspect

    def filter_contours(self, contours, params):
        selected = []
        for contour in contours:
            x, y, width, height, area, fill, aspect = self.contour_stats(contour)
            if area < float(params.get("min_area_pixels", 0)):
                continue
            max_area = float(params.get("max_area_pixels", 0))
            if max_area and area > max_area:
                continue
            if width < int(params.get("min_width_pixels", 0)):
                continue
            max_width = int(params.get("max_width_pixels", 0))
            if max_width and width > max_width:
                continue
            if height < int(params.get("min_height_pixels", 0)):
                continue
            max_height = int(params.get("max_height_pixels", 0))
            if max_height and height > max_height:
                continue
            min_aspect = float(params.get("min_aspect_ratio", 0.0))
            max_aspect = float(params.get("max_aspect_ratio", 0.0))
            if min_aspect and aspect < min_aspect:
                continue
            if max_aspect and aspect > max_aspect:
                continue
            # Avoid partial robot/gripper parts at the image edge.
            if x <= 2 or y <= 2 or x + width >= self.last_color.shape[1] - 2:
                continue
            max_x = int(params.get("max_x_pixels", 0))
            max_y = int(params.get("max_y_pixels", 0))
            if max_x and x + width > max_x:
                continue
            if max_y and y + height > max_y:
                continue
            selected.append((area, contour, (x, y, width, height, area, fill, aspect)))
        return sorted(selected, key=lambda item: item[0], reverse=True)

    @staticmethod
    def bbox_iou(first, second):
        ax, ay, aw, ah = first[:4]
        bx, by, bw, bh = second[:4]
        x0, y0 = max(ax, bx), max(ay, by)
        x1, y1 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
        if x1 <= x0 or y1 <= y0:
            return 0.0
        intersection = float((x1 - x0) * (y1 - y0))
        union = float(aw * ah + bw * bh) - intersection
        return intersection / max(1.0, union)

    def depth_at_bbox(self, bbox):
        x, y, width, height, _, _, _ = bbox
        depth_cfg = self.config.get("depth", {})
        min_depth = float(depth_cfg.get("min_depth", 0.05))
        max_depth = float(depth_cfg.get("max_depth", 2.0))
        cx = x + width // 2
        cy = y + height // 2
        half = max(2, int(depth_cfg.get("window_pixels", 11)) // 2)
        for radius in (half, half * 2, half * 4):
            y0, y1 = max(0, cy - radius), min(self.last_depth.shape[0], cy + radius + 1)
            x0, x1 = max(0, cx - radius), min(self.last_depth.shape[1], cx + radius + 1)
            values = self.last_depth[y0:y1, x0:x1]
            values = values[(values > min_depth) & (values < max_depth)]
            if values.size:
                return float(np.median(values)), cx, cy
        raise RuntimeError("No valid depth near bbox center {}".format(bbox[:4]))

    def pixel_to_base(self, u, v, z):
        info = self.last_info
        fx, fy = float(info.K[0]), float(info.K[4])
        cx, cy = float(info.K[2]), float(info.K[5])
        point_camera = np.array([(float(u) - cx) * z / fx, (float(v) - cy) * z / fy, z])
        frame_id = info.header.frame_id or self.config.get("camera_frame")
        transform = self.tf_buffer.lookup_transform(
            self.config["planning_frame"], frame_id, rospy.Time(0), rospy.Duration(3.0)
        ).transform
        quat = (
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )
        rotation = tf.transformations.quaternion_matrix(quat)[:3, :3]
        translation = np.array(
            [transform.translation.x, transform.translation.y, transform.translation.z]
        )
        return rotation.dot(point_camera) + translation

    def cloud_center_from_contour(self, contour):
        mask = np.zeros(self.last_depth.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
        rows, cols = np.nonzero(mask > 0)
        depth_cfg = self.config.get("depth", {})
        if len(rows) > int(depth_cfg.get("max_pixels_for_cloud", 5000)):
            step = max(1, len(rows) // int(depth_cfg.get("max_pixels_for_cloud", 5000)))
            rows, cols = rows[::step], cols[::step]
        z = self.last_depth[rows, cols]
        valid = (z > float(depth_cfg.get("min_depth", 0.05))) & (
            z < float(depth_cfg.get("max_depth", 2.0))
        )
        rows, cols, z = rows[valid], cols[valid], z[valid]
        if len(rows) < 80:
            raise RuntimeError("Not enough valid depth pixels for contour")
        info = self.last_info
        fx, fy = float(info.K[0]), float(info.K[4])
        cx, cy = float(info.K[2]), float(info.K[5])
        points_camera = np.stack(((cols - cx) * z / fx, (rows - cy) * z / fy, z), axis=1)
        frame_id = info.header.frame_id or self.config.get("camera_frame")
        transform = self.tf_buffer.lookup_transform(
            self.config["planning_frame"], frame_id, rospy.Time(0), rospy.Duration(3.0)
        ).transform
        rotation = tf.transformations.quaternion_matrix(
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            )
        )[:3, :3]
        translation = np.array(
            [transform.translation.x, transform.translation.y, transform.translation.z]
        )
        points_base = (rotation.dot(points_camera.T)).T + translation
        q = float(depth_cfg.get("bounds_quantile", 0.06))
        lower = np.quantile(points_base, q, axis=0)
        upper = np.quantile(points_base, 1.0 - q, axis=0)
        return ((lower + upper) * 0.5).tolist()

    def classify_sticker(self, crop):
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        colored = (s > 45) & (v > 45)
        total = max(1, int(np.count_nonzero(colored)))
        red = np.count_nonzero(colored & ((h < 14) | (h > 166)))
        yellow = np.count_nonzero(colored & (h >= 16) & (h <= 42))
        green = np.count_nonzero(colored & (h >= 43) & (h <= 88))
        blue = np.count_nonzero(colored & (h >= 90) & (h <= 135))
        dark = np.count_nonzero(v < 80)
        if red / float(total) > 0.55 and total < 0.20 * crop.shape[0] * crop.shape[1]:
            return "clock"
        if red > max(yellow, blue, green) * 0.8 and red + yellow > blue + green:
            return "apple"
        if yellow >= max(red, blue, green) * 0.8 and yellow > 8:
            return "banana"
        if blue + green > red + yellow * 0.5 and blue + green > 8:
            return "bird"
        if dark > 0.18 * crop.shape[0] * crop.shape[1]:
            return "bird"
        return "unknown"

    def detect_stickers(self, color, hsv, solid_bboxes=None):
        params = self.config.get("sticker", {})
        solid_bboxes = solid_bboxes or []
        max_iou = float(params.get("solid_overlap_iou_max", 0.0))
        mask = cv2.inRange(
            hsv,
            np.array([0, 0, int(params.get("white_value_min", 135))], dtype=np.uint8),
            np.array([180, int(params.get("white_saturation_max", 95)), 255], dtype=np.uint8),
        )
        mask = apply_roi(mask, self.config.get("roi", {}), color.shape)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        items = []
        for _area, contour, bbox in self.filter_contours(contours, params):
            if max_iou and any(self.bbox_iou(bbox, solid) > max_iou for solid in solid_bboxes):
                continue
            x, y, width, height, *_ = bbox
            crop = color[y : y + height, x : x + width]
            label = self.classify_sticker(crop)
            try:
                z, u, v = self.depth_at_bbox(bbox)
                center = self.pixel_to_base(u, v, z).tolist()
            except Exception as exc:
                rospy.logwarn("Skipping sticker bbox=%s: %s", bbox[:4], exc)
                continue
            offset = self.config.get("sticker_center_offset", [0.0, 0.0, -0.020])
            center = [center[i] + float(offset[i]) for i in range(3)]
            items.append({"label": label, "bbox": bbox, "center": center, "kind": "sticker"})
        return self.assign_sticker_fallbacks(items)

    def assign_sticker_fallbacks(self, items):
        if self.config.get("sticker", {}).get("force_spatial_layout_labels", False) and len(items) >= 4:
            for item in items:
                item["label"] = "unknown"
            top_two = sorted(items, key=lambda item: (item["bbox"][1], item["bbox"][0]))[:2]
            for index, item in enumerate(sorted(top_two, key=lambda item: item["bbox"][0]), start=1):
                item["label"] = "clock_{}".format(index)
            remaining = [item for item in items if item not in top_two]
            if remaining:
                bird = min(remaining, key=lambda item: item["bbox"][0])
                bird["label"] = "bird"
                remaining.remove(bird)
            if remaining:
                banana = max(remaining, key=lambda item: item["bbox"][0])
                banana["label"] = "banana"
                remaining.remove(banana)
            if remaining:
                apple = max(remaining, key=lambda item: item["bbox"][1])
                apple["label"] = "apple"
                remaining.remove(apple)
            for item in remaining:
                item["label"] = "unknown"
            return items

        labels = [item["label"] for item in items]
        missing = [name for name in ("bird", "banana", "apple") if name not in labels]
        unknown = [item for item in items if item["label"] == "unknown"]
        if missing and unknown:
            # Current teaching layout fallback: bird is leftmost, banana is rightmost,
            # apple is the lower sticker. This only runs when color features are weak.
            if "bird" in missing and unknown:
                left = min(unknown, key=lambda item: item["bbox"][0])
                left["label"] = "bird"
                unknown.remove(left)
            if "banana" in missing and unknown:
                right = max(unknown, key=lambda item: item["bbox"][0])
                right["label"] = "banana"
                unknown.remove(right)
            if "apple" in missing and unknown:
                low = max(unknown, key=lambda item: item["bbox"][1])
                low["label"] = "apple"
                unknown.remove(low)
        for item in unknown:
            item["label"] = "clock"
        clocks = sorted([item for item in items if item["label"] == "clock"], key=lambda item: item["bbox"][0])
        for index, item in enumerate(clocks, start=1):
            item["label"] = "clock_{}".format(index)
        return items

    def detect_solid(self, label, hsv):
        params = self.config.get(label, {})
        if label == "red":
            low_1 = np.array(params.get("low_1", [0, 80, 45]), dtype=np.uint8)
            high_1 = np.array(params.get("high_1", [14, 255, 255]), dtype=np.uint8)
            low_2 = np.array(params.get("low_2", [166, 80, 45]), dtype=np.uint8)
            high_2 = np.array(params.get("high_2", [180, 255, 255]), dtype=np.uint8)
            mask = cv2.inRange(hsv, low_1, high_1) | cv2.inRange(hsv, low_2, high_2)
        else:
            mask = cv2.inRange(
                hsv,
                np.array([0, int(params.get("saturation_min", 20)), 0], dtype=np.uint8),
                np.array([180, 255, int(params.get("value_max", 80))], dtype=np.uint8),
            )
        mask = apply_roi(mask, self.config.get("roi", {}), self.last_color.shape)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = self.filter_contours(contours, params)
        if not candidates:
            return None
        _area, contour, bbox = candidates[0]
        try:
            center = self.cloud_center_from_contour(contour)
        except Exception:
            z, u, v = self.depth_at_bbox(bbox)
            center = self.pixel_to_base(u, v, z).tolist()
        offset = self.config.get("solid_center_offset", [0.0, 0.0, 0.0])
        center = [center[i] + float(offset[i]) for i in range(3)]
        return {"label": label, "bbox": bbox, "center": center, "kind": "solid"}

    def detect(self):
        color, depth, info = self.capture()
        del depth, info
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        solids = []
        for label in ("red", "black"):
            item = self.detect_solid(label, hsv)
            if item is not None:
                solids.append(item)
        items = self.detect_stickers(color, hsv, [item["bbox"] for item in solids])
        items.extend(solids)
        return items

    def save_debug_image(self, items):
        if self.last_color is None:
            return
        image = self.last_color.copy()
        for item in items:
            x, y, width, height, *_ = item["bbox"]
            label = item["label"]
            color = color_name(label)
            cv2.rectangle(image, (x, y), (x + width, y + height), color, 2)
            cv2.putText(
                image,
                label,
                (x, max(18, y - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )
        path = os.path.expanduser(self.config.get("debug_image_path", "/tmp/markerless_debug.png"))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        cv2.imwrite(path, image)
        rospy.loginfo("Saved markerless detection debug image: %s", path)


def sequence_from_config(config, sequence_arg):
    if sequence_arg.strip():
        return [part.strip() for part in sequence_arg.split(",") if part.strip()]
    return list(config.get("default_sequence", ["banana", "apple", "clock", "bird", "red", "black"]))


def expand_sequence(items, sequence, allow_missing):
    by_label = {}
    for item in items:
        by_label.setdefault(item["label"], []).append(item)
    for label_items in by_label.values():
        label_items.sort(key=lambda item: (item["bbox"][1], item["bbox"][0]))
    ordered = []
    missing = []
    used = set()
    for token in sequence:
        matches = []
        if token == "clock":
            for label in sorted(by_label):
                if label.startswith("clock_"):
                    matches.extend(by_label[label])
        else:
            matches = by_label.get(token, [])
        if not matches:
            missing.append(token)
            continue
        for item in matches:
            identity = id(item)
            if identity not in used:
                ordered.append(item)
                used.add(identity)
    if missing and not allow_missing:
        raise RuntimeError("Missing requested targets: {}".format(", ".join(missing)))
    if missing:
        rospy.logwarn("Skipping missing targets: %s", ", ".join(missing))
    return ordered


def log_items(items):
    if not items:
        rospy.logwarn("No markerless targets detected.")
        return
    rospy.loginfo("Detected markerless targets:")
    for item in sorted(items, key=lambda entry: (entry["bbox"][1], entry["bbox"][0])):
        center = item["center"]
        rospy.loginfo(
            "  %-8s kind=%s bbox=%s center_%s=[%.3f, %.3f, %.3f]",
            item["label"],
            item["kind"],
            item["bbox"][:4],
            "base",
            center[0],
            center[1],
            center[2],
        )


def label_override(config, label, key, default=None):
    overrides = config.get(key + "_by_label", {})
    if not isinstance(overrides, dict):
        return default
    if label in overrides:
        return overrides[label]
    prefix = label.split("_", 1)[0]
    return overrides.get(prefix, default)


def apply_target_offset(config, label, target):
    offset = label_override(config, label, "target_offset", [0.0, 0.0, 0.0])
    return [target[index] + float(offset[index]) for index in range(3)]


def run_sequence(config, detector, sequence, execute, allow_missing):
    moveit_commander.roscpp_initialize(sys.argv)
    picker = RightArmVisualPick(config, execute)
    try:
        picker.ensure_external_control()
        picker.publish_gripper(config.get("open_position", 0), "open")
        items = detector.detect()
        detector.save_debug_image(items)
        ordered = expand_sequence(items, sequence, allow_missing)
        order_text = " -> ".join(item["label"] for item in ordered)
        if execute:
            rospy.loginfo("Execution order: %s", order_text)
        else:
            rospy.loginfo("Planned order: %s", order_text)
        for item in ordered:
            pick_one(picker, item)
    finally:
        moveit_commander.roscpp_shutdown()


def pick_one(picker, item):
    label = item["label"]
    target = apply_target_offset(picker.config, label, item["center"])
    original_values = {}
    for key in ("tool_to_grasp_center", "pre_grasp_clearance"):
        override = label_override(picker.config, label, key)
        if override is not None:
            original_values[key] = picker.config.get(key)
            picker.config[key] = override
    try:
        pre_grasp, grasp, lift, shift = picker.grasp_poses(target)
    finally:
        for key, value in original_values.items():
            picker.config[key] = value
    rospy.loginfo(
        "Target %s center=[%.3f, %.3f, %.3f]",
        label,
        target[0],
        target[1],
        target[2],
    )
    picker.virtual_start_state = None if picker.execute else picker.virtual_start_state
    picker.plan_to_pose(label + "_pre", pre_grasp)
    picker.cartesian_to_pose(label + "_grasp", grasp)
    picker.publish_gripper(picker.config.get("close_position", 220), "close " + label)
    picker.require_grasp()
    if picker.config.get("post_grasp_retreat_to_pregrasp", False):
        picker.cartesian_to_pose(label + "_retreat", pre_grasp)
        current = pre_grasp
        lift = picker.add(pre_grasp, picker.config.get("post_retreat_lift", [0.0, 0.0, 0.04]))
        shift = picker.add(lift, picker.config.get("post_grasp_shift", [0.0, -0.03, 0.0]))
    else:
        current = grasp
    if picker.distance(current, lift) > 1e-4:
        picker.cartesian_to_pose(label + "_lift", lift)
        current = lift
    if picker.distance(current, shift) > 1e-4:
        picker.cartesian_to_pose(label + "_shift", shift)
    if picker.execute:
        picker.publish_gripper(picker.config.get("open_position", 0), "open after " + label)


def main():
    args = parse_args()
    config = load_yaml(args.config)
    if args.execute and not config.get("enabled", False):
        raise RuntimeError("Real execution is locked: enabled is not true in YAML")
    rospy.init_node("right_arm_markerless_sequence_pick")
    if args.execute and not args.yes:
        answer = input("Markerless targets verified, workspace clear, E-stop reachable? Type EXECUTE: ")
        if answer != "EXECUTE":
            rospy.logwarn("Cancelled.")
            return
    detector = MarkerlessSceneDetector(config)
    sequence = sequence_from_config(config, args.sequence)
    rospy.loginfo("Requested markerless sequence: %s", " -> ".join(sequence))
    if args.detect_only:
        items = detector.detect()
        detector.save_debug_image(items)
        log_items(items)
        ordered = expand_sequence(items, sequence, args.allow_missing)
        rospy.loginfo("Resolved order: %s", " -> ".join(item["label"] for item in ordered))
        return
    mode = "REAL EXECUTION" if args.execute else "PLAN ONLY"
    rospy.loginfo("Mode: %s", mode)
    run_sequence(config, detector, sequence, args.execute, args.allow_missing)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
