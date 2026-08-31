#!/usr/bin/env python3

import json
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "python"))
sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import cv2
import numpy as np
import rospy
from PIL import Image as PilImage
from PIL import ImageDraw
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ur3_graspnet6dof.ros_image import decode_color
from ur3_graspnet6dof.detection_tracking import DetectionStabilizer


OUTPUT_TOPIC = "/ur3_graspnet6d/object_detection_image"
OBJECTS_TOPIC = "/ur3_graspnet6d/detected_objects_json"


def components(mask, minimum_area=30):
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), 8
    )
    result = []
    for index in range(1, count):
        x, y, width, height, area = [int(v) for v in stats[index]]
        if area < minimum_area:
            continue
        rows, cols = np.nonzero(labels == index)
        major_axis = np.array([1.0, 0.0], dtype=float)
        if len(cols) >= 3:
            points = np.column_stack((cols, rows)).astype(float)
            covariance = np.cov(points, rowvar=False)
            eigenvalues, eigenvectors = np.linalg.eigh(covariance)
            major_axis = eigenvectors[:, int(np.argmax(eigenvalues))]
            # A 2-D object axis is bidirectional.  Canonicalizing the sign makes
            # live JSON stable instead of letting eigenvector sign jitter.
            if major_axis[0] < 0.0 or (
                abs(float(major_axis[0])) < 1e-9 and major_axis[1] < 0.0
            ):
                major_axis = -major_axis
        result.append(
            {
                "bbox": [x, y, x + width, y + height],
                "area": area,
                "center": [float(centroids[index][0]), float(centroids[index][1])],
                "major_axis_image": major_axis.tolist(),
            }
        )
    return result


def merge_split_upright_rivets(detections, max_center_distance=45.0):
    """Merge highlight-separated head/shaft blobs belonging to one rivet."""
    result = []
    for detection in detections:
        if detection.get("category") != "upright_rivet":
            result.append(detection)
            continue
        center = np.asarray(detection["center"], dtype=float)
        matching_index = None
        for index, existing in enumerate(result):
            if existing.get("category") != "upright_rivet":
                continue
            existing_center = np.asarray(existing["center"], dtype=float)
            if np.linalg.norm(center - existing_center) <= max_center_distance:
                matching_index = index
                break
        if matching_index is None:
            result.append(detection)
            continue
        existing = result[matching_index]
        x1 = min(existing["bbox"][0], detection["bbox"][0])
        y1 = min(existing["bbox"][1], detection["bbox"][1])
        x2 = max(existing["bbox"][2], detection["bbox"][2])
        y2 = max(existing["bbox"][3], detection["bbox"][3])
        merged = dict(existing)
        merged["bbox"] = [x1, y1, x2, y2]
        merged["center"] = [(x1 + x2) * 0.5, (y1 + y2) * 0.5]
        result[matching_index] = merged
    return result


def expanded_box(box, margin, shape):
    x1, y1, x2, y2 = box
    height, width = shape[:2]
    return [
        max(0, x1 - margin),
        max(0, y1 - margin),
        min(width, x2 + margin),
        min(height, y2 + margin),
    ]


def in_box(point, box):
    return box[0] <= point[0] <= box[2] and box[1] <= point[1] <= box[3]


def detect_scene(rgb):
    height, width = rgb.shape[:2]
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    hue, saturation, value = cv2.split(hsv)
    detections = []
    excluded_boxes = []

    # Visible objects left of the calibrated pick workspace are reported for
    # scene understanding, but deliberately excluded from sequence planning.
    outside_bright = ((saturation < 80) & (value > 75)).astype(np.uint8)
    outside_bright[:, int(0.22 * width) :] = 0
    outside_bright[int(0.35 * height) :, :] = 0
    outside_bright = cv2.morphologyEx(
        outside_bright, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8)
    )
    outside_candidates = [
        item
        for item in components(outside_bright, minimum_area=450)
        if item["bbox"][2] - item["bbox"][0] >= 38
        and item["bbox"][3] - item["bbox"][1] >= 35
    ]
    if outside_candidates:
        item = max(outside_candidates, key=lambda candidate: candidate["area"])
        detections.append(
            {
                "category": "large_hex_fitting_outside_workspace",
                "category_zh": "大型六角件（工作区外）",
                "bbox": expanded_box(item["bbox"], 5, rgb.shape),
                "center": item["center"],
                "pickable": False,
                "priority": 0,
            }
        )

    outside_dark = (value < 92).astype(np.uint8)
    outside_dark[:, int(0.28 * width) :] = 0
    outside_dark[int(0.20 * height) :, :] = 0
    outside_dark = cv2.morphologyEx(
        outside_dark, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)
    )
    dark_candidates = []
    for item in components(outside_dark, minimum_area=40):
        x1, y1, x2, y2 = item["bbox"]
        component_width, component_height = x2 - x1, y2 - y1
        if 6 <= component_width <= 30 and 18 <= component_height <= 60:
            dark_candidates.append(item)
    if dark_candidates:
        item = max(dark_candidates, key=lambda candidate: candidate["area"])
        detections.append(
            {
                "category": "black_fastener_outside_workspace",
                "category_zh": "黑色紧固件（工作区外）",
                "bbox": expanded_box(item["bbox"], 5, rgb.shape),
                "center": item["center"],
                "pickable": False,
                "priority": 0,
            }
        )

    # The large calibration board is visible but must never become a pick target.
    board_mask = ((saturation < 75) & (value > 95)).astype(np.uint8)
    # Only the central worktable may contain the checkerboard.  This prevents
    # the white paper and aluminium frame at the right from joining its mask.
    board_mask[: int(0.40 * height), :] = 0
    board_mask[int(0.82 * height) :, :] = 0
    board_mask[:, : int(0.25 * width)] = 0
    board_mask[:, int(0.72 * width) :] = 0
    board_mask = cv2.morphologyEx(
        board_mask, cv2.MORPH_CLOSE, np.ones((21, 21), np.uint8)
    )
    board_candidates = components(board_mask, minimum_area=12000)
    if board_candidates:
        board = max(board_candidates, key=lambda item: item["area"])
        box = expanded_box(board["bbox"], 8, rgb.shape)
        detections.append(
            {
                "category": "calibration_board",
                "category_zh": "标定棋盘（排除）",
                "bbox": box,
                "center": board["center"],
                "pickable": False,
                "priority": 0,
            }
        )
        excluded_boxes.append(box)

    # The current silver cylinder lies horizontally near the upper image edge.
    # Detect its complete neutral body directly instead of relying on a blue
    # specular stripe, which appears and disappears with auto exposure.
    cylinder_mask = ((saturation < 105) & (value > 85)).astype(np.uint8)
    cylinder_mask[int(0.13 * height) :, :] = 0
    cylinder_mask[:, : int(0.42 * width)] = 0
    cylinder_mask[:, int(0.70 * width) :] = 0
    cylinder_mask = cv2.morphologyEx(
        cylinder_mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8)
    )
    cylinder_candidates = []
    for item in components(cylinder_mask, minimum_area=160):
        x1, y1, x2, y2 = item["bbox"]
        item_width, item_height = x2 - x1, y2 - y1
        if (
            50 <= item_width <= 140
            and 8 <= item_height <= 42
            and item_width / max(item_height, 1) >= 2.0
        ):
            cylinder_candidates.append(item)
    if cylinder_candidates:
        item = max(cylinder_candidates, key=lambda candidate: candidate["area"])
        box = expanded_box(item["bbox"], 7, rgb.shape)
        detections.append(
            {
                "category": "horizontal_cylinder",
                "category_zh": "横放银色圆柱体",
                "bbox": box,
                "center": item["center"],
                "major_axis_image": item["major_axis_image"],
                "pickable": True,
                "grasp_region": "middle",
                "priority": 1,
            }
        )
        excluded_boxes.append(box)

    # Detect the complete yellow tool before its blue handle caps.  Previously
    # those caps were also reported as a separate blue cutter instance.
    colour_specs = [
        ("yellow_pliers", "黄色尖嘴钳", (12, 42), 5),
        ("blue_cutters", "蓝色剪钳", (74, 132), 6),
    ]
    for category, category_zh, (low_hue, high_hue), priority in colour_specs:
        mask = (
            (hue >= low_hue)
            & (hue <= high_hue)
            & (saturation >= 90)
            & (value >= 45)
        ).astype(np.uint8)
        # Coloured tools are on the upper central work surface.  Limiting the
        # proposal region rejects the brown checkerboard rim and right frame.
        mask[int(0.31 * height) :, :] = 0
        mask[:, : int(0.25 * width)] = 0
        mask[:, int(0.73 * width) :] = 0
        for box in excluded_boxes:
            mask[box[1] : box[3], box[0] : box[2]] = 0
        mask = cv2.dilate(mask, np.ones((11, 11), np.uint8), iterations=1)
        candidates = components(mask, minimum_area=260)
        if not candidates:
            continue
        item = max(candidates, key=lambda candidate: candidate["area"])
        box = expanded_box(item["bbox"], 18, rgb.shape)
        detected_category = category
        detected_category_zh = category_zh
        detected_center = item["center"]
        # In the current scene the silver horizontal cylinder has a blue
        # specular stripe.  Its isolated top-edge elongated proposal is not a
        # blue cutter; expose the physical class required by the grasp policy.
        if (
            category == "blue_cutters"
            and item["center"][1] < 0.12 * height
            and (box[2] - box[0]) / max(box[3] - box[1], 1) >= 1.45
        ):
            detected_category = "horizontal_cylinder"
            detected_category_zh = "横放银色圆柱体"
        if category == "yellow_pliers":
            # Clamp across both yellow handles, away from the metal jaws. The
            # component box is pre-expansion and therefore follows the tool.
            component_box = item["bbox"]
            detected_center = [
                component_box[0] + 0.25 * (component_box[2] - component_box[0]),
                component_box[1] + 0.50 * (component_box[3] - component_box[1]),
            ]
            detected_category_zh = "黄色尖嘴钳（握把）"
        detections.append(
            {
                "category": detected_category,
                "category_zh": detected_category_zh,
                "bbox": box,
                "center": detected_center,
                "major_axis_image": item["major_axis_image"],
                "pickable": True,
                "grasp_region": "handles" if category == "yellow_pliers" else "body",
                "priority": priority,
            }
        )
        excluded_boxes.append(box)

    # Neutral/dark components provide class-independent proposals for metal parts.
    neutral = (((saturation < 82) & (value > 35)) | (value < 78)).astype(np.uint8)
    # Upright rivet-like cylinders may be placed close to the upper image edge.
    # Keep a small border for camera noise without discarding those targets.
    neutral[: int(0.02 * height), :] = 0
    neutral[int(0.72 * height) :, :] = 0
    neutral[:, : int(0.34 * width)] = 0
    neutral[:, int(0.72 * width) :] = 0
    for box in excluded_boxes:
        neutral[box[1] : box[3], box[0] : box[2]] = 0
    neutral = cv2.morphologyEx(neutral, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    neutral = cv2.morphologyEx(neutral, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    used = []
    for item in sorted(components(neutral, 32), key=lambda candidate: candidate["area"], reverse=True):
        x1, y1, x2, y2 = item["bbox"]
        box_width, box_height = x2 - x1, y2 - y1
        center = item["center"]
        if any(in_box(center, box) for box in used):
            continue
        patch_v = value[y1:y2, x1:x2]
        mean_v = float(np.mean(patch_v)) if patch_v.size else 255.0
        category = category_zh = None
        priority = 99
        # The taller/wider upright flange must be checked before the disc rule;
        # the two overlap in width under strong specular highlights.
        if (
            82 <= box_width <= 125
            and 38 <= box_height <= 78
            and box_width / max(box_height, 1) >= 1.25
        ):
            category, category_zh, priority = "metal_flange", "金属法兰/滑轮件", 3
        # Reflective discs can become narrow components under auto exposure.
        # Classify a bright elongated component before the similarly sized
        # black-connector rule so lighting changes do not rename one object.
        elif (
            45 <= box_width <= 90
            and 18 <= box_height <= 46
            and box_width / max(box_height, 1) >= 1.45
            and mean_v >= 145
        ):
            category, category_zh, priority = "metal_disc", "金属圆盘", 4
        # A real black connector must remain dark; image position alone is not
        # sufficient because the current metal disc occupies the same band.
        elif (
            18 <= box_width <= 58
            and 10 <= box_height <= 38
            and box_width / max(box_height, 1) >= 1.20
            and mean_v < 145
        ):
            category, category_zh, priority = "black_connector", "黑色小接头", 6
        elif (
            center[1] > 0.30 * height
            and 55 <= box_width <= 120
            and 14 <= box_height <= 48
            and box_width / max(box_height, 1) >= 1.65
        ):
            category, category_zh, priority = "silver_bolt", "横放银色螺栓", 1
        elif (
            58 <= box_width <= 105
            and 52 <= box_height <= 105
            and 0.72 <= box_width / max(box_height, 1) <= 1.35
            and mean_v >= 145
        ):
            category, category_zh, priority = "large_washer", "厚大垫圈", 2
        elif box_height >= 42 and 10 <= box_width <= 70 and box_height / max(box_width, 1) >= 1.35:
            category, category_zh, priority = "silver_bolt", "银色螺栓", 1
        elif 42 <= box_width <= 110 and 42 <= box_height <= 110:
            category, category_zh, priority = "large_ring", "薄金属圆环", 7
        elif 9 <= box_width <= 42 and 9 <= box_height <= 42:
            # The current scene contains three upright rivet-like cylinders.
            # Their reflective caps and dark stems form compact square neutral
            # components; give them an explicit category with a conservative
            # table-plane RGB grasp fallback instead of treating them as flat nuts.
            category, category_zh, priority = "upright_rivet", "立放圆柱铆钉", 2
        if category is None:
            continue
        box = expanded_box(item["bbox"], 5, rgb.shape)
        detections.append(
            {
                "category": category,
                "category_zh": category_zh,
                "bbox": box,
                "center": center,
                "major_axis_image": item["major_axis_image"],
                "pickable": True,
                "priority": priority,
            }
        )
        used.append(box)

    detections = merge_split_upright_rivets(detections)
    detections.sort(key=lambda item: (not item["pickable"], item["priority"]))
    return detections


def annotate(rgb, detections):
    image = PilImage.fromarray(rgb.astype(np.uint8), mode="RGB")
    draw = ImageDraw.Draw(image)
    pick_index = 0
    for detection in detections:
        x1, y1, x2, y2 = detection["bbox"]
        if detection["pickable"]:
            pick_index += 1
            colour = (255, 55, 35)
            label = "{}: {}".format(pick_index, detection["category"])
        else:
            colour = (180, 180, 180)
            label = "EXCLUDE: {}".format(detection["category"])
        draw.rectangle((x1, y1, x2, y2), outline=colour, width=4)
        text_y = max(0, y1 - 17)
        draw.rectangle((x1, text_y, x1 + 8 * len(label) + 6, y1), fill=(0, 0, 0))
        draw.text((x1 + 3, text_y + 2), label, fill=colour)
    return np.asarray(image)


def image_message(rgb, header):
    message = Image()
    message.header = header
    message.height, message.width = rgb.shape[:2]
    message.encoding = "rgb8"
    message.is_bigendian = 0
    message.step = message.width * 3
    message.data = np.ascontiguousarray(rgb).tobytes()
    return message


def main():
    rospy.init_node("ur3_graspnet6d_object_classifier")
    source_topic = rospy.get_param("~source_topic", "/camera/color/image_raw")
    annotated_pub = rospy.Publisher(OUTPUT_TOPIC, Image, queue_size=1, latch=True)
    objects_pub = rospy.Publisher(OBJECTS_TOPIC, String, queue_size=1, latch=True)
    runtime = PROJECT_ROOT / "runtime"
    runtime.mkdir(parents=True, exist_ok=True)
    state = {"last_process": 0.0, "last_save": 0.0}
    stabilizer = DetectionStabilizer()
    update_period = 1.0 / max(float(rospy.get_param("~update_rate", 2.0)), 0.1)

    def handle_image(source):
        now = rospy.get_time()
        if now - state["last_process"] < update_period:
            return
        state["last_process"] = now
        try:
            rgb = decode_color(source)
            raw_detections = detect_scene(rgb)
            detections = stabilizer.update(raw_detections)
            annotated = annotate(rgb, detections)
            payload = {
                "schema_version": 1,
                "frame_id": source.header.frame_id,
                "stamp": {
                    "secs": int(source.header.stamp.secs),
                    "nsecs": int(source.header.stamp.nsecs),
                },
                "image_size": [int(source.width), int(source.height)],
                "grasp_order": [
                    item["category"] for item in detections if item["pickable"]
                ],
                "objects": detections,
            }
            annotated_pub.publish(image_message(annotated, source.header))
            objects_pub.publish(
                String(
                    data=json.dumps(
                        payload, ensure_ascii=False, separators=(",", ":")
                    )
                )
            )
            # Disk artifacts are for inspection only; update them less often
            # than the GUI stream to avoid unnecessary PNG encoding load.
            if now - state["last_save"] >= 2.0:
                temporary = runtime / "latest_objects.json.tmp"
                temporary.write_text(
                    json.dumps(payload, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
                os.replace(str(temporary), str(runtime / "latest_objects.json"))
                PilImage.fromarray(annotated).save(runtime / "latest_objects.png")
                state["last_save"] = now
            rospy.loginfo_throttle(
                5.0,
                "Live classification: %d scene objects at %.1f Hz",
                len(detections),
                1.0 / update_period,
            )
        except Exception as exc:
            rospy.logerr_throttle(2.0, "Live classification failed: %s", exc)

    rospy.Subscriber(
        source_topic,
        Image,
        handle_image,
        queue_size=1,
        buff_size=16 * 1024 * 1024,
    )
    rospy.loginfo(
        "Live classifier subscribed to %s; output=%s at %.1f Hz",
        source_topic,
        OUTPUT_TOPIC,
        1.0 / update_period,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
