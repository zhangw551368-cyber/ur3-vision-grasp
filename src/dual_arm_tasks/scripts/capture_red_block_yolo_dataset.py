#!/usr/bin/env python3

import argparse
import os
import random
import sys
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture RealSense images and create one-class YOLO labels for the visible red block."
    )
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--output", default="/home/gzu/gzu_ws/datasets/red_block_autolabel")
    parser.add_argument("--count", type=int, default=120)
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--min-area", type=int, default=500)
    parser.add_argument("--debug-dir", default="/tmp/red_block_yolo_autolabel")
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def ensure_dirs(root):
    for split in ("train", "val"):
        os.makedirs(os.path.join(root, "images", split), exist_ok=True)
        os.makedirs(os.path.join(root, "labels", split), exist_ok=True)


def red_bbox(bgr, min_area):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 80, 40]), np.array([12, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([168, 80, 40]), np.array([180, 255, 255]))
    mask = mask1 | mask2
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < min_area:
        return None, mask
    x, y, w, h = cv2.boundingRect(contour)
    return (x, y, x + w, y + h), mask


def yolo_label_from_bbox(bbox, width, height):
    x1, y1, x2, y2 = bbox
    cx = ((x1 + x2) * 0.5) / width
    cy = ((y1 + y2) * 0.5) / height
    bw = (x2 - x1) / width
    bh = (y2 - y1) / height
    return "0 {:.6f} {:.6f} {:.6f} {:.6f}\n".format(cx, cy, bw, bh)


def write_data_yaml(root):
    path = os.path.join(root, "data.yaml")
    with open(path, "w", encoding="utf-8") as stream:
        stream.write("path: {}\n".format(root))
        stream.write("train: images/train\n")
        stream.write("val: images/val\n")
        stream.write("names:\n")
        stream.write("  0: red\n")
    return path


def main():
    args = parse_args()
    ensure_dirs(args.output)
    os.makedirs(args.debug_dir, exist_ok=True)

    rospy.init_node("capture_red_block_yolo_dataset", anonymous=True)
    bridge = CvBridge()
    interval = 1.0 / max(args.rate, 0.1)
    saved = 0
    attempts = 0

    rospy.loginfo(
        "Capturing %d auto-labeled red-block frames from %s",
        args.count,
        args.image_topic,
    )
    while saved < args.count and not rospy.is_shutdown():
        attempts += 1
        msg = rospy.wait_for_message(args.image_topic, Image, timeout=3.0)
        bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        bbox, mask = red_bbox(bgr, args.min_area)
        if bbox is None:
            rospy.logwarn_throttle(1.0, "No red block bbox found; adjust lighting or min-area")
            continue

        split = "val" if random.random() < args.val_ratio else "train"
        stem = "red_block_{:05d}".format(saved)
        image_path = os.path.join(args.output, "images", split, stem + ".jpg")
        label_path = os.path.join(args.output, "labels", split, stem + ".txt")
        cv2.imwrite(image_path, bgr)
        height, width = bgr.shape[:2]
        with open(label_path, "w", encoding="utf-8") as stream:
            stream.write(yolo_label_from_bbox(bbox, width, height))

        debug = bgr.copy()
        x1, y1, x2, y2 = bbox
        cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.imwrite(os.path.join(args.debug_dir, stem + ".jpg"), debug)

        saved += 1
        rospy.loginfo_throttle(
            1.0,
            "Saved %d/%d red-block YOLO samples after %d attempts",
            saved,
            args.count,
            attempts,
        )
        time.sleep(interval)

    data_yaml = write_data_yaml(args.output)
    rospy.loginfo("Dataset ready: %s", data_yaml)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        sys.exit(1)
