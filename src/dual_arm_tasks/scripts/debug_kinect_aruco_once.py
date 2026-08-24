#!/usr/bin/python3

import argparse
import os
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def parse_args():
    parser = argparse.ArgumentParser(
        description="Detect ArUco markers once from a ROS image topic and save an annotated frame."
    )
    parser.add_argument(
        "--image-topic",
        default="/kinect_0/kinect2/qhd/image_color_rect",
        help="ROS image topic to inspect.",
    )
    parser.add_argument(
        "--aruco-dictionary",
        default="DICT_4X4_50",
        help="OpenCV ArUco dictionary name, for example DICT_4X4_50.",
    )
    parser.add_argument(
        "--detection-scale",
        type=float,
        default=2.0,
        help="Upscale factor before marker detection.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=20.0,
        help="Seconds to wait for a detectable marker.",
    )
    parser.add_argument(
        "--output",
        default="/tmp/gzu_kinect_aruco_debug.png",
        help="Path for the annotated debug image.",
    )
    return parser.parse_args(rospy.myargv()[1:])


def aruco_dictionary(name):
    if not hasattr(cv2.aruco, name):
        raise RuntimeError("OpenCV has no ArUco dictionary named {}".format(name))
    dictionary_id = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, "Dictionary_get"):
        return cv2.aruco.Dictionary_get(dictionary_id)
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


def detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return params


def detect_markers(frame, dictionary, params, scale):
    detect_frame = frame
    if scale and abs(scale - 1.0) > 1e-6:
        detect_frame = cv2.resize(
            frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR
        )
    gray = cv2.cvtColor(detect_frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        return [], [], rejected, detect_frame
    if scale and abs(scale - 1.0) > 1e-6:
        corners = [corner / scale for corner in corners]
    return corners, ids.flatten().tolist(), rejected, frame.copy()


def main():
    args = parse_args()
    rospy.init_node("debug_kinect_aruco_once")
    bridge = CvBridge()
    dictionary = aruco_dictionary(args.aruco_dictionary)
    params = detector_parameters()
    deadline = time.time() + args.timeout
    last_frame = None
    last_stamp = None

    rospy.loginfo(
        "Waiting for ArUco markers on %s dictionary=%s scale=%.2f",
        args.image_topic,
        args.aruco_dictionary,
        args.detection_scale,
    )

    while not rospy.is_shutdown() and time.time() < deadline:
        remaining = max(0.1, deadline - time.time())
        try:
            msg = rospy.wait_for_message(args.image_topic, Image, timeout=min(remaining, 2.0))
        except rospy.ROSException:
            continue
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        last_frame = frame
        last_stamp = msg.header.stamp
        corners, ids, _rejected, annotated = detect_markers(
            frame, dictionary, params, args.detection_scale
        )
        if ids:
            cv2.aruco.drawDetectedMarkers(annotated, corners, None)
            os.makedirs(os.path.dirname(args.output), exist_ok=True)
            cv2.imwrite(args.output, annotated)
            rospy.loginfo("Detected ArUco IDs: %s", ids)
            rospy.loginfo("Annotated image saved to %s", args.output)
            return 0
        rospy.logwarn("No marker in current frame stamp=%s", last_stamp)

    if last_frame is not None:
        os.makedirs(os.path.dirname(args.output), exist_ok=True)
        cv2.imwrite(args.output, last_frame)
        rospy.logerr("No ArUco marker detected. Last raw frame saved to %s", args.output)
    else:
        rospy.logerr("No image received from %s", args.image_topic)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
