#!/usr/bin/python3

import math

import cv2
import numpy as np
import rospy
import tf.transformations
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image


class CheckerboardPosePublisher:
    def __init__(self):
        self.pattern_cols = int(rospy.get_param("~pattern_cols", 5))
        self.pattern_rows = int(rospy.get_param("~pattern_rows", 5))
        self.square_size = float(rospy.get_param("~square_size", 0.03))
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )
        self.camera_frame = rospy.get_param("~camera_frame", "")
        self.checkerboard_frame = rospy.get_param(
            "~checkerboard_frame", "camera_checkerboard"
        )
        self.center_origin = bool(rospy.get_param("~center_origin", True))
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", True))
        self.use_sb_detector = bool(rospy.get_param("~use_sb_detector", True))
        self.roi_x_min = int(rospy.get_param("~roi_x_min", 0))
        self.roi_y_min = int(rospy.get_param("~roi_y_min", 0))
        self.roi_x_max = int(rospy.get_param("~roi_x_max", 0))
        self.roi_y_max = int(rospy.get_param("~roi_y_max", 0))

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion = None
        self.last_rotation = None
        self.object_points = self.make_object_points(reverse=False)
        self.reversed_object_points = self.make_object_points(reverse=True)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.pose_pub = rospy.Publisher("checkerboard_pose", PoseStamped, queue_size=1)
        self.debug_pub = rospy.Publisher(
            "checkerboard_debug_image", Image, queue_size=1
        )
        rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1
        )
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        rospy.loginfo(
            "Checkerboard detector: pattern=%dx%d square=%.3fm image=%s info=%s frame=%s roi=[%d,%d,%d,%d]",
            self.pattern_cols,
            self.pattern_rows,
            self.square_size,
            self.image_topic,
            self.camera_info_topic,
            self.checkerboard_frame,
            self.roi_x_min,
            self.roi_y_min,
            self.roi_x_max,
            self.roi_y_max,
        )

    def make_object_points(self, reverse=False):
        points = []
        x_center = (self.pattern_cols - 1) * self.square_size / 2.0
        y_center = (self.pattern_rows - 1) * self.square_size / 2.0
        for row in range(self.pattern_rows):
            for col in range(self.pattern_cols):
                x = col * self.square_size
                y = row * self.square_size
                if self.center_origin:
                    x -= x_center
                    y -= y_center
                points.append([x, y, 0.0])
        if reverse:
            points.reverse()
        return np.asarray(points, dtype=np.float32)

    def camera_info_callback(self, info):
        self.camera_matrix = np.asarray(info.K, dtype=np.float64).reshape(3, 3)
        self.distortion = np.asarray(info.D, dtype=np.float64)
        if not self.camera_frame:
            self.camera_frame = info.header.frame_id or "camera_color_optical_frame"

    def detection_roi(self, gray):
        height, width = gray.shape[:2]
        x0 = max(0, min(width, self.roi_x_min))
        y0 = max(0, min(height, self.roi_y_min))
        x1 = self.roi_x_max if self.roi_x_max > 0 else width
        y1 = self.roi_y_max if self.roi_y_max > 0 else height
        x1 = max(x0, min(width, x1))
        y1 = max(y0, min(height, y1))
        if x1 <= x0 or y1 <= y0:
            return gray, 0, 0
        return gray[y0:y1, x0:x1], x0, y0

    def find_corners(self, gray):
        detection_image, offset_x, offset_y = self.detection_roi(gray)
        pattern = (self.pattern_cols, self.pattern_rows)
        if self.use_sb_detector and hasattr(cv2, "findChessboardCornersSB"):
            flags = cv2.CALIB_CB_NORMALIZE_IMAGE
            if hasattr(cv2, "CALIB_CB_EXHAUSTIVE"):
                flags |= cv2.CALIB_CB_EXHAUSTIVE
            if hasattr(cv2, "CALIB_CB_ACCURACY"):
                flags |= cv2.CALIB_CB_ACCURACY
            ok, corners = cv2.findChessboardCornersSB(
                detection_image, pattern, flags=flags
            )
            if ok:
                corners = corners.astype(np.float32)
                corners[:, 0, 0] += offset_x
                corners[:, 0, 1] += offset_y
                return corners

        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        ok, corners = cv2.findChessboardCorners(detection_image, pattern, flags)
        if not ok:
            return None
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )
        cv2.cornerSubPix(detection_image, corners, (5, 5), (-1, -1), criteria)
        corners[:, 0, 0] += offset_x
        corners[:, 0, 1] += offset_y
        return corners

    def solve_pose(self, object_points, corners):
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            corners,
            self.camera_matrix,
            self.distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None
        rotation, _ = cv2.Rodrigues(rvec)
        return rotation, tvec.reshape(3), rvec

    @staticmethod
    def rotation_distance(first, second):
        relative = first.T.dot(second)
        trace = np.trace(relative)
        value = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
        return math.acos(value)

    def choose_pose(self, corners):
        pose = self.solve_pose(self.object_points, corners)
        reversed_pose = self.solve_pose(self.reversed_object_points, corners)
        if pose is None:
            return reversed_pose
        if reversed_pose is None or self.last_rotation is None:
            return pose

        normal_error = self.rotation_distance(self.last_rotation, pose[0])
        reversed_error = self.rotation_distance(self.last_rotation, reversed_pose[0])
        if reversed_error < normal_error:
            return reversed_pose
        return pose

    def image_callback(self, image_msg):
        if self.camera_matrix is None:
            rospy.logwarn_throttle(2.0, "Waiting for camera info on %s", self.camera_info_topic)
            return

        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners = self.find_corners(gray)
        if corners is None:
            rospy.logwarn_throttle(
                2.0,
                "Checkerboard %dx%d was not detected",
                self.pattern_cols,
                self.pattern_rows,
            )
            if self.publish_debug_image and self.debug_pub.get_num_connections() > 0:
                self.debug_pub.publish(
                    self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                )
            return

        pose = self.choose_pose(corners)
        if pose is None:
            rospy.logwarn_throttle(2.0, "solvePnP failed for checkerboard")
            return
        rotation, translation, rvec = pose
        self.last_rotation = rotation
        self.publish_pose(image_msg.header.stamp, translation, rotation)

        if self.publish_debug_image and self.debug_pub.get_num_connections() > 0:
            debug = image.copy()
            cv2.drawChessboardCorners(
                debug,
                (self.pattern_cols, self.pattern_rows),
                corners,
                True,
            )
            try:
                cv2.drawFrameAxes(
                    debug,
                    self.camera_matrix,
                    self.distortion,
                    rvec,
                    translation.reshape(3, 1),
                    self.square_size * 2.0,
                )
            except AttributeError:
                pass
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            )

    def publish_pose(self, stamp, translation, rotation):
        transform = TransformStamped()
        transform.header.stamp = stamp if stamp != rospy.Time() else rospy.Time.now()
        transform.header.frame_id = self.camera_frame
        transform.child_frame_id = self.checkerboard_frame
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])

        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        quaternion = tf.transformations.quaternion_from_matrix(matrix)
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self.tf_broadcaster.sendTransform(transform)

        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.pose_pub.publish(pose)


if __name__ == "__main__":
    rospy.init_node("checkerboard_pose_publisher")
    CheckerboardPosePublisher()
    rospy.spin()
