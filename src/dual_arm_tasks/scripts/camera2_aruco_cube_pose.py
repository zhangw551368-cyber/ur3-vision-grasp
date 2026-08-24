#!/usr/bin/python3

import math

import cv2
import numpy as np
import rospy
import tf2_geometry_msgs  # noqa: F401, registers PoseStamped transforms.
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


FACE_NAMES = {
    0: "top",
    1: "bottom",
    2: "front",
    3: "left",
    4: "back",
    5: "right",
}


def aruco_dictionary(name):
    if not hasattr(cv2.aruco, name):
        raise RuntimeError("Unknown OpenCV ArUco dictionary: {}".format(name))
    dictionary_id = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, "Dictionary_get"):
        return cv2.aruco.Dictionary_get(dictionary_id)
    return cv2.aruco.getPredefinedDictionary(dictionary_id)


def detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 51
    params.adaptiveThreshWinSizeStep = 4
    params.minMarkerPerimeterRate = 0.01
    params.maxMarkerPerimeterRate = 0.5
    params.polygonalApproxAccuracyRate = 0.06
    if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return params


def rotation_matrix_to_quaternion(matrix):
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (matrix[2, 1] - matrix[1, 2]) / s
        qy = (matrix[0, 2] - matrix[2, 0]) / s
        qz = (matrix[1, 0] - matrix[0, 1]) / s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        qw = (matrix[2, 1] - matrix[1, 2]) / s
        qx = 0.25 * s
        qy = (matrix[0, 1] + matrix[1, 0]) / s
        qz = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        qw = (matrix[0, 2] - matrix[2, 0]) / s
        qx = (matrix[0, 1] + matrix[1, 0]) / s
        qy = 0.25 * s
        qz = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        qw = (matrix[1, 0] - matrix[0, 1]) / s
        qx = (matrix[0, 2] + matrix[2, 0]) / s
        qy = (matrix[1, 2] + matrix[2, 1]) / s
        qz = 0.25 * s
    quat = np.array([qx, qy, qz, qw], dtype=float)
    quat /= np.linalg.norm(quat)
    return quat


def pose_from_rt(frame_id, stamp, rotation, translation):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = float(translation[0])
    pose.pose.position.y = float(translation[1])
    pose.pose.position.z = float(translation[2])
    qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation)
    pose.pose.orientation.x = float(qx)
    pose.pose.orientation.y = float(qy)
    pose.pose.orientation.z = float(qz)
    pose.pose.orientation.w = float(qw)
    return pose


class Camera2ArucoCubePose:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera2/color/image_raw")
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera2/aligned_depth_to_color/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera2/color/camera_info_calibrated"
        )
        self.camera_frame = rospy.get_param("~camera_frame", "camera2_color_optical_frame")
        self.base_frame = rospy.get_param("~base_frame", "right_arm_base")
        self.dictionary_name = rospy.get_param("~aruco_dictionary", "DICT_4X4_50")
        self.marker_size = float(rospy.get_param("~marker_size", 0.047))
        self.cube_size = float(rospy.get_param("~cube_size", 0.055))
        self.depth_window = int(rospy.get_param("~depth_window", 11))
        self.min_depth = float(rospy.get_param("~min_depth", 0.05))
        self.max_depth = float(rospy.get_param("~max_depth", 1.50))
        self.use_depth_for_top = bool(rospy.get_param("~use_depth_for_top", True))
        self.primary_id = int(rospy.get_param("~primary_id", 0))
        self.require_primary_id = bool(rospy.get_param("~require_primary_id", True))
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", True))
        self.max_center_spread = float(rospy.get_param("~max_center_spread", 0.020))
        target_ids = rospy.get_param("~target_ids", [0, 1, 2, 3, 4, 5])
        self.target_ids = set(int(value) for value in target_ids)

        self.bridge = CvBridge()
        self.dictionary = aruco_dictionary(self.dictionary_name)
        self.parameters = detector_parameters()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.camera_info = None
        self.depth_msg = None

        self.center_pub = rospy.Publisher(
            "/camera2_aruco_cube/center_pose_base", PoseStamped, queue_size=1
        )
        self.marker_pose_pub = rospy.Publisher(
            "/camera2_aruco_cube/marker_pose_base", PoseStamped, queue_size=1
        )
        self.ids_pub = rospy.Publisher(
            "/camera2_aruco_cube/visible_ids", Int32MultiArray, queue_size=1
        )
        self.marker_array_pub = rospy.Publisher(
            "/camera2_aruco_cube/rviz_markers", MarkerArray, queue_size=1
        )
        self.debug_pub = rospy.Publisher(
            "/camera2_aruco_cube/debug_image", Image, queue_size=1
        )

        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_cb, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)

        rospy.loginfo(
            "camera2 ArUco cube pose: ids=%s dictionary=%s marker=%.3fm cube=%.3fm",
            sorted(self.target_ids),
            self.dictionary_name,
            self.marker_size,
            self.cube_size,
        )

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def depth_cb(self, msg):
        self.depth_msg = msg

    def transform_to_base(self, pose):
        pose.header.stamp = rospy.Time(0)
        return self.tf_buffer.transform(pose, self.base_frame, rospy.Duration(1.0))

    def point_to_base(self, point):
        point.header.stamp = rospy.Time(0)
        return self.tf_buffer.transform(point, self.base_frame, rospy.Duration(1.0))

    def depth_center_in_base(self, center_u, center_v):
        if self.depth_msg is None:
            return None
        depth = self.bridge.imgmsg_to_cv2(self.depth_msg, desired_encoding="passthrough")
        window = max(3, self.depth_window)
        if window % 2 == 0:
            window += 1
        radius = window // 2
        u0 = max(0, int(round(center_u)) - radius)
        u1 = min(depth.shape[1], int(round(center_u)) + radius + 1)
        v0 = max(0, int(round(center_v)) - radius)
        v1 = min(depth.shape[0], int(round(center_v)) + radius + 1)
        patch = np.asarray(depth[v0:v1, u0:u1], dtype=np.float64)
        if self.depth_msg.encoding in ("16UC1", "mono16"):
            patch *= 0.001
        valid = patch[
            np.isfinite(patch)
            & (patch > self.min_depth)
            & (patch < self.max_depth)
        ]
        if len(valid) == 0:
            return None

        depth_m = float(np.median(valid))
        fx = float(self.camera_info.K[0])
        fy = float(self.camera_info.K[4])
        cx = float(self.camera_info.K[2])
        cy = float(self.camera_info.K[5])
        point = PointStamped()
        point.header.frame_id = self.camera_frame
        point.header.stamp = rospy.Time(0)
        point.point.x = (float(center_u) - cx) * depth_m / fx
        point.point.y = (float(center_v) - cy) * depth_m / fy
        point.point.z = depth_m
        return self.point_to_base(point)

    def image_cb(self, msg):
        if self.camera_info is None:
            rospy.logwarn_throttle(2.0, "Waiting for %s", self.camera_info_topic)
            return

        color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )
        if ids is None or len(ids) == 0:
            rospy.logwarn_throttle(1.0, "No ArUco cube marker detected.")
            return

        camera_matrix = np.asarray(self.camera_info.K, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.asarray(self.camera_info.D, dtype=np.float64).reshape(-1, 1)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, camera_matrix, dist_coeffs
        )

        visible_ids = []
        marker_poses_base = []
        center_poses_base = []
        primary_marker_pose_base = None
        primary_center_pose_base = None
        half = 0.5 * self.cube_size
        inward_offset_marker = np.array([0.0, 0.0, -half], dtype=float)

        annotated = color.copy()
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

        for index, marker_id in enumerate(ids.flatten().tolist()):
            marker_id = int(marker_id)
            if marker_id not in self.target_ids:
                continue
            selected_corners = corners[index].reshape(4, 2)
            rotation, _ = cv2.Rodrigues(rvecs[index].reshape(3))
            translation = tvecs[index].reshape(3)
            center_camera = translation + rotation.dot(inward_offset_marker)

            stamp = rospy.Time(0)
            marker_pose_camera = pose_from_rt(
                self.camera_frame, stamp, rotation, translation
            )
            center_pose_camera = pose_from_rt(
                self.camera_frame, stamp, rotation, center_camera
            )
            try:
                marker_pose_base = self.transform_to_base(marker_pose_camera)
                center_pose_base = self.transform_to_base(center_pose_camera)
            except tf2_ros.TransformException as exc:
                rospy.logwarn_throttle(1.0, "TF transform failed: %s", exc)
                continue

            if marker_id == 0 and self.use_depth_for_top:
                center_u, center_v = selected_corners.mean(axis=0)
                try:
                    top_center_base = self.depth_center_in_base(center_u, center_v)
                except tf2_ros.TransformException as exc:
                    rospy.logwarn_throttle(1.0, "Depth center TF failed: %s", exc)
                    top_center_base = None
                if top_center_base is not None:
                    center_pose_base.pose.position.x = top_center_base.point.x
                    center_pose_base.pose.position.y = top_center_base.point.y
                    center_pose_base.pose.position.z = (
                        top_center_base.point.z - half
                    )

            visible_ids.append(marker_id)
            marker_poses_base.append(marker_pose_base)
            center_poses_base.append(center_pose_base)
            if marker_id == self.primary_id:
                primary_marker_pose_base = marker_pose_base
                primary_center_pose_base = center_pose_base
            cv2.aruco.drawAxis(
                annotated,
                camera_matrix,
                dist_coeffs,
                rvecs[index],
                tvecs[index],
                self.marker_size * 0.5,
            )

        if not center_poses_base:
            rospy.logwarn_throttle(1.0, "Detected markers, but none in target IDs.")
            return

        if self.require_primary_id:
            self.ids_pub.publish(Int32MultiArray(data=visible_ids))
            if self.publish_debug_image:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))
            if primary_center_pose_base is None:
                rospy.logwarn_throttle(
                    1.0,
                    "Primary cube marker ID %d is not visible; visible_ids=%s. "
                    "Not publishing center pose for grasp.",
                    self.primary_id,
                    visible_ids,
                )
                return
            marker_poses_base = [primary_marker_pose_base]
            center_poses_base = [primary_center_pose_base]
            visible_ids = [self.primary_id]

        center_xyz = np.array(
            [
                [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ]
                for pose in center_poses_base
            ],
            dtype=float,
        )
        mean_xyz = np.mean(center_xyz, axis=0)
        spread = float(np.max(np.linalg.norm(center_xyz - mean_xyz, axis=1)))
        if spread > self.max_center_spread:
            rospy.logwarn(
                "Cube center estimates disagree: visible_ids=%s spread=%.3fm",
                visible_ids,
                spread,
            )

        output = PoseStamped()
        output.header.frame_id = self.base_frame
        output.header.stamp = rospy.Time.now()
        output.pose = center_poses_base[0].pose
        output.pose.position.x = float(mean_xyz[0])
        output.pose.position.y = float(mean_xyz[1])
        output.pose.position.z = float(mean_xyz[2])

        self.center_pub.publish(output)
        self.marker_pose_pub.publish(marker_poses_base[0])
        self.ids_pub.publish(Int32MultiArray(data=visible_ids))
        self.marker_array_pub.publish(self.make_markers(output, visible_ids))
        if self.publish_debug_image:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))

        faces = [FACE_NAMES.get(marker_id, "?") for marker_id in visible_ids]
        rospy.loginfo_throttle(
            0.5,
            "cube center in %s xyz=[%.3f, %.3f, %.3f], visible IDs=%s faces=%s spread=%.3fm",
            self.base_frame,
            mean_xyz[0],
            mean_xyz[1],
            mean_xyz[2],
            visible_ids,
            faces,
            spread,
        )

    def make_markers(self, pose, visible_ids):
        cube = Marker()
        cube.header = pose.header
        cube.ns = "camera2_aruco_cube"
        cube.id = 0
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose = pose.pose
        cube.pose.orientation.x = 0.0
        cube.pose.orientation.y = 0.0
        cube.pose.orientation.z = 0.0
        cube.pose.orientation.w = 1.0
        cube.scale.x = self.cube_size
        cube.scale.y = self.cube_size
        cube.scale.z = self.cube_size
        cube.color.r = 0.1
        cube.color.g = 0.8
        cube.color.b = 0.2
        cube.color.a = 0.45

        text = Marker()
        text.header = pose.header
        text.ns = "camera2_aruco_cube"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose = pose.pose
        text.pose.position.z += self.cube_size
        text.scale.z = 0.025
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = "cube ids {}".format(",".join(str(value) for value in visible_ids))
        return MarkerArray(markers=[cube, text])


def main():
    rospy.init_node("camera2_aruco_cube_pose")
    Camera2ArucoCubePose()
    rospy.spin()


if __name__ == "__main__":
    main()
