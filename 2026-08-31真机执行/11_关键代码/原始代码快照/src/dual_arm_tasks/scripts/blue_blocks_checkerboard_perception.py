#!/usr/bin/env python3

"""Detect multiple blue blocks and board-relative placement points.

This node is perception only.  It never sends robot or gripper commands.
"""

import math
import threading

import cv2
import numpy as np
import rospy
import tf.transformations
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


class BlueBlocksCheckerboardPerception:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/camera/aligned_depth_to_color/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )
        self.camera_frame = rospy.get_param(
            "~camera_frame", "camera_color_optical_frame"
        )
        self.target_frame = rospy.get_param("~target_frame", "right_arm_base")
        self.process_rate = float(rospy.get_param("~process_rate", 10.0))
        self.max_color_depth_dt = float(
            rospy.get_param("~max_color_depth_dt", 0.12)
        )

        self.h_min = int(rospy.get_param("~h_min", 90))
        self.h_max = int(rospy.get_param("~h_max", 108))
        self.s_min = int(rospy.get_param("~s_min", 128))
        self.s_max = int(rospy.get_param("~s_max", 255))
        self.v_min = int(rospy.get_param("~v_min", 135))
        self.v_max = int(rospy.get_param("~v_max", 255))
        self.close_kernel = self.odd_kernel(rospy.get_param("~close_kernel", 5))
        self.dilate_kernel = self.odd_kernel(
            rospy.get_param("~dilate_kernel", 3)
        )
        self.dilate_iterations = int(rospy.get_param("~dilate_iterations", 1))
        self.min_area = float(rospy.get_param("~min_area", 1200.0))
        self.max_area = float(rospy.get_param("~max_area", 15000.0))
        self.min_width = int(rospy.get_param("~min_width", 25))
        self.max_width = int(rospy.get_param("~max_width", 150))
        self.min_height = int(rospy.get_param("~min_height", 20))
        self.max_height = int(rospy.get_param("~max_height", 150))
        self.min_fill_ratio = float(rospy.get_param("~min_fill_ratio", 0.35))
        self.min_depth_m = float(rospy.get_param("~min_depth_m", 0.05))
        self.max_depth_m = float(rospy.get_param("~max_depth_m", 3.0))
        self.min_depth_samples = int(rospy.get_param("~min_depth_samples", 80))
        self.depth_erode_kernel = self.odd_kernel(
            rospy.get_param("~depth_erode_kernel", 3)
        )
        # Empirical camera-to-grasp correction in the aligned color image.
        # Positive v moves the commanded grasp downward in the overhead view.
        self.object_center_offset_u_px = float(
            rospy.get_param("~object_center_offset_u_px", 0.0)
        )
        self.object_center_offset_v_px = float(
            rospy.get_param("~object_center_offset_v_px", 0.0)
        )
        self.object_center_offset_min_aspect_ratio = float(
            rospy.get_param("~object_center_offset_min_aspect_ratio", 1.40)
        )
        self.object_center_offsets_px = rospy.get_param(
            "~object_center_offsets_px", []
        )
        self.expected_object_count = int(
            rospy.get_param("~expected_object_count", 3)
        )
        self.sort_mode = rospy.get_param("~sort_mode", "right_to_left")

        self.pattern_cols = int(rospy.get_param("~pattern_cols", 8))
        self.pattern_rows = int(rospy.get_param("~pattern_rows", 6))
        self.square_size = float(rospy.get_param("~square_size", 0.03))
        self.square_size_confirmed = bool(
            rospy.get_param("~square_size_confirmed", False)
        )
        self.min_object_height = float(
            rospy.get_param("~min_object_height", 0.015)
        )
        self.max_object_height = float(
            rospy.get_param("~max_object_height", 0.080)
        )
        self.roi_x_min = int(rospy.get_param("~board_roi_x_min", 350))
        self.roi_y_min = int(rospy.get_param("~board_roi_y_min", 160))
        self.roi_x_max = int(rospy.get_param("~board_roi_x_max", 950))
        self.roi_y_max = int(rospy.get_param("~board_roi_y_max", 600))
        self.board_exclusion_margin_px = int(
            rospy.get_param("~board_exclusion_margin_px", 10)
        )
        raw_offsets = rospy.get_param(
            "~placement_offsets",
            [[-0.075, 0.0, 0.0], [0.0, 0.0, 0.0], [0.075, 0.0, 0.0]],
        )
        if len(raw_offsets) != self.expected_object_count:
            raise RuntimeError(
                "placement_offsets must contain exactly {} xyz entries".format(
                    self.expected_object_count
                )
            )
        self.placement_offsets = np.asarray(raw_offsets, dtype=np.float64)
        if self.placement_offsets.shape != (self.expected_object_count, 3):
            raise RuntimeError("placement_offsets must be an Nx3 list")

        self.task_marker_topic = rospy.get_param(
            "~task_marker_topic", "/hsv_grasp/grasp_debug_markers"
        )
        self.task_grasp_above_center = float(
            rospy.get_param("~task_grasp_above_center", 0.020)
        )
        self.task_lift_distance = float(
            rospy.get_param("~task_lift_distance", 0.140)
        )
        self.task_release_tcp_clearance = float(
            rospy.get_param("~task_release_tcp_clearance", 0.120)
        )
        self.task_release_clearance = float(
            rospy.get_param("~task_release_clearance", 0.050)
        )
        self.task_active_object_indices = [
            int(value)
            for value in rospy.get_param("~task_active_object_indices", [0, 1, 2])
        ]
        self.task_placement_indices = [
            int(value)
            for value in rospy.get_param("~task_placement_indices", [0, 1, 2])
        ]
        if len(self.task_active_object_indices) != len(self.task_placement_indices):
            raise RuntimeError(
                "task_active_object_indices and task_placement_indices must match"
            )

        self.show_window = bool(rospy.get_param("~show_window", True))
        self.window_name = rospy.get_param(
            "~window_name", "blue blocks + checkerboard perception"
        )

        self.bridge = CvBridge()
        self.mutex = threading.Lock()
        self.color = None
        self.color_header = None
        self.depth = None
        self.depth_header = None
        self.camera_matrix = None
        self.distortion = None

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.objects_camera_pub = rospy.Publisher(
            "/hsv_grasp/blue_object_points_camera", PoseArray, queue_size=1
        )
        self.objects_target_pub = rospy.Publisher(
            "/hsv_grasp/blue_object_points_base", PoseArray, queue_size=1
        )
        self.selected_camera_pub = rospy.Publisher(
            "/hsv_grasp/selected_blue_object_camera", PointStamped, queue_size=1
        )
        self.board_pose_pub = rospy.Publisher(
            "/hsv_grasp/checkerboard_pose_camera", PoseStamped, queue_size=1
        )
        self.places_camera_pub = rospy.Publisher(
            "/hsv_grasp/checkerboard_place_points_camera", PoseArray, queue_size=1
        )
        self.places_target_pub = rospy.Publisher(
            "/hsv_grasp/checkerboard_place_points_base", PoseArray, queue_size=1
        )
        self.ready_pub = rospy.Publisher(
            "/hsv_grasp/three_block_scene_ready", Bool, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            "/hsv_grasp/three_block_scene_status", String, queue_size=1, latch=True
        )
        self.debug_pub = rospy.Publisher(
            "/hsv_grasp/blue_checkerboard_debug", Image, queue_size=1
        )
        self.task_marker_pub = rospy.Publisher(
            self.task_marker_topic, MarkerArray, queue_size=1, latch=True
        )

        rospy.Subscriber(
            self.image_topic, Image, self.color_callback, queue_size=1, buff_size=2**24
        )
        rospy.Subscriber(
            self.depth_topic, Image, self.depth_callback, queue_size=1, buff_size=2**24
        )
        rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.info_callback, queue_size=1
        )
        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.process_rate)), self.process)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "Blue/checkerboard perception only: HSV H[%d,%d] S[%d,%d] V[%d,%d], "
            "board=%dx%d square=%.4fm, expected_objects=%d",
            self.h_min,
            self.h_max,
            self.s_min,
            self.s_max,
            self.v_min,
            self.v_max,
            self.pattern_cols,
            self.pattern_rows,
            self.square_size,
            self.expected_object_count,
        )

    @staticmethod
    def odd_kernel(value):
        value = max(0, int(value))
        if value > 0 and value % 2 == 0:
            value += 1
        return value

    def color_callback(self, message):
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        with self.mutex:
            self.color = image
            self.color_header = message.header

    def depth_callback(self, message):
        image = np.asarray(
            self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough"),
            dtype=np.float64,
        )
        if message.encoding in ("16UC1", "mono16"):
            image *= 0.001
        with self.mutex:
            self.depth = image
            self.depth_header = message.header

    def info_callback(self, message):
        with self.mutex:
            self.camera_matrix = np.asarray(message.K, dtype=np.float64).reshape(3, 3)
            self.distortion = np.asarray(message.D, dtype=np.float64)
            if message.header.frame_id:
                self.camera_frame = message.header.frame_id

    def snapshot(self):
        with self.mutex:
            if (
                self.color is None
                or self.depth is None
                or self.camera_matrix is None
                or self.color_header is None
                or self.depth_header is None
            ):
                return None
            return (
                self.color.copy(),
                self.depth.copy(),
                self.color_header,
                self.depth_header,
                self.camera_matrix.copy(),
                self.distortion.copy(),
            )

    def board_object_points(self):
        x_center = (self.pattern_cols - 1) * self.square_size / 2.0
        y_center = (self.pattern_rows - 1) * self.square_size / 2.0
        return np.asarray(
            [
                [col * self.square_size - x_center, row * self.square_size - y_center, 0.0]
                for row in range(self.pattern_rows)
                for col in range(self.pattern_cols)
            ],
            dtype=np.float32,
        )

    def find_board(self, image, camera_matrix, distortion):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape[:2]
        x0 = max(0, min(width, self.roi_x_min))
        y0 = max(0, min(height, self.roi_y_min))
        x1 = max(x0, min(width, self.roi_x_max or width))
        y1 = max(y0, min(height, self.roi_y_max or height))
        cropped = gray[y0:y1, x0:x1]
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE
        if hasattr(cv2, "CALIB_CB_EXHAUSTIVE"):
            flags |= cv2.CALIB_CB_EXHAUSTIVE
        if hasattr(cv2, "CALIB_CB_ACCURACY"):
            flags |= cv2.CALIB_CB_ACCURACY
        ok, corners = cv2.findChessboardCornersSB(
            cropped, (self.pattern_cols, self.pattern_rows), flags=flags
        )
        if not ok:
            return None
        corners = corners.astype(np.float32)
        corners[:, 0, 0] += x0
        corners[:, 0, 1] += y0
        # findChessboardCornersSB may return the same symmetric board with the
        # corner order reversed by 180 degrees on isolated frames.  That makes
        # asymmetric placement offsets jump to the opposite side of the board.
        # In this fixed overhead setup the board's column direction is left to
        # right in the image, so canonicalize the endpoint order before solvePnP
        # and homography construction.
        if corners[0, 0, 0] > corners[-1, 0, 0]:
            corners = corners[::-1].copy()
        object_points = self.board_object_points()
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            corners,
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None
        rotation, _ = cv2.Rodrigues(rvec)

        grid_points = np.asarray(
            [[col, row] for row in range(self.pattern_rows) for col in range(self.pattern_cols)],
            dtype=np.float32,
        )
        homography, _ = cv2.findHomography(grid_points, corners.reshape(-1, 2))
        outer_grid = np.asarray(
            [[[-1.0, -1.0], [self.pattern_cols, -1.0],
              [self.pattern_cols, self.pattern_rows], [-1.0, self.pattern_rows]]],
            dtype=np.float32,
        )
        polygon = cv2.perspectiveTransform(outer_grid, homography)[0]
        center = np.mean(polygon, axis=0)
        radial = polygon - center
        lengths = np.linalg.norm(radial, axis=1, keepdims=True)
        polygon += radial / np.maximum(lengths, 1.0) * self.board_exclusion_margin_px
        return corners, rotation, tvec.reshape(3), rvec, np.round(polygon).astype(np.int32)

    def make_blue_mask(self, image, board_polygon):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.asarray([self.h_min, self.s_min, self.v_min], dtype=np.uint8),
            np.asarray([self.h_max, self.s_max, self.v_max], dtype=np.uint8),
        )
        if self.close_kernel > 0:
            kernel = np.ones((self.close_kernel, self.close_kernel), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        if self.dilate_kernel > 0 and self.dilate_iterations > 0:
            kernel = np.ones((self.dilate_kernel, self.dilate_kernel), np.uint8)
            mask = cv2.dilate(mask, kernel, iterations=self.dilate_iterations)
        cv2.fillConvexPoly(mask, board_polygon, 0)
        return mask

    def contour_to_point(self, contour, depth, camera_matrix):
        x, y, width, height = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        fill_ratio = area / float(max(1, width * height))
        if not (self.min_area <= area <= self.max_area):
            return None
        if not (self.min_width <= width <= self.max_width):
            return None
        if not (self.min_height <= height <= self.max_height):
            return None
        if fill_ratio < self.min_fill_ratio:
            return None

        component = np.zeros(depth.shape[:2], dtype=np.uint8)
        cv2.drawContours(component, [contour], -1, 255, thickness=cv2.FILLED)
        if self.depth_erode_kernel > 0:
            kernel = np.ones(
                (self.depth_erode_kernel, self.depth_erode_kernel), np.uint8
            )
            component = cv2.erode(component, kernel, iterations=1)
        rows, cols = np.nonzero(
            (component > 0)
            & np.isfinite(depth)
            & (depth >= self.min_depth_m)
            & (depth <= self.max_depth_m)
        )
        if len(rows) < self.min_depth_samples:
            return None
        z = depth[rows, cols]
        fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
        cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
        points = np.column_stack(
            ((cols - cx) * z / fx, (rows - cy) * z / fy, z)
        )
        point = np.median(points, axis=0)
        rectangle = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rectangle)
        edges = [box[(index + 1) % 4] - box[index] for index in range(4)]
        long_edge = max(edges, key=lambda edge: float(np.linalg.norm(edge)))
        long_length = float(np.linalg.norm(long_edge))
        short_length = min(float(rectangle[1][0]), float(rectangle[1][1]))
        aspect_ratio = long_length / max(short_length, 1.0)
        raw_center = (int(np.median(cols)), int(np.median(rows)))
        if long_length <= 1.0:
            long_axis_pixel = np.asarray([1.0, 0.0], dtype=float)
        else:
            long_axis_pixel = np.asarray(long_edge, dtype=float) / long_length
        # A rectangle axis is unsigned.  Canonicalize it to prevent 180-degree
        # flips between frames before circular locking in the task planner.
        if (
            long_axis_pixel[0] < 0.0
            or (abs(long_axis_pixel[0]) < 1e-6 and long_axis_pixel[1] < 0.0)
        ):
            long_axis_pixel *= -1.0
        return {
            "point": point,
            "raw_center": raw_center,
            "center": raw_center,
            "bbox": (x, y, width, height),
            "area": area,
            "fill": fill_ratio,
            "long_axis_pixel": long_axis_pixel,
            "aspect_ratio": aspect_ratio,
            "center_offset_applied": False,
        }

    def sort_objects(self, objects):
        if self.sort_mode == "left_to_right":
            return sorted(objects, key=lambda item: item["center"][0])
        if self.sort_mode == "top_to_bottom":
            return sorted(objects, key=lambda item: item["center"][1])
        if self.sort_mode == "bottom_to_top":
            return sorted(objects, key=lambda item: -item["center"][1])
        if self.sort_mode != "right_to_left":
            rospy.logwarn_throttle(5.0, "Unknown sort_mode=%s; using right_to_left", self.sort_mode)
        return sorted(objects, key=lambda item: -item["center"][0])

    def apply_object_center_offsets(self, objects, fx, fy):
        """Apply sorted-object-specific empirical image-plane corrections."""
        for index, item in enumerate(objects):
            if index < len(self.object_center_offsets_px):
                values = self.object_center_offsets_px[index]
                if len(values) != 2:
                    raise RuntimeError(
                        "object_center_offsets_px entries must be [u_px, v_px]"
                    )
                offset_u, offset_v = float(values[0]), float(values[1])
            elif (
                item["aspect_ratio"]
                >= self.object_center_offset_min_aspect_ratio
            ):
                offset_u = self.object_center_offset_u_px
                offset_v = self.object_center_offset_v_px
            else:
                offset_u, offset_v = 0.0, 0.0
            raw_u, raw_v = item["raw_center"]
            item["center"] = (
                int(round(raw_u + offset_u)),
                int(round(raw_v + offset_v)),
            )
            item["point"] = np.asarray(item["point"], dtype=float).copy()
            item["point"][0] += offset_u * item["point"][2] / fx
            item["point"][1] += offset_v * item["point"][2] / fy
            item["center_offset_applied"] = bool(offset_u or offset_v)
        return objects

    @staticmethod
    def pose_array(header, points, yaws=None):
        message = PoseArray()
        message.header = header
        if yaws is None:
            yaws = [None] * len(points)
        for point, yaw in zip(points, yaws):
            pose = Pose()
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = float(point[2])
            if yaw is None:
                pose.orientation.w = 1.0
            else:
                quaternion = tf.transformations.quaternion_from_euler(
                    0.0, 0.0, float(yaw)
                )
                pose.orientation.x = float(quaternion[0])
                pose.orientation.y = float(quaternion[1])
                pose.orientation.z = float(quaternion[2])
                pose.orientation.w = float(quaternion[3])
            message.poses.append(pose)
        return message

    def transform_points(self, points, stamp):
        if not self.target_frame or self.target_frame == self.camera_frame:
            return points, self.camera_frame
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            self.camera_frame,
            stamp,
            rospy.Duration(0.05),
        ).transform
        quaternion = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
        rotation = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
        translation = np.asarray(
            [transform.translation.x, transform.translation.y, transform.translation.z]
        )
        return [rotation.dot(point) + translation for point in points], self.target_frame

    def transform_vectors(self, vectors, stamp):
        if not self.target_frame or self.target_frame == self.camera_frame:
            return vectors
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            self.camera_frame,
            stamp,
            rospy.Duration(0.05),
        ).transform
        quaternion = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
        rotation = tf.transformations.quaternion_matrix(quaternion)[:3, :3]
        return [rotation.dot(vector) for vector in vectors]

    def publish_board_pose(self, header, rotation, translation):
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        quaternion = tf.transformations.quaternion_from_matrix(matrix)
        message = PoseStamped()
        message.header = header
        message.pose.position.x = float(translation[0])
        message.pose.position.y = float(translation[1])
        message.pose.position.z = float(translation[2])
        message.pose.orientation.x = float(quaternion[0])
        message.pose.orientation.y = float(quaternion[1])
        message.pose.orientation.z = float(quaternion[2])
        message.pose.orientation.w = float(quaternion[3])
        self.board_pose_pub.publish(message)

    @staticmethod
    def marker_point(xyz):
        return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))

    def publish_task_markers(self, header, objects, places, heights, long_axis_yaws):
        """Show numbered pick/lift/high-release goals; never commands motion."""
        markers = []
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.append(clear)
        for object_index, place_index in zip(
            self.task_active_object_indices, self.task_placement_indices
        ):
            index = object_index + 1
            obj = objects[object_index]
            place = places[place_index]
            height = heights[object_index]
            long_axis_yaw = long_axis_yaws[object_index]
            grasp = np.asarray(obj, dtype=float).copy()
            grasp[2] = (
                obj[2] - height / 2.0 + self.task_grasp_above_center
            )
            lift = grasp + np.asarray([0.0, 0.0, self.task_lift_distance])
            release = np.asarray(place, dtype=float).copy()
            release[2] = max(
                place[2] + self.task_release_tcp_clearance,
                place[2] + height / 2.0 + self.task_grasp_above_center
                + self.task_release_clearance,
            )

            path = Marker()
            path.header = header
            path.ns = "three_blue_task_path"
            path.id = index
            path.type = Marker.LINE_STRIP
            path.action = Marker.ADD
            path.scale.x = 0.008
            path.color.r, path.color.g, path.color.b, path.color.a = (0.2, 0.8, 1.0, 0.9)
            path.points = [
                self.marker_point(grasp), self.marker_point(lift),
                self.marker_point(release),
            ]
            markers.append(path)

            axis = np.asarray(
                [math.cos(long_axis_yaw), math.sin(long_axis_yaw), 0.0]
            )
            axis_marker = Marker()
            axis_marker.header = header
            axis_marker.ns = "three_blue_long_axis"
            axis_marker.id = index
            axis_marker.type = Marker.ARROW
            axis_marker.action = Marker.ADD
            axis_marker.scale.x = 0.010
            axis_marker.scale.y = 0.020
            axis_marker.scale.z = 0.020
            axis_marker.color.r, axis_marker.color.g, axis_marker.color.b = (1.0, 0.75, 0.0)
            axis_marker.color.a = 1.0
            axis_marker.points = [
                self.marker_point(np.asarray(grasp) - 0.045 * axis),
                self.marker_point(np.asarray(grasp) + 0.045 * axis),
            ]
            markers.append(axis_marker)

            for phase_id, (phase, point, color) in enumerate(
                [
                    ("PICK", grasp, (1.0, 0.2, 0.1, 0.95)),
                    ("LIFT", lift, (0.1, 1.0, 0.2, 0.95)),
                    ("HIGH RELEASE", release, (1.0, 0.1, 1.0, 0.95)),
                ]
            ):
                sphere = Marker()
                sphere.header = header
                sphere.ns = "three_blue_task_goal"
                sphere.id = index * 10 + phase_id
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position = self.marker_point(point)
                sphere.pose.orientation.w = 1.0
                sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.030
                sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = color
                markers.append(sphere)

                text = Marker()
                text.header = header
                text.ns = "three_blue_task_text"
                text.id = index * 10 + phase_id
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position = self.marker_point(
                    np.asarray(point) + np.asarray([0.0, 0.0, 0.035])
                )
                text.pose.orientation.w = 1.0
                text.scale.z = 0.032
                text.color.r = text.color.g = text.color.b = text.color.a = 1.0
                suffix = " >=120mm" if phase == "HIGH RELEASE" else ""
                if phase == "PICK" and index <= 2:
                    suffix += " long-axis {:.1f}deg".format(
                        math.degrees(long_axis_yaw)
                    )
                text.text = "{} {}{}".format(index, phase, suffix)
                markers.append(text)
        self.task_marker_pub.publish(MarkerArray(markers=markers))

    def process(self, _event):
        snapshot = self.snapshot()
        if snapshot is None:
            return
        image, depth, color_header, depth_header, camera_matrix, distortion = snapshot
        if image.shape[:2] != depth.shape[:2]:
            self.publish_status(False, "color/depth size mismatch")
            return
        stamp_delta = abs((color_header.stamp - depth_header.stamp).to_sec())
        if stamp_delta > self.max_color_depth_dt:
            self.publish_status(False, "color/depth timestamp mismatch")
            return

        board = self.find_board(image, camera_matrix, distortion)
        if board is None:
            self.publish_status(False, "8x6 checkerboard not detected in configured ROI")
            return
        corners, rotation, translation, rvec, board_polygon = board
        self.publish_board_pose(color_header, rotation, translation)

        mask = self.make_blue_mask(image, board_polygon)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        objects = []
        for contour in contours:
            detected = self.contour_to_point(contour, depth, camera_matrix)
            if detected is not None:
                objects.append(detected)
        objects = self.sort_objects(objects)
        fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
        objects = self.apply_object_center_offsets(objects, fx, fy)
        object_points = [item["point"] for item in objects]
        object_axes_camera = []
        for item in objects:
            du, dv = item["long_axis_pixel"]
            axis = np.asarray([du / fx, dv / fy, 0.0], dtype=float)
            axis /= max(float(np.linalg.norm(axis)), 1e-9)
            object_axes_camera.append(axis)
        camera_header = color_header
        camera_header.frame_id = self.camera_frame
        self.objects_camera_pub.publish(self.pose_array(camera_header, object_points))
        if object_points:
            selected = PointStamped()
            selected.header = camera_header
            selected.point.x = float(object_points[0][0])
            selected.point.y = float(object_points[0][1])
            selected.point.z = float(object_points[0][2])
            self.selected_camera_pub.publish(selected)

        place_points = [
            rotation.dot(offset) + translation for offset in self.placement_offsets
        ]
        self.places_camera_pub.publish(self.pose_array(camera_header, place_points))

        transform_ok = False
        try:
            object_target, target_frame = self.transform_points(
                object_points, color_header.stamp
            )
            object_axes_target = self.transform_vectors(
                object_axes_camera, color_header.stamp
            )
            object_axis_yaws = []
            for axis in object_axes_target:
                horizontal = np.asarray([axis[0], axis[1]], dtype=float)
                horizontal /= max(float(np.linalg.norm(horizontal)), 1e-9)
                if (
                    horizontal[0] < 0.0
                    or (abs(horizontal[0]) < 1e-6 and horizontal[1] < 0.0)
                ):
                    horizontal *= -1.0
                object_axis_yaws.append(
                    math.atan2(float(horizontal[1]), float(horizontal[0]))
                )
            place_target, _ = self.transform_points(place_points, color_header.stamp)
            target_header = color_header
            target_header.frame_id = target_frame
            self.objects_target_pub.publish(
                self.pose_array(target_header, object_target, object_axis_yaws)
            )
            self.places_target_pub.publish(self.pose_array(target_header, place_target))
            transform_ok = True
        except tf2_ros.TransformException as exc:
            rospy.logwarn_throttle(2.0, "Waiting for camera-to-%s TF: %s", self.target_frame, exc)

        board_normal = rotation[:, 2]
        object_heights = [
            -float(np.dot(point - translation, board_normal))
            for point in object_points
        ]
        geometry_ok = len(object_heights) == self.expected_object_count and all(
            self.min_object_height <= height <= self.max_object_height
            for height in object_heights
        )
        if transform_ok and geometry_ok:
            self.publish_task_markers(
                target_header, object_target, place_target, object_heights,
                object_axis_yaws,
            )
        ready = (
            len(objects) == self.expected_object_count
            and transform_ok
            and geometry_ok
            and self.square_size_confirmed
        )
        status = "objects={}/{} board=ok heights_mm={} square={} tf={}".format(
            len(objects),
            self.expected_object_count,
            ",".join("{:.1f}".format(1000.0 * value) for value in object_heights),
            "confirmed" if self.square_size_confirmed else "UNCONFIRMED",
            "ok" if transform_ok else "missing",
        )
        self.publish_status(ready, status)
        self.publish_debug(
            image, mask, corners, board_polygon, objects, place_points,
            camera_matrix, distortion, rvec, translation, status
        )

    def publish_status(self, ready, status):
        self.ready_pub.publish(Bool(data=bool(ready)))
        self.status_pub.publish(String(data=status))
        if ready:
            rospy.loginfo_throttle(2.0, "Three-block scene ready: %s", status)
        else:
            rospy.logwarn_throttle(2.0, "Three-block scene not ready: %s", status)

    def publish_debug(
        self, image, mask, corners, board_polygon, objects, place_points,
        camera_matrix, distortion, rvec, translation, status
    ):
        debug = image.copy()
        cv2.drawChessboardCorners(
            debug, (self.pattern_cols, self.pattern_rows), corners, True
        )
        cv2.polylines(debug, [board_polygon], True, (0, 165, 255), 3)
        for index, item in enumerate(objects, start=1):
            x, y, width, height = item["bbox"]
            u, v = item["center"]
            raw_u, raw_v = item["raw_center"]
            cv2.rectangle(debug, (x, y), (x + width, y + height), (0, 0, 255), 2)
            cv2.circle(debug, (raw_u, raw_v), 5, (0, 0, 255), -1)
            if (u, v) != (raw_u, raw_v):
                cv2.line(debug, (raw_u, raw_v), (u, v), (255, 255, 0), 2)
            cv2.drawMarker(
                debug, (u, v), (255, 255, 0), cv2.MARKER_CROSS, 18, 3
            )
            cv2.putText(
                debug, "pick {} area {:.0f} AR {:.2f}".format(
                    index, item["area"], item["aspect_ratio"]
                ),
                (x, max(24, y - 7)), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                (0, 0, 255), 2, cv2.LINE_AA
            )
            axis = item["long_axis_pixel"]
            endpoint_1 = (
                int(round(u - 40.0 * axis[0])), int(round(v - 40.0 * axis[1]))
            )
            endpoint_2 = (
                int(round(u + 40.0 * axis[0])), int(round(v + 40.0 * axis[1]))
            )
            cv2.arrowedLine(
                debug, endpoint_1, endpoint_2, (0, 215, 255), 3,
                cv2.LINE_AA, tipLength=0.20,
            )
        projected, _ = cv2.projectPoints(
            np.asarray(self.placement_offsets, dtype=np.float32),
            rvec, translation.reshape(3, 1), camera_matrix, distortion
        )
        for index, pixel in enumerate(projected.reshape(-1, 2), start=1):
            u, v = int(round(pixel[0])), int(round(pixel[1]))
            cv2.circle(debug, (u, v), 10, (255, 0, 255), 3)
            cv2.putText(
                debug, "place {}".format(index), (u + 12, v),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 255), 2, cv2.LINE_AA
            )
        cv2.putText(
            debug, status, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
            (0, 255, 0) if len(objects) == self.expected_object_count else (0, 0, 255),
            2, cv2.LINE_AA
        )
        message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        message.header.frame_id = self.camera_frame
        self.debug_pub.publish(message)
        if self.show_window:
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            preview = np.hstack(
                [cv2.resize(debug, (640, 360)), cv2.resize(mask_bgr, (640, 360))]
            )
            cv2.imshow(self.window_name, preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                rospy.signal_shutdown("perception window closed")

    def shutdown(self):
        if self.show_window:
            cv2.destroyWindow(self.window_name)


if __name__ == "__main__":
    rospy.init_node("blue_blocks_checkerboard_perception")
    BlueBlocksCheckerboardPerception()
    rospy.spin()
