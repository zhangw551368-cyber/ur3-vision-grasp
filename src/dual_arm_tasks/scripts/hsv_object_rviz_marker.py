#!/usr/bin/env python3

import math

import rospy
import tf2_geometry_msgs  # Registers PointStamped conversions with tf2.
import tf2_ros
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker


class HsvObjectRvizMarker:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/hsv_grasp/object_point_base"
        )
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/hsv_grasp/object_marker"
        )
        self.target_frame = rospy.get_param("~target_frame", "base")
        self.cube_size = float(rospy.get_param("~cube_size", 0.055))
        self.object_point_semantic = rospy.get_param(
            "~object_point_semantic", "top_center"
        )
        self.snap_cube_to_support_plane = self.get_bool_param(
            "snap_cube_to_support_plane", False
        )
        self.support_plane_z = float(rospy.get_param("~support_plane_z", 0.0))
        self.marker_type_name = rospy.get_param("~marker_type", "cube").strip().lower()
        self.marker_z_offset = float(rospy.get_param("~marker_z_offset", 0.0))
        self.tf_timeout = rospy.Duration(float(rospy.get_param("~tf_timeout", 0.5)))

        self.marker_type = self.parse_marker_type(self.marker_type_name)
        self.validate_params()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            self.marker_topic, Marker, queue_size=1, latch=True
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, PointStamped, self.point_callback, queue_size=1
        )

        rospy.loginfo(
            "HSV object RViz marker started: input_topic=%s marker_topic=%s target_frame=%s marker_type=%s object_point_semantic=%s cube_size=%.6f marker_z_offset=%.6f snap_cube_to_support_plane=%s support_plane_z=%.6f",
            self.input_topic,
            self.marker_topic,
            self.target_frame,
            self.marker_type_name,
            self.object_point_semantic,
            self.cube_size,
            self.marker_z_offset,
            self.snap_cube_to_support_plane,
            self.support_plane_z,
        )

    @staticmethod
    def parse_marker_type(marker_type_name):
        if marker_type_name == "cube":
            return Marker.CUBE
        if marker_type_name == "sphere":
            return Marker.SPHERE
        raise RuntimeError(
            "Unsupported marker_type={!r}; use 'cube' or 'sphere'".format(
                marker_type_name
            )
        )

    def validate_params(self):
        if not self.target_frame:
            raise RuntimeError("Parameter ~target_frame must not be empty")
        if not math.isfinite(self.cube_size) or self.cube_size <= 0.0:
            raise RuntimeError("Parameter ~cube_size must be finite and > 0")
        if self.object_point_semantic not in ("top_center", "cube_center"):
            raise RuntimeError(
                "Unsupported object_point_semantic={!r}; use 'top_center' or 'cube_center'".format(
                    self.object_point_semantic
                )
            )
        if not math.isfinite(self.marker_z_offset):
            raise RuntimeError("Parameter ~marker_z_offset must be finite")
        if not math.isfinite(self.support_plane_z):
            raise RuntimeError("Parameter ~support_plane_z must be finite")

    @staticmethod
    def get_bool_param(name, default):
        value = rospy.get_param("~{}".format(name), default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in ("true", "1", "yes", "on"):
                return True
            if normalized in ("false", "0", "no", "off"):
                return False
        return bool(value)

    def marker_center_z(self, point):
        if self.snap_cube_to_support_plane:
            return self.support_plane_z + self.cube_size / 2.0
        z = point.point.z + self.marker_z_offset
        if self.object_point_semantic == "top_center":
            z -= self.cube_size / 2.0
        return z

    @staticmethod
    def point_is_valid(point):
        values = (point.x, point.y, point.z)
        return all(math.isfinite(value) for value in values)

    @staticmethod
    def copy_point_stamped(point):
        copied = PointStamped()
        copied.header.stamp = point.header.stamp
        copied.header.frame_id = point.header.frame_id
        copied.point.x = point.point.x
        copied.point.y = point.point.y
        copied.point.z = point.point.z
        return copied

    def transform_to_target_frame(self, msg):
        input_frame = msg.header.frame_id.strip()
        if not input_frame:
            raise RuntimeError("Input PointStamped has empty frame_id")

        if input_frame == self.target_frame:
            point = self.copy_point_stamped(msg)
            point.header.frame_id = self.target_frame
            return input_frame, point

        transformed = self.tf_buffer.transform(
            msg, self.target_frame, timeout=self.tf_timeout
        )
        return input_frame, transformed

    def make_marker(self, point):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.target_frame
        marker.ns = "hsv_object"
        marker.id = 0
        marker.type = self.marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = point.point.x
        marker.pose.position.y = point.point.y
        marker.pose.position.z = self.marker_center_z(point)
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.cube_size
        marker.scale.y = self.cube_size
        marker.scale.z = self.cube_size
        marker.color.r = 0.0
        marker.color.g = 0.2
        marker.color.b = 1.0
        marker.color.a = 0.8
        return marker

    def point_callback(self, msg):
        if not self.point_is_valid(msg.point):
            rospy.logwarn_throttle(
                1.0,
                "Invalid HSV object point: frame=%r x=%.6f y=%.6f z=%.6f; marker not published",
                msg.header.frame_id,
                msg.point.x,
                msg.point.y,
                msg.point.z,
            )
            return

        try:
            input_frame, point_base = self.transform_to_target_frame(msg)
        except (tf2_ros.TransformException, RuntimeError) as exc:
            rospy.logwarn_throttle(
                1.0,
                "Failed to transform HSV object point from %r to %r: %s; marker not published",
                msg.header.frame_id,
                self.target_frame,
                exc,
            )
            return

        marker = self.make_marker(point_base)
        self.publisher.publish(marker)

        rospy.loginfo_throttle(
            1.0,
            "HSV marker: input_frame=%s semantic=%s snap_to_support=%s marker_center_in_%s=(%.6f, %.6f, %.6f) marker_scale=(%.6f, %.6f, %.6f) marker_topic=%s",
            input_frame,
            self.object_point_semantic,
            self.snap_cube_to_support_plane,
            self.target_frame,
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z,
            marker.scale.x,
            marker.scale.y,
            marker.scale.z,
            self.marker_topic,
        )


def main():
    rospy.init_node("hsv_object_rviz_marker")
    HsvObjectRvizMarker()
    rospy.spin()


if __name__ == "__main__":
    main()
