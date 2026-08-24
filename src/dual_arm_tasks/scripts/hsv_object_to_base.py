#!/usr/bin/env python3

import math

import rospy
import tf2_geometry_msgs  # Registers PointStamped conversions with tf2.
import tf2_ros
from geometry_msgs.msg import PointStamped


class HsvObjectToBase:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/hsv_grasp/object_point_camera"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/hsv_grasp/object_point_base"
        )
        self.expected_input_frame = rospy.get_param(
            "~expected_input_frame", "camera_color_optical_frame"
        )
        self.target_frame = rospy.get_param("~target_frame", "right_arm_base")
        self.tf_timeout = rospy.Duration(rospy.get_param("~tf_timeout", 0.2))
        self.max_abs_coordinate_m = rospy.get_param("~max_abs_coordinate_m", 10.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            self.output_topic, PointStamped, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, PointStamped, self.callback, queue_size=1
        )

        rospy.loginfo(
            "HSV object TF node: %s (%s) -> %s, publishing %s",
            self.input_topic,
            self.expected_input_frame,
            self.target_frame,
            self.output_topic,
        )

    def is_valid_point(self, point):
        values = (point.x, point.y, point.z)
        if not all(math.isfinite(value) for value in values):
            return False
        return all(abs(value) <= self.max_abs_coordinate_m for value in values)

    def callback(self, msg):
        input_frame = msg.header.frame_id.strip()
        if input_frame != self.expected_input_frame:
            rospy.logwarn_throttle(
                2.0,
                "Unexpected input frame_id=%r, expected %r; not publishing base point",
                msg.header.frame_id,
                self.expected_input_frame,
            )
            return

        if not self.is_valid_point(msg.point):
            rospy.logwarn_throttle(
                2.0,
                "Invalid camera point received: Xc=%.6f Yc=%.6f Zc=%.6f; not publishing base point",
                msg.point.x,
                msg.point.y,
                msg.point.z,
            )
            return

        try:
            base_point = self.tf_buffer.transform(
                msg, self.target_frame, timeout=self.tf_timeout
            )
        except (
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.LookupException,
            tf2_ros.InvalidArgumentException,
        ) as exc:
            rospy.logwarn_throttle(
                2.0,
                "Cannot transform HSV object from %s to %s: %s; not publishing base point",
                input_frame,
                self.target_frame,
                exc,
            )
            return

        if not self.is_valid_point(base_point.point):
            rospy.logwarn_throttle(
                2.0,
                "Invalid transformed base point: Xb=%.6f Yb=%.6f Zb=%.6f; not publishing base point",
                base_point.point.x,
                base_point.point.y,
                base_point.point.z,
            )
            return

        base_point.header.frame_id = self.target_frame
        self.publisher.publish(base_point)
        rospy.loginfo(
            "camera_color_optical_frame: Xc=%.6f Yc=%.6f Zc=%.6f | right_arm_base: Xb=%.6f Yb=%.6f Zb=%.6f",
            msg.point.x,
            msg.point.y,
            msg.point.z,
            base_point.point.x,
            base_point.point.y,
            base_point.point.z,
        )


if __name__ == "__main__":
    rospy.init_node("hsv_object_to_base")
    HsvObjectToBase()
    rospy.spin()
