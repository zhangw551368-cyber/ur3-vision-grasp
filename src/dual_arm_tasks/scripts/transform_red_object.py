#!/usr/bin/python3

import rospy
import tf2_geometry_msgs  # Registers PointStamped conversions with tf2.
import tf2_ros
from geometry_msgs.msg import PointStamped


class RedObjectTransformer:
    def __init__(self):
        self.target_frame = rospy.get_param("~target_frame", "base")
        input_topic = rospy.get_param("~input_topic", "/red_object/point_camera")
        output_topic = rospy.get_param("~output_topic", "/red_object/point_base")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(output_topic, PointStamped, queue_size=1)
        rospy.Subscriber(input_topic, PointStamped, self.callback, queue_size=1)
        rospy.loginfo(
            "Transforming red object points from %s to frame=%s, publishing %s",
            input_topic,
            self.target_frame,
            output_topic,
        )

    def callback(self, point):
        try:
            transformed = self.tf_buffer.transform(
                point, self.target_frame, timeout=rospy.Duration(0.2)
            )
            self.publisher.publish(transformed)
        except (
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.LookupException,
        ) as exc:
            rospy.logwarn_throttle(
                3.0,
                "Cannot transform red object from %s to %s: %s",
                point.header.frame_id,
                self.target_frame,
                exc,
            )


if __name__ == "__main__":
    rospy.init_node("transform_red_object")
    RedObjectTransformer()
    rospy.spin()
