#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray


class RedObjectMarker:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/red_object/point_base")
        self.marker_topic = rospy.get_param("~marker_topic", "/red_object/markers")
        self.frame_id = rospy.get_param("~frame_id", "base")
        self.cube_size = rospy.get_param("~cube_size", [0.05, 0.05, 0.08])
        self.z_offset = rospy.get_param("~z_offset", 0.0)
        self.publisher = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=1, latch=True
        )
        rospy.Subscriber(self.input_topic, PointStamped, self.callback, queue_size=1)
        rospy.loginfo(
            "Publishing red object RViz markers from %s to %s",
            self.input_topic,
            self.marker_topic,
        )

    def make_cube(self, point):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = point.header.frame_id or self.frame_id
        marker.ns = "red_object"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = point.point.x
        marker.pose.position.y = point.point.y
        marker.pose.position.z = point.point.z + self.z_offset
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.cube_size[0]
        marker.scale.y = self.cube_size[1]
        marker.scale.z = self.cube_size[2]
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.02
        marker.color.a = 0.85
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def make_text(self, point):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = point.header.frame_id or self.frame_id
        marker.ns = "red_object"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = point.point.x
        marker.pose.position.y = point.point.y
        marker.pose.position.z = point.point.z + self.z_offset + 0.08
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.035
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.02
        marker.color.a = 1.0
        marker.text = "red block"
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def callback(self, point):
        markers = MarkerArray()
        markers.markers.append(self.make_cube(point))
        markers.markers.append(self.make_text(point))
        self.publisher.publish(markers)


if __name__ == "__main__":
    rospy.init_node("red_object_marker")
    RedObjectMarker()
    rospy.spin()
