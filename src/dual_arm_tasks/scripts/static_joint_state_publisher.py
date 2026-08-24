#!/usr/bin/python3

import rospy
from sensor_msgs.msg import JointState


def main():
    rospy.init_node("static_joint_state_publisher")
    prefix = rospy.get_param("~prefix", "left_arm_")
    topic = rospy.get_param("~joint_state_topic", "/left_arm/joint_states")
    rate_hz = rospy.get_param("~rate", 20.0)
    joint_names = rospy.get_param(
        "~joint_names",
        [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ],
    )
    positions = rospy.get_param("~positions", [0.0] * len(joint_names))
    names = [prefix + name for name in joint_names]
    publisher = rospy.Publisher(topic, JointState, queue_size=1)
    rate = rospy.Rate(rate_hz)
    rospy.loginfo("Publishing static joint states to %s: %s", topic, names)
    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = names
        msg.position = positions
        publisher.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
