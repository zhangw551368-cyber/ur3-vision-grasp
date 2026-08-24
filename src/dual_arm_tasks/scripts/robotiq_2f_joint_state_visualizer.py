#!/usr/bin/python3

import rospy
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from sensor_msgs.msg import JointState


class Robotiq2FJointStateVisualizer:
    def __init__(self):
        self.prefix = rospy.get_param("~prefix", "right_")
        self.command_topic = rospy.get_param(
            "~command_topic", "/right_arm/Robotiq2FGripperRobotOutput"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/right_arm/Robotiq2FGripperRobotInput"
        )
        self.preview_topic = rospy.get_param(
            "~preview_topic", "/right_gripper/preview_command"
        )
        self.output_topic = rospy.get_param(
            "~joint_state_topic", "/right_gripper/joint_states"
        )
        self.max_joint_position = rospy.get_param("~max_joint_position", 0.8)
        self.position = 0.0
        self.last_status_time = rospy.Time(0)
        self.preview_until = rospy.Time(0)
        self.publisher = rospy.Publisher(self.output_topic, JointState, queue_size=1)
        rospy.Subscriber(
            self.command_topic,
            Robotiq2FGripper_robot_output,
            self.command_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.status_topic,
            Robotiq2FGripper_robot_input,
            self.status_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.preview_topic,
            Robotiq2FGripper_robot_output,
            self.preview_callback,
            queue_size=1,
        )
        rospy.loginfo(
            "Publishing %sfinger_joint visualization from %s/%s/%s to %s",
            self.prefix,
            self.command_topic,
            self.status_topic,
            self.preview_topic,
            self.output_topic,
        )

    def robotiq_position_to_joint(self, position):
        bounded = max(0.0, min(255.0, float(position)))
        return bounded / 255.0 * self.max_joint_position

    def command_callback(self, command):
        if rospy.Time.now() < self.preview_until:
            return
        if rospy.Time.now() - self.last_status_time > rospy.Duration(0.5):
            self.position = self.robotiq_position_to_joint(command.rPR)

    def status_callback(self, status):
        if rospy.Time.now() < self.preview_until:
            return
        self.last_status_time = rospy.Time.now()
        self.position = self.robotiq_position_to_joint(status.gPO)

    def preview_callback(self, command):
        self.preview_until = rospy.Time.now() + rospy.Duration(2.0)
        self.position = self.robotiq_position_to_joint(command.rPR)

    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = [self.prefix + "finger_joint"]
            msg.position = [self.position]
            self.publisher.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("robotiq_2f_joint_state_visualizer")
    Robotiq2FJointStateVisualizer().spin()
