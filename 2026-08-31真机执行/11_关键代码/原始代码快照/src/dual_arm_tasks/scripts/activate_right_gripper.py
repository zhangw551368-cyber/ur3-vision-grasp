#!/usr/bin/python3

import rospy
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output


def make_command(ract, rpr, speed, force):
    command = Robotiq2FGripper_robot_output()
    command.rACT = ract
    command.rGTO = 1 if ract else 0
    command.rATR = 0
    command.rPR = rpr
    command.rSP = speed
    command.rFR = force
    return command


def publish_repeated(publisher, command, duration):
    end_time = rospy.Time.now() + rospy.Duration(duration)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        publisher.publish(command)
        rate.sleep()


def main():
    rospy.init_node("activate_right_gripper")
    command_topic = rospy.get_param(
        "~command_topic", "/right_arm/Robotiq2FGripperRobotOutput"
    )
    reset_duration = rospy.get_param("~reset_duration", 1.0)
    settle_duration = rospy.get_param("~settle_duration", 0.5)
    activate_duration = rospy.get_param("~activate_duration", 2.0)
    open_position = rospy.get_param("~open_position", 0)
    speed = rospy.get_param("~speed", 120)
    force = rospy.get_param("~force", 80)

    publisher = rospy.Publisher(
        command_topic, Robotiq2FGripper_robot_output, queue_size=1, latch=True
    )

    rospy.loginfo("Waiting for right gripper command subscribers on %s", command_topic)
    timeout = rospy.Time.now() + rospy.Duration(15.0)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and publisher.get_num_connections() == 0:
        if rospy.Time.now() > timeout:
            rospy.logwarn(
                "No gripper driver subscribed to %s yet; sending activation anyway",
                command_topic,
            )
            break
        rate.sleep()

    rospy.loginfo("Resetting right gripper with rACT=0")
    publish_repeated(
        publisher,
        make_command(ract=0, rpr=0, speed=speed, force=force),
        reset_duration,
    )

    rospy.sleep(settle_duration)

    rospy.loginfo(
        "Activating right gripper with rACT=1 and opening to rPR=%d", open_position
    )
    publish_repeated(
        publisher,
        make_command(ract=1, rpr=open_position, speed=speed, force=force),
        activate_duration,
    )
    rospy.loginfo("Right gripper activation command sequence finished")


if __name__ == "__main__":
    main()
