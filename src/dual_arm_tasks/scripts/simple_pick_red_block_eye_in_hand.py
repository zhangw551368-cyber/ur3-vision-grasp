#!/usr/bin/python3

import sys

import moveit_commander
import rospy

from simple_pick_red_block import SimplePickRedBlock


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("simple_pick_red_block_eye_in_hand")
    try:
        SimplePickRedBlock().run()
    except Exception as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
