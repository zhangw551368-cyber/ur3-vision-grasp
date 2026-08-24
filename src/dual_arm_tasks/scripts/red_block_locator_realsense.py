#!/usr/bin/python3

import rospy

from red_block_locator_kinect import RedBlockLocatorKinect


if __name__ == "__main__":
    rospy.init_node("red_block_locator_realsense")
    RedBlockLocatorKinect(
        label="RealSense",
        default_color_topic="/camera/color/image_raw",
        default_depth_topic="/camera/aligned_depth_to_color/image_raw",
        default_info_topic="/camera/color/camera_info",
        default_output_topic="/red_block/point_base",
        default_use_plane_if_depth_missing=False,
    )
    rospy.spin()
