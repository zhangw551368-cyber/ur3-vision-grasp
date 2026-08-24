#!/usr/bin/env python3

"""Apply repeatable manual RGB settings after a RealSense node starts."""

import sys

import rospy
from dynamic_reconfigure.client import Client


def main():
    rospy.init_node("set_realsense_color_manual", anonymous=False)
    server = rospy.get_param("~server", "/camera2/rgb_camera")
    timeout = float(rospy.get_param("~timeout", 20.0))
    settings = {
        "enable_auto_exposure": False,
        "exposure": int(rospy.get_param("~exposure", 250)),
        "gain": int(rospy.get_param("~gain", 16)),
        "enable_auto_white_balance": False,
        "white_balance": float(rospy.get_param("~white_balance", 4600.0)),
    }

    try:
        client = Client(server, timeout=timeout)
        applied = client.update_configuration(settings)
    except Exception as exc:  # dynamic_reconfigure raises several ROS exception types
        rospy.logerr("Failed to configure %s: %s", server, exc)
        return 1

    rospy.loginfo(
        "Locked %s: exposure=%s gain=%s white_balance=%s",
        server,
        applied.get("exposure"),
        applied.get("gain"),
        applied.get("white_balance"),
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
