#!/usr/bin/python3

import argparse
import sys
import time

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input
from std_msgs.msg import Bool


def parse_args():
    parser = argparse.ArgumentParser(
        description="Check whether the right-arm visual pick stack is ready."
    )
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--skip-gripper", action="store_true")
    parser.add_argument("--skip-moveit", action="store_true")
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


class ReadinessCheck:
    def __init__(self, timeout):
        self.timeout = timeout
        self.errors = []
        self.warnings = []

    def ok(self, message):
        rospy.loginfo("[OK] %s", message)

    def warn(self, message):
        self.warnings.append(message)
        rospy.logwarn("[WARN] %s", message)

    def fail(self, message):
        self.errors.append(message)
        rospy.logerr("[FAIL] %s", message)

    def wait_for_topic_name(self, topic):
        deadline = time.time() + self.timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            topics = [name for name, _ in rospy.get_published_topics()]
            if topic in topics:
                self.ok("topic exists: {}".format(topic))
                return True
            time.sleep(0.2)
        self.fail("topic missing: {}".format(topic))
        return False

    def wait_for_message(self, topic, msg_type, description):
        try:
            msg = rospy.wait_for_message(topic, msg_type, timeout=self.timeout)
        except rospy.ROSException:
            self.fail("no message on {} ({})".format(topic, description))
            return None
        self.ok("message received on {} ({})".format(topic, description))
        return msg

    def check_tf(self, target, source):
        buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(buffer)
        try:
            transform = buffer.lookup_transform(
                target, source, rospy.Time(0), rospy.Duration(self.timeout)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.fail("TF missing {} <- {}: {}".format(target, source, exc))
            return None
        t = transform.transform.translation
        self.ok(
            "TF {} <- {} xyz=[{:.3f}, {:.3f}, {:.3f}]".format(
                target, source, t.x, t.y, t.z
            )
        )
        return transform

    def check_red_block(self):
        point = self.wait_for_message(
            "/red_object/point_base", PointStamped, "red block in robot base"
        )
        if point is None:
            return
        if point.header.frame_id != "base":
            self.fail(
                "/red_object/point_base frame_id is {}, expected base".format(
                    point.header.frame_id
                )
            )
            return
        p = point.point
        self.ok(
            "red block base point x={:.3f} y={:.3f} z={:.3f}".format(
                p.x, p.y, p.z
            )
        )
        if p.z < -0.05 or p.z > 0.30:
            self.warn(
                "red block z={:.3f} is outside the expected table/conveyor range".format(
                    p.z
                )
            )

    def check_gripper(self):
        status = self.wait_for_message(
            "/right_arm/Robotiq2FGripperRobotInput",
            Robotiq2FGripper_robot_input,
            "right Robotiq status",
        )
        if status is None:
            return
        self.ok(
            "right gripper status gACT={} gSTA={} gOBJ={} gFLT={} gPO={}".format(
                status.gACT, status.gSTA, status.gOBJ, status.gFLT, status.gPO
            )
        )
        if status.gACT != 1:
            self.warn("right gripper is not activated yet; rACT 0 -> 1 may be needed")
        if status.gFLT != 0:
            self.warn("right gripper reports fault code gFLT={}".format(status.gFLT))

    def check_external_control(self):
        topic = "/right_arm/ur_hardware_interface/robot_program_running"
        running = self.wait_for_message(topic, Bool, "right UR External Control")
        if running is not None and not running.data:
            self.fail(
                "right UR External Control is not running; start the External Control "
                "program on the right teach pendant before real execution"
            )

    def run(self, skip_gripper=False, skip_moveit=False):
        self.check_tf("base", "right_arm_tool0")
        self.check_tf("base", "camera_color_optical_frame")
        self.check_red_block()
        self.check_external_control()

        if skip_moveit:
            self.warn("MoveIt topic check skipped")
        else:
            self.wait_for_topic_name("/move_group/status")
            self.wait_for_topic_name(
                "/right_arm/scaled_pos_joint_traj_controller/follow_joint_trajectory/status"
            )

        if skip_gripper:
            self.warn("right gripper check skipped")
        else:
            self.check_gripper()

        if self.errors:
            rospy.logerr("Readiness check failed with %d error(s)", len(self.errors))
            return 1
        rospy.loginfo(
            "Readiness check passed with %d warning(s). Plan-only pick can be tried.",
            len(self.warnings),
        )
        return 0


def main():
    args = parse_args()
    rospy.init_node("right_arm_pick_readiness")
    checker = ReadinessCheck(args.timeout)
    sys.exit(
        checker.run(skip_gripper=args.skip_gripper, skip_moveit=args.skip_moveit)
    )


if __name__ == "__main__":
    main()
