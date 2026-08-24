#!/usr/bin/python3

import argparse
import math
import sys
import time

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Pose
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from std_msgs.msg import Bool, String


def parse_args():
    parser = argparse.ArgumentParser(
        description="Use URScript to move from the current taught grasp to explicit place poses."
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--base-frame", default="right_arm_base")
    parser.add_argument("--tool-frame", default="right_arm_tool0")
    parser.add_argument("--tcp-offset", nargs=3, type=float, default=[0.0, 0.0, 0.155])
    parser.add_argument("--input-frame", choices=("tool", "tcp"), default="tool")
    parser.add_argument(
        "--pre-place",
        nargs=7,
        type=float,
        required=True,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
    )
    parser.add_argument(
        "--place",
        nargs=7,
        type=float,
        required=True,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
    )
    parser.add_argument(
        "--home",
        nargs=7,
        type=float,
        default=[-0.298, 0.191, 0.541, -0.037, 0.378, 0.605, 0.700],
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
    )
    parser.add_argument("--robot-program-topic", default="/right_arm/ur_hardware_interface/robot_program_running")
    parser.add_argument("--script-topic", default="/right_arm/ur_hardware_interface/script_command")
    parser.add_argument("--gripper-topic", default="/right_arm/Robotiq2FGripperRobotOutput")
    parser.add_argument("--open-position", type=int, default=0)
    parser.add_argument("--close-position", type=int, default=255)
    parser.add_argument("--gripper-speed", type=int, default=80)
    parser.add_argument("--gripper-force", type=int, default=80)
    parser.add_argument("--accel", type=float, default=0.08)
    parser.add_argument("--vel", type=float, default=0.04)
    parser.add_argument("--target-tolerance", type=float, default=0.025)
    parser.add_argument("--move-timeout", type=float, default=25.0)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def normalize_quat(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if norm <= 0:
        raise ValueError("zero quaternion")
    return q / norm


def quat_matrix(q):
    x, y, z, w = normalize_quat(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def quat_to_rotvec(q):
    x, y, z, w = normalize_quat(q)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    angle = 2.0 * math.atan2(math.sqrt(x * x + y * y + z * z), w)
    s = math.sin(angle / 2.0)
    if abs(s) < 1e-9:
        return [0.0, 0.0, 0.0]
    return [angle * x / s, angle * y / s, angle * z / s]


def tcp_from_tool(tool_xyz, quat, offset):
    return np.asarray(tool_xyz, dtype=float) + quat_matrix(quat).dot(
        np.asarray(offset, dtype=float)
    )


def pose_values_to_tcp(values, input_frame, tcp_offset):
    xyz = np.asarray(values[:3], dtype=float)
    quat = normalize_quat(values[3:])
    if input_frame == "tool":
        xyz = tcp_from_tool(xyz, quat, tcp_offset)
    return xyz, quat


def make_gripper_command(position, speed, force):
    msg = Robotiq2FGripper_robot_output()
    msg.rACT = 1
    msg.rGTO = 1
    msg.rATR = 0
    msg.rPR = int(position)
    msg.rSP = int(speed)
    msg.rFR = int(force)
    return msg


class URScriptCurrentGraspPlace:
    def __init__(self, args):
        self.args = args
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.script_pub = rospy.Publisher(args.script_topic, String, queue_size=1)
        self.gripper_pub = rospy.Publisher(
            args.gripper_topic, Robotiq2FGripper_robot_output, queue_size=1
        )

    def wait_ready(self):
        msg = rospy.wait_for_message(self.args.robot_program_topic, Bool, timeout=3.0)
        if not msg.data:
            raise RuntimeError("Right arm External Control program is not running.")

    def current_tool_xyz(self):
        transform = self.tf_buffer.lookup_transform(
            self.args.base_frame,
            self.args.tool_frame,
            rospy.Time(0),
            rospy.Duration(3.0),
        )
        t = transform.transform.translation
        return np.array([t.x, t.y, t.z], dtype=float)

    def command_gripper(self, position, label):
        rospy.loginfo("Gripper %s position=%d", label, position)
        if not self.args.execute:
            return
        deadline = time.time() + 5.0
        while self.gripper_pub.get_num_connections() == 0 and time.time() < deadline:
            rospy.sleep(0.1)
        if self.gripper_pub.get_num_connections() == 0:
            raise RuntimeError("No subscriber on {}".format(self.args.gripper_topic))
        msg = make_gripper_command(
            position, self.args.gripper_speed, self.args.gripper_force
        )
        for _ in range(8):
            self.gripper_pub.publish(msg)
            rospy.sleep(0.05)
        rospy.sleep(1.0)

    def script_for_tcp_pose(self, name, xyz, quat):
        rx, ry, rz = quat_to_rotvec(quat)
        return (
            "def codex_{name}():\n"
            "  set_tcp(p[{ox:.6f},{oy:.6f},{oz:.6f},0,0,0])\n"
            "  movej(get_inverse_kin(p[{x:.6f},{y:.6f},{z:.6f},{rx:.6f},{ry:.6f},{rz:.6f}]), a={a:.5f}, v={v:.5f})\n"
            "end\n"
        ).format(
            name=name,
            ox=self.args.tcp_offset[0],
            oy=self.args.tcp_offset[1],
            oz=self.args.tcp_offset[2],
            x=xyz[0],
            y=xyz[1],
            z=xyz[2],
            rx=rx,
            ry=ry,
            rz=rz,
            a=self.args.accel,
            v=self.args.vel,
        )

    def publish_script(self, name, script):
        rospy.loginfo("Publishing URScript move: %s", name)
        rospy.loginfo("URScript:\n%s", script.strip())
        if not self.args.execute:
            return
        deadline = time.time() + 5.0
        while self.script_pub.get_num_connections() == 0 and time.time() < deadline:
            rospy.sleep(0.1)
        if self.script_pub.get_num_connections() == 0:
            raise RuntimeError("No subscriber on {}".format(self.args.script_topic))
        self.script_pub.publish(String(data=script))

    def wait_until_near_tool(self, target_tool_xyz, name):
        if not self.args.execute:
            return
        deadline = time.time() + self.args.move_timeout
        last_error = None
        while time.time() < deadline and not rospy.is_shutdown():
            current = self.current_tool_xyz()
            last_error = float(np.linalg.norm(current - target_tool_xyz))
            if last_error <= self.args.target_tolerance:
                rospy.loginfo("%s reached, tool error=%.3fm", name, last_error)
                return
            rospy.sleep(0.2)
        raise RuntimeError(
            "{} did not reach target; last tool error={:.3f}m".format(
                name, last_error if last_error is not None else float("nan")
            )
        )

    def move_to(self, name, values, input_frame):
        tcp_xyz, quat = pose_values_to_tcp(values, input_frame, self.args.tcp_offset)
        target_tool_xyz = np.asarray(values[:3], dtype=float)
        if input_frame == "tcp":
            target_tool_xyz = tcp_xyz - quat_matrix(quat).dot(
                np.asarray(self.args.tcp_offset, dtype=float)
            )
        rospy.loginfo(
            "%s target tcp=[%.3f, %.3f, %.3f] tool=[%.3f, %.3f, %.3f]",
            name,
            tcp_xyz[0],
            tcp_xyz[1],
            tcp_xyz[2],
            target_tool_xyz[0],
            target_tool_xyz[1],
            target_tool_xyz[2],
        )
        self.publish_script(name, self.script_for_tcp_pose(name, tcp_xyz, quat))
        self.wait_until_near_tool(target_tool_xyz, name)

    def run(self):
        self.wait_ready()
        current = self.current_tool_xyz()
        rospy.loginfo(
            "Current tool xyz=[%.3f, %.3f, %.3f]",
            current[0],
            current[1],
            current[2],
        )
        self.command_gripper(self.args.close_position, "close")
        self.move_to("pre_place", self.args.pre_place, self.args.input_frame)
        self.move_to("place", self.args.place, self.args.input_frame)
        self.command_gripper(self.args.open_position, "open")
        self.move_to("home", self.args.home, "tcp")


def main():
    args = parse_args()
    rospy.init_node("urscript_current_grasp_place")
    rospy.loginfo("Mode: %s", "REAL EXECUTION" if args.execute else "PLAN ONLY")
    URScriptCurrentGraspPlace(args).run()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logerr("%s", exc)
        sys.exit(1)
