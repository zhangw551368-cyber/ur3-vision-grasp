#!/usr/bin/python3

import math
import sys
import time

import moveit_commander
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


RIGHT_START_JOINTS = {
    "right_arm_elbow_joint": -0.4332788626300257,
    "right_arm_shoulder_lift_joint": -2.4318178335772913,
    "right_arm_shoulder_pan_joint": 2.5958969593048096,
    "right_arm_wrist_1_joint": 4.361816883087158,
    "right_arm_wrist_2_joint": 0.7190194129943848,
    "right_arm_wrist_3_joint": 3.250563383102417,
}

SIDE_GRASP_Q = (-0.5, 0.5, 0.5, 0.5)

# Computed from the current TF geometry of both inner finger pads.
# With SIDE_GRASP_Q this is approximately base [0, +0.130, 0].
TOOL_TO_GRASP_CENTER_TOOL = (0.00047704, -0.00086203, 0.13032177)
TOOL_TO_GRASP_CENTER_BASE = (0.00086203, 0.13032177, -0.00047704)


def make_pose(xyz, quat=SIDE_GRASP_Q):
    pose = PoseStamped()
    pose.header.frame_id = "base"
    pose.header.stamp = rospy.Time.now()
    pose.pose = Pose()
    pose.pose.position.x = xyz[0]
    pose.pose.position.y = xyz[1]
    pose.pose.position.z = xyz[2]
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    return pose


def tool_from_center(center):
    return [
        center[0] - TOOL_TO_GRASP_CENTER_BASE[0],
        center[1] - TOOL_TO_GRASP_CENTER_BASE[1],
        center[2] - TOOL_TO_GRASP_CENTER_BASE[2],
    ]


def color(r, g, b, a=0.85):
    msg = ColorRGBA()
    msg.r = r
    msg.g = g
    msg.b = b
    msg.a = a
    return msg


class SimPickPlace:
    def __init__(self):
        self.arm = moveit_commander.MoveGroupCommander("right_arm")
        self.arm.set_pose_reference_frame("base")
        self.arm.set_end_effector_link("right_arm_tool0")
        self.arm.set_max_velocity_scaling_factor(0.08)
        self.arm.set_max_acceleration_scaling_factor(0.08)
        self.arm.set_planning_time(10.0)
        self.arm.set_num_planning_attempts(10)
        self.arm.set_goal_position_tolerance(0.02)
        self.arm.set_goal_orientation_tolerance(0.28)
        self.display = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=1,
            latch=True,
        )
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.markers = rospy.Publisher(
            "/sim_pick_place/markers", MarkerArray, queue_size=1, latch=True
        )
        self.executed = []

    def publish_static_markers(self, red_real, pick_center, place_center, board_center):
        marker_array = MarkerArray()
        marker_array.markers.append(
            self.box_marker(
                1,
                "real_detected_low_reference",
                red_real,
                (0.045, 0.035, 0.050),
                color(1.0, 0.05, 0.02, 0.25),
            )
        )
        marker_array.markers.append(
            self.box_marker(
                2,
                "sim_pick_block_start",
                pick_center,
                (0.045, 0.035, 0.050),
                color(1.0, 0.05, 0.02, 0.90),
            )
        )
        marker_array.markers.append(
            self.box_marker(
                3,
                "place_on_board",
                place_center,
                (0.045, 0.035, 0.050),
                color(0.1, 0.8, 1.0, 0.65),
            )
        )
        board = self.box_marker(
            4,
            "aruco_board",
            board_center,
            (0.297, 0.210, 0.004),
            color(1.0, 1.0, 1.0, 0.45),
        )
        board.pose.orientation.z = math.sin(math.radians(8.35) / 2.0)
        board.pose.orientation.w = math.cos(math.radians(8.35) / 2.0)
        marker_array.markers.append(board)
        self.markers.publish(marker_array)

        self.scene.remove_world_object("sim_red_block")
        self.scene.remove_world_object("aruco_board")
        time.sleep(0.2)
        self.scene.add_box(
            "sim_red_block",
            make_pose(pick_center, (0.0, 0.0, 0.0, 1.0)),
            size=(0.045, 0.035, 0.050),
        )
        board_pose = make_pose(board_center, (0.0, 0.0, 0.0, 1.0))
        board_pose.pose.orientation.z = math.sin(math.radians(8.35) / 2.0)
        board_pose.pose.orientation.w = math.cos(math.radians(8.35) / 2.0)
        self.scene.add_box(
            "aruco_board",
            board_pose,
            size=(0.297, 0.210, 0.004),
        )

    def publish_block_marker_world(self, center, label):
        marker = self.box_marker(
            10,
            label,
            center,
            (0.045, 0.035, 0.050),
            color(1.0, 0.05, 0.02, 0.95),
        )
        self.markers.publish(MarkerArray(markers=[marker]))

    def publish_block_marker_attached(self):
        marker = Marker()
        marker.header.frame_id = "right_arm_tool0"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "sim_pick_place"
        marker.id = 10
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = TOOL_TO_GRASP_CENTER_TOOL[0]
        marker.pose.position.y = TOOL_TO_GRASP_CENTER_TOOL[1]
        marker.pose.position.z = TOOL_TO_GRASP_CENTER_TOOL[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.045
        marker.scale.y = 0.035
        marker.scale.z = 0.050
        marker.color = color(1.0, 0.05, 0.02, 0.95)
        self.markers.publish(MarkerArray(markers=[marker]))

    def attach_block(self):
        touch_links = [
            "right_robotiq_arg2f_base_link",
            "right_left_inner_finger",
            "right_left_inner_finger_pad",
            "right_right_inner_finger",
            "right_right_inner_finger_pad",
            "right_left_outer_finger",
            "right_right_outer_finger",
        ]
        self.scene.attach_box(
            "right_arm_tool0",
            "sim_red_block",
            touch_links=touch_links,
        )
        self.publish_block_marker_attached()
        rospy.loginfo("Attached sim_red_block to right_arm_tool0.")
        time.sleep(0.5)

    def detach_block(self, place_center):
        self.scene.remove_attached_object("right_arm_tool0", name="sim_red_block")
        time.sleep(0.3)
        self.scene.add_box(
            "sim_red_block",
            make_pose(place_center, (0.0, 0.0, 0.0, 1.0)),
            size=(0.045, 0.035, 0.050),
        )
        self.publish_block_marker_world(place_center, "sim_block_placed_on_board")
        rospy.loginfo("Detached sim_red_block at board place center.")
        time.sleep(0.5)

    @staticmethod
    def box_marker(marker_id, name, center, scale, marker_color):
        marker = Marker()
        marker.header.frame_id = "base"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "sim_pick_place"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color = marker_color
        marker.text = name
        return marker

    def execute_joint_start(self):
        rospy.loginfo("Moving fake robot to copied real start joints.")
        self.arm.set_joint_value_target(RIGHT_START_JOINTS)
        plan = self.plan_current_target("start_joints")
        self.execute_plan("start_joints", plan)

    def plan_current_target(self, name):
        result = self.arm.plan()
        trajectory = result[1] if isinstance(result, tuple) else result
        if not trajectory.joint_trajectory.points:
            raise RuntimeError("No plan for {}".format(name))
        rospy.loginfo(
            "Planned %-12s points=%d duration=%.2fs",
            name,
            len(trajectory.joint_trajectory.points),
            trajectory.joint_trajectory.points[-1].time_from_start.to_sec(),
        )
        return trajectory

    def plan_pose(self, name, xyz):
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(make_pose(xyz))
        plan = self.plan_current_target(name)
        self.arm.clear_pose_targets()
        return plan

    def execute_plan(self, name, plan):
        if not self.arm.execute(plan, wait=True):
            self.arm.stop()
            raise RuntimeError("Fake execution failed at {}".format(name))
        self.arm.stop()
        self.executed.append(name)
        rospy.loginfo("Fake executed %s", name)
        time.sleep(0.3)

    def run(self):
        # Current Kinect2 detections from the real scene. The real red block is
        # lower than the first reachable fake grasp center; keeping both markers
        # visible makes the remaining TCP/model error obvious in RViz.
        red_real = [0.5436, 0.0059, 0.0225]
        board_center = [0.669, -0.197, -0.007]

        pick_center = [red_real[0], red_real[1], 0.200]
        place_center = [0.544, -0.124, 0.200]

        pre_pick_tool = tool_from_center([pick_center[0], pick_center[1] - 0.080, 0.240])
        pick_tool = tool_from_center(pick_center)
        lift_tool = tool_from_center([pick_center[0], pick_center[1], 0.250])
        pre_place_tool = tool_from_center([place_center[0], place_center[1] - 0.080, 0.240])
        place_tool = tool_from_center(place_center)
        retreat_tool = tool_from_center([place_center[0], place_center[1] - 0.080, 0.250])

        rospy.loginfo("Detected red real center marker: %s", red_real)
        rospy.loginfo("Sim pick center: %s", pick_center)
        rospy.loginfo("Sim place center on board: %s", place_center)
        self.publish_static_markers(red_real, pick_center, place_center, board_center)
        self.publish_block_marker_world(pick_center, "sim_block_waiting_for_pick")

        self.execute_joint_start()
        for name, xyz in [
            ("pre_pick", pre_pick_tool),
            ("pick", pick_tool),
            ("lift", lift_tool),
            ("pre_place", pre_place_tool),
            ("place", place_tool),
            ("retreat", retreat_tool),
        ]:
            plan = self.plan_pose(name, xyz)
            self.execute_plan(name, plan)
            if name == "pick":
                self.attach_block()
            if name == "place":
                self.detach_block(place_center)

        rospy.loginfo("SIM_PICK_PLACE_SUCCESS stages=%s", ",".join(self.executed))


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("sim_right_arm_kinect2_pick_place")
    SimPickPlace().run()
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
