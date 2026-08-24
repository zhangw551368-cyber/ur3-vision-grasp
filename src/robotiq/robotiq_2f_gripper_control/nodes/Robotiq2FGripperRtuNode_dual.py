#!/usr/bin/env python3

import rospy
import sys
from robotiq_2f_gripper_control.baseRobotiq2FGripper import robotiqbaseRobotiq2FGripper
from robotiq_modbus_rtu.comModbusRtu import communication
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input, Robotiq2FGripper_robot_output

class DualArmGripperController:
    def __init__(self, left_device, right_device):
        # 初始化左臂夹爪
        self.left_gripper = robotiqbaseRobotiq2FGripper()
        self.left_gripper.client = communication()
        self.left_gripper.client.connectToDevice(left_device)
        
        # 初始化右臂夹爪
        self.right_gripper = robotiqbaseRobotiq2FGripper()
        self.right_gripper.client = communication()
        self.right_gripper.client.connectToDevice(right_device)

        # 创建带命名空间的话题
        self.left_pub = rospy.Publisher(
            '/left_arm/Robotiq2FGripperRobotInput', 
            Robotiq2FGripper_robot_input,
            queue_size=10
        )
        
        self.right_pub = rospy.Publisher(
            '/right_arm/Robotiq2FGripperRobotInput', 
            Robotiq2FGripper_robot_input,
            queue_size=10
        )

        # 订阅左右臂控制指令
        rospy.Subscriber(
            '/left_arm/Robotiq2FGripperRobotOutput',
            Robotiq2FGripper_robot_output,
            self.left_gripper.refreshCommand
        )
        
        rospy.Subscriber(
            '/right_arm/Robotiq2FGripperRobotOutput',
            Robotiq2FGripper_robot_output,
            self.right_gripper.refreshCommand
        )

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            # 获取并发布左臂状态；某一路串口无响应时不要让双夹爪节点整体退出。
            try:
                left_status = self.left_gripper.getStatus()
                self.left_pub.publish(left_status)
            except Exception as exc:
                rospy.logwarn_throttle(
                    5.0, "left gripper status read failed: %s", exc
                )
            
            # 发送左臂控制指令
            try:
                self.left_gripper.sendCommand()
            except Exception as exc:
                rospy.logwarn_throttle(
                    5.0, "left gripper command send failed: %s", exc
                )
            
            # 获取并发布右臂状态
            try:
                right_status = self.right_gripper.getStatus()
                self.right_pub.publish(right_status)
            except Exception as exc:
                rospy.logwarn_throttle(
                    5.0, "right gripper status read failed: %s", exc
                )
            
            # 发送右臂控制指令
            try:
                self.right_gripper.sendCommand()
            except Exception as exc:
                rospy.logwarn_throttle(
                    5.0, "right gripper command send failed: %s", exc
                )
            
            rate.sleep()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        rospy.logerr("请指定两个串口设备路径！")
        print("使用示例: robotiq_2f_gripper_modbus_dual_node.py /dev/ttyUSB0 /dev/ttyUSB1")
        sys.exit(1)
        
    try:
        rospy.init_node("dual_arm_robotiq_gripper")
        controller = DualArmGripperController(sys.argv[1], sys.argv[2])
        controller.run()
    except rospy.ROSInterruptException:
        pass
