#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 不使用 -e，以便某一项失败时仍可看到其余检查结果。
set -uo pipefail
# 加载 ROS Noetic。
source /opt/ros/noetic/setup.bash
# 加载工作空间消息和服务类型。
source /home/gzu/gzu_ws/devel/setup.bash
# 查询示教器 External Control 程序是否正在运行。
rosservice call /right_arm/ur_hardware_interface/dashboard/program_running
# 查询示教器当前程序状态。
rosservice call /right_arm/ur_hardware_interface/dashboard/program_state
# 读取一次机器人模式，正常运行期应为 mode=7。
rostopic echo -n 1 /right_arm/ur_hardware_interface/robot_mode
# 读取一次安全模式，正常期应为 mode=1。
rostopic echo -n 1 /right_arm/ur_hardware_interface/safety_mode
# 读取一次右夹爪输入状态，检查激活、故障和当前位置。
rostopic echo -n 1 /right_arm/Robotiq2FGripperRobotInput

