#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 不使用 -e，以便一次显示所有检查结果。
set -uo pipefail
# 加载 ROS Noetic。
source /opt/ros/noetic/setup.bash
# 加载工作空间消息和服务类型。
source /home/gzu/gzu_ws/devel/setup.bash
# 查询 External Control 是否运行。
rosservice call /right_arm/ur_hardware_interface/dashboard/program_running
# 查询示教器程序状态。
rosservice call /right_arm/ur_hardware_interface/dashboard/program_state
# 读取机器人模式，期望 mode=7。
rostopic echo -n 1 /right_arm/ur_hardware_interface/robot_mode
# 读取安全模式，期望 mode=1。
rostopic echo -n 1 /right_arm/ur_hardware_interface/safety_mode
# 检查右臂轨迹控制器处于 running。
rosservice call /right_arm/controller_manager/list_controllers
# 读取夹爪状态，检查 gACT、gSTA、gFLT 和 gPO。
rostopic echo -n 1 /right_arm/Robotiq2FGripperRobotInput

