#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 开启严格错误处理。
set -euo pipefail
# 加载 ROS Noetic。
source /opt/ros/noetic/setup.bash
# 加载包含夹爪激活脚本的工作空间。
source /home/gzu/gzu_ws/devel/setup.bash
# 激活右夹爪，并以速度 120、力 80 打开到位置 0。
exec rosrun dual_arm_tasks activate_right_gripper.py _command_topic:=/right_arm/Robotiq2FGripperRobotOutput _open_position:=0 _speed:=120 _force:=80

