#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 开启严格错误处理。
set -euo pipefail
# 加载 ROS Noetic，使 tf 工具可用。
source /opt/ros/noetic/setup.bash
# 加载当前工作空间。
source /home/gzu/gzu_ws/devel/setup.bash
# 持续显示右臂基座到 RealSense 彩色光学坐标系的组合变换。
exec rosrun tf tf_echo right_arm_base camera_color_optical_frame

