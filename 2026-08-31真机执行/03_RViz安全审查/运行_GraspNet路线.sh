#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 开启严格错误处理。
set -euo pipefail
# 进入 GraspNet 独立工程。
cd /home/gzu/gzu_ws/ur3_graspnet6dof_20260824
# 退出可能冲突的 Conda 环境。
if type conda >/dev/null 2>&1; then conda deactivate 2>/dev/null || true; fi
# 加载 ROS Noetic 环境。
source /opt/ros/noetic/setup.bash
# 加载工作空间内的机器人和 MoveIt 包。
source /home/gzu/gzu_ws/devel/setup.bash
# 使用专门显示 GraspNet Marker 和完整轨迹的 RViz 配置。
exec rosrun rviz rviz -d /home/gzu/gzu_ws/ur3_graspnet6dof_20260824/rviz/graspnet_planning.rviz

