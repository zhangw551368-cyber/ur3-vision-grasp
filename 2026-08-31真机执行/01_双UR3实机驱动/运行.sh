#!/usr/bin/env bash
# 上一行让系统使用当前环境中的 Bash 解释本脚本。
# 遇到命令失败、未定义变量或管道失败时立即停止。
set -euo pipefail
# 进入本次 ROS 工作空间，保证相对路径一致。
cd /home/gzu/gzu_ws
# 如果当前终端存在 Conda 命令，则退出 Conda 环境，避免污染 ROS Python。
if type conda >/dev/null 2>&1; then conda deactivate 2>/dev/null || true; fi
# 加载 ROS Noetic 的基础环境变量。
source /opt/ros/noetic/setup.bash
# 加载本工作空间编译生成的 ROS 包和消息。
source /home/gzu/gzu_ws/devel/setup.bash
# 启动双 UR3 真机驱动；exec 让 Ctrl+C 直接停止 roslaunch。
exec roslaunch ur_robot_driver ur3_dual_bringup.launch

