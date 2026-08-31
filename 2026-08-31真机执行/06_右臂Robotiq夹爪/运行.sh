#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 开启严格错误处理。
set -euo pipefail
# 进入工作空间。
cd /home/gzu/gzu_ws
# 退出可能冲突的 Conda 环境。
if type conda >/dev/null 2>&1; then conda deactivate 2>/dev/null || true; fi
# 加载 ROS Noetic。
source /opt/ros/noetic/setup.bash
# 加载工作空间内的 Robotiq 消息和节点。
source /home/gzu/gzu_ws/devel/setup.bash
# 把 Robotiq Modbus Python 模块加入模块搜索路径。
export PYTHONPATH="/home/gzu/gzu_ws/src/robotiq/robotiq_modbus_rtu/src:${PYTHONPATH:-}"
# 通过固定的 FTDI RS-485 设备路径启动右夹爪驱动节点。
exec rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode_right.py /dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DA6ACQ6P-if00-port0

