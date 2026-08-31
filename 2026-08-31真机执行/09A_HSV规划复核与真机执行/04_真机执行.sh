#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 开启严格错误处理。
set -euo pipefail
# 明确提示这条命令会驱动真实机械臂。
printf '%s\n' '警告：下一步将执行三蓝块真机轨迹。确认 RViz、锁点复核、External Control、现场和急停后，输入 EXECUTE。'
# 从键盘读取一行确认文本。
read -r confirmation
# 如果输入不是精确的 EXECUTE，就安全退出。
if [[ "$confirmation" != "EXECUTE" ]]; then printf '%s\n' '已取消，未发送运动命令。'; exit 1; fi
# 加载 ROS Noetic。
source /opt/ros/noetic/setup.bash
# 加载真机执行节点。
source /home/gzu/gzu_ws/devel/setup.bash
# 使用历史配置和锁点执行；--accept-provisional-board 表示接受尚未尺量确认的 22.7 mm 棋盘格。
exec rosrun dual_arm_tasks right_arm_three_blue_pick_place.py --config /home/gzu/gzu_ws/src/dual_arm_tasks/config/right_arm_three_blue_pick_place_20260822.yaml --targets-file /home/gzu/gzu_ws/calibration/environment/right_arm_three_blue_locked_targets_20260823.yaml --execute --yes --accept-provisional-board

