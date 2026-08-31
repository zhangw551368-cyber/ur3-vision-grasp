#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 开启严格错误处理。
set -euo pipefail
# 从调用环境读取当前场景的冻结目标；未提供时保持为空。
TARGET_SPECS="${TARGET_SPECS:-}"
# 不允许默认复用历史像素，防止物体移动后误规划。
if [[ -z "$TARGET_SPECS" ]]; then printf '%s\n' "用法：TARGET_SPECS='类别:u:v:半径,...' bash $0"; exit 2; fi
# 进入独立 GraspNet 工程。
cd /home/gzu/gzu_ws/ur3_graspnet6dof_20260824
# 加载 ROS、MoveIt 和工程 Python 路径。
source /home/gzu/gzu_ws/ur3_graspnet6dof_20260824/env.sh
# 对当前冻结目标完成全流程规划并写入缓存；没有 --execute，因此不会发送真机命令。
exec /usr/bin/python3 /home/gzu/gzu_ws/ur3_graspnet6dof_20260824/scripts/multi_object_sequence.py --target-specs "$TARGET_SPECS"

