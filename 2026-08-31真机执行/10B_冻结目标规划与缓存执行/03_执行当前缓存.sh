#!/usr/bin/env bash
# 上一行指定 Bash 解释器。
# 开启严格错误处理。
set -euo pipefail
# 指定经过验证的原始 GraspNet 配置文件。
CONFIG=/home/gzu/gzu_ws/ur3_graspnet6dof_20260824/config/right_arm_green_table.yaml
# 只有操作者已显式把执行锁改为 true 时才继续。
if ! sed -n '/^execution:/,/^[^ ]/p' "$CONFIG" | grep -Eq '^  enabled: true([[:space:]]|$)'; then printf '%s\n' '执行锁仍为 false。完成 RViz 和安全检查后，才可临时改为 true。'; exit 3; fi
# 明确提示下一步会执行当前缓存中的真实轨迹。
printf '%s\n' '警告：将执行当前 cached_sequence_plan.pkl。确认缓存为本轮生成、机器人未移动、现场清空且急停可触达后，输入 EXECUTE。'
# 读取人工确认文本。
read -r confirmation
# 非精确确认时立即取消。
if [[ "$confirmation" != "EXECUTE" ]]; then printf '%s\n' '已取消，未发送运动命令。'; exit 1; fi
# 进入独立 GraspNet 工程。
cd /home/gzu/gzu_ws/ur3_graspnet6dof_20260824
# 加载 ROS、MoveIt 和工程 Python 路径。
source /home/gzu/gzu_ws/ur3_graspnet6dof_20260824/env.sh
# 校验缓存年龄和关节起点后，执行当前缓存；程序内部仍会检查执行锁和夹爪反馈。
exec /usr/bin/python3 /home/gzu/gzu_ws/ur3_graspnet6dof_20260824/scripts/execute_cached_sequence.py --config "$CONFIG" --execute --yes

