# UR3 Dual-Arm Vision Pick-and-Place — 2026-08-23 Baseline

这是双 UR3 平台右臂三蓝色物块连续视觉抓放任务的可复现备份。仓库基线对应 2026-08-23 实机成功运行：三个物块全部抓取，在棋盘格上方高位松开，最后返回右臂初始位。

## 仓库命名

建议 GitHub 仓库名：

```text
ur3-dual-arm-vision-pick-place-20260823
```

建议创建为 **Private repository**。本备份保留了真实设备所需的局域网 IP、相机序列号、夹爪串口标识和标定参数，不适合未经脱敏直接公开。

## 已备份内容

| 路径 | 内容 |
|---|---|
| `src/dual_arm_tasks` | HSV/深度感知、三物块顺序规划、锁点复核、夹爪控制、真实环境碰撞场景 |
| `src/ur3_dual_moveit_config` | 双臂 SRDF、规划组、控制器、OMPL、RViz 和 MoveIt 启动配置 |
| `src/ur_description` | 双 UR3 URDF/Xacro、网格和双 Robotiq 装配模型 |
| `src/ur_robot_driver` | 当前实机使用的 UR ROS Driver 及定制双臂 bringup |
| `src/robotiq` | Robotiq 2F 串口控制、消息和可视化模型 |
| `calibration/environment` | 桌面/机柜环境、2026-08-23 三目标锁点及历史对照 YAML |
| `docs` | 2026-08-21～22 汇总和 2026-08-23 最终成功记录 |
| `dependencies` | 操作系统、ROS、Python 与关键软件包版本基线 |
| `scripts` | 新机器恢复和构建脚本 |

未上传的内容包括 `build/`、`devel/`、ROS 日志、视频、RGB-D 原始帧、大型训练数据和 Python 缓存。这些内容可重建或体积过大，不是复现昨天任务所必需。

## 成功运行摘要

```text
REAL EXECUTION complete: 3 block(s), 16 motion segments
```

- 运动前从无遮挡的最新相机画面一次性锁定三个物块；
- 执行开始后冻结目标，机械臂遮挡相机时不刷新坐标；
- 两个长方体用俯视抓取，夹爪方向对齐视觉长轴；
- 魔方沿用 2026-08-21 已成功的约 40° 斜抓四元数；
- 棋盘上方直接松爪，TCP 高于棋盘表面至少 120 mm；
- 三次夹持反馈均为 `gOBJ=2`，最终机器人和安全模式正常。

完整数据、终端命令和安全门控见：

- `docs/right_arm_three_blue_pick_place_success_20260823_zh.md`
- `docs/right_arm_real_environment_three_blue_pick_place_complete_20260821_22_zh.md`

## 新机器恢复

目标系统建议与原机一致：Ubuntu 20.04 + ROS Noetic。

```bash
git clone <YOUR_REPOSITORY_URL> ~/ur3-dual-arm-vision-pick-place-20260823
cd ~/ur3-dual-arm-vision-pick-place-20260823
bash scripts/restore_workspace.sh
```

脚本会运行 `rosdep` 并构建任务所需的核心包。系统包尚未安装时，先阅读 `dependencies/README.md`。

构建完成后：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## 规划与实机执行

规划命令：

```bash
rosrun dual_arm_tasks right_arm_three_blue_pick_place.py \
  --config src/dual_arm_tasks/config/right_arm_three_blue_pick_place_20260822.yaml \
  --targets-file calibration/environment/right_arm_three_blue_locked_targets_20260823.yaml
```

旧锁点仅用于复现和审计。只要物块、棋盘、相机或机器人底座发生移动，必须重新识别、锁定、复核和规划。

实机运动必须满足以下全部条件：

1. RViz 轨迹和碰撞场景已经人工检查；
2. 现场工作区清空；
3. 机器人处于 RUNNING，安全模式 NORMAL；
4. 右夹爪已激活、打开且无故障；
5. 示教器 External Control 已开启；
6. 现场操作者对本次轨迹明确确认执行。

## 仓库许可说明

本备份由多个来源组成，保留各上游包中的原始许可证。`dual_arm_tasks` 的包清单声明 MIT；UR Driver、UR Description 与 Robotiq 组件分别遵循其目录内许可证。不要用一个顶层许可证覆盖所有第三方代码。

