# UR3 + GraspNet 6-DoF 独立抓取工程

本目录是 2026-08-24 新建的独立工程。它不修改 `/home/gzu/gzu_ws/src`、现有 HSV 抓取、手眼标定、MoveIt 配置或碰撞环境文件，也不加入原 catkin 编译。

目标场景是固定俯视 RealSense、绿色工作台和图中平铺工件。第一版仅放行接近方向距竖直向下不超过 30° 的候选，适合优先测试钳子、剪钳、螺栓和较厚圆环。薄金属圆环、反光螺母和很小的黑色零件受深度空洞、夹爪指宽和最小开口影响，不应作为第一次真机目标。

## 目录边界

```text
ur3_graspnet6dof_20260824/
├── .conda_env/                   独立 Python/CUDA 环境
├── checkpoints/                 只放 checkpoint-rs.tar
├── third_party/                 官方 graspnet-baseline、graspnetAPI
├── config/                      本工程独立参数
├── python/                      推理与几何公共模块
├── scripts/                     感知、规划、执行、检查脚本
├── launch/                      可选 ROS launch
└── runtime/                     最新候选图、JSON 和运行记录
```

运行 `env.sh` 只修改当前终端的 `PATH`、`PYTHONPATH` 和 `ROS_PACKAGE_PATH`。

## 已实现的数据流

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
        ↓ 同步、单位/内参/ROI检查
GraspNet Baseline + checkpoint-rs.tar
        ↓ NMS + 模型自由碰撞检测
完整 Top-N 6-DoF 候选（原子 JSON + PoseArray + RViz Marker）
        ↓ camera_color_optical_frame → base
宽度、工作区、桌面高度、近似竖直 approach 过滤
        ↓ GraspNet 坐标轴 → Robotiq/tool0 坐标轴
逐候选 MoveIt：pre-grasp → 笛卡尔 grasp → 垂直 lift
        ↓
默认 plan-only；显式解锁后可低速 pick-hold
```

GraspNet 的 `+X` 是 approach、`+Y` 是夹爪开口方向。本机器人模型中 tool `+Z` 指向夹爪中心、tool `+Y` 是 Robotiq 开口方向，因此代码使用：

```text
tool Z = grasp X
tool Y = grasp Y
tool X = -grasp Z
```

并使用现有实机验证过的 `tool0 → 抓取中心 = 0.130 m`。
当前平铺工件配置会保留预测的夹爪开口方向，但将执行 approach 规范化为竖直向下，避免斜抓增加 UR3 外伸和横向扫碰。

## 1. 一次性安装

```bash
cd /home/gzu/gzu_ws/ur3_graspnet6dof_20260824
./scripts/setup_environment.sh
./scripts/download_checkpoint.sh
./scripts/validate_project.py
```

安装脚本不会修改全局 Conda 配置。环境固定使用 Python 3.8、PyTorch 2.1.2 + CUDA 12.1、NumPy 1.x 和 Open3D 0.18，并为 RTX 4090 编译 `sm_89` 的 pointnet2/knn 扩展。

## 2. 启动已有机器人系统

继续使用已经验证的终端和配置：

```bash
# 终端 A：UR 双臂 driver
conda deactivate
source /home/gzu/gzu_ws/devel/setup.bash
roslaunch ur_robot_driver ur3_dual_bringup.launch

# 终端 B：MoveIt
conda deactivate
source /home/gzu/gzu_ws/devel/setup.bash
roslaunch ur3_dual_moveit_config move_group.launch moveit_controller_manager:=simple_right

# 终端 C：右 Robotiq
conda deactivate
source /home/gzu/gzu_ws/devel/setup.bash
PYTHONPATH=/home/gzu/gzu_ws/src/robotiq/robotiq_modbus_rtu/src:$PYTHONPATH \
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode_right.py \
/dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DA6ACQ6P-if00-port0
```

另外启动：

- 当前 `/camera` RealSense，必须启用 `align_depth:=true enable_sync:=true`；
- 已验证的、只发布一个父节点的手眼 TF；
- 当前真实碰撞环境；
- RViz。

实时检查：

```bash
cd /home/gzu/gzu_ws/ur3_graspnet6dof_20260824
source env.sh
./scripts/validate_project.py --live
```

必须看到 RGB、aligned depth、camera_info 都是 `camera_color_optical_frame`，且存在 `base <- camera_color_optical_frame` TF。

## 3. 启动 GraspNet 感知

新终端：

```bash
cd /home/gzu/gzu_ws/ur3_graspnet6dof_20260824
./scripts/start_inference.sh
```

节点只缓存同步 RGB-D，不会持续推理。每次执行器调用 `/ur3_graspnet6d/infer` 时才推理一次。

可以单独触发：

```bash
rosservice call /ur3_graspnet6d/infer
```

结果：

```text
/ur3_graspnet6d/candidates_camera
/ur3_graspnet6d/best_grasp_camera
/ur3_graspnet6d/candidate_markers
/ur3_graspnet6d/candidates_json
runtime/latest_candidates.png
runtime/latest_result.json
```

`latest_candidates.png` 中红圈应落在工件而不是绿色桌面或右侧机器框架上。

## 4. 必须先 plan-only

确保 MoveIt 已加载绿色工作台碰撞环境，然后运行：

```bash
cd /home/gzu/gzu_ws/ur3_graspnet6dof_20260824
./scripts/plan_pick_once.sh
```

该命令结构上没有执行参数，不会发送轨迹或夹爪命令。它会：

1. 触发一帧新推理；
2. 检查拟合桌面是否接近当前观测中心值 `base z=-0.010 m`；
3. 用唯一名称添加与观测平面对齐的 MoveIt 安全薄板；
4. 过滤宽度、桌面高度、工作区和 approach；
5. 对 Top-20 候选及两个等价腕部方向逐个尝试；
6. 找到第一条完整的 `pre-grasp → grasp → lift` 路径；
7. 在 RViz 发布完整轨迹和红色 selected arrow。

如果要只检查某个网络排序，可使用：

```bash
./scripts/plan_pick_once.sh --network-rank 3
```

注意网络 rank 是抓取候选排序，不等同于“第几个物体”。

也可以按当前彩色图像像素锁定某个工件；例如截图中的银色螺栓：

```bash
./scripts/plan_pick_once.sh --target-pixel 790 260 --target-radius 65
```

目标圆只限制网络关注区域，几何工作区、桌面、碰撞和可达性检查仍全部执行。
对当前图片中的螺栓，也提供了固定的仅规划快捷命令：

```bash
./scripts/plan_pictured_bolt.sh
```

## 5. 解锁并执行一次 pick-hold

只有在以下项目全部通过后，才编辑本工程自己的文件：

```text
config/right_arm_green_table.yaml
execution.enabled: false → true
```

要求：

- 候选位于钳子、剪钳、螺栓或较厚圆环上；
- RViz 中 tool0/Robotiq 方向和候选方向一致；
- pre-grasp 保持 120 mm 安全距离；
- 笛卡尔下降不碰桌面或邻近工件；
- lift 垂直上升 140 mm；
- External Control 正在运行，急停可触达。

执行：

```bash
cd /home/gzu/gzu_ws/ur3_graspnet6dof_20260824
./scripts/execute_pick_hold.sh
```

当前图片中螺栓的等价快捷命令是：

```bash
./scripts/execute_pictured_bolt_hold.sh
```

它同样要求先把 `execution.enabled` 改为 `true`，并在终端键入 `EXECUTE`。

必须输入 `EXECUTE`。执行顺序：

```text
重新推理并锁定候选
→ 再次完整规划
→ 打开夹爪
→ 低速到 pre-grasp
→ 笛卡尔下降
→ 闭合夹爪
→ 检查 Robotiq gOBJ=2
→ 垂直抬升
→ 停止并保持工件
```

如果夹爪未检测到物体，节点会重新打开夹爪并报错，不继续抬升。

## 6. 放置与连续抓取

`pick_drop` 已实现，但默认锁定：

```yaml
execution:
  drop_enabled: false
```

需要先在空箱/接料区示教一个 `base` 下的 TCP 位姿，填写 `drop_pose`，完成 plan-only 后再设置 `drop_enabled: true`。在没有确认接料区之前，不应让程序根据截图猜放置位置。

每次放置后都必须重新调用推理；不能复用上一次候选，因为工件与点云已经变化。

## 针对截图工件的建议顺序

1. 黄色尖嘴钳：厚度和可夹区域较清楚，优先。
2. 蓝色剪钳：夹中部手柄，避开铰链和尖端。
3. 银色螺栓：抓圆柱中部，检查宽度预测。
4. 较厚圆环：仅选择跨两侧圆环壁的候选。
5. 螺母、薄环、小黑件：最后测试；深度空洞或预测宽度过小就应拒绝。

## 关键参数

所有参数仅在 [`config/right_arm_green_table.yaml`](config/right_arm_green_table.yaml) 中：

- `camera.roi_normalized`：限制到截图中央取料区，排除右侧机器及已观察到的伪深度；新工件应摆入该区域；
- `selector.workspace_bounds`：MoveIt `base` 下的安全抓取范围；
- `selector.support_plane_z=-0.010`：当前 RGB-D/TF 观测的绿色台面中心值；
- `safety_scene`：独立的观测桌面安全薄板，不覆盖旧碰撞文件；
- `selector.max_approach_tilt_deg=30`：第一版近似竖直抓取；
- `selector.max_gripper_width=0.080`：为 2F-85 留机械余量；
- `tool.tcp_offset_from_tool0=0.130`：现有实机 TCP 偏移；
- `moveit.velocity_scaling=0.05`：真机低速；
- `moveit.max_pose_duration=90`：配合 5% 速度，允许从当前高位姿缓慢到达预抓取；
- `execution.enabled`：真机总锁。
