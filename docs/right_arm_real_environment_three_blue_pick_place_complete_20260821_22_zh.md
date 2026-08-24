# 右臂真实环境、绿布桌面与三蓝色物块抓放完整记录

记录日期：2026-08-21 至 2026-08-22  
工作空间：`/home/gzu/gzu_ws`  
系统：ROS Noetic、MoveIt、双 UR3、右臂 Robotiq 2F、固定 RealSense D435I  
任务：在真实工作环境中识别三个蓝色物块，使用右臂逐个抓取，抬升后放到棋盘格区域，并返回初始位。

本文汇总并更新以下旧记录：

- `docs/right_arm_hsv_real_environment_moveit_workflow_20260821_zh.md`
- `docs/right_arm_real_environment_measurement_sheet_20260821_zh.md`
- `docs/right_arm_eye_on_base_hsv_cube_pick_complete_20260820_21_zh.md`

本文是 2026-08-22 当前工作台和程序的主复刻文档。旧文档仍保留原始测量、推导和单魔方实验过程，但今天再次运行时应优先以本文为准。

## 1. 两天工作最终完成了什么

完整链路已经在右臂实机上跑通：

```text
真实尺寸测量
-> MoveIt PlanningScene 建模
-> RealSense RGB-D 与安全手眼 TF
-> HSV 蓝色物块检测 + 8x6 棋盘定位
-> 多帧稳定锁点
-> 只规划与 RViz 检查
-> 操作者开启 External Control 并确认
-> 夹爪复位、激活、完全张开确认
-> 预抓取、下降、夹持检测、抬升
-> 移动到棋盘格上方、安全高度松开
-> 抬起并返回初始关节位置
```

2026-08-21 完成了单个 55 mm 魔方的视觉抓取、夹持检测、平移、释放和回位。2026-08-22 更换桌面并扩展机柜模型后，完成了三个蓝色物块的顺序抓放：先完成一个物块，再对重新摆放的两个剩余物块执行完整的 13 段轨迹。

最终两个剩余物块的实机结果：

| 项目 | 结果 |
|---|---|
| 规划物块数 | 2 |
| 运动段数 | 13 |
| 物块 1 夹持反馈 | `gOBJ=2, gPO=209` |
| 物块 2 夹持反馈 | `gOBJ=2, gPO=84` |
| 放置策略 | 物块底部距棋盘表面约 50 mm 时松开 |
| 执行结果 | `REAL EXECUTION complete: 2 block(s), 13 motion segments` |
| 最终夹爪 | `gACT=1, gSTA=3, gOBJ=3, gFLT=0, gPO=3` |
| 最终机器人模式 | `mode=7`，RUNNING |
| 最终安全模式 | `mode=1`，NORMAL |
| 回初始位最大关节误差 | 约 `0.00018 rad` |

三个物块均已完成抓放。第二阶段执行完成后 Dashboard 显示 External Control 程序不再运行，可以保持关闭。

## 2. 坐标约定与环境模型变化

### 2.1 `base` 坐标约定

- 原点为双臂中央支架的 URDF `base` 基准；
- `+Y` 从右臂侧指向左臂侧；
- 俯视时，`+X` 是从 `+Y` 顺时针旋转 90° 的方向；
- `+Z` 竖直向上；
- 所有 YAML 长度为米，角度为弧度；
- `box.pose` 表示几何中心，不是顶面。

### 2.2 2026-08-21 原始环境

昨天的主机柜边界为：

```text
x = [-0.8275, +0.1525] m
y = [-0.2330, +1.0270] m
z = [-0.8050, -0.0100] m
```

因此原机柜模型为：

```yaml
pose: [-0.3375, 0.3970, -0.4075, 0, 0, 0]
size: [0.9800, 1.2600, 0.7950]
```

昨天使用的是木质工作板。单魔方抓取成功记录中的木板模型为：

```yaml
pose: [0.600749, -0.227145, 0.209227, 0, 0, 0]
size: [0.2820, 0.4500, 0.0180]
```

### 2.3 2026-08-22 机柜向右延长 1.5 m

按照现场照片中“机械臂右侧”的方向，机柜沿 `base -Y` 延长 `1.500 m`。原来的 `+Y`、X、Z 边界不变：

```text
旧 y = [-0.2330, +1.0270] m
新 y = [-1.7330, +1.0270] m
```

新机柜模型为：

```yaml
pose: [-0.3375, -0.3530, -0.4075, 0, 0, 0]
size: [0.9800, 2.7600, 0.7950]
```

### 2.4 2026-08-22 绿布工作台

新桌面实测和现场约定：

```text
长：1200 mm
宽：900 mm
桌面比机柜顶面低：45 mm
机柜顶面：z=-0.010 m
绿布支撑表面：z=-0.055 m
宽度两边相对原点的垂直距离：100 mm 和 800 mm
```

绿板经过方向修正后，长边沿右臂方向 `-Y` 延伸，最终边界为：

```text
x = [-0.1000, +0.8000] m
y = [-1.2000, 0.0000] m
top z = -0.0550 m
```

MoveIt 只需要一个向下延伸的薄碰撞层表示承载表面，因此使用 20 mm 厚代理 box：

```yaml
pose: [0.3500, -0.6000, -0.0650, 0, 0, 0]
size: [0.9000, 1.2000, 0.0200]
```

20 mm 是 PlanningScene 表面代理厚度，不表示真实桌板厚度；代理体顶面仍精确位于 `z=-0.055 m`。

正式配置文件：

```text
src/dual_arm_tasks/config/right_arm_real_environment_20260822.yaml
```

`right_arm_real_environment.launch` 已默认加载这份 8 月 22 日配置。它沿用 `scene_id: right_arm_lab_20260821`，这样重新发布时会先替换旧机柜和旧木桌，不会在 RViz 中叠加两套碰撞体。

### 2.5 尚未建模的安全边界

以下对象仍因缺少精确测量而保持 `enabled: false`：

- 新桌面的腿、框架和下部结构；
- 固定相机立柱、底座和横梁；
- 线缆和气管的摆动包络。

因此当前场景只允许规划桌面以上的动作，不应据此规划钻到桌面下方的路径。

## 3. 视觉与棋盘格定位

### 3.1 相机与 TF

本次使用的 RealSense D435I 序列号为 `912112073732`，彩色与对齐深度均为约 30 Hz。

安全手眼入口：

```text
src/dual_arm_tasks/launch/right_arm_hsv_eye_on_base_runtime.launch
```

它完成两件事：

1. 发布经过验证的固定相机手眼 TF；
2. 将 HSV 相机坐标点转换到右臂基座坐标。

只能保留一个 `right_arm_base -> camera*` 外参发布器，不能同时启动其他历史手眼 launch。

### 3.2 三蓝色物块与棋盘感知

主节点：

```text
src/dual_arm_tasks/scripts/blue_blocks_checkerboard_perception.py
```

配置：

```text
src/dual_arm_tasks/config/blue_blocks_checkerboard_perception.yaml
```

主要处理流程：

```text
RGB 转 HSV
-> H[90,108] S[128,255] V[135,255] 提取蓝色
-> 形态学闭运算和膨胀
-> 面积、宽高、填充率过滤
-> 在轮廓内部读取对齐深度
-> 像素反投影到 camera_color_optical_frame
-> TF 转换到机器人坐标

8x6 内角点棋盘检测
-> solvePnP 求棋盘姿态
-> 计算棋盘多边形并从蓝色掩膜排除
-> 根据棋盘坐标偏移生成三个放置点
```

关键话题：

| 话题 | 含义 |
|---|---|
| `/hsv_grasp/blue_object_points_camera` | 相机坐标中的蓝色物块顶面点 |
| `/hsv_grasp/blue_object_points_base` | 转换到目标机器人坐标的物块点 |
| `/hsv_grasp/checkerboard_pose_camera` | 棋盘在相机中的姿态 |
| `/hsv_grasp/checkerboard_place_points_base` | 棋盘内三个候选放置表面点 |
| `/hsv_grasp/three_block_scene_ready` | 三物块、棋盘、TF、尺寸是否全部就绪 |
| `/hsv_grasp/three_block_scene_status` | 当前拒绝原因或高度诊断 |
| `/hsv_grasp/blue_checkerboard_debug` | 带轮廓、角点和文字的调试图 |

棋盘是 `8 x 6` 个内角点。当前单格尺寸 `0.0227 m` 来自 RGB-D 几何估计，尚未用尺子最终确认，因此配置中仍是：

```yaml
square_size_confirmed: false
```

实机执行时必须显式接受这个临时尺寸；更正确的后续做法是用尺测量后写入真实值并改成 `true`。

### 3.3 棋盘被已放物块遮挡后的处理

第一个物块放入棋盘后遮挡了部分角点，完整棋盘节点开始报告：

```text
8x6 checkerboard not detected in configured ROI
```

这不是剩余物块消失，而是棋盘姿态无法重新求解。处理方式是：

1. 棋盘未遮挡时先锁定棋盘放置点；
2. 人工重新摆放剩余两个物块；
3. 使用 HSV、对齐深度和已有 TF 对剩余物块做 30 帧只读复核；
4. 只比较仍需抓取的物块，不要求已经放置的物块回到原位置；
5. 复核通过后才允许使用已锁定的棋盘点。

本次最终复核结果：

```text
object1 live=[0.393752, -0.284595, 0.031154]
shift=6.10 mm, MAD=[0.82, 0.40, 4.00] mm

object2 live=[0.496060, -0.387070, 0.032080]
shift=4.07 mm, MAD=[1.64, 0.55, 0.99] mm
```

两者均小于 `12 mm` 位移阈值，稳定性也通过。

这套只读复核现已保存为：

```text
src/dual_arm_tasks/scripts/validate_locked_blue_targets.py
```

它只订阅 RGB、深度和 TF，不发布机器人或夹爪命令。

## 4. 抓取与放置几何

### 4.1 顶面点、物块中心与抓取点

视觉输出是物块顶面点，不是中心。程序先用棋盘平面估计物块高度 `h`，然后计算：

```text
object_center_z = object_top_z - h/2
grasp_z = object_center_z + 0.020
```

即夹爪抓在物块中心上方 20 mm，避免过低碰桌，同时保持足够夹持面积。

### 4.2 TCP 与 `tool0`

配置使用：

```yaml
pose_target_is_tcp: true
tcp_offset_from_end_effector: [0.0, 0.0, 0.130]
```

任务中的 XYZ 表示夹爪 TCP。程序会根据当前四元数把 130 mm TCP 偏移换算成 MoveIt 的 `right_arm_tool0` 目标，不能直接把 TCP 坐标当成 `tool0` 坐标。

### 4.3 抓取方向

低桌面抓取继续使用 8 月 21 日验证成功的 40° 倾斜姿态：

```text
xyzw = [-0.0189564460, 0.9436495767, -0.1063103131, 0.3128326179]
```

它来自实际可达 IK/FK 候选，并避免两根手指一高一低压到物块顶面。

### 4.4 放置方向与安全松开高度

放置姿态不是固定四元数，而是根据放置点相对 `base` 原点的径向方向，动态生成向外倾斜 40° 的姿态：

```yaml
place_orientation_mode: outward_tilt
place_outward_tilt_deg: 40.0
place_roll_deg: 0.0
```

第一轮放置曾让夹爪接近并碰到桌面。随后将：

```yaml
release_clearance: 0.050
```

放置 TCP 高度的公式为：

```text
release_z = board_surface_z + h/2 + grasp_offset + release_clearance
```

由于抓住物块后，物块底面相对 TCP 的距离是 `h/2 + grasp_offset`，所以上式保证松爪瞬间物块底面距棋盘约 50 mm。物块会从较低安全高度落到棋盘，而夹爪不再扎入桌面。

### 4.5 每个物块的动作序列

```text
全局规划到 pre_grasp
-> 笛卡尔垂直下降到 grasp
-> 闭合夹爪
-> 必须检测 gOBJ=2
-> 笛卡尔抬升到 lift
-> 全局规划到 pre_place
-> 笛卡尔下降到 release
-> 张开夹爪
-> 笛卡尔抬升到 retreat
```

所有物块完成后再全局规划回初始关节位。

两个物块共记录 13 段轨迹：每个物块 6 段机械臂轨迹，加最后 1 段回初始位。夹爪开合命令不计入 MoveIt 轨迹段数。

## 5. 今天新增或修改的有用代码

### 5.1 `publish_real_environment_scene.py`

路径：

```text
src/dual_arm_tasks/scripts/publish_real_environment_scene.py
```

用途：把 YAML 中的真实机柜、桌面等几何写入 MoveIt PlanningScene。它只管理具有指定 `scene_id__` 前缀的物体，不会清除其他节点添加的场景对象，也不会发送机械臂轨迹。

关键能力：

- 支持 box 和 cylinder；
- 校验尺寸、姿态、颜色和有限数；
- `require_measured=true` 时拒绝任何启用但未确认测量的对象；
- `dry_run=true` 只做语法检查；
- `remove_only=true` 只删除本场景对象；
- 等待 MoveIt 确认对象确实加入或删除；
- 通过 `/apply_planning_scene` 设置 RViz 颜色。

### 5.2 `right_arm_real_environment_20260822.yaml`

用途：保存今天的正式碰撞几何。主要包含：

- 沿 `-Y` 延长 1.5 m 的蓝色机柜；
- 1200 x 900 mm 的绿色工作表面；
- 尚未测量、默认禁用的桌下结构、相机架和线缆占位项。

### 5.3 `checkerboard_pose_publisher.py`

用途：通用棋盘姿态发布器。它检测棋盘角点，用 `solvePnP` 计算姿态，发布 Pose 和 TF，并支持 ROI、SB 角点检测、中心原点和 180° 解连续性选择。它是棋盘姿态调试工具，不会控制机器人。

### 5.4 `blue_blocks_checkerboard_perception.py`

用途：将蓝色物块检测和棋盘放置点生成整合到同一个只读节点。它同步使用彩色、深度、相机内参和 TF，发布多物块 PoseArray、棋盘姿态、放置点和调试图。

代码中特别处理了：

- 棋盘对称导致角点顺序偶发翻转 180°；
- 棋盘区域中的蓝色图案不能被误认为待抓物块；
- 深度轮廓边缘混入背景，因此先腐蚀再取中位数；
- 物块数量、高度、TF、棋盘尺寸共同决定 ready 状态。

### 5.5 `right_arm_three_blue_pick_place.py`

用途：右臂多物块抓取、放置和回位的主任务脚本。默认不带 `--execute` 时只规划和发布 RViz 轨迹。

主要组成：

- `TargetLock`：采集物块和放置点 PoseArray；
- `robust_lock()`：使用中位数、半径内点比例和 MAD 排除跳点；
- `geometry_from_points()`：由三个棋盘点拟合平面并估计物块高度；
- `write_targets()/load_targets()`：把本次移动前坐标锁入 YAML；
- `validate_live()`：执行前检查目标相对锁定值是否移动；
- `initialize_right_gripper()`：执行 reset、activate、open，并等待真实完成反馈；
- `plan_or_execute()`：按预抓取、抓取、抬升、放置、释放、撤离和回位顺序运行。

安全开关含义：

| 参数 | 含义 |
|---|---|
| 无 `--execute` | 只规划，不向机械臂和真实夹爪发送动作 |
| `--reuse-targets` | 只规划时复用已经锁定的 YAML |
| `--execute` | 允许进入实机执行分支 |
| `--accept-provisional-board` | 明确接受尚未尺量确认的 22.7 mm 棋盘格 |
| `--external-scene-validated` | 棋盘被遮挡时，声明已经用独立工具复核活动目标 |
| `--yes` | 跳过终端中输入 `EXECUTE`；只应在外部已经取得明确确认时使用 |

### 5.6 夹爪初始化改进

旧逻辑只看到 `gACT=1, gSTA=3` 就认为夹爪准备好，但当时可能仍是：

```text
gOBJ=0, gPO=152
```

这表示夹爪仍在运动。新逻辑必须同时满足：

```text
gFLT == 0
gACT == 1
gSTA == 3
gOBJ == 3
abs(gPO - open_position) <= 5
```

也就是无故障、激活完成、夹爪已停止并真正到达张开位置，机械臂才开始运动。

### 5.7 `right_arm_visual_pick.py` 的 Dashboard 双重联锁

驱动话题 `/right_arm/ur_hardware_interface/robot_program_running` 在重连或暂停后可能短时间残留 `True`。今天实际看到：

```text
ROS topic: True
Dashboard: Program running=false, state=PAUSED
```

因此 `ensure_external_control()` 现在同时检查：

1. ROS `robot_program_running` 话题；
2. `/right_arm/ur_hardware_interface/dashboard/program_running` 实时服务。

只有两者都确认运行才允许实机轨迹。回初始位前还会再次调用该联锁。

### 5.8 `activate_right_gripper.py`

用途：独立执行 Robotiq 的标准初始化序列：

```text
重复发布 rACT=0 复位
-> 等待
-> 重复发布 rACT=1, rPR=0 激活并完全张开
```

它适合单独调试夹爪；正式多物块任务已经在内部执行同样的初始化和反馈确认。

### 5.9 `validate_locked_blue_targets.py`

用途：棋盘已被物块遮挡时，只读复核锁定的蓝色源物块是否移动。它：

- 用 HSV 找蓝色候选；
- 用对齐深度反投影三维点；
- 转换到目标 YAML 的 `frame_id`；
- 把候选与锁定点做一一最小距离关联；
- 采集默认 30 帧；
- 检查中位位置偏移和 MAD；
- 超过阈值返回非零退出码；
- 不发布机械臂、控制器或夹爪命令。

### 5.10 成功目标归档

今天原来放在 `/tmp` 的目标已永久归档：

```text
calibration/environment/right_arm_three_blue_locked_targets_20260822_initial.yaml
calibration/environment/right_arm_remaining_two_locked_targets_20260822_success.yaml
```

这些文件只是实验记录。只要物块、棋盘、相机或桌面移动，就必须重新锁定，禁止直接复用归档值执行。

## 6. 推荐的完整终端启动顺序

### 6.1 每个终端的公共准备

每个新终端先执行：

```bash
cd /home/gzu/gzu_ws
conda deactivate 2>/dev/null || true
source /opt/ros/noetic/setup.bash
source /home/gzu/gzu_ws/devel/setup.bash
```

启动准备和只规划阶段保持右臂示教器的 External Control 为暂停/关闭。只有最终规划成功、目标未移动、工作区清空后才打开。

### 终端 1：双臂 UR 驱动

```bash
roslaunch ur_robot_driver ur3_dual_bringup.launch
```

用途：连接两台 UR3、发布关节状态/机器人模式/安全模式、启动 ros_control 控制器和 Dashboard 服务。

当前地址：

```text
右臂：192.168.1.44
左臂：192.168.1.43
电脑有线网卡：192.168.1.41/24
```

右臂检查：

```bash
ping -c 3 192.168.1.44
rostopic hz /right_arm/joint_states
rosservice call /right_arm/controller_manager/list_controllers
```

正常值约为 125 Hz，并且以下控制器都是 `running`：

```text
joint_state_controller
scaled_pos_joint_traj_controller
```

### 终端 2：MoveIt

```bash
roslaunch ur3_dual_moveit_config move_group.launch \
  allow_trajectory_execution:=true \
  moveit_controller_manager:=simple_right
```

用途：加载 URDF/SRDF、TRAC-IK、OMPL、PlanningScene 和右臂轨迹控制器接口。

虽然这里允许轨迹执行，但任务脚本默认仍是 plan-only；没有 `--execute` 不会运动。准备期间 External Control 也保持关闭。

### 终端 3：RViz

```bash
roslaunch ur3_dual_moveit_config moveit_rviz.launch \
  rviz_config:=/home/gzu/gzu_ws/src/ur3_dual_moveit_config/launch/moveit.rviz
```

用途：查看当前关节姿态、绿色桌面、蓝色机柜、规划轨迹和碰撞结果。

重点显示项：

```text
MotionPlanning -> Scene Geometry
MotionPlanning -> Planned Path
RobotModel
TF
```

### 终端 4：RealSense RGB-D

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

用途：发布彩色图、深度图、对齐深度和相机内参。

检查：

```bash
rostopic hz /camera/color/image_raw
rostopic hz /camera/aligned_depth_to_color/image_raw
```

两者应接近 30 Hz。

### 终端 5：安全手眼 TF

```bash
roslaunch dual_arm_tasks right_arm_hsv_eye_on_base_runtime.launch
```

用途：发布固定相机的安全外参并提供相机点到机器人坐标的 TF 链。

检查：

```bash
rosrun tf tf_echo base camera_color_optical_frame
```

### 终端 6：三物块和棋盘感知

```bash
roslaunch dual_arm_tasks blue_blocks_checkerboard_perception.launch \
  show_window:=true
```

用途：识别三个蓝色物块、棋盘姿态和棋盘内放置点。它是纯感知节点，不会运动机器人。

检查：

```bash
rostopic echo -n 1 /hsv_grasp/three_block_scene_status
rostopic echo -n 1 /hsv_grasp/blue_object_points_base
rostopic echo -n 1 /hsv_grasp/checkerboard_place_points_base
```

### 终端 7：右夹爪 RTU 驱动

```bash
export PYTHONPATH="/home/gzu/gzu_ws/src/robotiq/robotiq_modbus_rtu/src:$PYTHONPATH"

rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode_right.py \
  /dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DA6ACQ6P-if00-port0
```

用途：通过 USB-RS485 与真实 Robotiq 通信，订阅命令并发布反馈。

必须使用稳定的 `/dev/serial/by-id/...`，不要依赖可能变化的 `/dev/ttyUSB0`。

检查：

```bash
rostopic echo -n 1 /right_arm/Robotiq2FGripperRobotInput
```

### 终端 8：发布真实碰撞环境

必须在终端 2 的 MoveIt 启动后运行：

```bash
roslaunch dual_arm_tasks right_arm_real_environment.launch
```

用途：把机柜和绿布桌面加入 PlanningScene。节点成功后会显示：

```text
Applied environment scene 'right_arm_lab_20260821' with 2 collision objects
```

这是一次性发布节点，成功后自动退出是正常现象。

验证场景对象：

```bash
rosservice call /get_planning_scene \
  '{components: {components: 536}}' | \
  grep 'right_arm_lab_20260821__'
```

应包含：

```text
right_arm_lab_20260821__support_cabinet
right_arm_lab_20260821__green_cloth_work_surface
```

### 终端 9：状态监控

```bash
rostopic echo /right_arm/ur_hardware_interface/safety_mode
```

需要时另开终端检查：

```bash
rosservice call /right_arm/ur_hardware_interface/dashboard/program_running
rosservice call /right_arm/ur_hardware_interface/dashboard/program_state
rostopic echo -n 1 /right_arm/ur_hardware_interface/robot_mode
rostopic echo -n 1 /right_arm/Robotiq2FGripperRobotInput
```

## 7. 从新场景重新锁点和只规划

### 7.1 场景未遮挡时生成新的目标文件

摆好三个蓝色物块并保持棋盘完整可见。不要复用日期归档。运行：

```bash
rosrun dual_arm_tasks right_arm_three_blue_pick_place.py \
  --config /home/gzu/gzu_ws/src/dual_arm_tasks/config/right_arm_three_blue_pick_place_20260822.yaml \
  --targets-file /home/gzu/gzu_ws/calibration/environment/right_arm_three_blue_locked_targets_current.yaml
```

不带 `--execute`，所以只会：

1. 收集物块和棋盘点；
2. 做中位数、内点和 MAD 稳定性检查；
3. 写入新的目标 YAML；
4. 规划并在 RViz 显示；
5. 不执行实机运动。

当前正式配置的 `max_blocks: 2` 是今天“首个物块已经完成后，处理剩余两个”的状态。如果从三个物块重新开始，应先根据可达性逐个规划；不要直接把 `max_blocks` 改成 3 后盲目连续执行。

### 7.2 复用目标做纯规划

仅当物块和棋盘从锁点后没有移动时：

```bash
rosrun dual_arm_tasks right_arm_three_blue_pick_place.py \
  --config /home/gzu/gzu_ws/src/dual_arm_tasks/config/right_arm_three_blue_pick_place_20260822.yaml \
  --reuse-targets \
  --targets-file /home/gzu/gzu_ws/calibration/environment/right_arm_three_blue_locked_targets_current.yaml
```

看到：

```text
PLAN ONLY complete: N block(s), M motion segments
```

只代表几何和规划成功，仍不等于允许实机执行。

### 7.3 棋盘遮挡时复核剩余目标

若棋盘放置点已经锁定，但已放物块遮挡棋盘，可运行：

```bash
rosrun dual_arm_tasks validate_locked_blue_targets.py \
  --config /home/gzu/gzu_ws/src/dual_arm_tasks/config/blue_blocks_checkerboard_perception.yaml \
  --targets-file /home/gzu/gzu_ws/calibration/environment/right_arm_remaining_two_locked_targets_current.yaml \
  --samples 30 \
  --max-shift 0.012 \
  --max-mad 0.006
```

必须看到：

```text
LOCKED BLUE TARGET VALIDATION PASSED
```

否则不能传 `--external-scene-validated`，应重新摆正棋盘、清除遮挡并重新锁点。

## 8. 最终实机执行流程

### 8.1 规划成功后才开启 External Control

在右臂示教器启动 `right_con.urp` / External Control，然后检查：

```bash
rosservice call /right_arm/ur_hardware_interface/dashboard/program_running
rosservice call /right_arm/ur_hardware_interface/dashboard/program_state
rostopic echo -n 1 /right_arm/ur_hardware_interface/robot_mode
rostopic echo -n 1 /right_arm/ur_hardware_interface/safety_mode
```

必须同时得到：

```text
program_running: True
state: PLAYING
robot mode: 7
safety mode: 1
```

不要只看 `robot_program_running` 话题，因为重连后可能残留旧的 True。

### 8.2 推荐的人工确认执行命令

正常场景、完整棋盘仍可检测时，不使用外部复核绕过项：

```bash
rosrun dual_arm_tasks right_arm_three_blue_pick_place.py \
  --config /home/gzu/gzu_ws/src/dual_arm_tasks/config/right_arm_three_blue_pick_place_20260822.yaml \
  --execute \
  --accept-provisional-board \
  --targets-file /home/gzu/gzu_ws/calibration/environment/right_arm_three_blue_locked_targets_current.yaml
```

程序会要求输入：

```text
EXECUTE
```

棋盘被遮挡、且第 7.3 节独立复核刚刚通过时，才增加：

```bash
--external-scene-validated
```

自动化协作场景中，只有操作者已经明确回复“确认执行”后才可增加 `--yes` 跳过重复输入。`--yes` 本身不是安全检查的替代品。

### 8.3 执行中自动停止条件

以下任一情况会中止流程：

- External Control 话题或 Dashboard 不确认运行；
- 没有夹爪驱动订阅命令；
- 夹爪初始化未完成或 `gFLT != 0`；
- 目标文件 frame 不匹配；
- 棋盘尺寸未确认且没有显式接受；
- 实时物块/棋盘相对锁定值移动超阈值；
- MoveIt 无法规划某段轨迹；
- 笛卡尔路径比例低于 `0.995`；
- 闭合后没有得到 `gOBJ=2`；
- 控制器执行返回失败或 PREEMPTED。

## 9. 右夹爪独立操作

### 9.1 复位、激活并打开

```bash
conda deactivate 2>/dev/null || true
source /home/gzu/gzu_ws/devel/setup.bash

rosrun dual_arm_tasks activate_right_gripper.py \
  _command_topic:=/right_arm/Robotiq2FGripperRobotOutput \
  _open_position:=0 \
  _speed:=120 \
  _force:=80
```

### 9.2 手动关闭

```bash
rostopic pub -1 /right_arm/Robotiq2FGripperRobotOutput \
  robotiq_2f_gripper_control/Robotiq2FGripper_robot_output \
  "{rACT: 1, rGTO: 1, rATR: 0, rPR: 255, rSP: 120, rFR: 80}"
```

### 9.3 手动打开

```bash
rostopic pub -1 /right_arm/Robotiq2FGripperRobotOutput \
  robotiq_2f_gripper_control/Robotiq2FGripper_robot_output \
  "{rACT: 1, rGTO: 1, rATR: 0, rPR: 0, rSP: 120, rFR: 80}"
```

Robotiq 关键反馈：

```text
gOBJ=0：手指仍在运动
gOBJ=2：闭合途中被物体挡住，本任务判定抓取成功
gOBJ=3：到达请求位置；打开时表示已完全打开，闭合时通常表示未抓到物体
gFLT=0：无故障
```

## 10. 今天遇到的问题、证据与修复

### 10.1 轨迹一提交就 PREEMPTED

两次失败均表现为：

```text
pre_grasp 规划成功
-> 执行请求发出
-> 约 0.07 s 后 PREEMPTED
-> 机械臂未运动，夹爪未闭合
```

根因不是 IK 或 OMPL，而是电脑到机器人网络中断。当时证据为：

```text
enp3s0: NO-CARRIER
右臂 192.168.1.44 ping 100% 丢包
/right_arm/joint_states 没有新消息
Dashboard 请求超时
驱动持续 Could not get fresh data package from robot
```

网线恢复后：

```text
enp3s0 = UP, 192.168.1.41/24
ping 延迟约 0.12~0.16 ms, 0% 丢包
joint_states 约 125 Hz
robot mode=7
safety mode=1
```

以后遇到立即 PREEMPTED，应先检查通信，不要连续重试实机轨迹。

### 10.2 驱动恢复后右臂控制器为空

驱动在断网期间启动较慢，右臂 controller spawner 已提前退出，表现为：

```text
rosservice call /right_arm/controller_manager/list_controllers
controller: []
```

正常优先重启整个驱动启动链。若驱动节点已稳定、只缺右臂控制器，可恢复：

```bash
ROS_NAMESPACE=/right_arm \
rosrun controller_manager spawner \
  joint_state_controller scaled_pos_joint_traj_controller
```

确认两个控制器均为 `running` 后才能规划或执行。

### 10.3 ROS master 重启后旧进程仍在但节点消失

网断恢复时启动了新的 roscore。旧 MoveIt、RViz、相机、感知和夹爪进程虽然还能在 `ps` 中看到，但没有在新 master 中注册，`rosnode list` 里不存在。

处理原则：

1. 以 `rosnode list` 和话题数据为准，不以“窗口还开着”或 `ps` 为准；
2. 停止旧的孤立进程；
3. 按本文终端 1 到 8 的顺序重新启动；
4. 重新发布 PlanningScene；
5. 重新规划，不复用上一个 MoveIt 实例中的轨迹。

### 10.4 场景发布等待 `/get_planning_scene` 超时

原因是 MoveIt 尚未启动或已连接到旧 ROS master。正确顺序是先启动终端 2 的 `move_group`，看到：

```text
You can start planning now!
```

再运行场景发布终端。

### 10.5 场景是否真的存在

查询 PlanningScene 时不能只请求组件值 1；世界物体几何和颜色需要相应位掩码。本次使用：

```bash
rosservice call /get_planning_scene '{components: {components: 536}}'
```

并确认两个对象 ID 存在。

### 10.6 External Control 假阳性

用户暂停示教器程序后，ROS Bool 话题仍曾显示 True，但 Dashboard 明确为：

```text
Program running: false
PAUSED right_con.urp
```

这就是增加 Dashboard 双联锁的原因。以后人工检查也应优先使用 Dashboard 服务。

### 10.7 夹爪放置碰桌

第一次放置目标太贴近棋盘表面，夹爪触碰桌面。修复是把 `release_clearance` 从近表面值提高到 `0.050 m`，让物块底部在 50 mm 高度松开，再抬起夹爪。

## 11. 关键配置值

```yaml
arm_group: right_arm
planning_frame: base
end_effector_link: right_arm_tool0

velocity_scaling: 0.05
acceleration_scaling: 0.05
planning_time: 10.0
num_planning_attempts: 12
planner_id: RRTConnect

pre_pick_clearance: 0.180
lift_distance: 0.140
pre_place_clearance: 0.150
release_clearance: 0.050

cartesian_step: 0.003
cartesian_min_fraction: 0.995

open_position: 0
close_position: 255
gripper_speed: 120
gripper_force: 80
```

右臂初始关节位，顺序为 shoulder_pan、shoulder_lift、elbow、wrist_1、wrist_2、wrist_3：

```text
[ 3.7318120003,
 -1.5940335433,
 -1.8345826308,
 -1.1906345526,
  0.1394038796,
  1.6522442102 ]
```

## 12. 今天成功实验的目标记录

初始三物块目标保存在：

```text
calibration/environment/right_arm_three_blue_locked_targets_20260822_initial.yaml
```

最终两个剩余物块成功执行使用：

```yaml
object_top_points:
  - [0.3926334, -0.2846511, 0.0251562]
  - [0.4925150, -0.3871454, 0.0300812]

place_surface_points:
  - [0.5058373524, -0.0705696754, -0.0378890979]
  - [0.5651517030, -0.0838539189, -0.0354504870]

object_heights:
  - 0.0452828331
  - 0.0385998608
```

对应归档：

```text
calibration/environment/right_arm_remaining_two_locked_targets_20260822_success.yaml
```

再次强调：这些坐标用于复盘，不是永久目标。现场物体移动后必须重新检测和规划。

## 13. 安全停机顺序

任务完成后：

1. 确认右臂已返回初始位且速度为 0；
2. 确认夹爪已打开、`gFLT=0`；
3. 在示教器暂停/关闭 External Control；
4. 停止感知和调试节点；
5. 关闭 RViz 和 MoveIt；
6. 最后停止 UR driver；
7. 不要在机械臂运动中直接关闭驱动或拔网线。

如果只结束抓取任务而仍需查看机器人状态，可以保留 UR driver、相机和 RViz，仅关闭 External Control。

## 14. 后续建议

1. 用尺确认棋盘单格真实尺寸，取消 `--accept-provisional-board`；
2. 测量并加入桌腿、桌架、相机支架和线缆禁入区；
3. 将抓住的物块建模为 `AttachedCollisionObject`；
4. 给每轮任务保存时间戳、目标点、轨迹、夹爪反馈和最终关节状态；
5. 将全三物块任务改成“每完成一个就重新识别剩余物块”，减少对被遮挡棋盘的依赖；
6. 为有线网卡载波、Dashboard、控制器、关节数据新鲜度和相机帧率制作统一 readiness 节点；
7. ROS 日志目录已超过 1 GB，可在确认不再需要旧日志后单独执行 `rosclean`，不要在实验中途清理。

## 15. 核心经验

- PlanningScene 必须描述真实障碍物，RViz 中看见模型不等于模型尺寸正确；
- 物块顶面点、物块中心、TCP 和 `tool0` 是不同的几何语义；
- 规划成功不代表实机通信、External Control 或夹爪已经准备好；
- `gOBJ=2` 才是本任务的有效夹持信号；
- 目标移动后必须重新锁点；
- 棋盘被遮挡时可以复用移动前的棋盘点，但剩余活动物块必须独立复核；
- External Control 必须以 Dashboard 实时状态为准，不能只相信可能残留的 Bool 话题；
- 立即 PREEMPTED 时先查网卡、关节话题和控制器，不要重复下发轨迹；
- 放置不需要让夹爪贴桌，保持可控高度松开更安全；
- 每次执行结束都应打开夹爪并回到经过验证的初始关节位。
