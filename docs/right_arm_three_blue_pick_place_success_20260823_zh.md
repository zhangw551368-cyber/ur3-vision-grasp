# 右臂三蓝色物块连续抓放成功记录（2026-08-23）

记录日期：2026-08-23  
工作空间：`/home/gzu/gzu_ws`  
平台：Ubuntu 20.04、ROS Noetic、MoveIt、双 UR3、右臂 Robotiq 2F、固定 RealSense RGB-D  
结果：右臂连续完成三个蓝色物块的抓取、高位释放和回初始位。

> 本文只记录 2026-08-23 最终成功的一次运行。物块、棋盘、相机、机械臂或手眼标定发生变化后，必须重新识别和重新规划，不能照搬本文坐标直接执行。

## 1. 最终结果

最终终端输出：

```text
REAL EXECUTION complete: 3 block(s), 16 motion segments
```

执行顺序为：

```text
物块 1：预抓取 -> 抓取 -> 抬升 -> 棋盘上方高位释放 -> 撤离
物块 2：预抓取 -> 抓取 -> 抬升 -> 棋盘上方高位释放 -> 撤离
魔方 3：预抓取 -> 抓取 -> 抬升 -> 棋盘上方高位释放 -> 撤离
返回 right_initial
```

每个物块 5 段机械臂轨迹，共 `5 × 3 + 1 = 16` 段。三个物块均抓取并释放成功，最后回到右臂初始关节位。

最终状态：

| 检查项 | 结果 |
|---|---|
| 右臂机器人模式 | `mode=7`，RUNNING |
| 安全模式 | `mode=1`，NORMAL |
| 夹爪激活 | `gACT=1` |
| 夹爪状态 | `gSTA=3`，已激活 |
| 夹爪故障 | `gFLT=0` |
| 最终夹爪位置 | `gPO=3`，接近完全张开 |
| 执行物块数 | 3 |
| 运动段数 | 16 |

## 2. 本次成功的关键原则

### 2.1 抓取前锁定一次，运动中不再更新

最终采用的规则是：

1. 机械臂尚未开始运动、相机视线完整时，从当前 RGB-D 画面识别三个物块和棋盘；
2. 对三个物块进行多帧稳定采样，保存相机当时看到的原始中心坐标；
3. 规划前进行一次只读复核；
4. 操作者确认后开始执行；
5. 一旦机械臂开始运动，三个抓取坐标全部冻结，不再受机械臂遮挡相机的画面影响。

这样避免了右臂经过相机视野时，HSV 检测短暂丢失或把机械臂误认为物块，导致第二、第三个目标改变或任务停住。

本次所有经验性像素补偿均为零：

```yaml
object_center_offset_u_px: 0.0
object_center_offset_v_px: 0.0
object_center_offsets_px:
  - [0.0, 0.0]
  - [0.0, 0.0]
  - [0.0, 0.0]
```

也就是说，最终成功运行直接使用最新相机画面识别出的物块中心，不再对长方体向上、下、左或右做人为偏移。

### 2.2 只在棋盘上方高位松开

放置阶段不让夹爪下降接触棋盘或桌面。TCP 到棋盘表面的释放高度保持至少 `120 mm`，到达棋盘上方后直接张开夹爪，让物块落在棋盘格区域内，然后立即抬离。

正式配置：

```yaml
direct_high_release: true
release_tcp_clearance: 0.120
```

这消除了早期试验中夹爪向桌面“扎入”、手指碰桌或放置姿态约束过紧的问题。

### 2.3 两个长方体与魔方采用不同抓取姿态

- 物块 1、物块 2：优先俯视抓取，末端绕竖直轴旋转，使二指夹爪与视觉检测出的长边方向匹配；
- 魔方 3：采用 2026-08-21 实机成功过的固定约 40° 斜抓姿态；
- 放置阶段不要求竖直放置，使用向外倾斜的高位释放姿态，优先保证可达性和避碰。

## 3. 本次锁定坐标

正式锁点文件：

```text
/home/gzu/gzu_ws/calibration/environment/right_arm_three_blue_locked_targets_20260823.yaml
```

内容为：

```yaml
frame_id: base
object_top_points:
  - [0.4091723094325027, -0.3971029762972191, 0.019881432203895988]
  - [0.43904273477852884, -0.2743140328734306, 0.010271868659019523]
  - [0.5399850979839397, -0.3392816789501417, 0.032306983995222194]
place_surface_points:
  - [0.561669575539361, -0.1523808123467356, -0.02991661439618143]
  - [0.5588986333563, -0.033024015342917656, -0.04181620446702483]
  - [0.5802653248388231, -0.09215569868652279, -0.03508085772088526]
object_heights:
  - 0.03204581013716584
  - 0.03330321636895876
  - 0.044683419535744326
object_long_axis_yaws:
  - 0.026471281959838635
  - 0.02647128346181482
  - -1.4582348920005428
square_size_m: 0.0227
square_size_confirmed: false
created_at: 2026-08-23T11:47:16+0800
```

这里的 `object_top_points` 是物块顶面点，不是物块几何中心。抓取 TCP 高度按下式计算：

```text
grasp_z = object_top_z - object_height / 2 + 0.020 m
```

其中 `0.020 m` 是经过实机验证的夹爪中心相对物块中心的竖直偏置。

### 3.1 执行前 30 帧只读复核

执行前重新采集 30 帧，但只用于比较，不修改锁点文件：

```text
object1 live=[0.409547, -0.397263, 0.018884]
shift_mm=1.08  MAD_mm=[1.10, 0.62, 1.00]

object2 live=[0.438226, -0.274297, 0.010269]
shift_mm=0.82  MAD_mm=[0.88, 0.19, 1.00]

object3 live=[0.540029, -0.339470, 0.032307]
shift_mm=0.19  MAD_mm=[0.76, 0.60, 1.00]

LOCKED BLUE TARGET VALIDATION PASSED: samples=30
```

三者位移均明显小于 `12 mm` 阈值，说明锁定后物块没有被移动，且视觉采样稳定。

## 4. 最终规划目标与姿态

### 4.1 位置目标

| 物块 | 抓取 TCP `[x,y,z]` m | 预抓取 z | 抬升 z | 高位释放 TCP `[x,y,z]` m |
|---|---|---:|---:|---|
| 长方体 1 | `[0.40917,-0.39710,0.02386]` | `0.20386` | `0.16386` | `[0.56167,-0.15238,0.09008]` |
| 长方体 2 | `[0.43904,-0.27431,0.01362]` | `0.19362` | `0.15362` | `[0.55890,-0.03302,0.07818]` |
| 魔方 3 | `[0.53999,-0.33928,0.02997]` | `0.20997` | `0.16997` | `[0.58027,-0.09216,0.08492]` |

### 4.2 抓取姿态

两个长方体的检测长轴约为 `1.52°`，使用俯视姿态并对齐长轴，四元数近似为：

```yaml
[0.999912, 0.013235, 0.0, 0.0]
```

魔方采用 2026-08-21 已成功姿态，精确四元数为：

```yaml
[-0.0189564460, 0.9436495767, -0.1063103131, 0.3128326179]
```

这条魔方姿态是实机成功基线。除非当前位置规划失败，否则后续不应随意替换。

### 4.3 高位释放姿态

三个放置点使用根据棋盘方向动态生成的约 40° 向外倾斜姿态：

```text
物块 1：[0.638929, 0.689051, 0.158772, 0.302935]
物块 2：[0.658469, 0.670404, 0.224853, 0.257719]
魔方 3：[0.648692, 0.679868, 0.194369, 0.281422]
```

这些只是高位释放姿态，不表示必须用 40° 斜抓。抓取仍然优先使用最常见、最稳定的俯视姿态，只有魔方沿用历史成功斜抓姿态。

## 5. 实机执行证据

| 物块 | 预抓取误差 | 抓取误差 | 夹持反馈 | 抬升误差 | 释放点误差 | 结果 |
|---|---:|---:|---|---:|---:|---|
| 长方体 1 | `14.21 mm` | `0.01 mm` | `gOBJ=2, gPO=171` | `0.04 mm` | `8.79 mm` | 成功 |
| 长方体 2 | `6.50 mm` | `0.09 mm` | `gOBJ=2, gPO=176` | `0.06 mm` | `4.42 mm` | 成功 |
| 魔方 3 | `7.41 mm` | `0.08 mm` | `gOBJ=2, gPO=89` | `0.08 mm` | `6.58 mm` | 成功 |

夹爪 `gOBJ=2` 表示闭合过程中检测到物体，且三次 `gPO` 明显不同于空夹到底的反馈，因此程序允许进入抬升阶段。抓取段实际 TCP 最大误差限制为 `8 mm`，普通转场段允许 `20 mm`，两类误差不能混用。

## 6. 完整终端复刻流程

以下命令面向当前工作站。每个新终端先执行：

```bash
cd /home/gzu/gzu_ws
conda deactivate 2>/dev/null || true
source /opt/ros/noetic/setup.bash
source /home/gzu/gzu_ws/devel/setup.bash
```

### 终端 1：双 UR3 实机驱动

```bash
roslaunch ur_robot_driver ur3_dual_bringup.launch
```

作用：连接左右 UR3，发布关节状态、控制器、Dashboard 服务和 External Control 执行接口。

### 终端 2：MoveIt 规划与右臂执行控制器

```bash
roslaunch ur3_dual_moveit_config move_group.launch \
  allow_trajectory_execution:=true \
  moveit_controller_manager:=simple_right
```

作用：启动 `move_group`、碰撞检测、OMPL 规划及右臂轨迹执行接口。

### 终端 3：RViz

```bash
roslaunch ur3_dual_moveit_config moveit_rviz.launch \
  rviz_config:=/home/gzu/gzu_ws/src/ur3_dual_moveit_config/launch/moveit.rviz
```

作用：显示双臂模型、碰撞环境、感知点、抓取/释放目标 Marker 和规划轨迹。

### 终端 4：RealSense RGB-D

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

作用：发布彩色图、深度图、对齐深度和相机内参。

### 终端 5：固定相机手眼 TF

```bash
roslaunch dual_arm_tasks right_arm_hsv_eye_on_base_runtime.launch
```

作用：发布经验证的 `right_arm_base/base -> camera` 外参，并提供相机点到机器人坐标的转换链。不能同时启动另一套相同父子坐标的手眼 TF。

### 终端 6：三蓝色物块与棋盘感知

```bash
roslaunch dual_arm_tasks blue_blocks_checkerboard_perception.launch \
  show_window:=true
```

作用：HSV 检测三个蓝色物块、读取对齐深度、识别 8×6 棋盘、发布放置点、RViz Marker 和 HSV 调试窗口。

### 终端 7：右夹爪串口节点

```bash
export PYTHONPATH="/home/gzu/gzu_ws/src/robotiq/robotiq_modbus_rtu/src:$PYTHONPATH"

rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode_right.py \
  /dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DA6ACQ6P-if00-port0
```

作用：连接右臂 Robotiq 2F 夹爪，提供输入状态和输出命令话题。

如夹爪尚未激活，单独执行：

```bash
rosrun dual_arm_tasks activate_right_gripper.py \
  _command_topic:=/right_arm/Robotiq2FGripperRobotOutput \
  _open_position:=0 \
  _speed:=120 \
  _force:=80
```

### 终端 8：真实环境碰撞模型

```bash
roslaunch dual_arm_tasks right_arm_real_environment.launch
```

作用：向 MoveIt PlanningScene 发布双臂机柜、向右延长 1.5 m 的机柜区域，以及 1200×900 mm、比机柜顶面低 45 mm 的绿布桌面。

### 终端 9：规划、复核与执行

先规划，不执行：

```bash
rosrun dual_arm_tasks right_arm_three_blue_pick_place.py \
  --config /home/gzu/gzu_ws/src/dual_arm_tasks/config/right_arm_three_blue_pick_place_20260822.yaml \
  --targets-file /home/gzu/gzu_ws/calibration/environment/right_arm_three_blue_locked_targets_20260823.yaml
```

只读复核锁点：

```bash
rosrun dual_arm_tasks validate_locked_blue_targets.py \
  --config /home/gzu/gzu_ws/src/dual_arm_tasks/config/blue_blocks_checkerboard_perception.yaml \
  --targets-file /home/gzu/gzu_ws/calibration/environment/right_arm_three_blue_locked_targets_20260823.yaml \
  --samples 30 \
  --max-shift 0.012 \
  --max-mad 0.006 \
  --association-radius 0.080
```

执行前检查 External Control、机器人模式、安全模式和夹爪：

```bash
rosservice call /right_arm/ur_hardware_interface/dashboard/program_running
rosservice call /right_arm/ur_hardware_interface/dashboard/program_state
rostopic echo -n 1 /right_arm/ur_hardware_interface/robot_mode
rostopic echo -n 1 /right_arm/ur_hardware_interface/safety_mode
rostopic echo -n 1 /right_arm/Robotiq2FGripperRobotInput
```

只有在 RViz 轨迹检查通过、现场清空、夹爪正常、示教器 External Control 已开启，并收到操作者明确“确认执行”后，才运行：

```bash
rosrun dual_arm_tasks right_arm_three_blue_pick_place.py \
  --config /home/gzu/gzu_ws/src/dual_arm_tasks/config/right_arm_three_blue_pick_place_20260822.yaml \
  --targets-file /home/gzu/gzu_ws/calibration/environment/right_arm_three_blue_locked_targets_20260823.yaml \
  --execute \
  --yes \
  --accept-provisional-board
```

`--accept-provisional-board` 是因为棋盘单格 `0.0227 m` 尚未用尺确认。`--yes` 会越过程序内最后一次交互确认，只能在操作者已经明确授权本次执行时使用。

## 7. 核心代码和用途

| 文件 | 用途 |
|---|---|
| `scripts/blue_blocks_checkerboard_perception.py` | HSV 物块分割、深度采样、三维反投影、TF 转换、棋盘识别、抓取/释放 Marker |
| `scripts/right_arm_three_blue_pick_place.py` | 三物块锁点、姿态选择、完整顺序规划、夹爪状态门控、高位释放和回初始位 |
| `scripts/right_arm_visual_pick.py` | MoveIt 规划/执行公共实现、TCP 偏置、轨迹检查、实机状态检查和夹爪命令 |
| `scripts/validate_locked_blue_targets.py` | 用最新 30 帧只读比较锁点位置及 MAD 稳定度，不发送运动命令 |
| `scripts/publish_real_environment_scene.py` | 将机柜和绿布桌面作为碰撞物发布到 PlanningScene |
| `scripts/activate_right_gripper.py` | 激活右夹爪并打开到指定位置 |
| `config/right_arm_three_blue_pick_place_20260822.yaml` | 抓取/释放高度、姿态、规划器、速度、误差阈值、夹爪参数和初始位 |
| `config/blue_blocks_checkerboard_perception.yaml` | HSV 阈值、轮廓过滤、棋盘规格、ROI、排序及零像素偏移 |
| `config/right_arm_real_environment_20260822.yaml` | 机柜和绿布桌面的尺寸、位置、颜色及碰撞层厚度 |
| `launch/right_arm_real_environment.launch` | 启动真实环境场景发布器 |
| `launch/right_arm_hsv_eye_on_base_runtime.launch` | 启动固定相机手眼 TF 链 |
| `launch/blue_blocks_checkerboard_perception.launch` | 启动三物块和棋盘感知节点 |

## 8. 仿真/规划环境

本任务使用的“仿真环境”主要是 MoveIt PlanningScene 与 RViz 规划预览，并非用 Gazebo 模拟物理掉落。

关键模型：

- `ur_description/urdf/ur3_dual.xacro`：双 UR3 与双 Robotiq 夹爪的 URDF/Xacro；
- `ur3_dual_moveit_config/config/ur3_robot.srdf`：左右臂规划组、末端执行器、命名初始位及禁碰撞对；
- `ur3_dual_moveit_config/launch/moveit.rviz`：RViz 显示项，包括环境、相机点云、物块 Marker 和任务目标；
- `dual_arm_tasks/config/right_arm_real_environment_20260822.yaml`：真实机柜和绿布桌面的碰撞模型；
- `dual_arm_tasks/scripts/publish_real_environment_scene.py`：负责把上述环境写入 PlanningScene。

环境尺寸基线：

```text
机柜：沿机械臂右侧扩展 1.5 m
绿布桌面：长 1.20 m，宽 0.90 m
桌面顶面：比机柜顶面低 0.045 m
桌面模型顶面：base z = -0.055 m
绿板边界：x=[-0.10, 0.80] m，y=[-1.20, 0.00] m
```

## 9. 依赖基线

本次成功机器的软件基线：

```text
Ubuntu 20.04.6 LTS
ROS Noetic
Python 3.8.10
CMake 3.16.3
MoveIt 1.1.16
OpenCV 4.2.0
NumPy 1.17.4
PyYAML 5.3.1
realsense2_camera 2.3.2
Universal Robots ROS Driver（工作空间源码版本 2.1.2）
```

ROS 依赖应优先在新工作空间执行：

```bash
sudo apt update
sudo apt install python3-rosdep python3-catkin-tools python3-opencv python3-yaml python3-numpy
cd /path/to/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

Robotiq 驱动为工作空间源码依赖，不能只依赖 `rosdep`。UR 模型、双臂 MoveIt 配置和定制的双臂 bringup 也必须随备份恢复。

## 10. 后续调整建议

按优先级建议如下：

1. **实测棋盘格尺寸。** 用卡尺或直尺确认单格边长，更新 `square_size_m` 并将 `square_size_confirmed` 设为 `true`。
2. **保留“运动前一次锁定”机制。** 全局相机会被机械臂遮挡；在没有增加第二视角或遮挡跟踪前，不应运动中刷新三个目标。
3. **锁点不能跨摆放复用。** 只要任何物块、棋盘、相机或机器人底座移动，就重新生成带时间戳的 targets YAML。
4. **保留零像素偏移基线。** 本次直接使用视觉中心成功；不要重新加入未经同一工况验证的经验偏移。
5. **保留长方体长轴对齐。** 两个长方体继续用顶视抓取并根据检测长边旋转夹爪。
6. **保留魔方成功四元数。** 先使用 2026-08-21/23 已成功的魔方姿态；只有当前位置确实无解时再扫描其他倾角。
7. **保持高位释放。** TCP 与棋盘表面至少相差 `120 mm`，直接松爪，不再向桌面下降。
8. **提高视觉可信度输出。** 后续可给每个目标增加轮廓面积、深度有效率、中心 MAD、与锁点位移的综合置信度，并在 RViz 中以颜色显示。
9. **显示“已冻结目标”。** 机械臂遮挡相机后，RViz/终端应明确显示正在使用执行前锁定坐标，避免把视觉窗口暂时丢失误判为任务卡住。
10. **补全碰撞环境。** 实测相机支架、桌腿、线缆和桌下结构，在 PlanningScene 中建立保守包络。
11. **归档每次成功基线。** 保存 targets YAML、任务配置、计划摘要和执行日志，但只作为可追溯证据，不作为新摆放位置的执行输入。
12. **定期检查磁盘。** ROS 日志、RGB-D 录制和视频增长很快；删除前先人工确认，源码仓库不应包含这些大文件。

## 11. 实机安全约束

- 规划成功不等于允许执行；必须由现场操作者检查 RViz 并明确确认；
- 执行前确认工作区无人、无临时工具、线缆不会进入路径；
- External Control 只在执行窗口开启，调试和文档阶段保持关闭；
- 必须确认 `safety_mode=1`、机器人处于 RUNNING、夹爪 `gFLT=0`；
- 任一抓取未检测到 `gOBJ=2` 时，不允许携带动作继续到棋盘；
- 任一实际 TCP 偏差超过对应阈值时停止，不应为了“跑完三个”放宽抓取误差；
- 急停、保护停、夹爪故障、碰撞、目标移动或人员进入工作区时立即终止。

---

本次成功基线的核心结论是：**相机无遮挡时一次性锁定三个最新视觉中心，两个长方体按长轴俯视抓取，魔方沿用已验证斜抓四元数，全部在棋盘上方至少 120 mm 的 TCP 高度直接松爪。**
