# 右臂 MoveIt Calibration ChArUco 手眼标定流程

本文档用于双 UR3 工作站中右臂和外置 Kinect2 的手眼标定。使用的是 RViz 里的 MoveIt Calibration `HandEye Calibration` 插件，不是 `easy_handeye`，也不是 `easy_handeye2`。

## 最少终端指令

终端 1，连接双 UR3：

```bash
cd /home/gzu/gzu_ws
source devel/setup.bash
roslaunch ur_robot_driver ur3_dual_bringup.launch
```

作用：启动双臂 UR 驱动、`joint_state_publisher`、`robot_state_publisher`，发布双臂真实关节状态和 TF。

终端 2，如果右夹爪节点没开，启动右 Robotiq 夹爪：

```bash
cd /home/gzu/gzu_ws
source devel/setup.bash
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode_right.py /dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DA6ACQ6P-if00-port0
```

作用：启动 `/right_robotiq2FGripper`，订阅 `/right_arm/Robotiq2FGripperRobotOutput`，发布 `/right_arm/Robotiq2FGripperRobotInput`。

终端 3，激活并打开右夹爪：

```bash
cd /home/gzu/gzu_ws
source devel/setup.bash
rosrun dual_arm_tasks activate_right_gripper.py
```

作用：先发送 `rACT=0` 复位，再发送 `rACT=1` 激活，并把夹爪打开。

终端 4，启动右臂手眼标定界面：

```bash
cd /home/gzu/gzu_ws
source devel/setup.bash
roslaunch dual_arm_tasks right_arm_moveit_calibration_charuco.launch start_robot:=false
```

作用：启动 Kinect2、MoveIt、RViz HandEye Calibration 插件，以及 `/handeye_calibration/target_detection` 检测图窗口。`start_robot:=false` 表示不重复启动双臂驱动，因为终端 1 已经启动了。

标定完成后，发布保存出来的静态 TF：

```bash
cd /home/gzu/gzu_ws
source devel/setup.bash
roslaunch dual_arm_tasks right_arm_base_to_kinect20_0610.launch
```

作用：发布 `right_arm_base -> kinect2_0_rgb_optical_frame`，供后续视觉抓取把相机坐标转换到右臂基坐标。

采集结束后，如果只想关闭标定相关程序，直接在终端 4 按 `Ctrl+C`。终端 1 的双臂连接和终端 2 的右夹爪可以继续保留。

## 当前保存结果

当前保存的标定结果文件：

```bash
/home/gzu/gzu_ws/src/dual_arm_tasks/launch/right_arm_base_to_kinect20_0610.launch
```

里面发布的是：

```text
parent: right_arm_base
child:  kinect2_0_rgb_optical_frame
xyz:    -0.442579 0.197702 0.217105
quat:   0.353006 0.181478 0.839384 -0.371331
```

这正是当前外置 Kinect2 的结果方向：右臂基坐标到固定相机光学坐标。

## 当前标定类型

当前系统是：

```text
右臂: UR3
相机: Kinect2，固定在机器人外部观察桌面
类型: Eye-to-hand / Eye-on-base / 眼在手外
```

眼在手外时，ChArUco 标定板必须和右臂末端刚性一起运动。也就是说，标定板要被夹爪夹住，或者固定在右臂末端附近，并且采样过程中标定板相对 `right_arm_tool0` 的位置不能变化。外置 Kinect2 负责观察这块随右臂运动的板。

不要在当前眼在手外流程里把板固定在桌面上再只移动机械臂。板固定在桌面、相机随手移动，是后面眼在手上的流程。

## 标定板参数

已打印文件：

```bash
/home/gzu/gzu_ws/calib.io_charuco_200x150_8x11_15_11_DICT_4X4.pdf
```

实际参数：

```text
纸张尺寸:              200 x 150 mm
MoveIt 插件中方格数:   11 x 8
棋盘格边长:            15 mm
ArUco marker 边长:     11 mm
字典:                  DICT_4X4_250
longest board side:    0.165 m
measured marker size:  0.011 m
```

注意：`longest board side (m)` 是 ChArUco 棋盘区域最长边，不是整张纸的宽度。本标定板是 `11 * 0.015 = 0.165 m`。

## 基本操作流程

1. 按最上面的命令启动双臂、右夹爪和标定界面。
2. 把 ChArUco 板刚性固定在右夹爪/右末端上。
3. 在 RViz 的 `HandEye Calibration` 面板中进入 `Target` 页。
4. 选择 Kinect2 图像和 CameraInfo topic。
5. 确认检测窗口能看到绿色角点、marker id 和红绿蓝坐标轴。
6. 进入 `Context` 页，确认是 `Eye-to-hand`，并选择正确 frame。
7. 进入 `Calibrate` 页，移动右臂到多个不同姿态，每个稳定姿态点一次 `Take sample`。
8. 样本足够后点 `Solve`。
9. 重投影误差较小且结果稳定后点 `Save camera pose`。
10. 保存 launch 文件，再用保存的 launch 发布静态 TF。
11. 最后把 ArUco 物块放到之前固定抓取成功的位置，对比视觉转换后的抓取坐标和原 YAML 中的 grasp 坐标，误差在 `1-2 cm` 内再继续视觉抓取。

采样姿态建议：

- 一般 `15-30` 个质量好的样本就够。
- 样本要覆盖不同位置和不同姿态，不要在几乎相同的位姿重复点很多次。
- 右腕的 roll/pitch/yaw 要有变化，不能只平移。
- 每次点击 `Take sample` 前，检测图里标定板应完整、清晰、坐标轴稳定。

## 1. Target 页

当前应使用：

```text
Target Type:              HandEyeTarget/Charuco
squares, X:               11
squares, Y:               8
marker size (px):         110
square size (px):         150
margin size (px):         2
marker border (bits):     1
ArUco dictionary:         DICT_4X4_250
longest board side (m):   0.165
measured marker size (m): 0.011
Image Topic:              /kinect_0/kinect2/qhd/image_color_rect
CameraInfo Topic:         /kinect_0/kinect2/qhd/camera_info
```

每个选项含义：

- `Target Type`：标定目标类型。这里用 `HandEyeTarget/Charuco`，不要用单个 ArUco。
- `squares, X`：插件生成/识别的棋盘 X 方向方格数。当前填 `11`。
- `squares, Y`：插件生成/识别的棋盘 Y 方向方格数。当前填 `8`。
- `marker size (px)`：生成预览图时 marker 的像素边长，只影响预览/生成图，不是物理尺寸。
- `square size (px)`：生成预览图时棋盘格的像素边长。当前用 `150`，与 `marker size (px)=110` 对应 `15 mm:11 mm`。
- `margin size (px)`：生成目标图外边距。
- `marker border (bits)`：ArUco marker 黑边宽度，当前用 `1`。
- `ArUco dictionary`：ArUco 字典。当前打印板用 `DICT_4X4_250`。
- `longest board side (m)`：棋盘区域最长边的真实长度，当前 `0.165`。
- `measured marker size (m)`：marker 真实边长，当前 `0.011`。
- `Image Topic`：用于检测的彩色图像 topic。
- `CameraInfo Topic`：相机内参 topic，必须选，不能空着。

按钮含义：

- `Create Target`：生成右侧预览图。你已经打印好标定板，所以它不是必须步骤。
- `Save Target`：保存生成的目标图，不是保存手眼标定结果。

需要修改的地方：

- 换标定板时，改 `squares X/Y`、物理尺寸、字典。
- 换相机时，改 `Image Topic` 和 `CameraInfo Topic`。
- 如果 `CameraInfo Topic` 下拉为空，说明 RViz 打开时 camera topic 还没准备好；等 Kinect2 启动后重新打开 RViz 或重新选择 topic。

## 2. Context 页

当前应使用：

```text
Sensor configuration: Eye-to-hand
Sensor frame:         kinect2_0_rgb_optical_frame
Object frame:         handeye_target
End-effector frame:   right_arm_tool0
Robot base frame:     right_arm_base
```

每个选项含义：

- `Sensor configuration`：相机和机械臂的安装关系。当前外置 Kinect2 固定在机器人外部，所以选 `Eye-to-hand`。
- `Sensor frame`：相机光学坐标系。当前右侧 Kinect2 是 `kinect2_0_rgb_optical_frame`。
- `Object frame`：插件检测出来的 ChArUco 目标坐标系，选 `handeye_target`。
- `End-effector frame`：机械臂末端坐标系。右臂推荐 `right_arm_tool0`。
- `Robot base frame`：机器人基坐标。右臂视觉抓取使用 `right_arm_base`。
- `Camera Pose Initial Guess`：相机位姿初值。当前可以保持全 0，求解后由插件计算。

需要修改的地方：

- 如果 TF 里实际相机 frame 是 `kinect2_rgb_optical_frame`，就选实际存在的那个。
- 如果明确要用 UR 控制器 TCP，可以测试 `right_arm_tool0_controller`，但默认推荐 `right_arm_tool0`。
- 做左臂时，把 `right_arm_base`、`right_arm_tool0`、`right_arm` 改成左臂对应项。

## 3. Calibrate 页

当前应使用：

```text
AX=XB Solver:   crigroup/TsaiLenz1989
Planning Group: right_arm
```

主要按钮含义：

- `Take sample`：采集当前一组样本。只有检测稳定时才点。
- `Clear samples`：清空样本。如果 frame 选错、板移动松动、采错了，就清空重采。
- `Solve`：根据已有样本求解手眼变换。
- `Save camera pose`：保存最终相机位姿为静态 TF launch 文件。真正要保存标定结果时点这个。

文件按钮含义：

- `Load joint states`：加载记录过的关节状态序列。
- `Save joint states`：保存关节状态序列，可选。
- `Load samples`：加载之前保存的样本。
- `Save samples`：保存当前样本，方便以后复现或重新求解，可选但推荐保存一份。

自动回放按钮含义：

- `Plan`：为记录关节状态规划路径。
- `Execute`：执行规划路径。只有确认工作空间安全时才用。
- `Skip`：跳过当前记录点。
- `Recorded joint state progress`：自动回放关节状态的进度，不代表手动采样质量。

结果显示含义：

- `Pose samples`：已经采集的样本列表。
- `Reprojection error`：重投影误差，越小越好。你本次结果约 `0.0042 m`、`0.0028 rad`，看起来很好。

保存标定结果：

1. 点 `Solve`。
2. 点 `Save camera pose`。
3. 保存到：

```bash
/home/gzu/gzu_ws/src/dual_arm_tasks/launch/right_arm_base_to_kinect20_0610.launch
```

不要点 RViz 底部的 `Save` 来保存标定结果。RViz 底部 `Save` 只保存 RViz 界面布局。

## 采集后关闭

如果是用最上面的终端 4 启动的标定界面，采完后在终端 4 按 `Ctrl+C` 即可。

如果有残留窗口或节点，可以只清理标定相关节点：

```bash
source /home/gzu/gzu_ws/devel/setup.bash
rosnode kill /right_arm_charuco_detection_view /right_arm_moveit_calibration_rviz /move_group \
  /kinect_0/kinect2 /kinect_0/kinect2_bridge \
  /kinect_0/kinect2_points_xyzrgb_hd /kinect_0/kinect2_points_xyzrgb_qhd /kinect_0/kinect2_points_xyzrgb_sd
rosnode cleanup
```

如果还要保留机器人和右夹爪，确认只剩这些节点：

```text
/joint_state_publisher
/robot_state_publisher
/left_arm/ur_hardware_interface
/right_arm/ur_hardware_interface
/left_arm/controller_spawner
/right_arm/controller_spawner
/right_robotiq2FGripper
```

## 视觉抓取前验证

先发布保存的静态 TF：

```bash
cd /home/gzu/gzu_ws
source devel/setup.bash
roslaunch dual_arm_tasks right_arm_base_to_kinect20_0610.launch
```

把 ArUco 物块放到之前固定抓取成功的位置，运行视觉检测后比较视觉坐标和 YAML grasp 坐标：

```bash
rosrun dual_arm_tasks verify_visual_grasp_against_yaml.py \
  --config /home/gzu/gzu_ws/right_arm_visual_aruco_pick_place.yaml \
  --pose-topic /detected_object_pose_base \
  --threshold 0.02
```

误差在 `0.01-0.02 m` 内，再继续视觉抓取。

## 后续眼在手上流程

眼在手上是指相机刚性安装在末端，随右臂一起运动。

硬件关系：

```text
相机: 固定在右腕/右末端
ChArUco 板: 固定在桌面或外部支架上
标定类型: Eye-in-hand / 眼在手上
```

采样规则：

```text
移动右臂和相机，从不同角度观察固定的 ChArUco 板。
不要用夹爪夹着板移动。
保证板完整、清晰、角度多样。
```

Target 页：

- 继续使用 `HandEyeTarget/Charuco`。
- 如果还是同一块打印板，标定板参数保持不变。
- 把 `Image Topic` 和 `CameraInfo Topic` 改成腕部相机的话题。

Context 页右臂眼在手上设置：

```text
Sensor configuration: Eye-in-hand
Sensor frame:         <wrist_camera_optical_frame>
Object frame:         handeye_target
End-effector frame:   right_arm_tool0
Robot base frame:     right_arm_base
```

眼在手上保存出来的 TF 方向通常应为：

```text
parent: right_arm_tool0 或你选择的末端 frame
child:  <wrist_camera_optical_frame>
```

如果腕部相机有自己的启动文件，先单独启动腕部相机，再只启动标定界面，不启动 Kinect2：

```bash
cd /home/gzu/gzu_ws
source devel/setup.bash
roslaunch dual_arm_tasks right_arm_moveit_calibration_charuco.launch \
  start_robot:=false start_kinect2:=false
```

然后在 `Target` 页手动选择腕部相机的 image 和 camera_info topic。
