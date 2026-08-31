# 总览与关键指令

## 我们之前完成的主要工作

从新手眼标定到真机抓取，实际完成了以下链路：

1. 将 2026-08-17 的 `right_arm_base -> camera_color_optical_frame` 标定结果转换为不会破坏 RealSense 内部 TF 树的 safe 运行文件。
2. 启动双 UR3 实机驱动和右臂 MoveIt 规划/轨迹执行服务。
3. 启动对齐到彩色图的 RealSense RGB-D，并确认图像、深度、内参帧均为 `camera_color_optical_frame`。
4. 启动右臂 Robotiq 2F 串口节点，激活夹爪并使用 `gOBJ=2` 判断是否真正夹到物体。
5. 向 MoveIt PlanningScene 发布真实机柜和绿布桌面碰撞模型。
6. 路线 A 用 HSV、深度和棋盘识别锁定三个蓝色物块，运动前完成复核，执行期间不刷新目标。
7. 路线 B 用分类器冻结物理目标像素，用 GraspNet 产生抓取候选，再用 MoveIt 完成抓取、抬升、释放和返回轨迹。
8. 在 RViz 中检查完整轨迹后缓存计划；真机阶段只读缓存，不再读取被机械臂遮挡的相机画面。
9. 执行前检查 External Control、RUNNING/NORMAL 状态、右臂控制器和夹爪；抓取失败时撤离并返回初始位。

## 持续运行终端数量

### 路线 A：HSV 三蓝块，共 9 个

```text
01 driver
02 MoveIt
03 RViz
04 RealSense
05 新手眼 TF
06 Robotiq
07 真实碰撞环境
08A HSV 三蓝块/棋盘感知
09A 规划、锁点复核、状态检查、真机执行
```

### 路线 B：GraspNet 多样工件，共 10 个

```text
01 driver
02 MoveIt
03 RViz
04 RealSense
05 新手眼 TF
06 Robotiq
07 真实碰撞环境
08B GraspNet RGB-D 推理服务
09B 多样工件分类与像素冻结
10B 完整规划、缓存和真机执行
```

夹爪激活、`tf_echo` 和真机状态查询属于执行一次就结束的辅助命令，可在临时终端运行，不计入上述数量。

## 最关键的人工检查命令

```bash
rosrun tf tf_echo right_arm_base camera_color_optical_frame
rosservice call /right_arm/ur_hardware_interface/dashboard/program_running
rosservice call /right_arm/ur_hardware_interface/dashboard/program_state
rostopic echo -n 1 /right_arm/ur_hardware_interface/robot_mode
rostopic echo -n 1 /right_arm/ur_hardware_interface/safety_mode
rostopic echo -n 1 /right_arm/Robotiq2FGripperRobotInput
```

期望至少满足：External Control 程序在运行、`robot_mode=7`、`safety_mode=1`、夹爪 `gACT=1`、`gSTA=3`、`gFLT=0`。

## 启动约定

每个新终端的脚本都会执行等价的环境准备：

```bash
cd /home/gzu/gzu_ws
conda deactivate 2>/dev/null || true
source /opt/ros/noetic/setup.bash
source /home/gzu/gzu_ws/devel/setup.bash
```

直接运行脚本的方法：

```bash
cd /home/gzu/gzu_ws/2026-08-31真机执行
bash 01_双UR3实机驱动/运行.sh
```

其余终端按文件夹编号分别运行。各文件夹内的 Markdown 说明了成功标志和注意事项。
