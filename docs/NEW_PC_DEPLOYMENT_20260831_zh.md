# 新电脑克隆并恢复真实 UR3 工作站

## 能达到的结果

在一台装有 Ubuntu 20.04 和 ROS Noetic 的新电脑上，本仓库可以恢复：

- 左右 UR3 驱动、实机运动学标定和双臂 URDF；
- 右臂 MoveIt 规划组及真实轨迹控制器；
- 右臂 Robotiq 2F 串口节点、激活和 RViz 关节显示；
- 固定 RealSense 对齐 RGB-D；
- 2026-08-17 运行安全版眼在手外标定；
- 手眼训练/验证样本、离线候选结果、标定板 PDF 与完整中文总结；
- 最新实机机柜和绿布桌面碰撞空间；
- HSV 三蓝块抓取代码；
- GraspNet 多样工件轻量源码及其独立环境/模型下载脚本。

`git clone` 不能替代操作系统、ROS、网卡和示教器配置。完成下面的一次性步骤后，工作站可用一个脚本启动。

## 1. 硬件与系统前提

- Ubuntu 20.04.6 LTS；
- ROS Noetic Desktop Full 和 MoveIt；
- 与两台 UR3 相连的有线网卡；
- UR3 已安装并配置 External Control URCap；
- 右 Robotiq 2F 的 USB-RS485 适配器；
- 固定 RealSense，相机、桌子、机柜和机器人底座未改变相对位置。

实验室已验证网络：

```text
电脑：192.168.1.41/24
左臂：192.168.1.43
右臂：192.168.1.44
```

## 2. 克隆

```bash
cd ~
git clone git@github.com:zhangw551368-cyber/ur3-vision-grasp.git
cd ur3-vision-grasp
```

HTTPS 也可读取公开仓库：

```bash
git clone https://github.com/zhangw551368-cyber/ur3-vision-grasp.git
```

## 3. 安装依赖并编译

先按 [`dependencies/README.md`](../dependencies/README.md) 安装 ROS Noetic、MoveIt、RealSense ROS、rosdep 和 catkin-tools，然后执行：

```bash
bash scripts/restore_workspace.sh
```

脚本会初始化 rosdep、安装缺失依赖并构建驱动、双臂描述、MoveIt、Robotiq、任务代码和 GraspNet 轻量包。

## 4. 配置电脑有线网卡

先查看 NetworkManager 连接名：

```bash
nmcli connection show
```

将下面的 `有线连接名` 换成实际名称：

```bash
sudo nmcli connection modify "有线连接名" \
  ipv4.method manual \
  ipv4.addresses 192.168.1.41/24 \
  ipv4.gateway "" \
  ipv4.dns ""

sudo nmcli connection up "有线连接名"
```

如果现场 IP 或串口不同，编辑：

```text
config/lab_hardware.env
```

首次使用 USB-RS485 时，把当前用户加入串口组，然后注销并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

左右 UR3 的 External Control URCap 中，控制电脑地址应填写当前有线网卡地址（实验室默认 `192.168.1.41`）。

## 5. 只读硬件检查

```bash
bash scripts/check_hardware_connections.sh
```

它只检查电脑 IP、左右 UR3 ping、右夹爪设备和 RealSense，不启动 ROS，也不发送运动命令。

## 6. 启动整个真实工作站

```bash
bash scripts/start_real_workcell.sh
```

该命令同时启动：

```text
双 UR3 driver
-> robot_state_publisher / joint states
-> RealSense aligned RGB-D
-> 新手眼静态 TF
-> MoveIt move_group + right controller
-> 真实机柜和绿桌 PlanningScene
-> 右 Robotiq 节点与激活
-> RViz
```

默认碰撞物体为：

```text
right_arm_lab_20260824__support_cabinet
right_arm_lab_20260824__green_cloth_work_surface
```

绿桌碰撞体尺寸为 `1.200 x 0.900 x 0.020 m`，中心为 `[0.780, -0.340, -0.020] m`，顶面为 `base z=-0.010 m`。机柜尺寸为 `0.980 x 2.760 x 0.795 m`，中心为 `[-0.3375, -0.3530, -0.4075] m`。

## 7. 只读运行验证

保持统一启动终端运行，另开终端：

```bash
cd ~/ur3-vision-grasp
bash scripts/verify_running_workcell.sh
```

通过时显示：

```text
REAL WORKCELL READ-ONLY VERIFICATION PASSED
```

该检查要求右臂六个关节状态、Dashboard 服务、MoveIt PlanningScene、机柜、绿桌以及相机 TF 同时存在，不会运动机器人。

## 8. External Control

ROS driver 建立网络连接后，仍必须在左右 UR3 示教器中运行对应的 External Control 程序，才能接受轨迹。不要通过脚本绕过示教器确认。

在首次真机执行前必须检查：

```bash
rosservice call /right_arm/ur_hardware_interface/dashboard/program_running
rostopic echo -n 1 /right_arm/ur_hardware_interface/robot_mode
rostopic echo -n 1 /right_arm/ur_hardware_interface/safety_mode
rostopic echo -n 1 /right_arm/Robotiq2FGripperRobotInput
```

## 9. GraspNet 可选安装

GraspNet 的源码位于 `src/ur3_graspnet6dof_20260824`。仓库不保存 6 GB Conda 环境、第三方仓库、checkpoint 或历史运行缓存。需要 GraspNet 时：

```bash
cd src/ur3_graspnet6dof_20260824
bash scripts/setup_environment.sh
bash scripts/download_checkpoint.sh
./scripts/validate_project.py
```

这部分需要先安装 Conda 和 NVIDIA CUDA GPU。实验室 RTX 4090 默认使用 CUDA 架构 `8.9`；其他 GPU 应在安装前设置相应的 `TORCH_CUDA_ARCH_LIST`。未安装 GraspNet 不影响机械臂连接、MoveIt、手眼 TF、机柜和桌面碰撞空间。

## 安全边界

- 统一启动只建立连接和场景，不自动运行抓取轨迹；
- 机柜和桌面为已核对碰撞体，相机支架、桌腿、线缆仍未完成测量，禁止批准桌下或贴近线缆的轨迹；
- 相机、桌子、机柜或机器人底座移动后，旧手眼外参和碰撞坐标必须重新标定/测量；
- 历史目标 YAML 与历史像素只能用于审计，不得直接用于当前摆放；
- 必须人工检查 RViz、现场、急停、External Control、RUNNING/NORMAL 和夹爪状态后才允许实机运动。

## 本次发布验证记录

2026-08-31 已在仓库外的全新临时目录执行：

- 9 个必要 catkin 包首次构建成功；
- 统一 launch 的全部仓库依赖均解析到克隆目录，没有引用原工作空间；
- GraspNet 核心测试 `14 passed`；
- 关闭实体臂、相机、夹爪和轨迹执行后启动统一 launch；
- MoveIt 成功建立 PlanningScene；
- 机柜和绿桌两个碰撞体成功加入；
- 只读验证器输出 `REAL WORKCELL READ-ONLY VERIFICATION PASSED`。

验证时当前实验室两台 UR3 的 `192.168.1.43/44` 均未响应 ping，因此没有冒险启动实体 driver，也没有发送任何机器人或夹爪运动命令。实体连接仍须在机器人上电、网线接通、网络配置正确时由现场操作者完成。
