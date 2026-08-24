# 依赖与恢复说明

## 已验证平台

```text
Ubuntu 20.04.6 LTS
ROS Noetic
Python 3.8.10
CMake 3.16.3
catkin_tools linked devel layout
```

关键版本快照见 `system-baseline.txt`。

## 建议安装方式

先安装 ROS Noetic Desktop Full，再安装构建和视觉基础包：

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-moveit \
  ros-noetic-realsense2-camera \
  python3-rosdep \
  python3-catkin-tools \
  python3-opencv \
  python3-numpy \
  python3-yaml
```

随后从仓库根目录解析其余 ROS 依赖：

```bash
source /opt/ros/noetic/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

仓库已包含当前使用的 `ur_robot_driver`、`ur_description`、`ur3_dual_moveit_config` 和 `robotiq` 源码。系统中的同名 Debian 包会被当前工作空间 overlay 覆盖。

## 非软件依赖

- 两台 UR3 与相应 External Control/URCap 配置；
- 右臂 Robotiq 2F 夹爪与 USB-RS485 适配器；
- 固定 RealSense RGB-D 相机；
- 相机内参和固定相机到机器人基座的手眼外参；
- 与环境 YAML 一致的机柜、绿布桌面和棋盘；
- 操作者现场安全确认与急停访问能力。

真实设备标识保留在启动文件和文档中，因此仓库应默认设为 Private。

