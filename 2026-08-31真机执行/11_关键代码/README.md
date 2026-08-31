# 关键代码汇总

## 两类代码

1. 各终端文件夹中的 `运行.sh`、检查脚本和执行脚本，是用户手动运行的入口。每一条有效命令附近都附有中文注释，说明该行意义。
2. `原始代码快照` 保存本次流程真正调用的核心 ROS、MoveIt、感知和 GraspNet 源码。快照用于阅读和追溯；运行脚本仍调用 `/home/gzu/gzu_ws` 中的原文件，避免复制版与 catkin 安装空间不一致。

## 快照范围

### 新手眼标定

- `calibration/handeye/right_arm_camera_eye_on_base_20260817_safe.launch`
- `src/dual_arm_tasks/launch/right_arm_hsv_eye_on_base_runtime.launch`

### HSV 三蓝块路线

- `blue_blocks_checkerboard_perception.py`：HSV 分割、深度反投影、TF、棋盘与 Marker。
- `right_arm_three_blue_pick_place.py`：三目标冻结、姿态选择、顺序规划与执行。
- `right_arm_visual_pick.py`：MoveIt、TCP 偏移、轨迹检查、夹爪和实机状态公共代码。
- `validate_locked_blue_targets.py`：30 帧锁点稳定度只读复核。
- 对应 launch、任务配置、碰撞环境配置和历史成功锁点。

### GraspNet 多样工件路线

- `graspnet_inference_node.py`：同步 RGB-D、网络推理、候选与 Marker 发布。
- `classify_pictured_objects.py`：工件分类、像素框、冻结目标 JSON。
- `graspnet_pick_executor.py`：候选过滤、MoveIt 规划、真机门控和失败恢复。
- `multi_object_sequence.py`：运动前完成全部感知和规划并写入缓存。
- `execute_cached_sequence.py`：校验缓存和关节起点后执行，不在遮挡期间读相机。
- `python/ur3_graspnet6dof`：配置、几何、图像、候选与目标验证公共模块。
- `config/right_arm_green_table.yaml`：相机、桌面、工作区、速度、夹爪和执行锁。

## 为什么不把中文注释直接插入所有原始源码行

Python、YAML、XML 和 RViz 文件中的每个空行、括号续行或纯数据项并不都适合追加行尾注释；强行逐行插入会改变 shebang、YAML 结构、XML 属性或序列化文件语义。为保证代码仍与实机成功版本完全一致，原始源码按原样快照；可直接运行的短入口脚本则逐条写明中文意义。长源码的函数级用途见 `核心代码功能说明.md`。

历史 `runtime/*.pkl` 缓存、checkpoint、Conda 环境和第三方库没有复制。尤其不能把旧 `.pkl` 轨迹当作当前真机输入。

