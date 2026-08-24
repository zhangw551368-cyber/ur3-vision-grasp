# Right Arm YOLO Red Block Grasp

本流程用 YOLO 识别红色物块，但不让 YOLO 直接决定下探高度。YOLO 只提供物块在
`right_arm_base` 下的水平位置；抓取姿态、抓取高度、放置点和安全边界来自已经实机成功的
右臂固定点位几何。

## 为什么之前会乱跑

之前危险的点在于把视觉得到的 3D 点直接当成抓取 TCP 点，并且让视觉 Z 参与了下探。
只要手眼标定、目标中心、TCP 偏置或深度点定义有几厘米误差，机械臂就会向错误高度运动。

现在改成：

- YOLO 负责检测红色物块框。
- 对框中心取对齐深度，转换到 `right_arm_base`。
- 加固定的视觉点到 TCP 抓取点偏置。
- `grasp.z` 固定为已成功点位的安全高度 `0.369`。
- 姿态使用已成功点位的 `grasp` 四元数。
- `pre_grasp/lift` 使用已成功固定抓取的相对位移。
- 任何生成的抓取点超出安全边界，节点直接拒绝运动。

## 公开数据集

可选公开数据集/模型：

- Roboflow `Colored blocks`：513 张图片，类为 `red/green/blue`，页面显示有模型和指标。
  URL: https://universe.roboflow.com/autonomous-object-picking-robot/colored-blocks
- Roboflow `red,green,blue cube detection`：207 张图片，类为 `red cube/green cube/bluecube`。
  URL: https://universe.roboflow.com/jakub-slof/red-green-blue-cube-detection

这类公开模型可以做初始权重，但实验台面、相机角度、光照和物块材质不同，最好再用现场
采集的 20 到 50 张图微调一次。

## 训练并放置模型

先用 Roboflow 导出 YOLOv8 格式数据集，得到 `data.yaml`。如果用 Roboflow Python SDK，
需要自己的 API key。

训练命令示例：

```bash
cd /home/gzu/gzu_ws
yolo detect train \
  model=yolov8n.pt \
  data=/home/gzu/gzu_ws/datasets/colored-blocks/data.yaml \
  epochs=80 \
  imgsz=640 \
  batch=8 \
  project=/home/gzu/gzu_ws/models/yolo \
  name=colored_blocks_red
```

训练完成后放到默认路径：

```bash
mkdir -p /home/gzu/gzu_ws/src/ultralytics_ros/models
cp /home/gzu/gzu_ws/models/yolo/colored_blocks_red/weights/best.pt \
   /home/gzu/gzu_ws/src/ultralytics_ros/models/red_block.pt
```

也可以用本仓库脚本训练并自动复制：

```bash
/home/gzu/anaconda3/bin/python3 \
  /home/gzu/gzu_ws/src/dual_arm_tasks/scripts/train_public_colored_blocks_yolo.py \
  --data /home/gzu/gzu_ws/datasets/colored-blocks/data.yaml \
  --model yolov8n.pt \
  --epochs 80 \
  --imgsz 640 \
  --batch 8 \
  --device cpu
```

也可以不复制，启动时传 `model_path:=/path/to/best.pt`。

如果公开数据集暂时无法直接下载，可以先用现场相机自动标注红块，训练一个临时 YOLO：

```bash
/home/gzu/anaconda3/bin/python3 \
  /home/gzu/gzu_ws/src/dual_arm_tasks/scripts/capture_red_block_yolo_dataset.py \
  --output /home/gzu/gzu_ws/datasets/red_block_autolabel \
  --count 120

/home/gzu/anaconda3/bin/python3 \
  /home/gzu/gzu_ws/src/dual_arm_tasks/scripts/train_public_colored_blocks_yolo.py \
  --data /home/gzu/gzu_ws/datasets/red_block_autolabel/data.yaml \
  --model yolov8n.yaml \
  --epochs 40 \
  --imgsz 320 \
  --batch 16 \
  --device 0 \
  --name red_block_autolabel
```

这个临时模型只适合当前实验台面验证 YOLO 抓取链路；正式版本仍建议用公开彩色物块数据集加现场图片微调。

## 启动 YOLO 检测

确认 RealSense、眼在手 TF、TCP TF、MoveIt、右臂驱动已启动后：

```bash
roslaunch dual_arm_tasks right_arm_yolo_red_block_grasp.launch \
  start_detector:=true \
  start_pick:=false \
  model_path:=/home/gzu/gzu_ws/src/ultralytics_ros/models/red_block.pt
```

检查输出：

```bash
rostopic echo /yolo_red_block/point_base
rostopic echo /yolo_red_block/pixel
rqt_image_view /yolo_red_block/image
```

调试图也会保存到：

```bash
/tmp/yolo_red_block_debug.jpg
```

## 只规划不运动

YOLO 点稳定后先跑 plan-only：

```bash
roslaunch dual_arm_tasks right_arm_yolo_red_block_grasp.launch \
  start_detector:=false \
  start_pick:=true \
  execute:=false
```

必须看到生成的 `grasp TCP` 在安全范围内，并且 MoveIt 能完成：

```text
pre_grasp -> grasp -> lift -> pre_place -> place -> retreat -> home
```

## 真机执行

确认 RViz 规划和现场空间无误，示教器 External Control 正在运行后：

```bash
roslaunch dual_arm_tasks right_arm_yolo_red_block_grasp.launch \
  start_detector:=false \
  start_pick:=true \
  execute:=true \
  yes:=true
```

## 关键配置

文件：

```bash
/home/gzu/gzu_ws/src/dual_arm_tasks/config/right_arm_yolo_red_block_grasp.yaml
```

需要重点调的参数：

- `model_path`：YOLO 权重路径。
- `class_names`：公开模型的类别名，例如 `red` 或 `red cube`。
- `yolo_point_to_grasp_bias_base`：YOLO 检测点到真实 TCP 抓取点的偏置。
- `fixed_grasp_z`：固定安全抓取高度，当前 `0.369`。
- `safe_grasp_bounds`：生成抓取点的安全边界。
- `go_home_first`：是否先回固定 home；默认 `false`，避免无意义抬远。

如果 YOLO 框中心和 HSV 检测中心不同，先不要执行真机，只调
`yolo_point_to_grasp_bias_base`，每次按 5 到 10 mm 改。
