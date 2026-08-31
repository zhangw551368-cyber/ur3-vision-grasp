# 终端 04：RealSense 对齐 RGB-D

运行 `bash 运行.sh`。必须同时启用深度对齐和同步，以便彩色图、aligned depth 与 camera info 可按时间配对。

关键话题为：

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

三者的有效帧应为 `camera_color_optical_frame`。

