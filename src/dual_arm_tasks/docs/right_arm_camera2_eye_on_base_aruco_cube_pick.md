# Right Arm Camera2 Eye-On-Base ArUco Cube Pick

Today we use only the eye-on-base camera2 setup. Eye-in-hand calibration is not used.

## Final Hand-Eye Result

Calibration archive:

```bash
/home/gzu/gzu_ws/calibration/handeye/eye_on_base/right_arm_camera2/
```

Current optimized result:

```bash
/home/gzu/gzu_ws/calibration/handeye/eye_on_base/right_arm_camera2/right_arm_camera2_eye_on_base_lie_optimized_current.yaml
```

Package copy used by launch files:

```bash
$(rospack find dual_arm_tasks)/config/handeye/right_arm_camera2_eye_on_base_lie_optimized.yaml
```

Final TF launch:

```bash
roslaunch dual_arm_tasks right_arm_camera2_eye_on_base_tf.launch
```

It publishes:

```text
right_arm_base -> camera2_link
```

The RealSense driver publishes the internal transform:

```text
camera2_link -> camera2_color_optical_frame
```

Together they recover the optimized hand-eye transform:

```text
right_arm_base -> camera2_color_optical_frame
```

Do not run the old easy_handeye launch or MoveIt Calibration RViz camera-pose publisher at the same time.

## Bring Up

If the two RealSense cameras are already running, start the planning environment without
the pick node:

```bash
cd ~/gzu_ws
source devel/setup.bash

roslaunch dual_arm_tasks right_arm_camera2_eye_on_base_aruco_cube_pick.launch \
  start_cameras:=false \
  run_pick_node:=false
```

This starts MoveIt, the optimized eye-on-base TF, and calibrated camera2 camera_info.

Start cube localization in another terminal:

```bash
cd ~/gzu_ws
source devel/setup.bash

roslaunch dual_arm_tasks right_arm_camera2_aruco_cube_localization.launch \
  start_cameras:=false \
  start_handeye_tf:=false \
  start_camera_info:=false \
  view_debug:=true
```

## Plan-Only Cube Pick

Use this first. It detects the cube marker and plans, but does not execute:

```bash
roslaunch dual_arm_tasks right_arm_camera2_eye_on_base_aruco_cube_pick.launch \
  start_cameras:=false \
  start_moveit:=false \
  start_handeye_tf:=false \
  start_camera_info:=false \
  execute:=false
```

Default marker settings:

```text
marker_id: 0
dictionary: DICT_4X4_50
marker black edge: 0.047 m
cube edge: 0.055 m
image: /camera2/color/image_raw
depth: /camera2/aligned_depth_to_color/image_raw
camera_info: /camera2/color/camera_info_calibrated
```

The localization node publishes the 55 mm cube center directly:

```text
/camera2_aruco_cube/center_pose_base
```

The pick config subscribes to that topic and uses:

```text
visual_compensation_mode: absolute_offset
aruco_to_grasp_offset: [0.0, 0.0, 0.0]
pre_grasp_lift: 0.080
lift_height: 0.080
target_max_spread: 0.006
```

Face IDs:

```text
ID 0: top
ID 1: bottom
ID 2: front
ID 3: left
ID 4: back
ID 5: right
```

Detection-only localization:

```bash
roslaunch dual_arm_tasks right_arm_camera2_aruco_cube_localization.launch \
  start_cameras:=false
```

It publishes:

```text
/camera2_aruco_cube/center_pose_base
/camera2_aruco_cube/visible_ids
/camera2_aruco_cube/rviz_markers
/camera2_aruco_cube/debug_image
```

## Verified Plan

Verified on 2026-06-11 with camera2 seeing marker ID 0:

```text
center in right_arm_base: x=-0.039 y=0.421 z=0.416
sample spread: about 0.001 m
pre_grasp: x=-0.039 y=0.421 z=0.496
grasp:     x=-0.039 y=0.421 z=0.416
lift:      x=-0.039 y=0.421 z=0.496
```

The plan-only run succeeded for:

```text
home -> pre_grasp -> grasp -> lift -> pre_place -> place -> retreat -> home
```

## Real Execution

Only after checking the detected pose and plan:

```bash
roslaunch dual_arm_tasks right_arm_camera2_eye_on_base_aruco_cube_pick.launch \
  start_cameras:=false \
  start_moveit:=false \
  start_handeye_tf:=false \
  start_camera_info:=false \
  execute:=true
```

The script will ask for `EXECUTE` before moving. Use `yes:=true` only when you
intentionally want to skip the typed confirmation:

```bash
roslaunch dual_arm_tasks right_arm_camera2_eye_on_base_aruco_cube_pick.launch \
  start_cameras:=false \
  start_moveit:=false \
  start_handeye_tf:=false \
  start_camera_info:=false \
  execute:=true \
  yes:=true
```
