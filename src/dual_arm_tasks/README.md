# Dual Arm Tasks

Start with a guarded left-arm pick-and-place cycle. The default configuration only plans trajectories.

```bash
source /home/gzu/gzu_ws/devel/setup.bash
rosrun dual_arm_tasks single_arm_pick_place.py \
  --config /home/gzu/gzu_ws/src/dual_arm_tasks/config/left_arm_pick_place.yaml
```

Inspect every pose in RViz. Measure the real pick and place coordinates, update the YAML file, and set
`enabled: true` only after the plan-only run succeeds. Real execution requires an additional flag and a
typed confirmation:

```bash
rosrun dual_arm_tasks single_arm_pick_place.py \
  --config /home/gzu/gzu_ws/src/dual_arm_tasks/config/left_arm_pick_place.yaml \
  --execute
```

## Right Arm Visual Red Block Pick

The hand-eye calibration result is reused from:

```text
/home/gzu/.ros/easy_handeye/ur3_right_realsense_handeyecalibration_eye_on_base.yaml
```

This is currently identical to:

```text
/home/gzu/gzu_ws/ur3_right_realsense_handeyecalibration_eye_on_base.yaml
```

Current gripper serial mapping for this workcell, rechecked on 2026-06-04:

```text
DA6ACN7O (/dev/ttyUSB1): left-arm gripper
DA6ACQ6P (/dev/ttyUSB0): right-arm gripper adapter, but no Robotiq status was read during the check
```

Do not use `/dev/ttyUSB1` for the right gripper. It was observed to close the physical left gripper.
Prefer the stable by-id path for the right gripper:

```bash
/dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DA6ACQ6P-if00-port0
```

For camera + right gripper testing without connecting the UR3 arm:

```bash
source /home/gzu/gzu_ws/devel/setup.bash
roslaunch dual_arm_tasks right_arm_vision_gripper.launch
```

`right_arm_vision_gripper.launch` does not start either UR3 arm or MoveIt. For a real right-arm pick, first
make sure the workcell is clear, E-stop is reachable, both UR controllers are reachable, the right pendant
is running External Control, and the right Robotiq responds on `DA6ACQ6P`.
Then start:

```bash
source /home/gzu/gzu_ws/devel/setup.bash
roslaunch dual_arm_tasks right_arm_pick_system.launch \
  start_rviz:=true \
  right_gripper_device:=/dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DA6ACQ6P-if00-port0 \
  moveit_controller_manager:=simple_right
```

This launch starts the RealSense, dual UR3 driver, MoveIt, the calibrated camera TF, red-object detection,
camera-to-base conversion, the right Robotiq 2F RTU node, and a right-gripper activation sequence
(`rACT=0`, then `rACT=1` opened).

Check the important outputs:

```bash
roslaunch dual_arm_tasks right_arm_pick_readiness.launch
```

This must report no `[FAIL]` lines. In particular, start the External Control program on the right teach
pendant if it reports that right UR External Control is not running.

If you are intentionally testing without the right gripper or without MoveIt, use
`skip_gripper:=true` or `skip_moveit:=true`.

Plan the visual pick. This plans `pre_grasp -> grasp -> lift -> shift_right` in RViz, but does not move the
robot or gripper:

```bash
roslaunch dual_arm_tasks right_arm_visual_pick.launch
```

For the first real attempt, move to hover and stop:

```bash
roslaunch dual_arm_tasks right_arm_visual_pick.launch phase:=approach execute:=true
```

Check the hover position. If it is correct, continue from hover to grasp, close, lift 30 mm, and shift
20 mm toward the right-arm side:

```bash
roslaunch dual_arm_tasks right_arm_visual_pick.launch phase:=grasp execute:=true
```

After the staged cycle is repeatedly accurate, one command performs the complete cycle:

```bash
roslaunch dual_arm_tasks right_arm_visual_pick.launch execute:=true
```

For the current hand-eye residual issue, first tune `target_offset` and `tool_to_grasp_center` with small guarded trials. Do not run reinforcement learning directly on the real arm until the residual search space, safety limits, and reward signal have been validated from these trials.

To back away from the block while keeping the gripper open:

```bash
roslaunch dual_arm_tasks right_arm_visual_pick.launch phase:=retreat execute:=true
```

## Left Arm Hand-Eye Calibration

The RealSense is fixed in the workcell, so this remains an eye-on-base calibration. Use the left MoveIt group
and the left tool frame:

```bash
source /home/gzu/gzu_ws/devel/setup.bash
roslaunch dual_arm_tasks calibrate_left_arm_camera.launch
```

The saved result will be:

```text
/home/gzu/.ros/easy_handeye/ur3_left_realsense_handeyecalibration_eye_on_base.yaml
```

After saving, publish the left calibration TF for checks with:

```bash
roslaunch dual_arm_tasks publish_left_arm_camera_tf.launch
```
