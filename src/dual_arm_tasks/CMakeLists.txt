cmake_minimum_required(VERSION 3.0.2)
project(dual_arm_tasks)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  moveit_commander
  moveit_msgs
  rospy
  robotiq_2f_gripper_control
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  ur_msgs
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/red_object_detector.py
  scripts/red_block_locator_kinect.py
  scripts/red_block_locator_realsense.py
  scripts/transform_red_object.py
  scripts/hsv_object_to_base.py
  scripts/sync_hsv_cube_scene.py
  scripts/calibrate_table_from_live_cube.py
  scripts/right_arm_hsv_approach_test.py
  scripts/publish_camera_link_tf.py
  scripts/red_object_marker.py
  scripts/robotiq_2f_joint_state_visualizer.py
  scripts/activate_right_gripper.py
  scripts/right_arm_gripper_control.py
  scripts/checkerboard_pose_publisher.py
  scripts/blue_blocks_checkerboard_perception.py
  scripts/validate_locked_blue_targets.py
  scripts/right_arm_jacquard_click_pick.py
  scripts/right_arm_visual_pick.py
  scripts/right_arm_pick_readiness.py
  scripts/single_arm_pick_place.py
  scripts/visual_aruco_pick_place.py
  scripts/static_joint_state_publisher.py
  scripts/verify_visual_grasp_against_yaml.py
  scripts/debug_kinect_aruco_once.py
  scripts/hsv_threshold_tuner.py
  scripts/right_arm_markerless_sequence_pick.py
  scripts/right_arm_three_blue_pick_place.py
  scripts/simple_pick_red_block.py
  scripts/simple_pick_red_block_eye_in_hand.py
  scripts/vision_biased_fixed_pick_place.py
  scripts/yolo_red_block_locator.py
  scripts/yolo_red_block_pick.py
  scripts/capture_red_block_yolo_dataset.py
  scripts/capture_kinect_graspnet_depth.py
  scripts/check_graspnet_capture_pairs.py
  scripts/train_public_colored_blocks_yolo.py
  scripts/camera_info_from_yaml_publisher.py
  scripts/handeye_eye_on_base_lie_optimizer.py
  scripts/camera2_aruco_cube_pose.py
  scripts/set_realsense_color_manual.py
  scripts/publish_real_environment_scene.py
  scripts/verify_real_workcell.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config docs launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
