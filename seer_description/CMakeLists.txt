cmake_minimum_required(VERSION 3.8)
project(seer_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 找到所有需要的目录并将它们安装到 share 目录下
install(
  DIRECTORY
    meshes urdf launch rviz scripts config maps
  DESTINATION share/${PROJECT_NAME}/
)

# 将 Python 脚本安装到 lib/ 目录下，这样 ros2 run 才能找到它
install(PROGRAMS
  scripts/scan_filter.py
  scripts/camera_video.py
  scripts/camera_mono.py
  scripts/generate_mono_camera_usd.py
  scripts/generate_mono_robot_models.py
  scripts/generate_finger_gripper_usd.py
  scripts/generate_finger_robot_models.py
  scripts/test_mono_camera.py
  scripts/start_warehouse_stick_mono_demo.sh
  scripts/start_warehouse_finger_mono_demo.sh
  scripts/cmd_vel_watchdog.py
  scripts/test_isaac_navigation.py
  scripts/test_isaac_arm.py
  scripts/test_isaac_mapping.py
  scripts/keyboard_teleop.py
  scripts/circle_move.py
  scripts/action_bridge.py
  scripts/generate_warehouse_scene.py
  scripts/isaac_ros2_runner.py
  scripts/mobile_manipulator_control.py
  scripts/start_isaac_ros2_stack.sh
  scripts/start_warehouse_demo.sh
  scripts/start_warehouse_stick_demo.sh
  scripts/amr_gripper_cmd.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  README_ISAAC_ROS2.md
  ISAAC_NAVIGATION_CAMERA.md
  README_MONO_CAMERA.md
  README_FINGER_GRIPPER.md
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY test/results DESTINATION share/${PROJECT_NAME}/test)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
