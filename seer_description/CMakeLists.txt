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
  scripts/start_warehouse_finger_etm6m_demo.sh
  scripts/start_warehouse_finger_etm6m_rebar_tester_demo.sh
  scripts/start_warehouse_finger_rebar_lab_demo.sh
  scripts/start_warehouse_stick_rebar_mono_demo.sh
  scripts/start_warehouse_finger_rebar_mono_demo.sh
  scripts/generate_rebar_station.py
  scripts/generate_rebar_visual.py
  scripts/test_rebar_grasp.py
  scripts/measure_rebar_settling.py
  scripts/rebar_loading_scene.py
  scripts/rebar_tester_bridge.py
  scripts/start_warehouse_finger_rebar_loading_demo.sh
  scripts/cmd_vel_watchdog.py
  scripts/test_isaac_navigation.py
  scripts/test_isaac_arm.py
  scripts/test_isaac_mapping.py
  scripts/keyboard_teleop.py
  scripts/circle_move.py
  scripts/action_bridge.py
  scripts/generate_warehouse_scene.py
  scripts/generate_etm6m_scene.py
  scripts/generate_rebar_test_machine_scene.py
  scripts/generate_rebar_lab_scene.py
  scripts/isaac_ros2_runner.py
  scripts/mobile_manipulator_control.py
  scripts/start_isaac_ros2_stack.sh
  scripts/start_warehouse_demo.sh
  scripts/start_warehouse_stick_demo.sh
  scripts/amr_gripper_cmd.py
  scripts/rebar_tester_geometry.py
  scripts/rebar_tester_gui.py
  scripts/rebar_tester_gui_qt.py
  scripts/test_rebar_tester_load.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/rebar_experiment_geometry.py scripts/rebar_tester_control.py DESTINATION lib/${PROJECT_NAME})

install(FILES
  README_ISAAC_ROS2.md
  ISAAC_NAVIGATION_CAMERA.md
  README_MONO_CAMERA.md
  README_FINGER_GRIPPER.md
  README_REBAR_GRASP.md
  README_ETM6M.md
  README_REBAR_TEST_MACHINE.md
  README_REBAR_LAB.md
  README_REBAR_TESTER_LOAD.md
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
