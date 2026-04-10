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
    meshes urdf launch rviz scripts config
  DESTINATION share/${PROJECT_NAME}/
)

# 将 Python 脚本安装到 lib/ 目录下，这样 ros2 run 才能找到它
install(PROGRAMS
  scripts/keyboard_teleop.py
  scripts/circle_move.py
  scripts/action_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
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
