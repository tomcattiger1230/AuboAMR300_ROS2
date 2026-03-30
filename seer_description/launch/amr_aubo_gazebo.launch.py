#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-03-30 16:30:45
LastEditors: Wei Luo
LastEditTime: 2026-03-30 16:48:49
Note: Note
"""
#!/usr/bin/env python3
# coding=UTF-8

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "seer_description"
    pkg_share = FindPackageShare(pkg_name)

    # 解决 Gazebo 模型加载路径问题
    model_pkg_share = get_package_share_directory(pkg_name)
    workspace_share_dir = os.path.join(model_pkg_share, "..")
    set_env_action = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=workspace_share_dir
    )

    # 指向我们融合后的总装 URDF
    urdf_model_path = PathJoinSubstitution(
        [pkg_share, "urdf", "composite_robot.urdf.xacro"]
    )

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(["xacro ", urdf_model_path]),
                "use_sim_time": True,
            }
        ],
    )

    # 2. 启动 Gazebo (加载空世界)
    ros_gz_sim_pkg = FindPackageShare("ros_gz_sim")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_pkg, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # 3. 在 Gazebo 中生成模型
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "composite_robot",
            "-topic",
            "robot_description",
            "-z",
            "0.1",
        ],
        output="screen",
    )

    # 4. 启动 ros_gz_bridge (打通底盘控制和雷达)
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # 引入仿真时钟（极其关键！）
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            # 注意：这里去掉了 joint_states 桥接，因为机械臂的关节状态由 ros2_control 接管发布了
        ],
        output="screen",
    )

    # 5. 激活 ros2_control 状态广播器
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    # 6. 激活机械臂轨迹控制器
    load_arm_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "aubo_arm_controller",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            set_env_action,
            robot_state_publisher_node,
            gazebo_launch,
            spawn_entity_node,
            bridge_node,
            load_joint_state_broadcaster,
            load_arm_controller,
        ]
    )
