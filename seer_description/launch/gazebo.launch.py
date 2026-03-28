#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-03-20 15:35:47
LastEditors: Wei Luo
LastEditTime: 2026-03-27 14:12:43
Note: Note
"""


import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import AppendEnvironmentVariable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "seer_description"
    pkg_share = FindPackageShare(pkg_name)
    # 找到拥有 meshes 文件的那个包的路径 (注意替换为你实际报错的包名，比如 seer_description)
    model_pkg_share = get_package_share_directory(
        pkg_name
    )  # 这里假设 meshes 文件就在 seer_description 包里，如果在其他包里请替换 pkg_name

    # 获取这个包的上一级目录 (即 install/<pkg_name>/share 目录)
    # 因为 Gazebo 看到 model://seer_description 时，会在这个目录下寻找名为 seer_description 的文件夹
    workspace_share_dir = os.path.join(model_pkg_share, "..")

    # 创建环境变量注入动作
    set_env_action = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=workspace_share_dir
    )
    urdf_model_path = PathJoinSubstitution(
        [pkg_share, "urdf", "seer_robot_base.urdf.xacro"]
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

    # 2. 启动新版 Gazebo (加载一个空世界)
    ros_gz_sim_pkg = FindPackageShare("ros_gz_sim")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_pkg, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf"
        }.items(),  # -r 表示启动后立刻开始运行仿真
    )

    # 3. 在新版 Gazebo 中生成模型 (可执行文件变成了 create)
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "seer_amb", "-topic", "robot_description", "-z", "0.1"],
        output="screen",
    )

    # 4. 【核心】设置 ros_gz_bridge 桥接话题
    # 将 Gazebo 的 cmd_vel 桥接到 ROS2 的 /cmd_vel，类型为 geometry_msgs/msg/Twist
    bridge_params = os.path.join(
        FindPackageShare(pkg_name).find(pkg_name),
        "config",
        "ros_gz_bridge.yaml",  # 如果话题多，最好用 yaml 配置，但这里为了简便我们直接写参数
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # 控制与底盘状态
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
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
        ]
    )
