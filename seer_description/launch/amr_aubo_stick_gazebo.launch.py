#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-04-30 10:05:50
LastEditors: Wei Luo
LastEditTime: 2026-04-30 11:15:51
Note: Note
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction


def generate_launch_description():
    pkg_name = "seer_description"
    pkg_share = FindPackageShare(pkg_name)
    pkg_moveit = FindPackageShare("seer_aubo_stick_moveit_config")

    # 解决 Gazebo 模型加载路径问题
    model_pkg_share = get_package_share_directory(pkg_name)
    workspace_share_dir = os.path.join(model_pkg_share, "..")
    set_env_action = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=workspace_share_dir
    )

    # 指向我们融合后的总装 URDF
    urdf_model_path = PathJoinSubstitution(
        [pkg_share, "urdf", "composite_robot_stick.urdf.xacro"]
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
            "composite_robot_stick",
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
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            # "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/model/composite_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # 桥接 IMU
            "/camera/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            # 桥接左目图像与相机内参
            "/camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            # 桥接右目图像与相机内参
            "/camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/model/composite_robot/tf", "/tf"),
        ],
        output="screen",
    )

    # # 5. 激活 ros2_control 状态广播器
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["aubo_arm_controller"],
    )

    # 6. 激活机械臂轨迹控制器
    # 终极防御机制：事件锁！
    # 监听 spawn_entity_node，只有当它成功把机器人放入世界并退出后，才允许加载控制器
    delay_controllers = TimerAction(
        period=8.0, actions=[load_joint_state_broadcaster, load_arm_controller]
    )

    # 7. 唤醒 MoveIt2 大脑 (必须开启仿真时间)
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_moveit, "launch", "move_group.launch.py"])
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # 8. 启动 RViz2 视觉界面
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_moveit, "launch", "moveit_rviz.launch.py"])
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            set_env_action,
            robot_state_publisher_node,
            gazebo_launch,
            spawn_entity_node,
            bridge_node,
            delay_controllers,
            move_group_launch,
            rviz_launch,
        ]
    )
