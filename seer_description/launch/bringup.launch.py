#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-03-31 12:47:48
LastEditors: Wei Luo
LastEditTime: 2026-03-31 12:58:47
Note: Note
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import SetParameter


def generate_launch_description():
    pkg_desc = FindPackageShare("seer_description")
    pkg_moveit = FindPackageShare("seer_aubo_moveit_config")

    # 1. 设置 Gazebo 资源路径 (防止报找不到 Mesh 的错)
    model_pkg_share = get_package_share_directory("seer_description")
    workspace_share_dir = os.path.join(model_pkg_share, "..")
    set_env_action = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=workspace_share_dir
    )

    urdf_model_path = PathJoinSubstitution(
        [pkg_desc, "urdf", "composite_robot.urdf.xacro"]
    )

    # 2. 机器人状态发布者 (TF 树)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(["xacro ", urdf_model_path]),
                "use_sim_time": True,
            }
        ],
    )

    # 3. 启动 Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # 4. 在物理世界中生成复合机器人
    spawn_entity = Node(
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

    # 5. 神经系统桥接 (增加点云数据流！)
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry]gz.msgs.Odometry",
            # 【新增】桥接底层轮子的关节状态
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            # 【新增】桥接 Gazebo 内部的 TF 坐标变换 (解决 odom 丢失)
            "/model/composite_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # 桥接 IMU
            "/camera/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            # 桥接左目图像与相机内参
            "/camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            # 桥接右目图像与相机内参
            "/camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            # 将 Gazebo 的点云打通给 MoveIt
            "/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        # 将 Gazebo 专用的 tf 话题重映射到 ROS 2 的全局 /tf 话题上
        remappings=[
            ("/model/composite_robot/tf", "/tf"),
        ],
        output="screen",
    )

    # 6. 激活底层伺服控制器
    load_jsb = ExecuteProcess(
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
    load_arm_ctrl = ExecuteProcess(
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

    # ========================================== #
    # 7. 唤醒 MoveIt2 大脑 (延时 6 秒启动，等底层硬件准备好)
    # ========================================== #
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_moveit, "launch", "move_group.launch.py"])
        ),
        # 强制 MoveIt 同步 Gazebo 的物理时间
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_moveit, "launch", "moveit_rviz.launch.py"])
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    delay_moveit = TimerAction(period=6.0, actions=[move_group_launch, rviz_launch])

    set_sim_time = SetParameter(name="use_sim_time", value=True)

    return LaunchDescription(
        [
            set_sim_time,
            set_env_action,
            rsp_node,
            gazebo_launch,
            spawn_entity,
            bridge_node,
            load_jsb,
            load_arm_ctrl,
            delay_moveit,
        ]
    )
