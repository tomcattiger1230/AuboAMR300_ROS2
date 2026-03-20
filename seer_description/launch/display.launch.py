#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-03-20 10:54:39
LastEditors: Wei Luo
LastEditTime: 2026-03-20 14:22:40
Note: Note
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. 定义包名
    pkg_name = "seer_description"
    pkg_share = FindPackageShare(package=pkg_name)

    # 2. 定义文件路径
    default_model_path = PathJoinSubstitution(
        [pkg_share, "urdf", "seer_robot_base.urdf.xacro"]
    )
    default_rviz_config_path = PathJoinSubstitution([pkg_share, "rviz", "display.rviz"])

    # 3. 声明 Launch 参数
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Absolute path to robot urdf file",
    )

    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=default_rviz_config_path,
        description="Absolute path to rviz config file",
    )

    # 4. 定义节点
    # robot_state_publisher 节点：调用 xacro 并发布 TF 树
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("model")])}
        ],
    )

    # joint_state_publisher_gui 节点：提供一个带有滑块的 GUI 窗口来控制关节角度
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # rviz2 节点：用于 3D 可视化
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    # 5. 返回 LaunchDescription
    return LaunchDescription(
        [
            model_arg,
            rviz_arg,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
