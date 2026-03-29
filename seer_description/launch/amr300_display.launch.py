#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-03-29 21:07:40
LastEditors: Wei Luo
LastEditTime: 2026-03-29 21:07:42
Note: Note
"""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # 提取 Launch 参数
    model = LaunchConfiguration("model")
    use_joint_state_gui = LaunchConfiguration("use_joint_state_gui")
    rvizconfig = LaunchConfiguration("rvizconfig")

    # 使用 xacro 命令解析复合模型文件
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            model,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 1. 机器人状态发布者 (核心：发布 TF 树)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 2. 关节状态发布者 (无 GUI 模式，当 use_joint_state_gui 为 false 时启动)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="log",
        condition=UnlessCondition(use_joint_state_gui),
        parameters=[robot_description],
    )

    # 3. 关节状态发布者 (带 GUI 滑块模式，当 use_joint_state_gui 为 true 时启动)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="log",
        condition=IfCondition(use_joint_state_gui),
        parameters=[robot_description],
    )

    # 4. RViz2 节点
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rvizconfig],
        parameters=[robot_description],  # 把模型参数传给 rviz2 以便正确解析材质
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]


def generate_launch_description():
    pkg_name = "seer_description"
    pkg_share = FindPackageShare(package=pkg_name)

    # 声明 Launch 参数，默认指向我们新写的复合机器人 URDF
    declared_arguments = [
        DeclareLaunchArgument(
            "model",
            default_value=PathJoinSubstitution(
                [pkg_share, "urdf", "composite_robot.urdf.xacro"]
            ),
            description="Absolute path to the composite robot urdf (xacro) file",
        ),
        DeclareLaunchArgument(
            "use_joint_state_gui",
            default_value="true",
            description="Whether to start joint_state_publisher_gui instead of joint_state_publisher.",
        ),
        DeclareLaunchArgument(
            "rvizconfig",
            default_value=PathJoinSubstitution([pkg_share, "rviz", "display.rviz"]),
            description="Absolute path to rviz config file",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
