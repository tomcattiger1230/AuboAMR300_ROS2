#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-05-06
Launch Aubo i16H robot with simplified collision geometry for RViz visualization.
Collision models use primitive shapes (cylinders/spheres) instead of STL files
for faster loading and display.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = "seer_description"
    pkg_share = FindPackageShare(package=pkg_name)

    # i16H simple URDF with primitive collision geometries
    default_model_path = PathJoinSubstitution(
        [pkg_share, "urdf", "aubo_i16H_simple.urdf.xacro"]
    )
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share, "rviz", "display_i16H_simple.rviz"]
    )

    # Launch arguments
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Path to robot URDF xacro file",
    )

    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=default_rviz_config_path,
        description="Path to RViz config file",
    )

    # Robot state publisher - parses xacro and publishes TF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": ParameterValue(
                Command(["xacro ", LaunchConfiguration("model")]),
                value_type=str
            )}
        ],
    )

    # Joint state publisher GUI - interactive joint slider control
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    # RViz2 node for 3D visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription(
        [
            model_arg,
            rviz_arg,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
