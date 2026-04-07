#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-04-03 21:43:44
LastEditors: Wei Luo
LastEditTime: 2026-04-07 16:40:24
Note: Note
"""


import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
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

    model_pkg_share = get_package_share_directory("seer_description")
    workspace_share_dir = os.path.join(model_pkg_share, "..")
    set_env_action = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=workspace_share_dir
    )

    urdf_model_path = PathJoinSubstitution(
        [pkg_desc, "urdf", "composite_robot.urdf.xacro"]
    )

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

    # 唤醒 MoveIt2 大脑
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_moveit, "launch", "move_group.launch.py"])
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_moveit, "launch", "moveit_rviz.launch.py"])
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    delay_moveit = TimerAction(period=3.0, actions=[move_group_launch, rviz_launch])

    set_sim_time = SetParameter(name="use_sim_time", value=True)

    return LaunchDescription(
        [
            set_sim_time,
            set_env_action,
            rsp_node,
            delay_moveit,
        ]
    )
