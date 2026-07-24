#!/usr/bin/env python3
# coding=UTF-8
"""Launch MoveIt and the trajectory bridge for an Isaac Sim articulation."""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_desc = FindPackageShare("seer_description")
    pkg_moveit = FindPackageShare("seer_aubo_moveit_config")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    action_name = LaunchConfiguration("action_name")
    command_topic = LaunchConfiguration("command_topic")

    urdf_model_path = PathJoinSubstitution(
        [pkg_desc, "urdf", "composite_robot.urdf.xacro"]
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(["xacro ", urdf_model_path]),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # 唤醒 MoveIt2 大脑
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_moveit, "launch", "move_group.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_moveit, "launch", "moveit_rviz.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(start_rviz),
    )

    delay_moveit = TimerAction(period=3.0, actions=[move_group_launch, rviz_launch])

    start_action_bridge = Node(
        package="seer_description",
        executable="action_bridge.py",
        parameters=[
            {
                "action_name": action_name,
                "command_topic": command_topic,
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument(
                "action_name",
                default_value=(
                    "/aubo_arm_controller_wo_gripper/follow_joint_trajectory"
                ),
            ),
            DeclareLaunchArgument(
                "command_topic", default_value="/isaac_joint_commands"
            ),
            rsp_node,
            start_action_bridge,
            delay_moveit,
        ]
    )
