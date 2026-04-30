#!/usr/bin/env python
# coding=UTF-8
"""
Author: Wei Luo
Date: 2026-04-30 16:38:10
LastEditors: Wei Luo
LastEditTime: 2026-04-30 16:40:50
Note: Note
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 1. 自动构建并加载你配置包里的所有参数 (URDF, SRDF, Kinematics 等)
    # 注意：确保包名 "seer_aubo_moveit_config" 是你实际的 MoveIt 配置包名
    moveit_config = (
        MoveItConfigsBuilder("seer_aubo_stick")
        .robot_description(file_path="config/seer_aubo_composite.urdf.xacro")
        .to_moveit_configs()
    )

    # 2. 启动你刚刚写的 Python 脚本，并把参数全塞给它
    gripper_cmd_node = Node(
        package="seer_description",  # 如果你把 py 脚本放在了其他包，改这里
        executable="amr_gripper_cmd.py",  # 确保在 CMakeLists 或 setup.py 里注册了该脚本
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},  # 极其关键：继承仿真时间！
        ],
    )

    return LaunchDescription([gripper_cmd_node])
