#!/usr/bin/env python3
"""
Aubo Bridge ROS2 Launch File
Launches the Aubo robot bridge node for material handling applications.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aubo_bridge',
            executable='aubo_bridge_node',
            name='aubo_bridge',
            output='screen',
            parameters=[{
                'robot_host': '127.0.0.1',
                'robot_port': 8899,
            }],
            remappings=[
                ('/aubo/trajectory_command', '/aubo/trajectory_command'),
                ('/aubo/joint_move_command', '/aubo/joint_move_command'),
                ('/aubo/events', '/aubo/events'),
                ('/aubo/joint_states_ex', '/aubo/joint_states_ex'),
                ('/aubo/status', '/aubo/status'),
                ('/joint_states', '/joint_states'),
            ],
        ),
    ])
