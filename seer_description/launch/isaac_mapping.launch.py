"""SLAM and wrist video on top of the running Isaac robot stack."""
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('video_port', default_value='8080'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        Node(package='seer_description', executable='scan_filter.py', output='screen', prefix=sys.executable,
             parameters=[{'use_sim_time': True}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
            launch_arguments={'use_sim_time': 'true', 'slam_params_file': PathJoinSubstitution([
                FindPackageShare('seer_description'), 'config', 'isaac_slam.yaml'])}.items()),
        Node(package='seer_description', executable='camera_video.py', output='screen', prefix=sys.executable,
             parameters=[{'host': LaunchConfiguration('video_host'),
                          'port': LaunchConfiguration('video_port')}]),
        Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': True}],
             arguments=['-d', PathJoinSubstitution([FindPackageShare('seer_description'),
                                                   'rviz', 'isaac_navigation.rviz'])],
             condition=IfCondition(LaunchConfiguration('start_rviz'))),
    ])
