"""Static map + AMCL for Isaac. Stop SLAM before launching this stack."""
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('seer_description'), 'config', 'isaac_localization.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('map', description='YAML absolute path on the ROS host'),
        Node(package='seer_description', executable='scan_filter.py', prefix=sys.executable,
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'use_sim_time': True, 'yaml_filename': LaunchConfiguration('map')}], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl', parameters=[params], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='localization_lifecycle_manager', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server','amcl']}]),
    ])
