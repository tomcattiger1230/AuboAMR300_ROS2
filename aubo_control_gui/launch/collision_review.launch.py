from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target_name',default_value='存储位 1'),
        Node(package='aubo_control_gui',executable='show_storage_collision.py',output='screen',
             parameters=[{'target_name':LaunchConfiguration('target_name')}]),
        Node(package='rviz2',executable='rviz2',name='storage_collision_rviz',output='screen',
             arguments=['-d',PathJoinSubstitution([FindPackageShare('aubo_control_gui'),'rviz','storage_collision.rviz'])]),
    ])
