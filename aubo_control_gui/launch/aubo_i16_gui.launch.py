"""GUI client only. Start the corresponding MoveIt backend separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='isaac', choices=['isaac', 'real']),
        DeclareLaunchArgument('enable_motion', default_value='false', description='False: plan only; true: plan and execute'),
        Node(package='aubo_control_gui', executable='aubo_control_gui', output='screen',
             parameters=[{'use_sim_time': ParameterValue(EqualsSubstitution(LaunchConfiguration('backend'),'isaac'),value_type=bool),
                          'enable_motion': ParameterValue(LaunchConfiguration('enable_motion'),value_type=bool)}]),
    ])
