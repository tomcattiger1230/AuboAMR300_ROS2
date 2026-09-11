from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("seer_aubo_composite", package_name="seer_aubo_finger_mono_moveit_config").to_moveit_configs()
    generated = generate_move_group_launch(moveit_config)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        *generated.entities,
    ])
