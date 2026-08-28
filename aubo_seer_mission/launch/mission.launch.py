from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_yaml = os.path.join(
        get_package_share_directory("aubo_seer_mission"), "config", "mission.yaml"
    )

    mission_yaml_arg = DeclareLaunchArgument(
        "mission_yaml",
        default_value=default_yaml,
        description="Path to the mission configuration YAML",
    )

    mission_node = Node(
        package="aubo_seer_mission",
        executable="mission_node",
        name="mission_node",
        output="screen",
        parameters=[{"mission_yaml": LaunchArgument("mission_yaml")}],
    )

    return LaunchDescription([mission_yaml_arg, mission_node])
