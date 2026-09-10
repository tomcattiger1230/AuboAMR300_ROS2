"""Start Nav2 after isaac_mapping has produced a map and map->odom TF."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    share = FindPackageShare('seer_description')
    params = PathJoinSubstitution([share, 'config', 'isaac_nav2.yaml'])
    nodes = []
    for package, executable in [('nav2_planner', 'planner_server'),
                                ('nav2_controller', 'controller_server'),
                                ('nav2_bt_navigator', 'bt_navigator')]:
        extra = {'default_nav_to_pose_bt_xml': PathJoinSubstitution([
            share, 'config', 'isaac_navigate.xml'])} if executable == 'bt_navigator' else {}
        nodes.append(Node(package=package, executable=executable, name=executable,
                          output='screen', parameters=[params, extra]))
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='navigation_lifecycle_manager', output='screen',
                      parameters=[{'use_sim_time': True, 'autostart': True,
                                   'node_names': ['planner_server', 'controller_server', 'bt_navigator']}]))
    return LaunchDescription(nodes)
