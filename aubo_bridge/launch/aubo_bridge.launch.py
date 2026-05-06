#!/usr/bin/env python3
"""
Aubo Bridge ROS2 Launch File
Launches the Aubo robot bridge node for material handling applications.

Starts the pure Python aubo_bridge_node and exposes both service and msg-based
control topics:
  /aubo/get_robot_info
  /aubo/move_joint
  /aubo/move_line
  /aubo/joint_move_command
  /aubo/line_move_command
  /aubo/trajectory_command
  /aubo/info_command

This launch version also exposes quaternion validation parameters used by
aubo_bridge_node.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 函数说明：生成 launch 描述：声明所有可调参数并启动 aubo_bridge_node.py。
def generate_launch_description():
    # LaunchConfiguration 只是读取 launch 参数的“占位对象”，
    # 真正的默认值在下面 DeclareLaunchArgument 中定义。
    robot_host = LaunchConfiguration("robot_host")
    robot_port = LaunchConfiguration("robot_port")
    collision_level = LaunchConfiguration("collision_level")
    state_publish_rate = LaunchConfiguration("state_publish_rate")
    startup_robot = LaunchConfiguration("startup_robot")
    default_joint_acc = LaunchConfiguration("default_joint_acc")
    default_joint_vel = LaunchConfiguration("default_joint_vel")
    default_line_acc = LaunchConfiguration("default_line_acc")
    default_line_vel = LaunchConfiguration("default_line_vel")
    auto_normalize_quaternion = LaunchConfiguration("auto_normalize_quaternion")
    quat_norm_tolerance = LaunchConfiguration("quat_norm_tolerance")

    return LaunchDescription(
        [
            # 控制柜 IP：根据实际机械臂控制器地址修改。
            DeclareLaunchArgument(
                "robot_host",
                default_value="192.168.3.250",
                description="Aubo controller IP address",
            ),
            DeclareLaunchArgument(
                "robot_port",
                default_value="8899",
                description="Aubo controller port",
            ),
            DeclareLaunchArgument(
                "collision_level",
                default_value="6",
                description="Robot startup collision level",
            ),
            DeclareLaunchArgument(
                "state_publish_rate",
                default_value="10.0",
                description="State publishing rate in Hz",
            ),
            DeclareLaunchArgument(
                "startup_robot",
                default_value="true",
                description="Call robot_startup() during bridge initialization",
            ),
            # topic 关节运动默认速度/加速度：TrajectoryPoint.msg 没有速度字段时使用这里的值。
            DeclareLaunchArgument(
                "default_joint_acc",
                default_value="0.5",
                description="Default joint command acceleration for topic commands",
            ),
            DeclareLaunchArgument(
                "default_joint_vel",
                default_value="0.5",
                description="Default joint command velocity for topic commands",
            ),
            # topic 直线运动默认速度/加速度：line_move_command 使用这里的值。
            DeclareLaunchArgument(
                "default_line_acc",
                default_value="0.3",
                description="Default line command acceleration for topic commands",
            ),
            DeclareLaunchArgument(
                "default_line_vel",
                default_value="0.3",
                description="Default line command velocity for topic commands",
            ),
            # 四元数保护参数：防止非单位四元数直接进入 IK。
            DeclareLaunchArgument(
                "auto_normalize_quaternion",
                default_value="true",
                description="Normalize non-unit input quaternions before IK/motion",
            ),
            DeclareLaunchArgument(
                "quat_norm_tolerance",
                default_value="0.001",
                description="Allowed quaternion norm error before normalization/warning",
            ),
            # 真正启动 aubo_bridge_node.py，并把上面声明的 launch 参数传给节点。
            Node(
                package="aubo_bridge",
                executable="aubo_bridge_node.py",
                name="aubo_bridge",
                output="screen",
                parameters=[
                    {
                        "robot_host": robot_host,
                        "robot_port": robot_port,
                        "collision_level": collision_level,
                        "state_publish_rate": state_publish_rate,
                        "startup_robot": startup_robot,
                        "default_joint_acc": default_joint_acc,
                        "default_joint_vel": default_joint_vel,
                        "default_line_acc": default_line_acc,
                        "default_line_vel": default_line_vel,
                        "auto_normalize_quaternion": auto_normalize_quaternion,
                        "quat_norm_tolerance": quat_norm_tolerance,
                    }
                ],
                # 这里保持同名 remap，主要是把所有公开 topic 集中列出来，便于后续改名。
                remappings=[
                    ("/aubo/trajectory_command", "/aubo/trajectory_command"),
                    ("/aubo/joint_move_command", "/aubo/joint_move_command"),
                    ("/aubo/line_move_command", "/aubo/line_move_command"),
                    ("/aubo/events", "/aubo/events"),
                    ("/aubo/joint_states_ex", "/aubo/joint_states_ex"),
                    ("/aubo/status", "/aubo/status"),
                    ("/aubo/info_command", "/aubo/info_command"),
                ],
            ),
        ]
    )
