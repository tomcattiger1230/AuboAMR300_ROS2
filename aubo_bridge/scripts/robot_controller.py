#!/usr/bin/env python3
"""
Aubo Robot Python Controller
High-level Python interface for controlling Aubo robot via ROS2 topics.
Works with the aubo_bridge C++ node.

Usage:
    from robot_controller import RobotController
    robot = RobotController()
    robot.move_to_joints([0, 0, 0, 0, 0, 0])
    robot.execute_trajectory(waypoints)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from aubo_bridge_msgs.msg import (
    TrajectoryCommand, TrajectoryPoint, RobotEvent,
    JointStateEx, RobotStatus
)
from aubo_bridge_msgs.srv import MoveToPose, MoveToJointAngles


class RobotController(Node):
    """High-level Python controller for Aubo robot via ROS2."""

    def __init__(self, node_name='robot_controller'):
        super().__init__(node_name)

        # Publishers
        self.traj_pub_ = self.create_publisher(
            TrajectoryCommand, '/aubo/trajectory_command', 10)
        self.joint_move_pub_ = self.create_publisher(
            TrajectoryPoint, '/aubo/joint_move_command', 10)

        # Subscribers
        self.event_sub_ = self.create_subscription(
            RobotEvent, '/aubo/events', self._on_event, 10)
        self.joint_state_sub_ = self.create_subscription(
            JointStateEx, '/aubo/joint_states_ex', self._on_joint_state, 10)
        self.status_sub_ = self.create_subscription(
            RobotStatus, '/aubo/status', self._on_status, 10)

        # Service clients
        self.move_pose_client_ = self.create_client(MoveToPose, '/aubo/move_to_pose')
        self.move_joint_client_ = self.create_client(
            MoveToJointAngles, '/aubo/move_to_joint_angles')

        # State
        self.last_joint_state_ = None
        self.last_status_ = None
        self.event_history_ = []

    def _on_event(self, msg: RobotEvent):
        self.event_history_.append(msg)
        self.get_logger().info(
            f"Robot Event: type={msg.event_type}, sev={msg.severity}, {msg.description}")

    def _on_joint_state(self, msg: JointStateEx):
        self.last_joint_state_ = msg

    def _on_status(self, msg: RobotStatus):
        self.last_status_ = msg

    def move_to_joints(
        self, joint_angles: list,
        max_velocity: float = 0.5,
        max_acceleration: float = 0.5,
        timeout: float = 30.0
    ) -> bool:
        """Move robot arm to specified joint angles (radians)."""
        request = MoveToJointAngles.Request()
        request.joint_angles = joint_angles
        request.max_velocity = max_velocity
        request.max_acceleration = max_acceleration
        request.blocking = True

        self.get_logger().info(f"Moving to joints: {joint_angles}")

        future = self.move_joint_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if future.result() is None:
            self.get_logger().error("Move to joints service call failed")
            return False

        response = future.result()
        if response.success:
            self.get_logger().info("Move completed successfully")
        else:
            self.get_logger().error(f"Move failed: {response.message}")

        return response.success

    def move_to_pose(
        self, x: float, y: float, z: float,
        qx: float = 0, qy: float = 0, qz: float = 0, qw: float = 1,
        max_velocity: float = 0.5,
        timeout: float = 30.0
    ) -> bool:
        """Move robot tool center to specified Cartesian pose."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        request = MoveToPose.Request()
        request.target_pose = pose
        request.max_velocity = max_velocity
        request.blocking = True

        self.get_logger().info(f"Moving to pose: ({x}, {y}, {z})")

        future = self.move_pose_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if future.result() is None:
            self.get_logger().error("Move to pose service call failed")
            return False

        return future.result().success

    def execute_trajectory(
        self, waypoints: list,
        max_joint_velocity: float = 0.5,
        max_joint_acceleration: float = 0.5
    ) -> bool:
        """Execute a list of waypoints as a trajectory.

        Args:
            waypoints: List of joint angle arrays (6 elements each)
            max_joint_velocity: Maximum joint velocity (rad/s)
            max_joint_acceleration: Maximum joint acceleration (rad/s^2)
        """
        cmd = TrajectoryCommand()
        cmd.command = TrajectoryCommand.EXECUTE_TRAJECTORY
        cmd.max_joint_velocity = max_joint_velocity
        cmd.max_joint_acceleration = max_joint_acceleration

        for i, joints in enumerate(waypoints):
            point = TrajectoryPoint()
            point.joint_positions = joints
            point.label = f"waypoint_{i}"
            cmd.trajectory.append(point)

        self.get_logger().info(f"Executing trajectory with {len(waypoints)} waypoints")
        self.traj_pub_.publish(cmd)
        return True

    def get_current_joints(self) -> list:
        """Get current joint positions (radians)."""
        if self.last_joint_state_:
            return list(self.last_joint_state_.joint_state.position)
        return None

    def get_current_pose(self) -> dict:
        """Get current TCP pose."""
        if self.last_status_:
            return {
                'x': self.last_status_.tool_position_x,
                'y': self.last_status_.tool_position_y,
                'z': self.last_status_.tool_position_z,
            }
        return None


def main():
    rclpy.init()
    robot = RobotController()

    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
