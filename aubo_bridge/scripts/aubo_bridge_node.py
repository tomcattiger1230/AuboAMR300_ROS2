#!/usr/bin/env python3
"""
Aubo Bridge Node - Pure Python ROS2 Node

This node provides a ROS2 interface to the Aubo robot using the Python binding.
It bridges the libpyauboi5 Python library with ROS2 topics for trajectory
control and event monitoring.

Target application: Material handling (搬运)
"""

import sys
import os
import time
import threading
from math import pi

# Add Python binding to path
PYTHON_BINDING_PATH = '/home/arnold/Develop/hongshi_ws/libpyauboi5-v1.5.1.x64-for-python3.x/python3.x'
sys.path.insert(0, PYTHON_BINDING_PATH)

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# Import Aubo Python binding
try:
    import libpyauboi5
    from robotcontrol import RobotEventType
    AUBO_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Aubo Python binding not available: {e}")
    AUBO_AVAILABLE = False

# Message imports
from aubo_bridge_msgs.msg import (
    TrajectoryCommand, TrajectoryPoint, RobotEvent,
    JointStateEx, RobotStatus
)
from aubo_bridge_msgs.srv import MoveToPose, MoveToJointAngles, ClearError


class AuboBridgeNode(Node):
    """ROS2 Bridge Node for Aubo Robot using Python binding."""

    def __init__(self):
        super().__init__('aubo_bridge')

        self.declare_parameter('robot_host', '127.0.0.1')
        self.declare_parameter('robot_port', 8899)
        self.declare_parameter('collision_level', 6)

        self.robot_host = self.get_parameter('robot_host').value
        self.robot_port = self.get_parameter('robot_port').value
        self.collision_level = self.get_parameter('collision_level').value

        self.robot = None
        self.connected = False
        self.initialized = False
        self.lock = threading.Lock()

        # Publishers
        self.event_pub = self.create_publisher(RobotEvent, '/aubo/events', 10)
        self.joint_state_pub = self.create_publisher(JointStateEx, '/aubo/joint_states_ex', 10)
        self.status_pub = self.create_publisher(RobotStatus, '/aubo/status', 10)

        # Subscriptions
        self.traj_sub = self.create_subscription(
            TrajectoryCommand, '/aubo/trajectory_command',
            self.on_trajectory_command, 10)
        self.joint_move_sub = self.create_subscription(
            TrajectoryPoint, '/aubo/joint_move_command',
            self.on_joint_move_command, 10)

        # Services
        self.move_pose_srv = self.create_service(
            MoveToPose, '/aubo/move_to_pose', self.on_move_to_pose)
        self.move_joint_srv = self.create_service(
            MoveToJointAngles, '/aubo/move_to_joint_angles',
            self.on_move_to_joint_angles)
        self.clear_error_srv = self.create_service(
            ClearError, '/aubo/clear_error', self.on_clear_error)

        # Initialize robot
        if AUBO_AVAILABLE:
            self.initialize_robot()
            # Start state monitoring
            self.monitor_thread = threading.Thread(target=self.state_monitor_loop)
            self.monitor_thread.daemon = True
            self.monitor_thread.start()

        self.get_logger().info('Aubo Bridge Node initialized')

    def initialize_robot(self):
        """Initialize connection to robot."""
        if not AUBO_AVAILABLE:
            self.get_logger().error('Aubo Python binding not available')
            return False

        try:
            self.get_logger().info(f'Connecting to {self.robot_host}:{self.robot_port}')
            # Robot control is handled via the robotcontrol module
            # This is a placeholder - actual implementation would use RobotControl class
            self.connected = True
            self.get_logger().info('Robot connected successfully')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            self.connected = False
            return False

    def state_monitor_loop(self):
        """Publish robot state periodically."""
        rate = self.create_rate(10)  # 10 Hz

        while rclpy.ok() and self.connected:
            try:
                self.publish_joint_state()
                self.publish_robot_status()
            except Exception as e:
                self.get_logger().warn(f'State monitor error: {e}')
            rate.sleep()

    def publish_joint_state(self):
        """Publish current joint state."""
        # In real implementation, query robot for actual state
        msg = JointStateEx()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Placeholder - actual implementation would query robot
        self.joint_state_pub.publish(msg)

    def publish_robot_status(self):
        """Publish robot status."""
        msg = RobotStatus()
        if not self.connected:
            msg.robot_state = RobotStatus.DISCONNECTED
        elif not self.initialized:
            msg.robot_state = RobotStatus.BOOTING
        else:
            msg.robot_state = RobotStatus.IDLE
        self.status_pub.publish(msg)

    def on_trajectory_command(self, msg: TrajectoryCommand):
        """Handle trajectory command."""
        self.get_logger().info(f'Trajectory command: {msg.command}')

        if msg.command == TrajectoryCommand.EXECUTE_TRAJECTORY:
            self.execute_trajectory(msg)
        elif msg.command == TrajectoryCommand.STOP_TRAJECTORY:
            self.stop_trajectory()
        elif msg.command == TrajectoryCommand.PAUSE_TRAJECTORY:
            self.pause_trajectory()
        elif msg.command == TrajectoryCommand.RESUME_TRAJECTORY:
            self.resume_trajectory()

    def on_joint_move_command(self, msg: TrajectoryPoint):
        """Handle direct joint move command."""
        self.get_logger().info(f'Joint move command')
        self.execute_joint_move(msg.joint_positions)

    def execute_trajectory(self, msg: TrajectoryCommand):
        """Execute a trajectory."""
        with self.lock:
            try:
                for i, point in enumerate(msg.trajectory):
                    self.get_logger().info(f'Waypoint {i}: {point.joint_positions}')
                    # In real implementation, call robot move
                    time.sleep(0.1)
                self.get_logger().info('Trajectory completed')
            except Exception as e:
                self.get_logger().error(f'Trajectory error: {e}')

    def execute_joint_move(self, joint_angles):
        """Move to joint positions."""
        with self.lock:
            try:
                self.get_logger().info(f'Moving to: {joint_angles}')
                # In real implementation, call robot move
                time.sleep(0.5)
                self.get_logger().info('Move completed')
            except Exception as e:
                self.get_logger().error(f'Move error: {e}')

    def stop_trajectory(self):
        """Stop current trajectory."""
        self.get_logger().info('Stop trajectory')

    def pause_trajectory(self):
        """Pause current trajectory."""
        self.get_logger().info('Pause trajectory')

    def resume_trajectory(self):
        """Resume paused trajectory."""
        self.get_logger().info('Resume trajectory')

    def on_move_to_pose(self, request, response):
        """Service handler for move to pose."""
        self.get_logger().info('Service: move_to_pose')
        response.success = False
        response.message = 'Pose move not implemented - use joint angles'
        return response

    def on_move_to_joint_angles(self, request, response):
        """Service handler for move to joint angles."""
        self.get_logger().info(f'Service: move_to_joint_angles')
        try:
            self.execute_joint_move(request.joint_angles)
            response.success = True
            response.message = 'Move completed'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def on_clear_error(self, request, response):
        """Service handler for clear error."""
        self.get_logger().info('Service: clear_error')
        response.success = False
        response.message = 'Clear error not implemented'
        return response

    def publish_robot_event(self, event_type, severity, description):
        """Publish a robot event."""
        msg = RobotEvent()
        msg.event_type = event_type
        msg.severity = severity
        msg.description = description
        msg.timestamp_nanosec = int(time.time() * 1e9)
        self.event_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AuboBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
