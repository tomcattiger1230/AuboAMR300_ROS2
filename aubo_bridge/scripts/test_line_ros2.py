#!/usr/bin/env python3
"""
Test Line Motion - ROS2 node for Cartesian straight-line motion via aubo_bridge services.

Usage:
    ros2 run aubo_bridge test_line_ros2

    # IK only (no motion):
    ros2 run aubo_bridge test_line_ros2 --ik-only

    # With motion:
    ros2 run aubo_bridge test_line_ros2

This wraps the logic from test_line.py into a proper ROS2 node:
  1. Set base coordinate system
  2. Get current flange pose (position + orientation)
  3. Compute target pose with relative offset
  4. Inverse kinematics to get target joint angles
  5. Execute move_line via /aubo/move_line service
"""

import sys
import argparse
import rclpy
from rclpy.node import Node

from aubo_bridge_msgs.srv import GetRobotInfo, MoveLine


class TestLineNode(Node):
    """ROS2 node for Cartesian line motion testing."""

    def __init__(self, enable_move=True, enable_ik_only=False):
        super().__init__("test_line_ros2")

        self.enable_move = enable_move
        self.enable_ik_only = enable_ik_only

        # Service clients
        self.robot_info_client = self.create_client(GetRobotInfo, "/aubo/get_robot_info")
        self.move_line_client = self.create_client(MoveLine, "/aubo/move_line")

        if not self.robot_info_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /aubo/get_robot_info not available")
            return
        if not self.move_line_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /aubo/move_line not available")
            return

        self.get_logger().info("Connected to aubo_bridge services")

    def print_section(self, title):
        print("\n" + "=" * 20 + f" {title} " + "=" * 20)

    def run(self):
        """Execute Cartesian line motion test."""
        self.print_section("Step 1: Get Robot Info")
        info_response = self.get_robot_info()
        if not info_response.success:
            self.get_logger().error(f"Failed to get robot info: {info_response.message}")
            return

        current_joint = tuple(info_response.current_joint)
        current_pos = tuple(info_response.current_pos)
        current_ori = tuple(info_response.current_ori)

        print(f"Current joint angles (rad): {current_joint}")
        print(f"Current flange position (m): {current_pos}")
        print(f"Current flange orientation (quat wxyz): {current_ori}")

        self.print_section("Step 2: Construct Target Pose")
        # Relative displacement in base frame (same as test_line.py)
        relative_pos = (-0.1, -0.1, -0.2)  # small displacement
        relative_ori = (1.0, 0.0, 0.0, 0.0)  # no rotation (w=1 means identity)

        target_pos = (
            current_pos[0] + relative_pos[0],
            current_pos[1] + relative_pos[1],
            current_pos[2] + relative_pos[2],
        )
        # Keep current orientation for move_line (as per test_line.py)
        target_ori = current_ori

        print(f"Relative position offset (m): {relative_pos}")
        print(f"Target flange position (m): {target_pos}")
        print(f"Target flange orientation (quat): {target_ori}")

        self.print_section("Step 3: Inverse Kinematics")
        ik_response = self.move_line(
            relative_pos=relative_pos,
            relative_ori=relative_ori,
            max_acc=(0.3, 0.3, 0.3, 0.3, 0.3, 0.3),
            max_vel=(0.3, 0.3, 0.3, 0.3, 0.3, 0.3),
            enable_move=False  # IK only
        )

        if not ik_response:
            self.get_logger().error("IK computation failed - service call returned None")
            return

        if not ik_response.success:
            self.get_logger().error(f"IK failed: {ik_response.message}")
            return

        target_joint = tuple(ik_response.result_joint)
        print(f"IK result - target joint angles (rad): {target_joint}")

        if not self.enable_ik_only:
            self.print_section("Step 4: Execute move_line")
            move_response = self.move_line(
                relative_pos=relative_pos,
                relative_ori=relative_ori,
                max_acc=(0.3, 0.3, 0.3, 0.3, 0.3, 0.3),
                max_vel=(0.3, 0.3, 0.3, 0.3, 0.3, 0.3),
                enable_move=self.enable_move
            )

            if move_response.success:
                print(f"Line move completed successfully")
                print(f"Result joint angles: {move_response.result_joint}")
                print(f"Target position reached: {move_response.target_pos}")
                print(f"Target orientation: {move_response.target_ori}")
            else:
                self.get_logger().error(f"Move failed: {move_response.message}")
        else:
            self.print_section("IK Only Mode - No Motion Executed")

    def get_robot_info(self):
        """Get current robot info."""
        request = GetRobotInfo.Request()
        future = self.robot_info_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result()

    def move_line(self, relative_pos, relative_ori, max_acc, max_vel, enable_move):
        """Call /aubo/move_line service."""
        request = MoveLine.Request()
        request.relative_pos = list(relative_pos)
        request.relative_ori = list(relative_ori)
        request.max_acc = list(max_acc)
        request.max_vel = list(max_vel)
        request.enable_move = enable_move

        self.get_logger().info(
            f"Calling /aubo/move_line: pos_offset={relative_pos}, enable_move={enable_move}"
        )
        future = self.move_line_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        if future.result() is None:
            self.get_logger().error("Service call returned None (exception?)")
            return None
        return future.result()


def main():
    parser = argparse.ArgumentParser(description="Test Cartesian line motion")
    parser.add_argument("--ik-only", action="store_true", help="Only compute IK, don't move")
    parser.add_argument("--no-move", action="store_true", help="Disable actual motion")
    args = parser.parse_args()

    rclpy.init()
    node = TestLineNode(enable_move=not args.no_move, enable_ik_only=args.ik_only)

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
