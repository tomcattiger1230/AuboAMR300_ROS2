#!/usr/bin/env python3
"""
Test Joint Motion - ROS2 node for joint space motion via aubo_bridge services.

Usage:
    ros2 run aubo_bridge test_joint_ros2

This wraps the logic from test_joint.py into a proper ROS2 node:
  1. Get current robot info (waypoint)
  2. Compute relative target position in base frame
  3. Inverse kinematics to get target joint angles
  4. Execute joint move via /aubo/move_joint service
"""

import sys
import argparse
import rclpy
from rclpy.node import Node

from aubo_bridge_msgs.srv import GetRobotInfo, MoveJoint, MoveLine


class TestJointNode(Node):
    """ROS2 node for joint space motion testing."""

    def __init__(self, enable_move=True, enable_ik_only=False):
        super().__init__("test_joint_ros2")

        self.enable_move = enable_move
        self.enable_ik_only = enable_ik_only

        # Service clients
        self.robot_info_client = self.create_client(GetRobotInfo, "/aubo/get_robot_info")
        self.move_joint_client = self.create_client(MoveJoint, "/aubo/move_joint")

        if not self.robot_info_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /aubo/get_robot_info not available")
            return
        if not self.move_joint_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /aubo/move_joint not available")
            return

        self.get_logger().info("Connected to aubo_bridge services")

    def print_section(self, title):
        print("\n" + "=" * 20 + f" {title} " + "=" * 20)

    def run(self):
        """Execute joint space motion test."""
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
        print(f"Current flange orientation (quat): {current_ori}")

        self.print_section("Step 2: Generate Target Pose")
        # Small relative displacement in base frame (same as test_joint.py)
        relative_pos = (0.05, 0.05, 0.05)  # 5mm each axis
        relative_ori = (1.0, 0.0, 0.0, 0.0)  # no rotation

        target_pos = (
            current_pos[0] + relative_pos[0],
            current_pos[1] + relative_pos[1],
            current_pos[2] + relative_pos[2],
        )
        target_ori = current_ori  # keep current orientation

        print(f"Relative position offset: {relative_pos}")
        print(f"Target flange position (m): {target_pos}")
        print(f"Target flange orientation (quat): {target_ori}")

        self.print_section("Step 3: Inverse Kinematics")
        # Use MoveLine with enable_move=False to compute IK only
        ik_response = self.compute_ik_via_move_line(
            relative_pos, relative_ori,
            max_acc=(0.5, 0.5, 0.5, 0.5, 0.5, 0.5),
            max_vel=(0.5, 0.5, 0.5, 0.5, 0.5, 0.5),
            enable_move=False
        )

        if not ik_response.success:
            self.get_logger().error(f"IK failed: {ik_response.message}")
            return

        target_joint = tuple(ik_response.result_joint)
        print(f"IK result - target joint angles (rad): {target_joint}")

        if not self.enable_ik_only:
            self.print_section("Step 4: Execute Joint Move")
            move_response = self.move_joint(
                target_joint,
                max_acc=(0.5, 0.5, 0.5, 0.5, 0.5, 0.5),
                max_vel=(0.5, 0.5, 0.5, 0.5, 0.5, 0.5),
                enable_move=self.enable_move
            )

            if move_response.success:
                print(f"Move completed successfully")
                print(f"Result joint angles: {move_response.result_joint}")
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

    def move_joint(self, target_joint, max_acc, max_vel, enable_move):
        """Call /aubo/move_joint service."""
        request = MoveJoint.Request()
        request.target_joint = list(target_joint)
        request.max_acc = list(max_acc)
        request.max_vel = list(max_vel)
        request.enable_move = enable_move

        self.get_logger().info(f"Calling /aubo/move_joint: {list(target_joint)}")
        future = self.move_joint_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        return future.result()

    def compute_ik_via_move_line(self, relative_pos, relative_ori, max_acc, max_vel, enable_move):
        """Use MoveLine service to compute IK (enable_move=False)."""
        client = self.create_client(MoveLine, "/aubo/move_line")
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /aubo/move_line not available")
            return None

        request = MoveLine.Request()
        request.relative_pos = list(relative_pos)
        request.relative_ori = list(relative_ori)
        request.max_acc = list(max_acc)
        request.max_vel = list(max_vel)
        request.enable_move = enable_move

        self.get_logger().info(f"Calling /aubo/move_line for IK computation")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        return future.result()


def main():
    parser = argparse.ArgumentParser(description="Test joint space motion")
    parser.add_argument("--ik-only", action="store_true", help="Only compute IK, don't move")
    parser.add_argument("--no-move", action="store_true", help="Disable actual motion")
    args = parser.parse_args()

    rclpy.init()
    node = TestJointNode(enable_move=not args.no_move, enable_ik_only=args.ik_only)

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
