#!/usr/bin/env python3
"""
Test Robot Info - ROS2 node to read robot status via aubo_bridge services.

Usage:
    ros2 run aubo_bridge test_robot_info_ros2

    ros2 service call /aubo/get_robot_info aubo_bridge_msgs/srv/GetRobotInfo "{}"
"""

import sys
import rclpy
from rclpy.node import Node

from aubo_bridge_msgs.srv import GetRobotInfo


class TestRobotInfo(Node):
    """Test node to get robot info via ROS2 service."""

    def __init__(self):
        super().__init__("test_robot_info")

        self.client = self.create_client(GetRobotInfo, "/aubo/get_robot_info")
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /aubo/get_robot_info not available")
            return

        self.get_logger().info("Connected to /aubo/get_robot_info service")

    def call_and_display(self):
        """Call the service and display results."""
        request = GetRobotInfo.Request()

        self.get_logger().info("Calling /aubo/get_robot_info...")
        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error("Service call failed")
            return

        response = future.result()

        print("\n" + "=" * 50)
        print(" Robot Info Response")
        print("=" * 50)
        print(f" Success: {response.success}")
        print(f" Message: {response.message}")
        print(f" Joint Status: {response.joint_status}")
        print(f"")
        print(f" Current Joint Angles (rad):")
        for i, val in enumerate(response.current_joint):
            print(f"   Joint {i}: {val:.6f}")
        print(f"")
        print(f" Current Flange Position (m):")
        print(f"   X: {response.current_pos[0]:.6f}")
        print(f"   Y: {response.current_pos[1]:.6f}")
        print(f"   Z: {response.current_pos[2]:.6f}")
        print(f"")
        print(f" Current Flange Orientation (quat):")
        print(f"   W: {response.current_ori[0]:.6f}")
        print(f"   X: {response.current_ori[1]:.6f}")
        print(f"   Y: {response.current_ori[2]:.6f}")
        print(f"   Z: {response.current_ori[3]:.6f}")
        print(f"")
        print(f" Max Joint Acceleration (rad/s^2):")
        for i, val in enumerate(response.joint_maxacc):
            print(f"   Joint {i}: {val:.4f}")
        print(f"")
        print(f" Max Joint Velocity (rad/s):")
        for i, val in enumerate(response.joint_maxvelc):
            print(f"   Joint {i}: {val:.4f}")
        print("=" * 50)


def main():
    rclpy.init()
    node = TestRobotInfo()

    try:
        node.call_and_display()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
