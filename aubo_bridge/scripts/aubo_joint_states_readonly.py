#!/usr/bin/env python3
"""Publish AUBO PE joint angles without enabling or configuring robot motion."""

import argparse
import math
import os
import sys
import time

sdk_path = os.environ.get("AUBO_SDK_PYTHON_PATH")
if sdk_path:
    sys.path.insert(0, sdk_path)

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from robotcontrol import Auboi5Robot, RobotErrorType


JOINT_NAMES = (
    "shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint",
)


class JointReader(Node):
    def __init__(self, host, port, rate):
        super().__init__("aubo_joint_states_readonly")
        self.host, self.port = host, port
        self.robot = None
        self.connected = False
        self.next_retry_at = 0.0
        self.sdk_initialized = False
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        result = Auboi5Robot.initialize()
        if result != RobotErrorType.RobotError_SUCC:
            raise RuntimeError(f"AUBO SDK initialization failed: {result}")
        self.sdk_initialized = True
        self.robot = Auboi5Robot()
        self.robot.create_context()
        self.timer = self.create_timer(1.0 / rate, self.poll)

    def poll(self):
        if not self.connected:
            if time.monotonic() < self.next_retry_at:
                return
            self.next_retry_at = time.monotonic() + 3.0
            try:
                result = self.robot.connect(self.host, self.port)
            except Exception as exc:
                self.get_logger().warn(f"AUBO connection failed: {exc}")
                return
            if result != RobotErrorType.RobotError_SUCC:
                self.get_logger().warn(f"AUBO connection failed: {result}")
                return
            self.connected = True
            self.get_logger().info(f"Reading joints from {self.host}:{self.port}")
        try:
            waypoint = self.robot.get_current_waypoint()
            joints = waypoint.get("joint") if isinstance(waypoint, dict) else None
            if joints is None or len(joints) != 6 or not all(math.isfinite(v) for v in joints):
                self.get_logger().warn("AUBO returned invalid joint angles; skipping sample")
                return
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(JOINT_NAMES)
            message.position = list(joints)
            self.publisher.publish(message)
        except Exception as exc:
            self.get_logger().error(f"Joint read failed: {exc}")
            try:
                self.robot.disconnect()
            except Exception:
                pass
            self.connected = False
            self.next_retry_at = time.monotonic() + 3.0

    def close(self):
        if self.robot is not None and self.connected:
            self.robot.disconnect()
        if self.sdk_initialized:
            Auboi5Robot.uninitialize()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=8899)
    parser.add_argument("--rate", type=float, default=10.0)
    args = parser.parse_args()
    if not 0 < args.rate <= 30:
        parser.error("rate must be between 0 and 30 Hz")
    rclpy.init()
    node = None
    try:
        node = JointReader(args.host, args.port, args.rate)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
