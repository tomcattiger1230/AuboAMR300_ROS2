#!/usr/bin/env python3
"""Expose independent rebar tester carriage and jaw targets over ROS 2."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from rebar_tester_control import DEFAULTS, atomic_write, clamp, paths, read_values


class RebarTesterBridge(Node):
    def __init__(self):
        super().__init__("rebar_tester_bridge")
        self.command_path, self.state_path = paths()
        self.targets = DEFAULTS.copy()
        self.last_state_mtime = None
        self.state_publishers = {}
        for key in DEFAULTS:
            group, axis = key.split("_", 1)
            topic = f"/rebar_tester/{group}/{axis}"
            self.create_subscription(Float64, topic + "_cmd", self.callback(key), 10)
            self.state_publishers[key] = self.create_publisher(Float64, topic + "_state", 10)
        atomic_write(self.command_path, self.targets)
        self.create_timer(0.1, self.publish_state)
        self.get_logger().info("Rebar tester command and state topics ready")

    def callback(self, key):
        def on_message(message):
            try:
                requested = float(message.data)
                value = clamp(key, requested)
            except ValueError as exc:
                self.get_logger().warning(str(exc))
                return
            if value != requested:
                self.get_logger().warning(f"{key} clamped to {value:.3f} m")
            self.targets[key] = value
            atomic_write(self.command_path, self.targets)
        return on_message

    def publish_state(self):
        try:
            mtime = self.state_path.stat().st_mtime_ns
            if mtime == self.last_state_mtime:
                return
            self.last_state_mtime = mtime
            state = read_values(self.state_path)
        except (FileNotFoundError, OSError, ValueError, KeyError) as exc:
            if not isinstance(exc, FileNotFoundError):
                self.get_logger().warning(f"Cannot read rebar tester state: {exc}")
            return
        for key, value in state.items():
            self.state_publishers[key].publish(Float64(data=value))


def main():
    rclpy.init()
    node = RebarTesterBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
