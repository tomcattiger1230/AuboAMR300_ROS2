#!/usr/bin/env python3
"""Expose independent rebar tester carriage and jaw targets over ROS 2."""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

from rebar_tester_control import DEFAULTS, atomic_write, clamp, paths, read_state


class RebarTesterBridge(Node):
    def __init__(self):
        super().__init__("rebar_tester_bridge")
        self.command_path, self.state_path = paths()
        self.targets = DEFAULTS.copy()
        self.targets["robot_attach"] = False
        self.targets["base_lock"] = False
        self.last_state_mtime = None
        self.last_state = None
        self.last_state_publish = self.get_clock().now() - Duration(seconds=10)
        self.state_publishers = {}
        self.gripped_publisher = self.create_publisher(
            Bool, "/rebar_tester/rebar_gripped", 10
        )
        self.attached_publisher = self.create_publisher(
            Bool, "/rebar_tester/robot_attached", 10
        )
        self.base_locked_publisher = self.create_publisher(
            Bool, "/rebar_tester/base_locked", 10
        )
        self.create_subscription(
            Bool, "/rebar_tester/robot_attach_cmd", self.on_robot_attach, 10
        )
        self.create_subscription(
            Bool, "/rebar_tester/base_lock_cmd", self.on_base_lock, 10
        )
        for key in DEFAULTS:
            group, axis = key.split("_", 1)
            topic = f"/rebar_tester/{group}/{axis}"
            self.create_subscription(Float64, topic + "_cmd", self.callback(key), 10)
            self.state_publishers[key] = self.create_publisher(Float64, topic + "_state", 10)
        atomic_write(self.command_path, self.targets)
        self.create_timer(0.1, self.publish_state)
        self.get_logger().info("Rebar tester command and state topics ready")

    def on_robot_attach(self, message):
        self.targets["robot_attach"] = bool(message.data)
        atomic_write(self.command_path, self.targets)

    def on_base_lock(self, message):
        self.targets["base_lock"] = bool(message.data)
        atomic_write(self.command_path, self.targets)

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
        # Publish on every state-file change, and additionally repeat the
        # last known state once per second: the Isaac side only rewrites
        # the file when a mechanism actually moves, so without the
        # heartbeat an idle tester looks disconnected to listeners.
        try:
            mtime = self.state_path.stat().st_mtime_ns
            now = self.get_clock().now()
            if mtime != self.last_state_mtime:
                self.last_state_mtime = mtime
                self.last_state = read_state(self.state_path)
                self.last_state_publish = now
            elif (self.last_state is not None
                  and (now - self.last_state_publish).nanoseconds < 1_000_000_000):
                return
            else:
                self.last_state_publish = now
        except (FileNotFoundError, OSError, ValueError, KeyError) as exc:
            if not isinstance(exc, FileNotFoundError):
                self.get_logger().warning(f"Cannot read rebar tester state: {exc}")
            return
        for key in DEFAULTS:
            value = self.last_state[key]
            self.state_publishers[key].publish(Float64(data=value))
        self.gripped_publisher.publish(
            Bool(data=self.last_state["rebar_gripped"])
        )
        self.attached_publisher.publish(
            Bool(data=self.last_state["robot_attached"])
        )
        self.base_locked_publisher.publish(
            Bool(data=self.last_state["base_locked"])
        )


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
