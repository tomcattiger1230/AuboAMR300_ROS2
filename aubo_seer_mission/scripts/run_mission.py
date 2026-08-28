#!/usr/bin/env python3
"""CLI trigger for the joint mission: calls /mission/start or /mission/stop."""

import argparse
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description="Start or stop an aubo_seer_mission task")
    parser.add_argument("command", choices=["start", "stop"], help="mission command")
    parser.add_argument("--wait", action="store_true", help="wait for mission to finish")
    args = parser.parse_args()

    rclpy.init()
    node = Node("run_mission")
    client = node.create_client(Trigger, f"/mission/{args.command}")

    if not client.wait_for_service(timeout_sec=5.0):
        print(f"error: /mission/{args.command} service unavailable", file=sys.stderr)
        sys.exit(1)

    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    print(f"{args.command}: success={result.success} message={result.message}")

    exit_code = 0 if result.success else 1

    if args.wait and args.command == "start" and result.success:
        from aubo_interfaces.msg import AgvStatus

        done = {"final": None, "text": ""}

        def on_status(msg):
            if msg.status in (AgvStatus.COMPLETED, AgvStatus.FAILED, AgvStatus.CANCELED):
                done["final"] = msg.status
                done["text"] = msg.message

        node.create_subscription(AgvStatus, "/mission/status", on_status, 10)
        while rclpy.ok() and done["final"] is None:
            rclpy.spin_once(node, timeout_sec=0.5)
        print(f"final: {done['text']}")
        exit_code = 0 if done["final"] == AgvStatus.COMPLETED else 1

    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
