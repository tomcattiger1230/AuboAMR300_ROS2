#!/usr/bin/env python3
"""Command the SEER mobile base and AUBO arm through ROS 2."""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = (
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
)
GRIPPER_JOINTS = ("gripper1_joint", "gripper2_joint")
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 1.0
MAX_MOTION_DURATION = 30.0


def duration_message(seconds):
    whole_seconds = int(seconds)
    nanoseconds = int(round((seconds - whole_seconds) * 1e9))
    if nanoseconds == 1_000_000_000:
        whole_seconds += 1
        nanoseconds = 0
    return Duration(sec=whole_seconds, nanosec=nanoseconds)


def finite_float(value):
    result = float(value)
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError("value must be finite")
    return result


def positive_duration(value):
    result = finite_float(value)
    if not 0.0 < result <= MAX_MOTION_DURATION:
        raise argparse.ArgumentTypeError(
            f"duration must be in (0, {MAX_MOTION_DURATION:g}] seconds"
        )
    return result


def joint_target(value):
    result = finite_float(value)
    if abs(result) > 2.0 * math.pi:
        raise argparse.ArgumentTypeError("joint target must be within +/- 2*pi rad")
    return result


def gripper_target(value):
    result = finite_float(value)
    if not 0.0 <= result <= 0.04:
        raise argparse.ArgumentTypeError(
            "gripper position must be between 0.0 and 0.04 m"
        )
    return result


def build_parser():
    parser = argparse.ArgumentParser(
        description="Control the SEER base, AUBO arm, or a combined motion."
    )
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument(
        "--action-name",
        default="/aubo_arm_controller_wo_gripper/follow_joint_trajectory",
    )
    parser.add_argument("--wait-timeout", type=positive_duration, default=10.0)

    subparsers = parser.add_subparsers(dest="command", required=True)

    base = subparsers.add_parser("base", help="Drive the mobile base")
    base.add_argument("--linear", type=finite_float, default=0.2)
    base.add_argument("--angular", type=finite_float, default=0.0)
    base.add_argument("--duration", type=positive_duration, default=2.0)

    arm = subparsers.add_parser("arm", help="Move all six AUBO joints")
    arm.add_argument(
        "--target",
        type=joint_target,
        nargs=6,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        required=True,
    )
    arm.add_argument("--duration", type=positive_duration, default=4.0)
    arm.add_argument("--steps", type=int, default=80)

    gripper = subparsers.add_parser(
        "gripper", help="Open or close both stick-gripper fingers"
    )
    gripper.add_argument(
        "--position",
        type=gripper_target,
        required=True,
        help="Finger travel in metres: 0.0=open, 0.04=closed",
    )
    gripper.add_argument("--duration", type=positive_duration, default=1.0)
    gripper.add_argument("--steps", type=int, default=20)

    sequence = subparsers.add_parser(
        "sequence", help="Drive the base, stop, then move the arm"
    )
    sequence.add_argument("--linear", type=finite_float, default=0.2)
    sequence.add_argument("--angular", type=finite_float, default=0.0)
    sequence.add_argument("--base-duration", type=positive_duration, default=2.0)
    sequence.add_argument(
        "--target",
        type=joint_target,
        nargs=6,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        default=(0.0, -0.35, 0.6, 0.0, 0.35, 0.0),
    )
    sequence.add_argument("--arm-duration", type=positive_duration, default=4.0)
    sequence.add_argument("--steps", type=int, default=80)

    subparsers.add_parser("stop", help="Publish zero base velocity")
    return parser


class MobileManipulatorController(Node):
    def __init__(self, args):
        super().__init__("mobile_manipulator_controller")
        self.args = args
        self._joint_positions = {}
        self._base_publisher = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self._joint_subscription = self.create_subscription(
            JointState,
            args.joint_state_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            args.action_name,
        )

    def _joint_state_callback(self, message):
        self._joint_positions.update(zip(message.name, message.position))

    @staticmethod
    def _validate_base_command(linear, angular):
        if abs(linear) > MAX_LINEAR_SPEED:
            raise ValueError(
                f"linear speed exceeds safety limit {MAX_LINEAR_SPEED:g} m/s"
            )
        if abs(angular) > MAX_ANGULAR_SPEED:
            raise ValueError(
                f"angular speed exceeds safety limit {MAX_ANGULAR_SPEED:g} rad/s"
            )

    def stop_base(self):
        stop_message = Twist()
        for _ in range(5):
            self._base_publisher.publish(stop_message)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

    def drive_base(self, linear, angular, duration):
        self._validate_base_command(linear, angular)
        command = Twist()
        period = 1.0 / 20.0
        ramp_time = min(0.5, duration * 0.25)
        started = time.monotonic()
        self.get_logger().info(
            f"Driving base: linear={linear:.3f} m/s, "
            f"angular={angular:.3f} rad/s, duration={duration:.2f} s"
        )
        try:
            while rclpy.ok():
                elapsed = time.monotonic() - started
                if elapsed >= duration:
                    break
                remaining = duration - elapsed
                scale = min(
                    1.0,
                    elapsed / ramp_time if ramp_time else 1.0,
                    remaining / ramp_time if ramp_time else 1.0,
                )
                command.linear.x = linear * scale
                command.angular.z = angular * scale
                self._base_publisher.publish(command)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(period)
        finally:
            self.stop_base()
        self.get_logger().info("Base motion completed and zero velocity published")

    def _current_joint_positions(self, joint_names, label):
        deadline = time.monotonic() + self.args.wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if all(joint in self._joint_positions for joint in joint_names):
                return [self._joint_positions[joint] for joint in joint_names]
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = [
            joint for joint in joint_names if joint not in self._joint_positions
        ]
        raise RuntimeError(
            f"timed out waiting for {label} joints on "
            f"{self.args.joint_state_topic}: "
            + ", ".join(missing)
        )

    def move_arm(self, target, duration, steps):
        self._move_joints(ARM_JOINTS, target, duration, steps, "arm")

    def move_gripper(self, position, duration, steps):
        self._move_joints(
            GRIPPER_JOINTS,
            (position, position),
            duration,
            steps,
            "gripper",
        )

    def _move_joints(self, joint_names, target, duration, steps, label):
        if steps < 2 or steps > 500:
            raise ValueError("steps must be between 2 and 500")
        if not self._trajectory_client.wait_for_server(
            timeout_sec=self.args.wait_timeout
        ):
            raise RuntimeError(
                f"action server is unavailable: {self.args.action_name}"
            )

        start = self._current_joint_positions(joint_names, label)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()

        for index in range(1, steps + 1):
            normalized_time = index / steps
            smooth_position = (
                3.0 * normalized_time**2 - 2.0 * normalized_time**3
            )
            smooth_velocity = (
                6.0 * normalized_time - 6.0 * normalized_time**2
            ) / duration
            point = JointTrajectoryPoint()
            point.positions = [
                initial + (final - initial) * smooth_position
                for initial, final in zip(start, target)
            ]
            point.velocities = [
                (final - initial) * smooth_velocity
                for initial, final in zip(start, target)
            ]
            point.time_from_start = duration_message(duration * normalized_time)
            goal.trajectory.points.append(point)

        self.get_logger().info(
            f"Sending smooth {label} trajectory to: "
            + ", ".join(f"{value:.3f}" for value in target)
        )
        send_future = self._trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"{label} trajectory was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError(f"{label} action returned no result")
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"{label} action failed with status {wrapped_result.status}"
            )
        if (
            wrapped_result.result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(wrapped_result.result.error_string)
        self.get_logger().info(f"{label.capitalize()} trajectory completed")


def main():
    parser = build_parser()
    cli_args = remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(cli_args)
    rclpy.init(args=sys.argv)
    node = MobileManipulatorController(args)
    exit_code = 0
    try:
        if args.command == "base":
            node.drive_base(args.linear, args.angular, args.duration)
        elif args.command == "arm":
            node.move_arm(args.target, args.duration, args.steps)
        elif args.command == "gripper":
            node.move_gripper(args.position, args.duration, args.steps)
        elif args.command == "sequence":
            node.drive_base(args.linear, args.angular, args.base_duration)
            node.move_arm(args.target, args.arm_duration, args.steps)
        elif args.command == "stop":
            node.stop_base()
            node.get_logger().info("Zero base velocity published")
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted; stopping the mobile base")
        exit_code = 130
    except Exception as error:
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        node.stop_base()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
