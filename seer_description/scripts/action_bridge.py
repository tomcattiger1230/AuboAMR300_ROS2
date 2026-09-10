#!/usr/bin/env python3
# coding=UTF-8
"""Translate MoveIt FollowJointTrajectory goals into Isaac JointState commands."""

import math
import threading
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from time import monotonic, sleep


class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__("action_to_joint_state_bridge")

        self.declare_parameter(
            "action_name",
            "/aubo_arm_controller_wo_gripper/follow_joint_trajectory",
        )
        self.declare_parameter("command_topic", "/isaac_joint_commands")
        self.declare_parameter("state_topic", "/joint_states")
        self.declare_parameter("settle_timeout", 10.0)
        action_name = self.get_parameter("action_name").value
        command_topic = self.get_parameter("command_topic").value

        self._command_lock = threading.Lock()
        self._cancel_pending = False
        self._goal_lock = threading.Lock()
        self._goal_active = False
        callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        self._joint_state = {}
        self._state_subscription = self.create_subscription(
            JointState, self.get_parameter("state_topic").value,
            self._on_joint_state, qos_profile_sensor_data,
            callback_group=callback_group,
        )

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group,
        )

        self.publisher_ = self.create_publisher(
            JointState,
            command_topic,
            10,
        )
        self.get_logger().info(
            f"MoveIt action {action_name} -> Isaac commands {command_topic}"
        )

    def goal_callback(self, goal_request):
        trajectory = goal_request.trajectory
        names = trajectory.joint_names
        if not trajectory.points or not names or len(set(names)) != len(names):
            self.get_logger().warning("Rejected an empty or ambiguous trajectory")
            return GoalResponse.REJECT
        previous_time = -1.0
        for point in trajectory.points:
            point_time = self._point_time(point)
            if (len(point.positions) != len(names)
                    or not all(math.isfinite(v) for v in point.positions)
                    or point_time < 0.0 or point_time < previous_time
                    or any(values and (len(values) != len(names)
                           or not all(math.isfinite(v) for v in values))
                           for values in (point.velocities, point.accelerations, point.effort))):
                self.get_logger().warning("Rejected malformed trajectory points")
                return GoalResponse.REJECT
            previous_time = point_time
        with self._goal_lock:
            if self._goal_active:
                self.get_logger().warning("Rejected a goal while another is active")
                return GoalResponse.REJECT
            self._goal_active = True
            self._cancel_pending = False
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Serialize cancellation with publishing: no target can follow this hold.
        with self._command_lock:
            self._cancel_pending = True
            self._hold_position(goal_handle.request.trajectory.joint_names)
        return CancelResponse.ACCEPT

    def _hold_position(self, names):
        actual = self._actual_positions(names)
        if actual is not None:
            command = JointState()
            command.header.stamp = self.get_clock().now().to_msg()
            command.name = list(names)
            command.position = actual
            command.velocity = [0.0] * len(names)
            self.publisher_.publish(command)

    def _on_joint_state(self, message):
        now = monotonic()
        with self._state_lock:
            self._joint_state.update(
                (name, (position, now))
                for name, position in zip(message.name, message.position)
                if math.isfinite(position)
            )

    def _actual_positions(self, names):
        now = monotonic()
        with self._state_lock:
            values = [self._joint_state.get(name) for name in names]
        if any(value is None or now - value[1] > 1.0 for value in values):
            return None
        return [value[0] for value in values]

    def _feedback(self, goal_handle, names, point):
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = list(names)
        feedback.desired = point
        actual = self._actual_positions(names)
        if actual is not None:
            feedback.actual.positions = actual
            feedback.error.positions = [a - b for a, b in zip(point.positions, actual)]
        goal_handle.publish_feedback(feedback)
        return actual

    def _wait_for_target(self, goal_handle, names, point):
        tolerances = {name: (0.001 if name.startswith("gripper") else 0.01)
                      for name in names}
        for tolerance in goal_handle.request.goal_tolerance:
            if tolerance.name in tolerances and tolerance.position != 0.0:
                tolerances[tolerance.name] = (
                    math.inf if tolerance.position < 0.0 else tolerance.position
                )
        deadline = monotonic() + self.get_parameter("settle_timeout").value
        while rclpy.ok() and monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            actual = self._feedback(goal_handle, names, point)
            if actual is not None and all(
                abs(target - observed) <= tolerances[name]
                for name, target, observed in zip(names, point.positions, actual)
            ):
                return True
            sleep(0.02)
        return False

    @staticmethod
    def _result(code, message):
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = message
        return result

    @staticmethod
    def _point_time(point):
        return point.time_from_start.sec + point.time_from_start.nanosec / 1e9

    def _wait_until(self, target_time, goal_handle):
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return False
            remaining = target_time - monotonic()
            if remaining <= 0.0:
                return True
            sleep(min(remaining, 0.01))
        return False

    def execute_callback(self, goal_handle):
        try:
            trajectory = goal_handle.request.trajectory
            start_time = monotonic()
            self.get_logger().info(
                f"Executing {len(trajectory.points)} trajectory points"
            )

            for point in trajectory.points:
                if not self._wait_until(
                    start_time + self._point_time(point), goal_handle
                ):
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        self.get_logger().info("Trajectory canceled")
                        return self._result(
                            FollowJointTrajectory.Result.INVALID_GOAL,
                            "Trajectory canceled",
                        )
                    goal_handle.abort()
                    return self._result(
                        FollowJointTrajectory.Result.INVALID_GOAL,
                        "ROS shutdown interrupted trajectory",
                    )

                command = JointState()
                command.header.stamp = self.get_clock().now().to_msg()
                command.name = list(trajectory.joint_names)
                command.position = list(point.positions)
                command.velocity = list(point.velocities)
                command.effort = list(point.effort)
                with self._command_lock:
                    if not self._cancel_pending:
                        self.publisher_.publish(command)

                self._feedback(goal_handle, trajectory.joint_names, point)

            if not self._wait_for_target(
                goal_handle, trajectory.joint_names, trajectory.points[-1]
            ):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    with self._command_lock:
                        self._hold_position(trajectory.joint_names)
                    goal_handle.abort()
                return self._result(
                    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                    "Canceled or no fresh joint feedback within goal tolerance",
                )

            goal_handle.succeed()
            self.get_logger().info("Trajectory completed")
            return self._result(
                FollowJointTrajectory.Result.SUCCESSFUL,
                "Trajectory completed",
            )
        finally:
            with self._goal_lock:
                self._goal_active = False


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
