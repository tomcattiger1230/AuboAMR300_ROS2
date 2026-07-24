#!/usr/bin/env python3
# coding=UTF-8
"""Translate MoveIt FollowJointTrajectory goals into Isaac JointState commands."""

import threading
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
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
        action_name = self.get_parameter("action_name").value
        command_topic = self.get_parameter("command_topic").value

        self._goal_lock = threading.Lock()
        self._goal_active = False
        callback_group = ReentrantCallbackGroup()

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
        if not goal_request.trajectory.points:
            self.get_logger().warning("Rejected an empty trajectory")
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_active:
                self.get_logger().warning("Rejected a goal while another is active")
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

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
                self.publisher_.publish(command)

                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = command.header.stamp
                feedback.joint_names = list(trajectory.joint_names)
                feedback.desired = point
                goal_handle.publish_feedback(feedback)

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
