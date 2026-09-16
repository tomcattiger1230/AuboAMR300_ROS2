"""ROS integration checks on private topics, without commanding a robot."""

import importlib.util
from pathlib import Path
import threading
import unittest

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

spec = importlib.util.spec_from_file_location(
    "action_bridge", Path(__file__).resolve().parents[1] / "scripts/action_bridge.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class BridgeFeedbackTest(unittest.TestCase):
    def test_requires_measured_target_and_rejects_invalid_points(self):
        rclpy.init(args=["--ros-args", "-p", "action_name:=/test_gripper_trajectory",
                         "-p", "command_topic:=/test_gripper_commands",
                         "-p", "state_topic:=/test_gripper_states"])
        bridge = module.TrajectoryBridge()
        bridge.set_parameters([Parameter("settle_timeout", value=0.5)])
        client_node = rclpy.create_node("bridge_test_client")
        client = ActionClient(client_node, FollowJointTrajectory, "/test_gripper_trajectory")
        publisher = client_node.create_publisher(JointState, "/test_gripper_states", 10)
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(bridge)
        executor.add_node(client_node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()

        def wait(future):
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            self.assertTrue(done.wait(5.0), "ROS request timed out")
            return future.result()

        def goal(positions, names=None):
            request = FollowJointTrajectory.Goal()
            request.trajectory.joint_names = names or [
                "gripper1_joint", "gripper2_joint"
            ]
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.nanosec = 100000000
            request.trajectory.points = [point]
            return request

        timer = None
        try:
            self.assertTrue(client.wait_for_server(timeout_sec=5.0))
            malformed = wait(client.send_goal_async(goal([float("nan"), 0.04])))
            self.assertFalse(malformed.accepted)

            handle = wait(client.send_goal_async(goal([0.04, 0.04])))
            self.assertTrue(handle.accepted)
            missing = wait(handle.get_result_async())
            self.assertEqual(missing.result.error_code,
                             FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED)

            state = JointState()
            state.name = ["gripper1_joint", "gripper2_joint"]
            state.position = [0.04, 0.04]
            timer = client_node.create_timer(0.02, lambda: publisher.publish(state))
            handle = wait(client.send_goal_async(goal([0.04, 0.04])))
            self.assertTrue(handle.accepted)
            arrived = wait(handle.get_result_async())
            self.assertEqual(arrived.result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)

            # A stalled gripper preload may remain active while the arm moves.
            state.name = ["gripper1_joint", "shoulder_joint"]
            state.position = [0.0, 0.1]
            preload = wait(
                client.send_goal_async(goal([0.04], ["gripper1_joint"]))
            )
            self.assertTrue(preload.accepted)
            overlap = wait(
                client.send_goal_async(goal([0.02], ["gripper1_joint"]))
            )
            self.assertFalse(overlap.accepted)
            arm = wait(client.send_goal_async(goal([0.1], ["shoulder_joint"])))
            self.assertTrue(arm.accepted)
            arm_result = wait(arm.get_result_async())
            self.assertEqual(
                arm_result.result.error_code,
                FollowJointTrajectory.Result.SUCCESSFUL,
            )
            wait(preload.cancel_goal_async())
            wait(preload.get_result_async())
        finally:
            if timer is not None:
                client_node.destroy_timer(timer)
            executor.shutdown()
            thread.join(timeout=5.0)
            client.destroy()
            client_node.destroy_node()
            bridge.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
