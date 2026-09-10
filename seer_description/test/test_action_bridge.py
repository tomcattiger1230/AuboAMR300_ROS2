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

        def goal(positions):
            request = FollowJointTrajectory.Goal()
            request.trajectory.joint_names = ["gripper1_joint", "gripper2_joint"]
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
