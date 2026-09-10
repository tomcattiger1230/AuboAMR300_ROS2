import importlib.util
from pathlib import Path
from types import SimpleNamespace
import rclpy
import pytest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from rclpy.action import CancelResponse


def test_cancel_publishes_feedback_hold_with_zero_velocity():
    pytest.importorskip("control_msgs", reason="Isaac controller test requires the Ubuntu backend dependencies")
    source=Path(__file__).resolve().parents[2]/'seer_description/scripts/action_bridge.py'
    spec=importlib.util.spec_from_file_location('sim_action_bridge',source)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    rclpy.init();node=module.TrajectoryBridge();published=[]
    try:
        node.publisher_=SimpleNamespace(publish=published.append)
        msg=JointState();msg.name=['shoulder_joint'];msg.position=[.123]
        node._on_joint_state(msg)
        trajectory=JointTrajectory();trajectory.joint_names=msg.name
        handle=SimpleNamespace(request=SimpleNamespace(trajectory=trajectory))
        assert node.cancel_callback(handle)==CancelResponse.ACCEPT
        assert node._cancel_pending
        assert list(published[-1].position)==[.123]
        assert list(published[-1].velocity)==[0.0]
    finally:
        node.destroy_node();rclpy.shutdown()
