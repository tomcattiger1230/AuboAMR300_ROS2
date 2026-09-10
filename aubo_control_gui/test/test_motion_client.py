"""ROS-independent transport fakes exercise cancellation and feedback policies."""
import math
import time
from types import SimpleNamespace
import pytest
import rclpy
from rclpy.task import Future
from sensor_msgs.msg import JointState
from aubo_control_gui.motion_client import MotionClient, ARM

@pytest.fixture
def client():
    rclpy.init()
    node=MotionClient()
    yield node
    node.destroy_node()
    rclpy.shutdown()

def feed(client):
    # Deliberately mix base/gripper states and reverse arm ordering.
    msg=JointState(); msg.name=['left_wheel_joint',*reversed(ARM)]
    msg.position=[99.,*reversed([.1,.2,.3,.4,.5,.6])]
    client._state(msg)

def test_joint_feedback_uses_names_and_expires(client):
    feed(client)
    assert [client.state[n] for n in ARM]==[.1,.2,.3,.4,.5,.6]
    assert client.fresh()
    client.received[ARM[0]]=time.monotonic()-2
    assert not client.fresh()
    with pytest.raises(ValueError,match='反馈'): client.joint_target([0.]*6)

def test_stop_before_acceptance_cancels_late_goal_and_blocks_new_motion(client):
    feed(client)
    pending=Future()
    client.move.server_is_ready=lambda:True
    client.move.send_goal_async=lambda goal:pending
    client.joint_target([0.]*6)
    client.stop()
    with pytest.raises(ValueError,match='尚未结束'): client.joint_target([0.]*6)
    result=Future(); cancellations=[]
    handle=SimpleNamespace(accepted=True,cancel_goal_async=lambda:cancellations.append(True),get_result_async=lambda:result)
    pending.set_result(handle)
    assert cancellations==[True]
    assert client.busy
    result.set_result(SimpleNamespace(status=5,result=SimpleNamespace(error_code=SimpleNamespace(val=-7))))
    assert not client.busy
    assert cancellations==[True]

def test_no_retry_after_execution_failure(client):
    feed(client);goals=[];pending=Future();result=Future()
    client.move.server_is_ready=lambda:True
    client.move.send_goal_async=lambda goal:(goals.append(goal) or pending)
    client.joint_target([0.]*6)
    pending.set_result(SimpleNamespace(accepted=True,get_result_async=lambda:result))
    result.set_result(SimpleNamespace(status=6,result=SimpleNamespace(error_code=SimpleNamespace(val=-4))))
    assert len(goals)==1 and not client.busy
    assert goals[0].planning_options.plan_only  # Explicitly enable execution.

def test_feedback_loss_requests_cancellation(client):
    feed(client);client.busy=True;calls=[]
    client.goal_handle=SimpleNamespace(cancel_goal_async=lambda:calls.append(True))
    client.received[ARM[0]]=time.monotonic()-2
    client._refresh_pose();client._refresh_pose()
    assert calls==[True] and client.busy

def test_nonfinite_state_cannot_refresh_old_sample(client):
    feed(client);stamp=client.received[ARM[0]]
    msg=JointState();msg.name=[ARM[0]];msg.position=[math.nan]
    client._state(msg)
    assert client.received[ARM[0]]==stamp
