"""ROS-independent transport fakes exercise cancellation and feedback policies."""
import math
import time
from types import SimpleNamespace
import pytest
import rclpy
from rclpy.task import Future
from sensor_msgs.msg import JointState
from aubo_control_gui.motion_client import MotionClient, ARM, GRIPPER

@pytest.fixture
def client():
    rclpy.init()
    node=MotionClient()
    yield node
    node.destroy_node()
    rclpy.shutdown()

def feed(client):
    # Deliberately mix base/gripper states and reverse arm ordering.
    msg=JointState(); msg.name=['left_wheel_joint',*GRIPPER,*reversed(ARM)]
    msg.position=[99.,0.,0.,*reversed([.1,.2,.3,.4,.5,.6])]
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


def test_i16h_j3_position_limits(client):
    sent = []
    client._send = lambda *args: sent.append(args)
    for degrees in (-161, 161):
        target = [0.0]*6
        target[2] = math.radians(degrees)
        client.joint_target(target)
    assert len(sent) == 2
    for degrees in (-161.01, 161.01, -360, 360):
        target = [0.0]*6
        target[2] = math.radians(degrees)
        with pytest.raises(ValueError, match='J3'):
            client.joint_target(target)
    assert len(sent) == 2


def planned(client):
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from rclpy.parameter import Parameter
    feed(client)
    client.set_parameters([Parameter('enable_motion',value=True)])
    pending=Future();result=Future();goals=[]
    client.move.server_is_ready=lambda:True
    client.move.send_goal_async=lambda goal:(goals.append(goal) or pending)
    client.joint_target([.1,.2,.4,.4,.5,.6])
    assert goals[0].planning_options.plan_only
    pending.set_result(SimpleNamespace(accepted=True,get_result_async=lambda:result))
    t=RobotTrajectory();t.joint_trajectory.joint_names=list(ARM)
    p=JointTrajectoryPoint();p.positions=[client.state[n] for n in ARM]
    end=JointTrajectoryPoint();end.positions=[.1,.2,.4,.4,.5,.6];end.time_from_start.sec=2
    t.joint_trajectory.points=[p,end]
    return result,t


def complete(future,trajectory):
    future.set_result(SimpleNamespace(status=4,result=SimpleNamespace(error_code=SimpleNamespace(val=1),planned_trajectory=trajectory)))


def test_plan_and_execute_are_separate_and_exact(client):
    future,t=planned(client);sent=[]
    client.execute.server_is_ready=lambda:True
    client.stop_event.get_subscription_count=lambda:1
    client.execute.send_goal_async=lambda goal:(sent.append(goal) or Future())
    complete(future,t)
    assert client.phase=='ready' and client.plan_ready() and not sent
    client.execute_planned()
    assert len(sent)==1 and sent[0].trajectory==t and sent[0].trajectory is not t
    assert client.plan is None and client.phase=='executing'
    with pytest.raises(ValueError):client.execute_planned()


@pytest.mark.parametrize('change',['target','state','stale','scene','stop'])
def test_cached_plan_invalidated(client,change):
    future,t=planned(client);complete(future,t)
    if change=='target':client.invalidate_plan()
    elif change=='state':client.state[ARM[1]]+=.02
    elif change=='stale':client.received[GRIPPER[0]]-=2
    elif change=='scene':
        from moveit_msgs.msg import PlanningScene, CollisionObject
        scene=PlanningScene();client._scene(scene)
        obj=CollisionObject();obj.id='new-obstacle';scene.world.collision_objects=[obj];client._scene(scene)
    else:client.stop()
    assert not client.plan_ready()
    with pytest.raises(ValueError):client.execute_planned()


def test_edit_during_planning_discards_late_result(client):
    future,t=planned(client);client.invalidate_plan();complete(future,t)
    assert client.plan is None and not client.busy


@pytest.mark.parametrize('bad',['empty','nan','start','names','time'])
def test_invalid_planned_trajectory_rejected(client,bad):
    future,t=planned(client)
    if bad=='empty':t.joint_trajectory.points=[]
    elif bad=='nan':t.joint_trajectory.points[-1].positions[0]=math.nan
    elif bad=='start':t.joint_trajectory.points[0].positions[0]+=.1
    elif bad=='names':t.joint_trajectory.joint_names[0]='unknown'
    else:t.joint_trajectory.points[-1].time_from_start.sec=0
    complete(future,t)
    assert client.plan is None and client.phase=='error'


def test_frame_jitter_tolerated_but_cumulative_motion_invalidates(client):
    from moveit_msgs.msg import PlanningScene
    from geometry_msgs.msg import TransformStamped
    scene=PlanningScene();scene.is_diff=True
    t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_footprint';t.transform.rotation.w=1.
    scene.fixed_frame_transforms=[t];client._scene(scene)
    future,traj=planned(client);complete(future,traj)
    for value in (.0001,.0003,.0008):
        t.transform.translation.x=value;client._scene(scene)
        assert client.plan_ready()
    t.transform.translation.x=.0012;client._scene(scene)
    assert not client.plan_ready()


def test_execute_stop_uses_moveit_event_until_terminal_result(client):
    future,t=planned(client);complete(future,t)
    accepted=Future();result=Future();events=[];cancels=[]
    client.execute.server_is_ready=lambda:True
    client.stop_event.get_subscription_count=lambda:1
    client.stop_event.publish=lambda msg:events.append(msg.data)
    client.execute.send_goal_async=lambda goal:accepted
    client.execute_planned();client.stop()
    assert events==['stop'] and client.busy
    accepted.set_result(SimpleNamespace(accepted=True,cancel_goal_async=lambda:cancels.append(True),get_result_async=lambda:result))
    assert cancels==[True] and events==['stop','stop']
    client._refresh_pose()
    assert events==['stop']*3
    result.set_result(SimpleNamespace(status=6,result=SimpleNamespace(error_code=SimpleNamespace(val=-7))))
    assert not client.busy and client.phase=='stopped'
    client._refresh_pose();assert len(events)==3
