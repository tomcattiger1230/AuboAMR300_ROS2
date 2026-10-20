from types import SimpleNamespace
import math
import pytest
import torch

from rebar_training.route import LoadingRoute
from rebar_training.reset import quat_multiply


def grasp_setup():
    route=LoadingRoute(1,'cpu')
    pose=torch.tensor([[-1.15,0.,.522,0.,0.,0.,1.]])
    route.reset(torch.tensor([0]),pose)
    state=SimpleNamespace(bar=pose[:,:3],tcp=pose[:,:3]+torch.tensor([0.,0.,.10]),
        fingers=torch.zeros(1,2),finger_forces=torch.zeros(1,2),jaw_forces=torch.zeros(1,4))
    logic=SimpleNamespace(contact_dwell=torch.zeros(1),pick_complete=torch.tensor([False]),phase=torch.zeros(1))
    return route,state,logic


def rotated(q,degrees):
    a=math.radians(degrees)/2
    return quat_multiply(torch.tensor([[0.,0.,math.sin(a),math.cos(a)]]),q)


def test_visual_grasp_cannot_descend_at_standoff_with_unsettled_orientation():
    route,s,logic=grasp_setup();goal=(0.,0.,1.5);joints=torch.zeros(1,6)
    for _ in range(20):
        route.advance(s,logic,joints,goal,grasp_quaternion=rotated(route.quaternion,22))
    assert route.stage.item()==0 and route.grasp_alignment_dwell.item()==0
    for _ in range(8):route.advance(s,logic,joints,goal,grasp_quaternion=route.quaternion)
    assert route.stage.item()==0
    route.advance(s,logic,joints,goal,grasp_quaternion=rotated(route.quaternion,5))
    assert route.grasp_alignment_dwell.item()==0
    # Quaternion sign and a harmless sensor normalization error do not change pose.
    for _ in range(9):route.advance(s,logic,joints,goal,grasp_quaternion=-route.quaternion*1.01)
    assert route.stage.item()==1
    torch.testing.assert_close(route.targets(s,goal)[0][:,2],s.bar[:,2]+.004)


@pytest.mark.parametrize('fingers,forces',[(.01,0.),(0.,.6)])
def test_visual_grasp_alignment_requires_open_unloaded_fingers(fingers,forces):
    route,s,logic=grasp_setup();s.fingers[:]=fingers;s.finger_forces[:]=forces
    for _ in range(12):route.advance(s,logic,torch.zeros(1,6),(0.,0.,1.5),grasp_quaternion=route.quaternion)
    assert route.stage.item()==0


def test_descent_orientation_drift_returns_to_standoff_only_if_empty():
    route,s,logic=grasp_setup();route.stage[:]=1;s.tcp[:,2]=s.bar[:,2]+.04
    route.advance(s,logic,torch.zeros(1,6),(0.,0.,1.5),grasp_quaternion=rotated(route.quaternion,8))
    assert route.stage.item()==0 and not route.grasp_alignment_blocked_contact.item()
    torch.testing.assert_close(route.targets(s,(0.,0.,1.5))[0][:,2],s.bar[:,2]+.10)
    route.stage[:]=1;s.tcp[:,2]=s.bar[:,2]+.004;s.finger_forces[:]=1.
    route.advance(s,logic,torch.zeros(1,6),(0.,0.,1.5),grasp_quaternion=rotated(route.quaternion,8))
    assert route.stage.item()==1 and route.grasp_alignment_blocked_contact.item()
    assert route.targets(s,(0.,0.,1.5))[2].item()==-1


@pytest.mark.parametrize('q',[torch.zeros(1,4),torch.full((1,4),float('nan')),torch.ones(1,3)])
def test_invalid_measured_grasp_pose_fails_closed(q):
    route,s,logic=grasp_setup()
    with pytest.raises(ValueError,match='Invalid measured grasp'):
        route.advance(s,logic,torch.zeros(1,6),(0.,0.,1.5),grasp_quaternion=q)


def test_route_requires_measured_descent_before_close_and_bilateral_hold_before_lift():
    route = LoadingRoute(1, 'cpu')
    pose = torch.tensor([[-1.15, 0., .522, 0., 0., 0., 1.]])
    route.reset(torch.tensor([0]), pose)
    s = SimpleNamespace(bar=pose[:, :3], tcp=pose[:, :3].clone(),
                        finger_forces=torch.zeros(1, 2), jaw_forces=torch.zeros(1, 4), fingers=torch.ones(1, 2))
    logic = SimpleNamespace(contact_dwell=torch.zeros(1), pick_complete=torch.tensor([False]), phase=torch.zeros(1))
    goal = (-.3, -.77, 1.5)
    joints = torch.zeros(1, 6)
    s.tcp[:, 2] += .10
    route.advance(s, logic, joints, goal)
    assert route.stage.item() == 1
    s.tcp[:, 2] = .542  # Too high: the regressed policy used to close here.
    route.advance(s, logic, joints, goal)
    assert route.targets(s, goal)[2].item() == -1
    s.tcp[:, 2] = .526
    route.advance(s, logic, joints, goal)
    assert route.stage.item() == 2 and route.targets(s, goal)[2].item() == 1
    logic.contact_dwell[:] = 8
    s.finger_forces[:] = torch.tensor([20., 0.])
    route.advance(s, logic, joints, goal)
    assert route.stage.item() == 2
    s.finger_forces[:] = 20
    route.advance(s, logic, joints, goal)
    assert route.stage.item() == 3


def test_route_partial_reset_preserves_other_episode_history():
    route = LoadingRoute(2, 'cpu')
    pose = torch.tensor([[-1.15, 0., .522, 0., 0., 0., 1.]])
    route.stage[:] = torch.tensor([7, 10])
    route.dwell[:] = 8
    route.reset(torch.tensor([0]), pose)
    assert route.stage.tolist() == [0, 10]
    assert route.dwell.tolist() == [0, 8]


def test_visual_retreat_waits_for_unloaded_fingers_and_continued_machine_support():
    route=LoadingRoute(1,'cpu',require_release_clearance=True);route.stage[:]=9
    state=SimpleNamespace(bar=torch.zeros(1,3),tcp=torch.zeros(1,3),fingers=torch.zeros(1,2),
        finger_forces=torch.tensor([[60.,0.]]),jaw_forces=torch.full((1,4),15.))
    logic=SimpleNamespace(contact_dwell=torch.zeros(1),pick_complete=torch.ones(1,dtype=torch.bool),phase=torch.tensor([2]))
    for _ in range(10):route.advance(state,logic,torch.zeros(1,6),(0.,0.,1.5))
    assert route.stage.item()==9
    state.finger_forces[:]=0
    for _ in range(2):route.advance(state,logic,torch.zeros(1,6),(0.,0.,1.5))
    assert route.stage.item()==9
    state.jaw_forces[:,0]=0
    route.advance(state,logic,torch.zeros(1,6),(0.,0.,1.5))
    assert route.release_dwell.item()==0
    state.jaw_forces[:]=15
    for _ in range(3):route.advance(state,logic,torch.zeros(1,6),(0.,0.,1.5))
    assert route.stage.item()==10
