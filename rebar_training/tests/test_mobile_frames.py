import numpy as np
from rebar_training.vision.capture import optical_transform
from rebar_training.vision.frames import BaseFrame
from rebar_training.vision.observer import CameraPacket,RobotPacket


def test_camera_and_robot_packets_remain_consistent_when_chassis_translates_and_turns():
    q=np.array([0,0,np.sin(.7/2),np.cos(.7/2)])
    base=BaseFrame([4.,3.,0.],q);T=base.world_from_base
    local=np.array([.2927,0,.715]);world=T[:3,:3]@local+T[:3,3]
    packet=RobotPacket(world,q,np.zeros(2),np.zeros(2),np.zeros(4),100)
    observed=base.robot_packet(packet)
    np.testing.assert_allclose(observed.tcp,local,atol=1e-12)
    np.testing.assert_allclose(observed.quaternion_xyzw,[0,0,0,1],atol=1e-12)
    camera=CameraPacket(np.zeros((16,16),np.uint8),np.eye(3),T@np.asarray(
        optical_transform([-.45,0,1.45],[0,0,0,1])),100,1)
    actual=base.camera_packet(camera)
    np.testing.assert_allclose(actual.T_scene_optical[:3,3],[-.45,0,1.45],atol=1e-12)
    assert actual.mono8 is camera.mono8


def test_loading_servo_uses_transformed_machine_goal_after_mobile_docking():
    import pytest
    torch=pytest.importorskip('torch')
    from types import SimpleNamespace
    from rebar_training.route import LoadingRoute
    from rebar_training.vision.servo import RobotServo
    route=LoadingRoute(1,'cpu');route.stage[:]=6
    state=SimpleNamespace(bar=torch.zeros(1,3),tcp=torch.zeros(1,3))
    env=SimpleNamespace(task_spec=SimpleNamespace(insert_goal=(-.87,-.08,1.508),arm_speed=.35),
        robot=SimpleNamespace(data=SimpleNamespace(joint_pos=SimpleNamespace(torch=torch.zeros(1,6)))),
        control_ids=list(range(6)))
    class CaptureServo(RobotServo):
        def cartesian_actions(self,desired,quaternion,grip):
            self.desired=desired;return torch.zeros(1,7)
    servo=CaptureServo(env)
    servo.route_actions(state,route,(-.3,-.77,1.508))
    np.testing.assert_allclose(servo.desired[0],[-.3,-.71,1.508],atol=1e-6)
    servo.route_actions(state,route)
    np.testing.assert_allclose(servo.desired[0],[-.87,-.02,1.508],atol=1e-6)


def test_mobile_alignment_reserves_visual_error_budget_before_clamping():
    import pytest
    torch=pytest.importorskip('torch')
    from types import SimpleNamespace
    from rebar_training.spec import TaskSpec
    from rebar_training.vision.progress import VisualProgress
    spec=TaskSpec(task='loading',position_tolerance=.003)
    logic=VisualProgress(spec,1,'cpu');logic.alignment_tolerance=.0008
    logic.pick_complete[:]=True;logic.phase[:]=1
    state=SimpleNamespace(bar=torch.tensor([spec.insert_goal])+torch.tensor([[.002,0,0]]),
        bar_axis=torch.tensor([[0.,0.,1.]]),linear_velocity=torch.zeros(1,3),angular_velocity=torch.zeros(1,3),
        finger_forces=torch.full((1,2),20.),jaw_forces=torch.zeros(1,4),fingers=torch.full((1,2),.015))
    state.tcp=state.bar+torch.tensor([[0.,0.,.004]])
    for _ in range(spec.align_hold_steps+1):logic.advance(state,torch.zeros(1,7),torch.tensor([True]))
    assert logic.phase.item()==1 and spec.position_tolerance==.003
    state.bar[:]=torch.tensor([spec.insert_goal])+torch.tensor([[.0005,0,0]])
    for _ in range(spec.align_hold_steps):logic.advance(state,torch.zeros(1,7),torch.tensor([True]))
    assert logic.phase.item()==2
