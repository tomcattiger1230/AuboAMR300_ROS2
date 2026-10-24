import ast
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from rebar_training.workcells import WorkcellLayout, Pose
from rebar_training.policy_compatibility import (StationPolicyAdapter, FrozenStationResidual,
    PolicySample, compose_residual_actions, REFERENCE_SOURCE, REFERENCE_MACHINE, SUPPORTED_ROUTES)
from rebar_training.offline_execution_check import synthetic_sample, equivalence_check, native_partial_audit
from rebar_training.workcell_mission import PICKUP_STAGES, LOADING_STAGES

ROOT=Path(__file__).parents[1]
CHECKPOINT=ROOT/'artifacts/mobile_curriculum_20261020/source_return_v2_frozen_acceptance_20261022/frozen_policy.pt'


def reference_layout():
    d=json.loads((ROOT/'config/multi_workcell_offline.json').read_text())
    d['stations'][0]['pose_map_xyz_yaw']=REFERENCE_SOURCE.values()
    d['stations'][2]['pose_map_xyz_yaw']=REFERENCE_MACHINE.values()
    return WorkcellLayout(d)


@pytest.fixture(scope='module')
def policy():
    if not CHECKPOINT.exists():pytest.skip('Frozen artifact must be restored before actual-model tests')
    return FrozenStationResidual(CHECKPOINT)


@pytest.fixture
def layout():return WorkcellLayout.load(ROOT/'config/multi_workcell_offline.json')


from rebar_training.legacy_observation_probe import legacy_observation


@pytest.mark.parametrize('phase', [*PICKUP_STAGES[:-1],*LOADING_STAGES])
def test_original_layout_reproduces_actual_legacy_encoder_and_activation_masks(phase):
    layout=reference_layout();adapter=StationPolicyAdapter(layout,'source_A','machine_1')
    for route in SUPPORTED_ROUTES[phase]:
        for ready,recovery in ((True,False),(False,False),(True,True)):
            sample=replace(synthetic_sample(layout,'source_A','machine_1',phase,route,10),motion_ready=ready,recovery_pending=recovery)
            old,active=legacy_observation(sample,layout);new=adapter.encode(sample)
            np.testing.assert_allclose(new.values,old,atol=3e-7,rtol=0)
            assert new.active==active


def test_independent_station_positions_preserve_frozen_context_but_not_motion_goal(layout,policy):
    report=equivalence_check(layout,policy)
    assert report['passed'] and report['pairs']==138
    assert report['max_actor_abs_difference']<=2e-6
    assert report['source_context_different_from_real_destination_cases']>0
    packet=StationPolicyAdapter(layout,'source_B','machine_3').encode(synthetic_sample(layout,'source_B','machine_3','pickup',3,10))
    assert math.dist(packet.actual_machine_goal_base,packet.policy_context_goal_base)>1
    actual=layout.machine_geometry('machine_3',refined_pose=layout.stations['machine_3'].pose_map,
                                    base_pose=layout.navigation_goal('source_B'))['goal']
    assert packet.actual_machine_goal_base==pytest.approx(actual)


@pytest.mark.parametrize('change', [dict(route=True),dict(route=11),dict(route=6),dict(phase='navigate'),dict(phase='observe_source'),
    dict(robot_stamp_s=8),dict(image_stamp_s=11),dict(vision_valid='true'),dict(q=[float('nan')]*6),
    dict(tcp_quaternion_xyzw=[0,0,0,0]),dict(bar_axis_base=[0,0,0]),dict(base_pose_map=[100,100,0,0]),
    dict(finger_forces_n=[-1,0])])
def test_bad_or_unrelated_inputs_never_reach_the_actor(layout,change):
    sample=replace(synthetic_sample(layout,'source_A','machine_1','pickup',3,10),**change)
    with pytest.raises(ValueError):StationPolicyAdapter(layout,'source_A','machine_1').encode(sample)


def test_no_privileged_fields_can_enter_typed_sensor_contract(layout):
    from dataclasses import asdict
    data=asdict(synthetic_sample(layout,'source_A','machine_1','pickup',3,10));data['true_bar_for_evaluation']=[1,2,3]
    with pytest.raises(TypeError):PolicySample(**data)


def test_actor_loader_does_not_construct_or_load_an_optimizer_or_save(monkeypatch):
    if not CHECKPOINT.exists():pytest.skip('Frozen artifact unavailable')
    def denied(*a,**k):raise AssertionError('Training/update/save API was called')
    monkeypatch.setattr(torch.optim,'Adam',denied);monkeypatch.setattr(torch,'save',denied)
    before=hashlib.sha256(CHECKPOINT.read_bytes()).hexdigest()
    policy=FrozenStationResidual(CHECKPOINT);proof=policy.verify_unchanged()
    assert proof['model_optimizer_tensors']==52 and proof['counters']==dict(updates=466,actor_updates=170,policy_steps=36602)
    assert all(p.device.type=='cpu' and p.dtype==torch.float32 and not p.requires_grad for p in policy.model.parameters())
    assert not hasattr(policy,'optimizer') and not hasattr(policy,'update') and not hasattr(policy,'save')
    assert hashlib.sha256(CHECKPOINT.read_bytes()).hexdigest()==before


def test_inactive_packets_skip_inference_and_residual_never_changes_gripper(layout,policy,monkeypatch):
    adapter=StationPolicyAdapter(layout,'source_A','machine_1')
    packet=adapter.encode(synthetic_sample(layout,'source_A','machine_1','pickup',0,10))
    with monkeypatch.context() as m:
        m.setattr(policy.model,'forward',lambda *_:(_ for _ in ()).throw(AssertionError('Inactive actor called')))
        assert policy.infer(packet)['scaled_delta']==[0.]*6
    active=adapter.encode(synthetic_sample(layout,'source_A','machine_1','pickup',3,10));decision=policy.infer(active)
    nominal=torch.tensor([[.99,-.99,.1,0,0,0,-1.]])
    mixed=compose_residual_actions(nominal,decision)
    assert mixed[0,6].item()==-1 and (mixed[:,:6]-nominal[:,:6]).abs().max()<=.080001
    assert mixed.abs().max()<=1
    policy.verify_unchanged()


def test_original_native_report_is_only_a_partial_measured_field_audit():
    path=CHECKPOINT.parent/'demo_holdout_9321042.json'
    if not path.exists():pytest.skip('Native report unavailable')
    audit=native_partial_audit(path)
    assert audit['measured_rows']>0 and not audit['ground_truth_fields_used']
    assert not audit['complete_76d_replay_available'] and audit['missing_fields']
    assert all('evaluation' not in k and not k.startswith('true_') for k in audit['permitted_fields'])


def test_real_controller_sensor_bridge_preserves_values_and_clock_staleness():
    from rebar_training.legacy_observation_probe import legacy_control_fixture
    from rebar_training.policy_compatibility import sample_from_legacy_control
    layout=reference_layout();sample=replace(synthetic_sample(layout,'source_A','machine_1','pickup',3,10),image_stamp_s=9.9)
    control=legacy_control_fixture(sample,layout)
    class Forbidden:
        def __getattribute__(self,name):raise AssertionError('Steel truth was accessed')
    control.env.rebar=Forbidden();control.env.true_bar_for_evaluation=Forbidden()
    captured=sample_from_legacy_control(control,inference_now_s=110,clock_offset_s=100)
    packet=StationPolicyAdapter(layout,'source_A','machine_1').encode(captured)
    old,_=legacy_observation(sample,layout)
    np.testing.assert_allclose(packet.values,old,atol=3e-7,rtol=0)
    assert captured.image_stamp_s==pytest.approx(109.9) and captured.robot_stamp_s==110
    stale=sample_from_legacy_control(control,inference_now_s=110,clock_offset_s=0)
    with pytest.raises(ValueError,match='Stale'):
        StationPolicyAdapter(layout,'source_A','machine_1').encode(stale)


def test_whole_map_relocation_does_not_change_base_local_sensor_values(layout,policy):
    import random
    rng=random.Random(1024);baseline=StationPolicyAdapter(layout,'source_B','machine_3')
    for _ in range(20):
        data=layout.to_dict();world=Pose(rng.uniform(-30,30),rng.uniform(-30,30),0,rng.uniform(-math.pi,math.pi))
        for station in data['stations']:
            station['pose_map_xyz_yaw']=world.compose(Pose.parse(station['pose_map_xyz_yaw'])).values()
        moved=WorkcellLayout(data);adapter=StationPolicyAdapter(moved,'source_B','machine_3')
        for phase,route in (('pickup',3),('loading',7)):
            original=synthetic_sample(layout,'source_B','machine_3',phase,route,10)
            translated=replace(original,base_pose_map=world.compose(Pose.parse(original.base_pose_map)).values())
            a=baseline.encode(original);b=adapter.encode(translated)
            np.testing.assert_allclose(a.values,b.values,atol=2e-6,rtol=0)
            assert policy.infer(a)['raw']==pytest.approx(policy.infer(b)['raw'],abs=2e-6)


def test_detached_layout_copy_preserves_identity_and_cannot_mutate_registry(layout):
    data=layout.to_dict();assert WorkcellLayout(data).sha256==layout.sha256
    data['stations'][0]['pose_map_xyz_yaw'][0]=100
    assert layout.stations['source_A'].pose_map.x==0


def test_tilted_chassis_frame_is_rejected_instead_of_silently_flattened():
    from rebar_training.legacy_observation_probe import legacy_control_fixture
    from rebar_training.policy_compatibility import sample_from_legacy_control
    layout=reference_layout();control=legacy_control_fixture(synthetic_sample(layout,'source_A','machine_1','pickup',3,10),layout)
    angle=.01;c,s=math.cos(angle),math.sin(angle)
    control.frame=lambda:SimpleNamespace(world_from_base=np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]]))
    with pytest.raises(ValueError,match='planar'):
        sample_from_legacy_control(control,inference_now_s=10)
