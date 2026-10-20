import pytest
from rebar_training.vision.recovery import tray_reobservation_allowed,TrayGraspRecovery


def state(**updates):
    return dict(dict(stage='pickup_tray',route=0,returned=True,valid=False,anchored=False,
        timestamp_ns=2_000_000_000,last_good_ns=1_000_000_000,
        fingers=[0.,0.],finger_forces=[0.,0.],tray_force=20.),**updates)


def test_stale_empty_wrist_over_supported_steel_can_reobserve():
    assert tray_reobservation_allowed(**state())
    assert tray_reobservation_allowed(**state(route=1))


@pytest.mark.parametrize('change',[
    dict(stage='loading'),dict(route=2),dict(returned=False),dict(valid=True),dict(anchored=True),
    dict(fingers=[.015,.015]),dict(finger_forces=[2.,0.]),dict(tray_force=0.),
    dict(tray_force=float('nan')),dict(fingers=[float('nan'),0.]),dict(finger_forces=[-1.,0.]),
    dict(last_good_ns=1_900_000_000),dict(last_good_ns=2_100_000_000),
])
def test_reobservation_never_moves_a_loaded_or_unreturned_wrist(change):
    assert not tray_reobservation_allowed(**state(**change))


def grasp(**updates):
    return dict(dict(state(route=2,fingers=[.017,.017],finger_forces=[55.,53.],tray_force=36.),
                     tcp=[.2927,0.,.724],reobservations=0),**updates)


def test_supported_partial_grasp_opens_before_any_reobservation():
    recovery=TrayGraspRecovery()
    assert recovery.step(**grasp())=='open' and recovery.pending
    assert recovery.step(**grasp(timestamp_ns=2_033_333_333,valid=True))=='open'
    for tick in range(2):
        assert recovery.step(**grasp(timestamp_ns=2_100_000_000+tick*34_000_000,
            fingers=[0.,0.],finger_forces=[0.,0.]))=='open'
    assert recovery.step(**grasp(timestamp_ns=2_200_000_000,
        fingers=[0.,0.],finger_forces=[0.,0.]))=='reobserve'
    assert not recovery.pending


@pytest.mark.parametrize('change',[dict(stage='loading'),dict(route=3),dict(anchored=True),
    dict(returned=False),dict(valid=True),dict(tray_force=0.),dict(tray_force=float('nan')),
    dict(tcp=[.2927,0.,.80]),dict(tcp=[.40,0.,.72]),dict(finger_forces=[float('nan'),0.]),
    dict(finger_forces=[251.,0.]),dict(last_good_ns=1_900_000_000)])
def test_partial_grasp_release_cannot_drop_carried_or_unobserved_steel(change):
    assert TrayGraspRecovery().step(**grasp(**change)) is None


def test_partial_grasp_release_exhaustion_or_support_loss_fails_closed():
    recovery=TrayGraspRecovery()
    assert recovery.step(**grasp(reobservations=2))=='failed'
    recovery=TrayGraspRecovery();assert recovery.step(**grasp())=='open'
    assert recovery.step(**grasp(timestamp_ns=2_100_000_000,tray_force=0.))=='failed'
    recovery=TrayGraspRecovery();assert recovery.step(**grasp())=='open'
    assert recovery.step(**grasp(timestamp_ns=5_100_000_000))=='failed'


def test_release_action_holds_all_arm_joints_and_excludes_residual():
    import ast
    from pathlib import Path
    from types import SimpleNamespace
    torch=pytest.importorskip('torch')
    tree=ast.parse((Path(__file__).parents[1]/'src/rebar_training/mobile_env.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MobileControl')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='action')
    namespace={};exec(compile(ast.Module(body=[method],type_ignores=[]),'mobile_action','exec'),namespace)
    controller=SimpleNamespace(env=SimpleNamespace(actions=torch.ones(1,7)),
                               grasp_recovery=SimpleNamespace(pending=True))
    action=namespace['action'](controller,residual=torch.ones(1,6))
    assert torch.equal(action,torch.tensor([[0.,0.,0.,0.,0.,0.,-1.]]))
