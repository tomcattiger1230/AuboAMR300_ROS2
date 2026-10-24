from copy import deepcopy
import math
from pathlib import Path
from types import SimpleNamespace
import pytest

torch = pytest.importorskip('torch')
from rebar_training.route import LoadingRoute
from rebar_training.vision.servo import RobotServo
from rebar_training.workcells import WorkcellLayout, Pose
from rebar_training.workcell_mission import MissionFlow, MissionTask
from rebar_training.workcell_check import fixture_result
from rebar_training.station_skill_adapter import StationLoadingAdapter, precision_dock_for_station, source_observer_for_command

ROOT = Path(__file__).parents[1]


def setup_loading(source='source_A', machine='machine_1'):
    layout = WorkcellLayout.load(ROOT/'config/multi_workcell_offline.json')
    flow = MissionFlow(layout, MissionTask('task', source, machine, layout.steel_id));now = 1
    while True:
        c = flow.command(now)
        if c['kind'] == 'load_machine':
            return layout, flow, c, now
        stamp = now+(.21 if c['kind'] == 'observe_source' else .01)
        flow.accept(fixture_result(flow, c, stamp), stamp);now = stamp+.01


def state_and_route():
    route = LoadingRoute(1, 'cpu')
    route.reset(torch.tensor([0]), torch.tensor([[.2927, 0, .715, 0, 0, 0, 1]]))
    state = SimpleNamespace(bar=torch.tensor([[.2927, 0, .715]]), tcp=torch.tensor([[.2927, 0, .815]]),
                            finger_forces=torch.zeros(1, 2), fingers=torch.zeros(1, 2), jaw_forces=torch.zeros(1, 4))
    return state, route


@pytest.mark.parametrize('source', ['source_A', 'source_B'])
@pytest.mark.parametrize('machine', ['machine_1', 'machine_2', 'machine_3'])
def test_all_station_commands_drive_existing_route_and_servo_math_without_an_environment(source, machine):
    layout, flow, c, now = setup_loading(source, machine);adapter = StationLoadingAdapter(layout, c)
    base = layout.navigation_goal(machine).values();state, route = state_and_route();route.stage[:] = 6
    kwargs = dict(base_pose_map=base, robot_stamp_s=now+.01, now=now+.01, measured_twist=[0, 0])
    g = adapter.geometry(**kwargs)
    assert g['goal'] == pytest.approx([-.3, -.77, 1.508])
    desired, quat, grip = adapter.route_targets(route, state, **kwargs)
    assert desired[0].tolist() == pytest.approx([-.3, -.71, 1.508])
    assert quat[0].tolist() == pytest.approx([math.sqrt(.5), 0, 0, math.sqrt(.5)])
    # Exercise the real RobotServo.route_actions adapter while capturing its
    # Cartesian request; no FK/Jacobian device or actuator is available here.
    env = SimpleNamespace(task_spec=SimpleNamespace(insert_goal=(999, 999, 999), arm_speed=.35),
                          robot=SimpleNamespace(data=SimpleNamespace(joint_pos=SimpleNamespace(torch=torch.zeros(1, 6)))),
                          control_ids=list(range(6)))
    class CaptureServo(RobotServo):
        def cartesian_actions(self, desired, quaternion, grip):
            self.target = desired.clone();self.orientation = quaternion.clone()
            return torch.zeros(1, 7)
    servo = CaptureServo(env);visual = fixture_result(flow, c, now+.01)['evidence']
    actions = adapter.servo_actions(servo, state, route, **kwargs, visual_evidence=visual)
    torch.testing.assert_close(servo.target, desired);torch.testing.assert_close(servo.orientation, quat)
    assert actions.shape == (1, 7) and flow.plan['policy']['execution_authorized'] is False


def test_rotated_fixture_requires_correct_measured_orientation_before_insertion():
    layout = WorkcellLayout.load(ROOT/'config/multi_workcell_offline.json')
    # Deliberately use a fixed base frame to expose world-yaw dependencies.
    g = layout.machine_geometry('machine_2', refined_pose=layout.stations['machine_2'].pose_map, base_pose=Pose(0, 0, 0, 0))
    state, route = state_and_route();route.stage[:] = 6
    state.tcp[:] = torch.tensor([g['standoff']])
    logic = SimpleNamespace(contact_dwell=torch.zeros(1), pick_complete=torch.ones(1, dtype=torch.bool), phase=torch.zeros(1))
    q = torch.tensor([[math.sqrt(.5), 0, 0, math.sqrt(.5)]])
    route.advance(state, logic, torch.zeros(1, 6), g['goal'], grasp_quaternion=q, loading_geometry=g)
    assert route.stage.item() == 6
    route.advance(state, logic, torch.zeros(1, 6), g['goal'], loading_geometry=g)
    assert route.stage.item() == 6  # Missing measured orientation is never enough.
    route.advance(state, logic, torch.zeros(1, 6), g['goal'], grasp_quaternion=torch.tensor([g['tcp_quaternion_xyzw']]), loading_geometry=g)
    assert route.stage.item() == 7
    desired, quat, _ = route.targets(state, g['goal'], loading_geometry=g)
    assert quat[0].tolist() == pytest.approx([.5, .5, .5, .5])


def test_legacy_default_targets_are_identical_to_original_machine_geometry():
    layout = WorkcellLayout.load(ROOT/'config/multi_workcell_offline.json')
    g = layout.machine_geometry('machine_1', refined_pose=layout.stations['machine_1'].pose_map,
                                base_pose=layout.navigation_goal('machine_1'))
    state, route = state_and_route()
    for stage in range(11):
        route.stage[:] = stage
        old = route.targets(state, g['goal']);new = route.targets(state, g['goal'], loading_geometry=g)
        for a, b in zip(old, new):
            torch.testing.assert_close(a, b)


@pytest.mark.parametrize('mutation', [lambda g:g.update(frame='map'), lambda g:g.update(refined=False),
                                    lambda g:g.update(goal=[0, 0, 0]), lambda g:g.update(standoff=g['goal']),
                                    lambda g:g.update(retreat_vector=[0, 0, 0]),
                                    lambda g:g.update(tcp_quaternion_xyzw=[0, 0, 0, 0]),
                                    lambda g:g.update(standoff=[float('nan'), 0, 0]), lambda g:g.update(goal=None)])
def test_malformed_geometry_never_advances_the_existing_route(mutation):
    layout, _, c, _ = setup_loading();g = deepcopy(c['args']['geometry']);goal = g['goal']
    state, route = state_and_route();route.stage[:] = 6;mutation(g)
    with pytest.raises(ValueError):
        route.advance(state, SimpleNamespace(), torch.zeros(1, 6), goal, loading_geometry=g)
    assert route.stage.item() == 6


def test_loading_adapter_refuses_stale_moving_base_and_policy_unblocking():
    layout, flow, c, now = setup_loading();adapter = StationLoadingAdapter(layout, c);base = layout.navigation_goal('machine_1').values()
    with pytest.raises(ValueError, match='Stale'):
        adapter.geometry(base, now, now+1, [0, 0])
    with pytest.raises(ValueError, match='settled'):
        adapter.geometry(base, now+.01, now+.01, [.02, 0])
    bad = deepcopy(c);bad['args']['policy']['execution_authorized'] = True
    with pytest.raises(ValueError, match='blocked'):
        StationLoadingAdapter(layout, bad)
    state, route = state_and_route();e = fixture_result(flow, c, now+.01)['evidence'];e['image_stamp_s'] -= 1
    with pytest.raises(ValueError, match='Stale'):
        adapter.advance_route(route, state, SimpleNamespace(), torch.zeros(1, 6), measured_quaternion=[0, 0, 0, 1],
                              base_pose_map=base, robot_stamp_s=now+.01, now=now+.01,
                              measured_twist=[0, 0], visual_evidence=e)


def test_each_source_configures_existing_image_observer_without_a_steel_pose_hint():
    layout = WorkcellLayout.load(ROOT/'config/multi_workcell_offline.json')
    for name in ('source_A', 'source_B'):
        flow = MissionFlow(layout, MissionTask('task', name, 'machine_1', layout.steel_id))
        c = flow.command(1);flow.accept(fixture_result(flow, c, 1.01), 1.01);c = flow.command(1.02)
        sentinel = object();base = layout.navigation_goal(name)
        observer = source_observer_for_command(layout, c, detector=sentinel, base_pose_map=base.values(),
                                               robot_stamp_s=1.03, now=1.03, measured_twist=[0, 0])
        assert observer.detector is sentinel and observer.plane_height == pytest.approx(.522)
        assert observer.survey_prior is None and observer.acquired is False and observer.fixed_support_prior
        dock = precision_dock_for_station(layout, name)
        assert dock.goal == pytest.approx([base.x, base.y, base.yaw])


@pytest.mark.parametrize('fault', ['emergency_stop', 'bad_state', 'unknown_origin', 'wrong_machine', 'machine_busy'])
def test_skill_bridge_never_computes_commands_from_invalid_feedback(fault):
    layout, flow, c, now = setup_loading();adapter = StationLoadingAdapter(layout, c)
    state, route = state_and_route();e = fixture_result(flow, c, now+.01)['evidence']
    if fault == 'emergency_stop':
        e['emergency_stop'] = True
    elif fault == 'unknown_origin':
        e['origin'] = 'simulated_truth'
    elif fault == 'wrong_machine':
        e['visual_id'] = 'another_machine'
    elif fault == 'machine_busy':
        e['machine_busy'] = True
    else:
        state.tcp[:] = float('nan')
    class NeverServo:
        def route_actions(self, *a, **kw):
            raise AssertionError('Invalid feedback reached the servo')
    with pytest.raises(ValueError):
        adapter.servo_actions(NeverServo(), state, route, base_pose_map=layout.navigation_goal('machine_1').values(),
                              robot_stamp_s=now+.01, now=now+.01, measured_twist=[0, 0], visual_evidence=e)
