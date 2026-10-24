from copy import deepcopy
import json
import math
from pathlib import Path
import random

import pytest

from rebar_training.workcells import Pose, WorkcellLayout
from rebar_training.workcell_mission import MissionFlow, MissionTask, plan_task
from rebar_training.workcell_check import validate_layout

CONFIG = Path(__file__).parents[1]/'config/multi_workcell_offline.json'


@pytest.fixture
def data():
    return json.loads(CONFIG.read_text())


@pytest.fixture
def layout(data):
    return WorkcellLayout(data)


def test_all_two_source_three_machine_routes_replay_with_explicit_synthetic_scope(layout):
    report = validate_layout(layout)
    assert report['passed'] and report['source_count'] == 2 and report['machine_count'] == 3
    assert len(report['combinations']) == 6
    assert all(r['completed_steps'] == 6 and r['state'] == 'completed' for r in report['combinations'])
    assert len(report['rejected_counterexamples']) == 20
    assert all(r['passed'] for r in report['rejected_counterexamples'])
    assert report['physical_acceptance'] is False and report['policy_inference_executed'] is False


def test_machine_geometry_and_docking_are_invariant_under_whole_layout_relocation(data):
    baseline = WorkcellLayout(data);rng = random.Random(61024)
    for _ in range(60):
        moved = deepcopy(data);world = Pose(rng.uniform(-30, 30), rng.uniform(-30, 30), 0, rng.uniform(-math.pi, math.pi))
        for s in moved['stations']:
            s['pose_map_xyz_yaw'] = world.compose(Pose.parse(s['pose_map_xyz_yaw'])).values()
        actual = WorkcellLayout(moved)
        for name in baseline.stations:
            old = baseline.navigation_goal(name);new = actual.navigation_goal(name)
            expected = world.compose(old)
            assert new.values()[:3] == pytest.approx(expected.values()[:3], abs=1e-10)
            assert math.sin(new.yaw) == pytest.approx(math.sin(expected.yaw), abs=1e-10)
            if baseline.profiles[baseline.stations[name].profile_id].kind == 'machine':
                a = actual.machine_geometry(name, refined_pose=actual.stations[name].pose_map, base_pose=new)
                b = baseline.machine_geometry(name, refined_pose=baseline.stations[name].pose_map, base_pose=old)
                for key in ('goal', 'standoff', 'retreat_vector', 'bar_axis', 'tcp_quaternion_xyzw'):
                    assert a[key] == pytest.approx(b[key], abs=1e-10)


def test_rotated_machine_changes_approach_direction_and_wrist_orientation(layout):
    base = Pose(0, 0, 0, 0);s = layout.stations['machine_2']
    g = layout.machine_geometry(s.id, refined_pose=s.pose_map, base_pose=base)
    assert g['goal'] == pytest.approx([10, 4, 1.508])
    assert g['standoff'] == pytest.approx([9.94, 4, 1.508])
    assert g['tcp_quaternion_xyzw'] == pytest.approx([.5, .5, .5, .5])
    assert g['bar_axis'] == pytest.approx([0, 0, 1])


def test_pose_inverse_composition_round_trip_and_wrap():
    p = Pose(4, -2, .3, math.pi*.99);q = Pose(-1, 2, .2, math.pi*.2)
    point = (.2, -.8, 1.5)
    assert p.inverse_point(p.point(point)) == pytest.approx(point)
    assert p.compose(q).point(point) == pytest.approx(p.point(q.point(point)))
    assert -math.pi <= p.compose(q).yaw <= math.pi


def test_source_region_is_not_a_ground_truth_bar_pose_and_checks_both_ends(layout):
    prior = layout.source_prior('source_B')
    assert prior['steel_pose_provided'] is False and 'bar_centre_map' not in prior
    s = layout.stations['source_B']
    assert layout.contains_bar(s.id, s.pose_map.point([0, 0, .522]), s.pose_map.vector([1, 0, 0]))
    assert not layout.contains_bar(s.id, s.pose_map.point([.7, 0, .522]), s.pose_map.vector([1, 0, 0]))
    assert not layout.contains_bar(s.id, s.pose_map.point([0, 0, .9]), s.pose_map.vector([1, 0, 0]))


@pytest.mark.parametrize('mutation', [
    lambda d:d['stations'].append(deepcopy(d['stations'][0])),
    lambda d:d['profiles'].append(deepcopy(d['profiles'][0])),
    lambda d:d['stations'][1].update(visual_id=d['stations'][0]['visual_id']),
    lambda d:d['stations'][1].update(navigation_id=d['stations'][0]['navigation_id']),
    lambda d:d['stations'][0].update(profile_id='missing'),
    lambda d:d['stations'][0].update(pose_map_xyz_yaw=[0, 0, 0, float('nan')]),
    lambda d:d['stations'][0].update(pose_map_xyz_yaw=[True, 0, 0, 0]),
    lambda d:d['profiles'][0].update(search_bounds_xy=[1, 0, -1, 0]),
    lambda d:d['profiles'][1].update(bar_axis=[0, 1, 0]),
    lambda d:d['profiles'][1].update(standoff_offset=[0, .01, 0]),
    lambda d:d['steel'].update(length_m=-1),
    lambda d:d.update(unknown_control_option='launch'),
])
def test_bad_layouts_are_rejected(data, mutation):
    mutation(data)
    with pytest.raises(ValueError):
        WorkcellLayout(data)


def test_layout_and_outward_plan_snapshots_are_immutable(layout):
    with pytest.raises(AttributeError):
        layout.map_id = 'other'
    with pytest.raises(TypeError):
        layout.stations['bad'] = layout.stations['source_A']
    flow = MissionFlow(layout, MissionTask('task', 'source_A', 'machine_1', layout.steel_id))
    plan = flow.plan;plan['policy']['execution_authorized'] = True
    assert flow.plan['policy']['execution_authorized'] is False
    c = flow.command(1);c['args']['goal_map'] = [999, 999, 0, 0]
    assert flow.command(1.1)['args']['goal_map'] != c['args']['goal_map']


@pytest.mark.parametrize('source,machine,steel', [('missing', 'machine_1', 'rebar_24x600'),
                                                ('machine_1', 'source_A', 'rebar_24x600'),
                                                ('source_A', 'machine_1', 'different_steel')])
def test_task_target_and_specification_must_exist(layout, source, machine, steel):
    with pytest.raises(ValueError):
        plan_task(layout, MissionTask('task', source, machine, steel))


def test_unused_source_profile_does_not_replace_an_actual_source_station(data):
    data['stations'] = [s for s in data['stations'] if s['profile_id'] == 'test_machine_v1']
    with pytest.raises(ValueError, match='Both source and machine'):
        WorkcellLayout(data)
