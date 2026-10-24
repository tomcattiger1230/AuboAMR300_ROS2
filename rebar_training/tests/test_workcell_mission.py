from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys

import pytest

from rebar_training.workcells import WorkcellLayout
from rebar_training.workcell_mission import MissionFlow, MissionTask
from rebar_training.workcell_check import fixture_result

ROOT = Path(__file__).parents[1]


@pytest.fixture
def layout():
    return WorkcellLayout.load(ROOT/'config/multi_workcell_offline.json')


def arrive_at(layout, kind, origin='synthetic_fixture'):
    flow = MissionFlow(layout, MissionTask('task', 'source_B', 'machine_3', layout.steel_id), evidence_origin=origin)
    now = 1
    while True:
        c = flow.command(now)
        if c['kind'] == kind:
            return flow, c, now
        stamp = now+(.21 if c['kind'] == 'observe_source' else .01)
        result = fixture_result(flow, c, stamp);result['evidence']['origin'] = origin
        assert flow.accept(result, stamp)['state'] in ('ready', 'completed')
        now = stamp+.01


@pytest.mark.parametrize('kind,mutate', [
    ('observe_source', lambda e:e.update(stable_frames=True)),
    ('observe_source', lambda e:e['stable_samples'][0].update(frame_id=e['frame_id'])),
    ('observe_source', lambda e:e['stable_samples'][1].update(centre_map=[10, 10, .522])),
    ('observe_source', lambda e:e.update(tf_stamp_s=e['image_stamp_s']-.01)),
    ('observe_source', lambda e:e.update(image_stamp_s=e['image_stamp_s']+1)),
    ('observe_source', lambda e:e.update(calibration_id='other')),
    ('observe_source', lambda e:e['T_map_optical'][0].__setitem__(0, -1)),
    ('observe_source', lambda e:e['T_map_optical'][1].__setitem__(1, 2)),
    ('observe_source', lambda e:e['K'][0].__setitem__(0, 0)),
    ('observe_source', lambda e:e.update(origin='untrusted_truth')),
    ('observe_source', lambda e:e.update(emergency_stop=True)),
    ('observe_machine', lambda e:e.update(station_pose_map=[100, 100, 0, 0])),
    ('observe_machine', lambda e:e.update(base_pose_map=[100, 100, 0, 0])),
    ('observe_machine', lambda e:e.update(measured_twist=[.02, 0])),
    ('navigate_machine', lambda e:e.update(tray_axis_base=[1, 0, 0])),
    ('navigate_machine', lambda e:e.update(tray_stamp_s=e['image_stamp_s']-1)),
    ('load_machine', lambda e:e.update(axis_error_deg=4)),
    ('load_machine', lambda e:e.update(retreat_m=float('nan'))),
    ('load_machine', lambda e:e.update(bar_linear_speed_mps=.041)),
    ('load_machine', lambda e:e.update(unsafe_peak_force_n=251)),
])
def test_bad_measured_evidence_blocks_without_advancing(layout, kind, mutate):
    flow, c, now = arrive_at(layout, kind);old = flow.index
    stamp = now+(.21 if kind == 'observe_source' else .01)
    result = fixture_result(flow, c, stamp);mutate(result['evidence'])
    hold = flow.accept(result, stamp)
    assert hold['kind'] == 'hold' and hold['preserve_grip'] and hold['stop_base'] and hold['hold_arm']
    assert flow.state == 'blocked' and flow.index == old
    assert flow.command(stamp+.01)['kind'] == 'hold'


def test_duplicate_or_out_of_order_receipt_cannot_complete_next_step(layout):
    flow, c, now = arrive_at(layout, 'navigate_source');r = fixture_result(flow, c, now+.01)
    flow.accept(r, now+.01);next_command = flow.command(now+.02)
    with pytest.raises(ValueError, match='Unmatched'):
        flow.accept(r, now+.03)
    assert flow.index == 1 and flow.state == 'waiting'
    assert flow.command(now+.04)['command_id'] == next_command['command_id']


def test_command_expiry_holds_and_does_not_retry_motion(layout):
    flow, c, now = arrive_at(layout, 'navigate_machine')
    hold = flow.command(c['deadline_s']+1)
    assert hold['kind'] == 'hold' and hold['preserve_grip']
    with pytest.raises(ValueError, match='Only blocked observation'):
        flow.retry_observation(c['deadline_s']+2)


def test_only_bounded_observation_retry_uses_new_identity_and_new_frames(layout):
    flow, c, now = arrive_at(layout, 'observe_source')
    old = fixture_result(flow, c, now+.21);old['evidence']['full_bar_visible'] = False
    assert flow.accept(old, now+.21)['kind'] == 'hold'
    new = flow.retry_observation(now+.22)
    assert new['command_id'] != c['command_id']
    with pytest.raises(ValueError, match='Unmatched'):
        flow.accept(old, now+.23)
    for _ in range(2):
        r = fixture_result(flow, new, new['issued_s']+.21);r['evidence']['full_bar_visible'] = False
        flow.accept(r, r['stamp_s'])
        if flow.attempt < 2:
            new = flow.retry_observation(r['stamp_s']+.01)
    with pytest.raises(ValueError, match='budget'):
        flow.retry_observation(flow.last_now+.01)


def test_stale_confirmation_cannot_start_pick_or_loading(layout):
    for kind in ('pickup_to_tray', 'load_machine'):
        flow, c, now = arrive_at(layout, kind)
        # The existing issued intent may still be cached; create expiry before first issue instead.
        flow.outstanding = None;flow.state = 'ready'
        assert flow.command(now+1)['kind'] == 'hold'
        assert flow.index == c['step']


def test_cancel_retains_payload_and_requires_no_simulator(layout):
    flow, c, now = arrive_at(layout, 'load_machine')
    hold = flow.cancel(now+.01)
    assert hold['preserve_grip'] and hold['stop_base'] and flow.state == 'cancelled'
    assert flow.command(now+.02)['kind'] == 'hold'
    with pytest.raises(ValueError):
        flow.accept(fixture_result(flow, c, now+.03), now+.03)


def test_recorded_replay_mode_does_not_accept_synthetic_receipts(layout):
    flow, c, now = arrive_at(layout, 'navigate_source', 'recorded_measurement')
    assert flow.accept(fixture_result(flow, c, now+.01), now+.01)['kind'] == 'hold'


def test_cli_help_and_plan_never_import_simulation_or_policy(tmp_path):
    code = """
import sys
from rebar_training.workcell_check import main
main(['plan','--layout',sys.argv[1],'--source','source_A','--machine','machine_2','--report',sys.argv[2]])
assert not any(n=='torch' or n.startswith(('isaac','gymnasium','pxr')) for n in sys.modules)
"""
    result = subprocess.run([sys.executable, '-c', code, str(ROOT/'config/multi_workcell_offline.json'), str(tmp_path/'plan.json')],
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    plan = json.loads((tmp_path/'plan.json').read_text())
    assert plan['policy']['execution_authorized'] is False
    assert plan['navigation_backend_connected'] is False


def test_replay_cli_accepts_recorded_contracts_and_does_not_overwrite_evidence(layout, tmp_path):
    flow, _, _ = arrive_at(layout, 'navigate_source', 'recorded_measurement');now = 1;events = []
    while flow.state != 'completed':
        c = flow.command(now);stamp = now+(.21 if c['kind'] == 'observe_source' else .01)
        r = fixture_result(flow, c, stamp);r['evidence']['origin'] = 'recorded_measurement'
        flow.accept(r, stamp);events.append(dict(issued_s=now, received_s=stamp, result=r));now = stamp+.01
    path = tmp_path/'events.json';path.write_text(json.dumps(events));report = tmp_path/'replay.json'
    # The event data here are test fixtures tagged for exercising the recorded-contract parser, not real sensor evidence.
    args = [sys.executable, '-m', 'rebar_training.workcell_check', 'replay', '--layout', str(ROOT/'config/multi_workcell_offline.json'),
            '--source', 'source_B', '--machine', 'machine_3', '--task-id', 'task', '--events', str(path), '--report', str(report)]
    result = subprocess.run(args, capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    assert json.loads(report.read_text())['physical_acceptance'] is False
    assert subprocess.run(args, capture_output=True, text=True, timeout=15).returncode != 0


def test_boolean_step_number_is_not_a_valid_command_identity(layout):
    flow, c, now = arrive_at(layout, 'observe_source');r = fixture_result(flow, c, now+.21);r['step'] = True
    with pytest.raises(ValueError, match='Unmatched result step'):
        flow.accept(r, now+.21)
    assert flow.index == 1
