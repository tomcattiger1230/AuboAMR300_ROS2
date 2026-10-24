"""Plan, replay recorded contracts, or validate a multi-workcell flow entirely offline.

The validate command fabricates explicitly labelled sensor/adapter receipts. It
never performs perception, physical navigation, policy inference, or simulation.
"""
import argparse
import hashlib
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

from .workcells import WorkcellLayout, FROZEN_POLICY_SHA256
from .workcell_mission import MissionFlow, MissionTask, PICKUP_STAGES, LOADING_STAGES, plan_task


def fixture_result(flow, command, now):
    """Synthetic interface fixture ONLY. Never a runtime sensor adapter."""
    layout = flow.layout;s = layout.station(command['station_id']);p = layout.profiles[s.profile_id]
    frame = 'fixture_'+str(command['step'])+'_'+str(round(now*1000000))
    e = dict(origin='synthetic_fixture', image_valid=True, calibration_id=layout.calibration_id,
             image_stamp_s=now, tf_stamp_s=now, frame_id=frame, tf_frame_id=frame,
             K=[[800, 0, 512], [0, 800, 307], [0, 0, 1]],
             T_map_optical=[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
             finger_positions_m=[0, 0], finger_forces_n=[0, 0], arm_stowed=True,
             emergency_stop=False, base_stopped=True, measured_twist=[0, 0], pose_stamp_s=now,
             base_pose_map=layout.navigation_goal(s.id).values(), visual_id=s.visual_id,
             machine_ready=True, machine_busy=False)
    kind = command['kind']
    if kind.startswith('navigate_'):
        e.update(localization_valid=True, path_checked=True, navigation_id=s.navigation_id)
    if kind in ('pickup_to_tray', 'navigate_machine'):
        e.update(tray_stamp_s=now, tray_load_n=20, tray_centre_base=[.2927, 0, .715],
                 tray_axis_base=[0, 1, 0], bar_stable=True)
    if kind == 'observe_source':
        centre = s.pose_map.point((0, 0, p.support_height_m));axis = s.pose_map.vector((1, 0, 0))
        e.update(full_bar_visible=True, stable_frames=3, bar_centre_map=centre, bar_axis_map=axis,
                 diameter_m=layout.diameter_m, length_m=layout.length_m,
                 stable_samples=[dict(stamp_s=now-dt, frame_id=frame+suffix, centre_map=centre, axis_map=axis)
                                 for dt, suffix in ((.2, '_a'), (.1, '_b'), (0, ''))])
    elif kind == 'pickup_to_tray':
        e.update(stages=list(PICKUP_STAGES), source_grasp_verified=True)
    elif kind == 'observe_machine':
        e.update(station_pose_map=s.pose_map.values())
    elif kind == 'load_machine':
        e.update(stages=list(LOADING_STAGES), jaw_forces_n=[15, 15, 15, 15],
                 position_error_m=.0012, axis_error_deg=.1, retreat_m=.085,
                 bar_linear_speed_mps=0, bar_angular_speed_radps=0, unsafe_peak_force_n=26,
                 terminal_arm_contact_n=0, peak_arm_contact_n=0, release_verified=True,
                 continuity_verified=True)
    return dict(**{k:command[k] for k in ('task_id', 'command_id', 'step', 'station_id', 'map_id', 'layout_sha256')},
                status='succeeded', stamp_s=now, evidence=e)


def run_fixture(layout, task, mutation=None):
    flow = MissionFlow(layout, task);now = 1.;events = [];rejection = None
    while flow.state not in ('completed', 'blocked', 'cancelled'):
        c = flow.command(now)
        if c['kind'] == 'hold':
            break
        stamp = now+(.21 if c['kind'] == 'observe_source' else .01)
        result = fixture_result(flow, c, stamp)
        applied = bool(mutation and mutation(c, result))
        try:
            outcome = flow.accept(result, stamp)
        except ValueError as ex:
            rejection = str(ex);outcome = dict(rejected=rejection)
        events.append(dict(issued_s=now, received_s=stamp, command=c, result=result, outcome=outcome))
        if rejection or applied:
            break
        now = stamp+.01
    return dict(task=asdict(task), state=flow.state, rejection=rejection, reason=flow.reason,
                completed_steps=flow.index, origin='synthetic_fixture', events=events,
                offline_contract_passed=flow.state == 'completed', physical_acceptance=False)


def validate_layout(layout):
    sources = [s.id for s in layout.stations.values() if layout.profiles[s.profile_id].kind == 'source']
    machines = [s.id for s in layout.stations.values() if layout.profiles[s.profile_id].kind == 'machine']
    successful = [run_fixture(layout, MissionTask(f'fixture_{a}_{b}', a, b, layout.steel_id))
                  for a in sources for b in machines]
    # Each malformed receipt is supplied at the actual stage it should block.
    mutations = [
        ('wrong_machine', 'observe_machine', lambda r:r['evidence'].update(visual_id='other_machine')),
        ('occupied_machine', 'observe_machine', lambda r:r['evidence'].update(machine_busy=True)),
        ('stale_image', 'observe_source', lambda r:r['evidence'].update(image_stamp_s=r['stamp_s']-1)),
        ('missing_intrinsics', 'observe_source', lambda r:r['evidence'].pop('K')),
        ('mixed_frames', 'observe_source', lambda r:r['evidence'].update(tf_frame_id='different_frame')),
        ('unloaded_tray', 'navigate_machine', lambda r:r['evidence'].update(tray_load_n=0)),
        ('wrong_tray_pose', 'navigate_machine', lambda r:r['evidence'].update(tray_centre_base=[2, 0, .715])),
        ('gripper_loaded', 'navigate_source', lambda r:r['evidence'].update(finger_forces_n=[5, 0])),
        ('unsettled_base', 'navigate_source', lambda r:r['evidence'].update(measured_twist=[.02, 0])),
        ('bar_outside_source', 'observe_source', lambda r:r['evidence'].update(bar_centre_map=[100, 100, .522])),
        ('missing_pickup_stage', 'pickup_to_tray', lambda r:r['evidence'].update(stages=['pickup'])),
        ('wrong_map', 'navigate_source', lambda r:r.update(map_id='other_map')),
        ('wrong_layout_revision', 'navigate_source', lambda r:r.update(layout_sha256='0'*64)),
        ('wrong_command', 'navigate_source', lambda r:r.update(command_id='old_command')),
        ('insufficient_support', 'load_machine', lambda r:r['evidence'].update(jaw_forces_n=[0, 15, 15, 15])),
        ('unreleased_grip', 'load_machine', lambda r:r['evidence'].update(finger_positions_m=[.02, .02])),
        ('failed_precision', 'load_machine', lambda r:r['evidence'].update(position_error_m=.004)),
        ('short_retreat', 'load_machine', lambda r:r['evidence'].update(retreat_m=.01)),
        ('arm_contact', 'load_machine', lambda r:r['evidence'].update(peak_arm_contact_n=1)),
        ('machine_becomes_busy', 'load_machine', lambda r:r['evidence'].update(machine_busy=True)),
    ]
    negative = []
    for name, kind, mutate in mutations:
        def change(c, r, kind=kind, mutate=mutate):
            if c['kind'] != kind:
                return False
            mutate(r);return True
        row = run_fixture(layout, MissionTask('reject_'+name, sources[0], machines[0], layout.steel_id), change)
        negative.append(dict(name=name, passed=bool(row['rejection'] or row['state'] == 'blocked'),
                             state=row['state'], reason=row['rejection'] or row['reason']))
    passed = all(r['offline_contract_passed'] for r in successful) and all(r['passed'] for r in negative)
    return dict(schema='multi_workcell_offline_validation_v1', passed=passed, layout_sha256=layout.sha256,
                map_id=layout.map_id, source_count=len(sources), machine_count=len(machines),
                combinations=successful, rejected_counterexamples=negative,
                scope='synthetic task/coordinate/evidence contract validation only',
                actual_perception_executed=False, navigation_backend_connected=False,
                legacy_skill_adapter_connected=False, policy_inference_executed=False,
                new_layout_policy_compatibility='not_validated', physical_acceptance=False,
                simulation_started=False, training_started=False, optimizer_loaded=False,
                frozen_policy_modified=False)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=('plan', 'validate', 'replay'))
    p.add_argument('--layout', required=True, type=Path)
    p.add_argument('--task-id', default='offline_task')
    p.add_argument('--source');p.add_argument('--machine')
    p.add_argument('--events', type=Path, help='Recorded issued_s/received_s/result events, not actuator commands')
    p.add_argument('--report', type=Path, required=True)
    p.add_argument('--checkpoint', type=Path, help='Read-only SHA verification; never loads tensors or optimizer')
    args = p.parse_args(argv)
    if args.report.exists():
        p.error('Use a new report path; existing evidence is never overwritten')
    layout = WorkcellLayout.load(args.layout)
    if args.checkpoint:
        checkpoint_sha = hashlib.sha256(args.checkpoint.read_bytes()).hexdigest()
        if checkpoint_sha != FROZEN_POLICY_SHA256:
            raise ValueError('Frozen policy/optimizer checkpoint SHA mismatch')
    if args.command == 'validate':
        report = validate_layout(layout)
    else:
        if not args.source or not args.machine:
            p.error('--source and --machine are required')
        task = MissionTask(args.task_id, args.source, args.machine, layout.steel_id)
        if args.command == 'plan':
            report = plan_task(layout, task)
        else:
            if not args.events:
                p.error('--events is required for replay')
            events = json.loads(args.events.read_text())
            if not isinstance(events, list):
                raise ValueError('Replay events must be an array')
            origin = 'recorded_measurement'
            flow = MissionFlow(layout, task, evidence_origin=origin);rows = [];rejection = None
            for event in events:
                c = flow.command(event['issued_s'])
                if c['kind'] == 'hold':
                    break
                try:
                    outcome = flow.accept(event['result'], event['received_s'])
                except ValueError as ex:
                    rejection = str(ex);rows.append(dict(command=c, rejected_result=event['result'], rejection=rejection));break
                rows.append(dict(command=c, outcome=outcome))
                if flow.state in ('blocked', 'cancelled', 'completed'):
                    break
            report = dict(passed=flow.state == 'completed' and rejection is None, state=flow.state, reason=rejection or flow.reason,
                          replay=rows, completed_steps=flow.index, origin=origin, physical_acceptance=False,
                          simulation_started=False, training_started=False, policy_inference_executed=False)
    if args.checkpoint:
        if hashlib.sha256(args.checkpoint.read_bytes()).hexdigest() != checkpoint_sha:
            raise ValueError('Checkpoint changed during offline validation')
        report['policy_optimizer_sha256_verified'] = checkpoint_sha
    report['created_unix'] = time.time()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open('x') as f:
        json.dump(report, f, indent=2, allow_nan=False);f.write('\n')
    print(json.dumps(dict(report=str(args.report.resolve()), passed=report.get('passed'),
                          simulation_started=False, training_started=False)))
    return 0 if report.get('passed', True) else 2


if __name__ == '__main__':
    sys.exit(main())
