"""Intent-only multi-station task flow with measured-evidence transition guards.

Commands describe navigation/legacy-skill adapter requests. This module does not
call a navigation service, load a policy/optimizer, or import a simulator. A
successful fixture replay is an interface check, never physical acceptance.
"""
from copy import deepcopy
from dataclasses import dataclass
import math

from .mobile import TransportFeedback, TransportGate
from .workcells import FROZEN_POLICY_SHA256, Pose, finite, identifier, unit, wrap

PICKUP_STAGES = ('pickup', 'tray_raise', 'tray_rotate', 'tray_via_side', 'tray_via_front',
                 'tray_above', 'tray_down', 'tray_release', 'tray_retreat', 'observe_tray')
LOADING_STAGES = ('pickup_tray', 'tray_lift_high', 'loading')
STEPS = (('navigate_source', 'source', 180), ('observe_source', 'source', 30),
         ('pickup_to_tray', 'source', 180), ('navigate_machine', 'machine', 240),
         ('observe_machine', 'machine', 30), ('load_machine', 'machine', 180))


@dataclass(frozen=True)
class MissionTask:
    task_id: str
    source_id: str
    machine_id: str
    steel_id: str

    def __post_init__(self):
        for v in (self.task_id, self.source_id, self.machine_id, self.steel_id):
            identifier(v)


def plan_task(layout, task):
    source = layout.station(task.source_id, 'source')
    machine = layout.station(task.machine_id, 'machine')
    if task.steel_id != layout.steel_id:
        raise ValueError('Unsupported steel specification')
    steps = []
    for kind, role, timeout in STEPS:
        s = source if role == 'source' else machine
        steps.append(dict(kind=kind, station_id=s.id, visual_id=s.visual_id,
                          navigation_id=s.navigation_id, timeout_s=timeout,
                          navigation_goal_map=layout.navigation_goal(s.id).values()
                          if kind.startswith('navigate_') else None))
    return dict(contract='multi_workcell_mission_v1', task_id=task.task_id,
                source_id=source.id, machine_id=machine.id, steel_id=task.steel_id,
                map_id=layout.map_id, layout_sha256=layout.sha256, layout_revision=layout.revision,
                geometry_status=layout.geometry_status, steps=steps,
                source_search_prior=layout.source_prior(source.id),
                machine_coarse_prior=layout.machine_geometry(machine.id),
                policy=dict(sha256=FROZEN_POLICY_SHA256, mutation_allowed=False,
                            execution_authorized=False, compatibility='unvalidated_new_layout',
                            unresolved_inputs=['base_xy', 'machine_goal_minus_tcp', 'route/phase semantics']),
                navigation_backend_connected=False, legacy_skill_adapter_connected=False,
                simulation_started=False, training_started=False)


def _true(e, *keys):
    for key in keys:
        if e.get(key) is not True:
            raise ValueError('Required measured condition: '+key)


def _number(e, key, *, minimum=None, maximum=None):
    value, = finite([e.get(key)], 1, key)
    if minimum is not None and value < minimum or maximum is not None and value > maximum:
        raise ValueError('Out-of-range measured condition: '+key)
    return value


def _fresh(stamp, now, issued, limit=.2):
    value, = finite([stamp], 1, 'timestamp')
    # Sensor and command clocks must share one timebase. Nanoseconds avoid 1 ulp issues.
    age = round(now*1e9)-round(value*1e9)
    if not 0 <= age <= round(limit*1e9) or round(value*1e9) < round(issued*1e9):
        raise ValueError('Stale, future or pre-command evidence')


def _empty_fingers(e):
    positions = finite(e.get('finger_positions_m'), 2, 'finger_positions_m')
    forces = finite(e.get('finger_forces_n'), 2, 'finger_forces_n')
    if any(abs(x) >= .003 for x in positions) or any(not 0 <= x < .5 for x in forces):
        raise ValueError('Fingers must be open and unloaded')


def _visual(e, layout, now, issued):
    _true(e, 'image_valid')
    if e.get('calibration_id') != layout.calibration_id:
        raise ValueError('Camera calibration mismatch')
    identifier(e.get('frame_id'), 'frame_id')
    if e.get('tf_frame_id') != e['frame_id']:
        raise ValueError('Image/TF frame mismatch')
    _fresh(e.get('image_stamp_s'), now, issued)
    _fresh(e.get('tf_stamp_s'), now, issued)
    if abs(e['image_stamp_s']-e['tf_stamp_s']) > .005:
        raise ValueError('Unsynchronised image/TF')
    # Intrinsics and rigid optical extrinsics are carried with the actual image.
    K = e.get('K');T = e.get('T_map_optical')
    if not isinstance(K, list) or len(K) != 3 or not isinstance(T, list) or len(T) != 4:
        raise ValueError('Missing image K/TF')
    K = [finite(row, 3, 'K') for row in K];T = [finite(row, 4, 'TF') for row in T]
    if K[0][0] <= 0 or K[1][1] <= 0 or any(abs(a-b) > 1e-6 for a, b in zip(K[2], (0, 0, 1))):
        raise ValueError('Invalid camera intrinsics')
    if any(abs(a-b) > 1e-6 for a, b in zip(T[3], (0, 0, 0, 1))):
        raise ValueError('Invalid homogeneous TF')
    R = [row[:3] for row in T[:3]]
    for i in range(3):
        for j in range(3):
            if abs(sum(R[k][i]*R[k][j] for k in range(3))-(1 if i == j else 0)) > 1e-5:
                raise ValueError('TF rotation is not orthonormal')
    det = sum(R[0][i]*(R[1][(i+1)%3]*R[2][(i+2)%3]-R[1][(i+2)%3]*R[2][(i+1)%3]) for i in range(3))
    if abs(det-1) > 1e-5:
        raise ValueError('TF reflection is not a valid rotation')


def _tray(e, layout, now, issued):
    _visual(e, layout, now, issued);_empty_fingers(e)
    _fresh(e.get('tray_stamp_s'), now, issued)
    _number(e, 'tray_load_n', minimum=5)
    centre = finite(e.get('tray_centre_base'), 3, 'tray_centre_base')
    axis = unit(e.get('tray_axis_base'), 'tray_axis_base')
    if math.dist(centre, (.2927, 0, .715)) >= .025 or abs(axis[1]) <= .95:
        raise ValueError('Steel is not verified in the chassis tray')
    _true(e, 'arm_stowed', 'bar_stable')
    if e.get('emergency_stop') is not False:
        raise ValueError('Emergency stop state must be explicitly clear')
    # Reuse the existing loaded-transport interlock, without granting wheel motion.
    gate = TransportGate();gate.command_twist(0, 0, now)
    gate.tick(TransportFeedback(now, e['image_stamp_s'], e['tray_stamp_s'], True, True, True, False))
    if gate.reason != 'enabled':
        raise ValueError('Transport guard: '+gate.reason)


class MissionFlow:
    """One task, one outstanding command, fail-closed evidence, no automatic motion retries."""
    def __init__(self, layout, task, *, evidence_origin='synthetic_fixture'):
        if evidence_origin not in ('synthetic_fixture', 'recorded_measurement'):
            raise ValueError('Only offline fixture/recorded evidence is supported')
        self.layout = layout;self._plan = plan_task(layout, task);self.origin = evidence_origin
        self.index = 0;self.attempt = 0;self.state = 'ready';self.reason = None
        self.outstanding = None;self.last_now = -math.inf;self.trace = []
        self.bar = None;self.fixture_pose = None;self.observed_base = None

    @property
    def plan(self):
        return deepcopy(self._plan)

    def _clock(self, now):
        value, = finite([now], 1, 'now')
        if value < self.last_now:
            raise ValueError('Task clock moved backwards')
        self.last_now = value

    def _hold(self):
        return dict(kind='hold', task_id=self._plan['task_id'], state=self.state, reason=self.reason,
                    stop_base=True, hold_arm=True, preserve_grip=True, actuator_dispatch=False)

    def _block(self, reason):
        self.state = 'blocked';self.reason = str(reason)
        return self._hold()

    def command(self, now):
        self._clock(now)
        if self.state in ('blocked', 'cancelled'):
            return self._hold()
        if self.state == 'completed':
            return dict(kind='completed', task_id=self._plan['task_id'], actuator_dispatch=False)
        if self.outstanding is not None:
            if now > self.outstanding['deadline_s']:
                return self._block('Command deadline expired')
            return deepcopy(self.outstanding)
        step = self._plan['steps'][self.index]
        args = dict(station_id=step['station_id'], visual_id=step['visual_id'],
                    navigation_id=step['navigation_id'], adapter_required=True)
        if step['kind'].startswith('navigate_'):
            args.update(goal_map=step['navigation_goal_map'], loaded=step['kind'] == 'navigate_machine',
                        path_validation_required=True, continuous_transport_guard_required=True)
        elif step['kind'] == 'observe_source':
            args.update(search_prior=self._plan['source_search_prior'], require_full_bar=True, stable_frames=3)
        elif step['kind'] == 'pickup_to_tray':
            try:
                _fresh(self.bar['stamp_s'], now, self.bar['issued_s'])
            except (TypeError, ValueError) as ex:
                return self._block('Reobserve source before skill dispatch: '+str(ex))
            args.update(visual_bar_hint=deepcopy(self.bar), legacy_stages=PICKUP_STAGES,
                        policy=self._plan['policy'])
        elif step['kind'] == 'observe_machine':
            args.update(coarse_prior=self._plan['machine_coarse_prior'], require_machine_ready=True)
        elif step['kind'] == 'load_machine':
            try:
                _fresh(self.observed_base['stamp_s'], now, self.observed_base['issued_s'])
            except (TypeError, ValueError) as ex:
                return self._block('Reobserve machine before skill dispatch: '+str(ex))
            args.update(geometry=self.layout.machine_geometry(self._plan['machine_id'], refined_pose=self.fixture_pose,
                        base_pose=self.observed_base['pose']), legacy_stages=LOADING_STAGES,
                        station_pose_map=self.fixture_pose.values(),
                        policy=self._plan['policy'], continuous_visual_servo_required=True)
        self.outstanding = dict(contract=self._plan['contract'], task_id=self._plan['task_id'],
                                command_id=f"{self._plan['task_id']}.{self.index}.{self.attempt}",
                                step=self.index, kind=step['kind'], station_id=step['station_id'],
                                map_id=self.layout.map_id, layout_sha256=self.layout.sha256,
                                issued_s=now, deadline_s=now+step['timeout_s'], args=args,
                                actuator_dispatch=False)
        self.state = 'waiting'
        return deepcopy(self.outstanding)

    def accept(self, result, now):
        self._clock(now)
        if self.state != 'waiting' or self.outstanding is None:
            raise ValueError('No outstanding command awaiting a result')
        c = self.outstanding
        if not isinstance(result, dict):
            raise ValueError('Result must be an object')
        for key in ('task_id', 'command_id', 'step', 'station_id', 'map_id', 'layout_sha256'):
            if (key == 'step' and type(result.get(key)) is not int) or result.get(key) != c[key]:
                raise ValueError('Unmatched result '+key)
        if now > c['deadline_s']:
            return self._block('Command deadline expired')
        if result.get('status') != 'succeeded':
            return self._block('Adapter did not succeed: '+str(result.get('status')))
        try:
            _fresh(result.get('stamp_s'), now, c['issued_s'])
            e = result.get('evidence')
            if not isinstance(e, dict) or e.get('origin') != self.origin:
                raise ValueError('Offline evidence origin mismatch')
            if e.get('emergency_stop') is not False:
                raise ValueError('Emergency stop state must be explicitly clear')
            kind = c['kind'];s = self.layout.station(c['station_id'])
            if kind.startswith('navigate_'):
                _true(e, 'localization_valid', 'path_checked', 'arm_stowed')
                if e.get('navigation_id') != s.navigation_id or e.get('emergency_stop') is not False:
                    raise ValueError('Navigation identity/stop state mismatch')
                _fresh(e.get('pose_stamp_s'), now, c['issued_s'])
                pose = Pose.parse(e.get('base_pose_map'));goal = self.layout.navigation_goal(s.id)
                if math.dist(pose.values()[:3], goal.values()[:3]) > .05 or abs(wrap(pose.yaw-goal.yaw)) > .05:
                    raise ValueError('Navigation has not reached the requested station')
                if max(abs(x) for x in finite(e.get('measured_twist'), 2, 'measured_twist')) >= .008:
                    raise ValueError('Base has not stopped')
                _empty_fingers(e)
                if kind == 'navigate_machine':
                    _tray(e, self.layout, now, c['issued_s'])
            elif kind == 'observe_source':
                _visual(e, self.layout, now, c['issued_s']);_true(e, 'full_bar_visible', 'base_stopped')
                if max(abs(x) for x in finite(e.get('measured_twist'), 2, 'measured_twist')) >= .008:
                    raise ValueError('Source observation requires a stopped base')
                if e.get('visual_id') != s.visual_id:
                    raise ValueError('Wrong source visual identity')
                frames = e.get('stable_frames')
                if type(frames) is not int or frames < 3:
                    raise ValueError('Three stable full-length observations required')
                centre = finite(e.get('bar_centre_map'), 3, 'bar_centre_map')
                axis = unit(e.get('bar_axis_map'), 'bar_axis_map')
                if not self.layout.contains_bar(s.id, centre, axis):
                    raise ValueError('Bar is outside requested source support/search region')
                diameter = _number(e, 'diameter_m', minimum=.001)
                length = _number(e, 'length_m', minimum=.01)
                if abs(diameter-self.layout.diameter_m) > .003 or abs(length-self.layout.length_m) > .02:
                    raise ValueError('Observed steel geometry differs from requested specification')
                samples = e.get('stable_samples')
                if not isinstance(samples, list) or len(samples) != 3:
                    raise ValueError('Three explicit stable observation samples required')
                previous_stamp = -math.inf;frame_ids = set();centres = []
                for sample in samples:
                    _fresh(sample.get('stamp_s'), now, c['issued_s'])
                    identifier(sample.get('frame_id'), 'frame_id')
                    if sample['stamp_s'] <= previous_stamp or sample['frame_id'] in frame_ids:
                        raise ValueError('Stable observations must be distinct ordered frames')
                    previous_stamp = sample['stamp_s'];frame_ids.add(sample['frame_id'])
                    centres.append(finite(sample.get('centre_map'), 3, 'stable centre'))
                    a = unit(sample.get('axis_map'), 'stable axis')
                    if abs(sum(x*y for x, y in zip(axis, a))) < math.cos(math.radians(2)):
                        raise ValueError('Unstable bar axis')
                if (samples[-1]['frame_id'] != e['frame_id'] or abs(samples[-1]['stamp_s']-e['image_stamp_s']) > 1e-9
                        or math.dist(centres[-1], centre) > 1e-6
                        or any(math.dist(a, b) > .004 for a in centres for b in centres)):
                    raise ValueError('Stable observations do not match the current estimate')
                self.bar = dict(centre_map=centre, axis_map=axis, stamp_s=e['image_stamp_s'],
                                frame_id=e['frame_id'], issued_s=c['issued_s'], origin=self.origin)
            elif kind == 'pickup_to_tray':
                if e.get('stages') != list(PICKUP_STAGES):
                    raise ValueError('Pickup/transfer/release stage continuity is incomplete')
                _true(e, 'source_grasp_verified');_tray(e, self.layout, now, c['issued_s'])
            elif kind == 'observe_machine':
                _visual(e, self.layout, now, c['issued_s']);_true(e, 'machine_ready', 'base_stopped')
                if e.get('visual_id') != s.visual_id or e.get('machine_busy') is not False:
                    raise ValueError('Wrong or occupied target machine')
                pose = Pose.parse(e.get('station_pose_map'));base = Pose.parse(e.get('base_pose_map'))
                _fresh(e.get('pose_stamp_s'), now, c['issued_s'])
                dock = self.layout.navigation_goal(s.id)
                if (math.dist(base.values()[:3], dock.values()[:3]) > .05
                        or abs(wrap(base.yaw-dock.yaw)) > .05
                        or max(abs(x) for x in finite(e.get('measured_twist'), 2, 'measured_twist')) >= .008):
                    raise ValueError('Machine observation requires a settled docking pose')
                if math.dist(pose.values()[:3], s.pose_map.values()[:3]) > .25 or abs(wrap(pose.yaw-s.pose_map.yaw)) > math.radians(15):
                    raise ValueError('Fixture disagrees with registered machine; relocalize')
                self.fixture_pose = pose
                self.observed_base = dict(pose=base, stamp_s=e['pose_stamp_s'], issued_s=c['issued_s'])
            elif kind == 'load_machine':
                _visual(e, self.layout, now, c['issued_s']);_empty_fingers(e)
                _true(e, 'machine_ready', 'base_stopped')
                if e.get('machine_busy') is not False:
                    raise ValueError('Machine is no longer ready')
                if e.get('stages') != list(LOADING_STAGES) or e.get('visual_id') != s.visual_id:
                    raise ValueError('Loading stage continuity/target identity mismatch')
                jaws = finite(e.get('jaw_forces_n'), 4, 'jaw_forces_n')
                if min(jaws) < .5 or sum(jaws) < 40:
                    raise ValueError('Machine support is insufficient for robot release')
                _number(e, 'position_error_m', minimum=0, maximum=.003)
                _number(e, 'axis_error_deg', minimum=0, maximum=3)
                _number(e, 'retreat_m', minimum=.06)
                _number(e, 'bar_linear_speed_mps', minimum=0, maximum=.04)
                _number(e, 'bar_angular_speed_radps', minimum=0, maximum=.2)
                _number(e, 'unsafe_peak_force_n', minimum=0, maximum=250)
                for key in ('terminal_arm_contact_n', 'peak_arm_contact_n'):
                    if _number(e, key, minimum=0) >= .5:
                        raise ValueError('Unexpected arm contact')
                _true(e, 'release_verified', 'continuity_verified')
        except (ValueError, TypeError, KeyError) as ex:
            return self._block(str(ex))
        self.trace.append(dict(command_id=c['command_id'], kind=c['kind'], station_id=c['station_id'],
                               completed_s=now, evidence=deepcopy(e)))
        self.index += 1;self.attempt = 0;self.outstanding = None
        self.state = 'completed' if self.index == len(STEPS) else 'ready'
        return dict(state=self.state, task_id=self._plan['task_id'], completed_steps=self.index,
                    offline_contract_only=True, physical_acceptance=False)

    def block(self, reason, now):
        """Execution adapter faults stop this task without fabricating completion."""
        self._clock(now)
        if self.state == 'completed':
            raise ValueError('Cannot block a completed task')
        return self._block(reason)

    def retry_observation(self, now):
        self._clock(now)
        if self.state != 'blocked' or not self._plan['steps'][self.index]['kind'].startswith('observe_'):
            raise ValueError('Only blocked observation requests may be retried')
        if self.attempt >= 2:
            raise ValueError('Observation retry budget exhausted')
        self.attempt += 1;self.outstanding = None;self.state = 'ready';self.reason = None
        return self.command(now)

    def cancel(self, now):
        self._clock(now)
        if self.state == 'completed':
            raise ValueError('Completed task cannot be cancelled')
        self.state = 'cancelled';self.reason = 'Task cancelled'
        return self._hold()
