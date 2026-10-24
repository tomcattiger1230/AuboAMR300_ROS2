"""Explicit, non-dispatching adapter into existing dock/route/visual-servo code.

A new-layout command does not authorize frozen PPO inference. Coordinates are
recomputed from current measured base feedback, rather than treating a surveyed
map point as a robot-frame target. Nothing here starts an environment.
"""
from copy import deepcopy
import math
from .mobile import PrecisionDock
from .workcells import Pose, FROZEN_POLICY_SHA256, finite, wrap
from .workcell_mission import _fresh, _visual


def _skill_visual(evidence, layout, station_id, now, issued):
    _visual(evidence, layout, now, issued)
    if evidence.get('origin') not in ('synthetic_fixture', 'recorded_measurement'):
        raise ValueError('Only offline measurement contracts are supported')
    if evidence.get('emergency_stop') is not False:
        raise ValueError('Emergency stop is not clear')
    if (evidence.get('visual_id') != layout.station(station_id, 'machine').visual_id
            or evidence.get('machine_ready') is not True or evidence.get('machine_busy') is not False):
        raise ValueError('Loading vision must confirm the requested ready machine')


def _measured_state(state):
    # Only this explicit arithmetic adapter needs torch; planning/CLI imports do not.
    import torch
    for name, width in (('bar', 3), ('tcp', 3), ('fingers', 2), ('finger_forces', 2), ('jaw_forces', 4)):
        value = getattr(state, name, None)
        if not isinstance(value, torch.Tensor) or value.shape != (1, width) or not torch.isfinite(value).all():
            raise ValueError('Invalid measured state: '+name)
    if (state.finger_forces < 0).any() or (state.jaw_forces < 0).any():
        raise ValueError('Contact force magnitudes cannot be negative')


def precision_dock_for_station(layout, station_id):
    pose = layout.navigation_goal(station_id)
    return PrecisionDock((pose.x, pose.y, pose.yaw))


class StationLoadingAdapter:
    def __init__(self, layout, command):
        c = deepcopy(command)
        if (c.get('kind') != 'load_machine' or c.get('layout_sha256') != layout.sha256
                or c.get('map_id') != layout.map_id or c.get('actuator_dispatch') is not False):
            raise ValueError('Expected a matching, non-dispatching station loading intent')
        s = layout.station(c['station_id'], 'machine');args = c['args']
        policy = args['policy']
        if (policy.get('sha256') != FROZEN_POLICY_SHA256 or policy.get('mutation_allowed') is not False
                or policy.get('execution_authorized') is not False):
            raise ValueError('Frozen policy compatibility must remain blocked')
        pose = Pose.parse(args['station_pose_map']);g = args['geometry']
        if (g.get('station_id') != s.id or g.get('profile_id') != s.profile_id
                or g.get('refined') is not True or g.get('frame') != 'base'):
            raise ValueError('Expected confirmed target machine geometry')
        if math.dist(pose.values()[:3], s.pose_map.values()[:3]) > .25 or abs(wrap(pose.yaw-s.pose_map.yaw)) > math.radians(15):
            raise ValueError('Confirmed machine differs from registry')
        self.layout = layout;self.command = c;self.station_pose = pose

    def geometry(self, base_pose_map, robot_stamp_s, now, measured_twist):
        _fresh(robot_stamp_s, now, self.command['issued_s'])
        if not self.command['issued_s'] <= now <= self.command['deadline_s']:
            raise ValueError('Loading intent expired or not yet issued')
        base = Pose.parse(base_pose_map);dock = self.layout.navigation_goal(self.command['station_id'])
        if (math.dist(base.values()[:3], dock.values()[:3]) > .05 or abs(wrap(base.yaw-dock.yaw)) > .05
                or max(abs(x) for x in finite(measured_twist, 2, 'measured_twist')) >= .008):
            raise ValueError('Loading requires a settled base at the requested station')
        return self.layout.machine_geometry(self.command['station_id'], refined_pose=self.station_pose, base_pose=base)

    def route_targets(self, route, state, *, base_pose_map, robot_stamp_s, now, measured_twist):
        _measured_state(state)
        g = self.geometry(base_pose_map, robot_stamp_s, now, measured_twist)
        return route.targets(state, g['goal'], loading_geometry=g)

    def advance_route(self, route, state, logic, joints, *, measured_quaternion, base_pose_map,
                      robot_stamp_s, now, measured_twist, visual_evidence):
        _skill_visual(visual_evidence, self.layout, self.command['station_id'], now, self.command['issued_s'])
        _measured_state(state)
        g = self.geometry(base_pose_map, robot_stamp_s, now, measured_twist)
        return route.advance(state, logic, joints, g['goal'], grasp_quaternion=measured_quaternion,
                             loading_geometry=g)

    def servo_actions(self, servo, state, route, *, base_pose_map, robot_stamp_s, now, measured_twist, visual_evidence):
        _skill_visual(visual_evidence, self.layout, self.command['station_id'], now, self.command['issued_s'])
        _measured_state(state)
        g = self.geometry(base_pose_map, robot_stamp_s, now, measured_twist)
        return servo.route_actions(state, route, g['goal'], loading_geometry=g)


def source_observer_for_command(layout, command, *, detector, base_pose_map, robot_stamp_s, now, measured_twist):
    """Create the existing image observer on the registered plane, with no steel-pose hint."""
    c = command
    if (c.get('kind') != 'observe_source' or c.get('layout_sha256') != layout.sha256
            or c.get('map_id') != layout.map_id or c.get('actuator_dispatch') is not False):
        raise ValueError('Expected a matching source observation intent')
    _fresh(robot_stamp_s, now, c['issued_s'])
    if not c['issued_s'] <= now <= c['deadline_s']:
        raise ValueError('Source observation intent expired')
    s = layout.station(c['station_id'], 'source');p = layout.profiles[s.profile_id]
    base = Pose.parse(base_pose_map);dock = layout.navigation_goal(s.id)
    if (math.dist(base.values()[:3], dock.values()[:3]) > .05 or abs(wrap(base.yaw-dock.yaw)) > .05
            or max(abs(x) for x in finite(measured_twist, 2, 'measured_twist')) >= .008):
        raise ValueError('Source observation requires a settled base')
    if abs(layout.diameter_m-.024) > 1e-9 or abs(layout.length_m-.6) > 1e-9:
        raise ValueError('Existing observer geometry supports 24x600 mm steel only')
    from .vision.observer import ImageObserver
    return ImageObserver(detector, plane_height=s.pose_map.z+p.support_height_m-base.z,
                         fixed_support_prior=True)
