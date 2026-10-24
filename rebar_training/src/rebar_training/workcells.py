"""Offline workcell registry and planar station transforms. No simulator or motor APIs.

Map XY/yaw locates a surveyed station frame; all skill geometry belongs to its
profile. A source search region is a prior, never a steel pose observation.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from types import MappingProxyType

FROZEN_POLICY_SHA256 = '393b47f8385a9163e3c9f88b53375923327aa3c80239a3af105e700cf1f93a82'
CONTRACT = 'multi_workcell_layout_v1'


def finite(values, size, name):
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise ValueError(f'{name}: expected {size} numbers')
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in values):
        raise ValueError(f'{name}: finite numbers required')
    return tuple(float(x) for x in values)


def identifier(value, name='id'):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_.-]{0,79}', value):
        raise ValueError(f'{name}: invalid identifier')
    return value


def exact(data, required, optional=()):
    if not isinstance(data, dict) or set(required) - set(data) or set(data) - set(required) - set(optional):
        raise ValueError(f'Expected fields {sorted(required)}; optional {sorted(optional)}')


def unit(vector, name):
    v = finite(vector, 3, name)
    if abs(sum(x*x for x in v)-1) > 1e-6:
        raise ValueError(f'{name}: unit vector required')
    return v


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose:
    """XYZ in metres, yaw in radians, Z-up; roll/pitch are outside this contract."""
    x: float
    y: float
    z: float
    yaw: float

    def __post_init__(self):
        finite(self.values(), 4, 'pose')

    @classmethod
    def parse(cls, values):
        return cls(*finite(values, 4, 'pose_xyz_yaw'))

    def values(self):
        return (self.x, self.y, self.z, self.yaw)

    def vector(self, local):
        x, y, z = finite(local, 3, 'vector')
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (c*x-s*y, s*x+c*y, z)

    def point(self, local):
        return tuple(a+b for a, b in zip(self.vector(local), self.values()[:3]))

    def inverse_point(self, world):
        d = tuple(a-b for a, b in zip(finite(world, 3, 'point'), self.values()[:3]))
        return Pose(0, 0, 0, -self.yaw).vector(d)

    def inverse_vector(self, world):
        return Pose(0, 0, 0, -self.yaw).vector(world)

    def compose(self, local):
        return Pose(*self.point(local.values()[:3]), wrap(self.yaw+local.yaw))


def content_sha(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class Profile:
    id: str
    kind: str
    navigation_offset: Pose
    search_bounds_xy: tuple | None = None
    support_height_m: float | None = None
    fixture_goal: tuple | None = None
    bar_axis: tuple | None = None
    standoff_offset: tuple | None = None
    retreat_offset: tuple | None = None

    @classmethod
    def parse(cls, data):
        common = {'id', 'kind', 'navigation_offset_xyz_yaw'}
        if not isinstance(data, dict):
            raise ValueError('Invalid profile')
        kind = data.get('kind')
        if kind == 'source':
            exact(data, common | {'search_bounds_xy', 'support_height_m'})
            bounds = finite(data['search_bounds_xy'], 4, 'search_bounds_xy')
            height, = finite([data['support_height_m']], 1, 'support_height_m')
            if bounds[0] >= bounds[2] or bounds[1] >= bounds[3] or height <= 0:
                raise ValueError('Invalid source support/search geometry')
            return cls(identifier(data['id']), kind, Pose.parse(data['navigation_offset_xyz_yaw']), bounds, height)
        if kind == 'machine':
            exact(data, common | {'fixture_goal', 'bar_axis', 'standoff_offset', 'retreat_offset'})
            goal = finite(data['fixture_goal'], 3, 'fixture_goal')
            axis = unit(data['bar_axis'], 'bar_axis')
            if any(abs(a-b) > 1e-6 for a, b in zip(axis, (0, 0, 1))):
                raise ValueError('This same-machine skill requires a vertical fixture axis')
            stand = finite(data['standoff_offset'], 3, 'standoff_offset')
            retreat = finite(data['retreat_offset'], 3, 'retreat_offset')
            if goal[2] <= 0 or math.dist(stand, (0, 0, 0)) < .06 or math.dist(retreat, (0, 0, 0)) < .06:
                raise ValueError('Invalid machine clearance geometry')
            return cls(identifier(data['id']), kind, Pose.parse(data['navigation_offset_xyz_yaw']),
                       fixture_goal=goal, bar_axis=axis, standoff_offset=stand, retreat_offset=retreat)
        raise ValueError('Unknown profile kind')


@dataclass(frozen=True)
class Station:
    id: str
    profile_id: str
    pose_map: Pose
    navigation_id: str
    visual_id: str

    @classmethod
    def parse(cls, data):
        exact(data, {'id', 'profile_id', 'pose_map_xyz_yaw', 'navigation_id', 'visual_id'})
        return cls(identifier(data['id']), identifier(data['profile_id']), Pose.parse(data['pose_map_xyz_yaw']),
                   identifier(data['navigation_id']), identifier(data['visual_id']))


class WorkcellLayout:
    def __setattr__(self, name, value):
        if self.__dict__.get("_sealed"):
            raise AttributeError("Layout is immutable; load a new revision")
        object.__setattr__(self, name, value)

    def __init__(self, data):
        exact(data, {'schema', 'map_id', 'revision', 'calibration_id', 'geometry_status', 'steel', 'profiles', 'stations'})
        if data['schema'] != CONTRACT or data['geometry_status'] not in ('synthetic', 'surveyed'):
            raise ValueError('Invalid layout schema/geometry status')
        self.map_id = identifier(data['map_id'], 'map_id')
        self.revision = identifier(data['revision'], 'revision')
        self.calibration_id = identifier(data['calibration_id'], 'calibration_id')
        self.geometry_status = data['geometry_status']
        exact(data['steel'], {'id', 'diameter_m', 'length_m'})
        self.steel_id = identifier(data['steel']['id'])
        self.diameter_m, self.length_m = finite([data['steel']['diameter_m'], data['steel']['length_m']], 2, 'steel')
        if not 0 < self.diameter_m < self.length_m:
            raise ValueError('Invalid steel geometry')
        if not isinstance(data['profiles'], list) or not isinstance(data['stations'], list):
            raise ValueError('Profiles and stations must be arrays')
        profiles = [Profile.parse(p) for p in data['profiles']]
        stations = [Station.parse(s) for s in data['stations']]
        self.profiles = {p.id: p for p in profiles}
        self.stations = {s.id: s for s in stations}
        if len(self.profiles) != len(profiles) or len(self.stations) != len(stations) or not stations:
            raise ValueError('Duplicate or empty profile/station registry')
        for s in stations:
            if s.profile_id not in self.profiles:
                raise ValueError('Unknown station profile: '+s.profile_id)
        for field in ('visual_id', 'navigation_id'):
            if len({getattr(s, field) for s in stations}) != len(stations):
                raise ValueError('Ambiguous '+field)
        if not any(self.profiles[s.profile_id].kind == 'source' for s in stations) or not any(self.profiles[s.profile_id].kind == 'machine' for s in stations):
            raise ValueError('Both source and machine profiles required')
        self.profiles = MappingProxyType(self.profiles)
        self.stations = MappingProxyType(self.stations)
        self.sha256 = content_sha(data)
        self._sealed = True

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text()))

    def station(self, station_id, kind=None):
        try:
            s = self.stations[station_id]
        except KeyError:
            raise ValueError('Unknown station: '+str(station_id)) from None
        if kind and self.profiles[s.profile_id].kind != kind:
            raise ValueError('Wrong station kind: '+station_id)
        return s

    def navigation_goal(self, station_id):
        s = self.station(station_id)
        return s.pose_map.compose(self.profiles[s.profile_id].navigation_offset)

    def source_prior(self, source_id):
        s = self.station(source_id, 'source');p = self.profiles[s.profile_id]
        x0, y0, x1, y1 = p.search_bounds_xy
        return dict(station_id=s.id, frame='map', support_height_m=s.pose_map.z+p.support_height_m,
                    search_polygon=[s.pose_map.point((x, y, p.support_height_m))
                                    for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))],
                    steel_pose_provided=False)

    def contains_bar(self, source_id, centre_map, axis_map):
        s = self.station(source_id, 'source');p = self.profiles[s.profile_id]
        centre = s.pose_map.inverse_point(centre_map);axis = s.pose_map.inverse_vector(unit(axis_map, 'bar_axis'))
        x0, y0, x1, y1 = p.search_bounds_xy
        endpoints = [tuple(c+sign*self.length_m*.5*a for c, a in zip(centre, axis)) for sign in (-1, 1)]
        return (abs(centre[2]-p.support_height_m) <= .03 and abs(axis[2]) <= .1
                and all(x0 <= q[0] <= x1 and y0 <= q[1] <= y1 for q in endpoints))

    def machine_geometry(self, machine_id, *, refined_pose=None, base_pose=None):
        s = self.station(machine_id, 'machine');p = self.profiles[s.profile_id]
        pose = refined_pose or s.pose_map
        goal = pose.point(p.fixture_goal)
        stand = pose.point(tuple(a+b for a, b in zip(p.fixture_goal, p.standoff_offset)))
        retreat = pose.vector(p.retreat_offset)
        axis = pose.vector(p.bar_axis)
        yaw = pose.yaw-(base_pose.yaw if base_pose else 0)
        a = math.sqrt(.5);c = math.cos(yaw/2);sn = math.sin(yaw/2)
        return dict(station_id=s.id, profile_id=p.id, frame='base' if base_pose else 'map',
                    tcp_quaternion_xyzw=(a*c, a*sn, a*sn, a*c),
                    goal=base_pose.inverse_point(goal) if base_pose else goal,
                    standoff=base_pose.inverse_point(stand) if base_pose else stand,
                    retreat_vector=base_pose.inverse_vector(retreat) if base_pose else retreat,
                    bar_axis=base_pose.inverse_vector(axis) if base_pose else axis,
                    refined=refined_pose is not None)
