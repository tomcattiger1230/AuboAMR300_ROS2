"""Check the executable curve against the reviewed local path and limits."""
import json
from pathlib import Path
import sys
import math

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "seer_description/scripts/rebar_tester_mission"))
from compact_curve import SCHEDULE, TARGET, VELOCITIES, nearest_target, sample, retime


def test_runtime_curve_matches_reviewed_offline_path():
    path = ROOT / "seer_description/test/results/rebar_compact_local_20260928"
    report = json.loads((path / "report.json").read_text())
    start = report["start_joints_rad"]
    joints = np.load(path / "trajectory.npz")["joints_rad"]
    reconstructed = [sample(start, report["target_joints_rad"], report["schedule_parameters"],
                            i/(len(joints)-1))[0] for i in range(len(joints))]
    assert np.allclose(joints, reconstructed, atol=1e-10)
    for u in (0., 1.):
        _, velocity, acceleration = sample(start, TARGET, SCHEDULE, u)
        assert max(abs(value) for value in velocity + acceleration) < 1e-10


def test_wrapped_wrist_target_stays_near_live_joint_and_inside_limits():
    target = nearest_target([-2.8, -1.5, -1.6, -1.6, 1.57, -4.4])
    assert -2*math.pi < target[-1] < -6.28
    assert abs(target[-1] + 4.4) < 2.


def test_default_curve_matches_replan_from_actual_pickup_feedback():
    path = ROOT / "seer_description/test/results/rebar_compact_experiment_20260928/planning"
    report = json.loads((path / "report.json").read_text())
    joints = np.load(path / "trajectory.npz")["joints_rad"]
    source = report["start_joints_rad"]
    reconstructed = [sample(source, nearest_target(source), SCHEDULE, i/(len(joints)-1))[0]
                     for i in range(len(joints))]
    assert np.allclose(joints, reconstructed, atol=1e-10)


def test_retiming_respects_slow_payload_velocity_and_acceleration():
    start = [-2.81348, -1.50368, -1.59933, -1.66645, 1.5708, 1.89891]
    rows, duration = retime(start, nearest_target(start), SCHEDULE)
    for _, speeds, accelerations in rows:
        assert all(abs(value)/duration <= limit*.06 for value, limit in zip(speeds, VELOCITIES))
        assert all(abs(value)/duration**2 <= .12 for value in accelerations)


def test_double_speed_preserves_path_and_halves_duration_with_scaled_limits():
    source = [3.4663, -.1704, 1.6013, .198, 1.5711, 1.8956]
    target = nearest_target(source)
    rows, reference = retime(source, target, SCHEDULE)
    faster, duration = retime(source, target, SCHEDULE, speed_scale=2.)
    assert faster == rows
    assert reference == pytest.approx(44.)
    assert duration == pytest.approx(22.)
    for _, speeds, accelerations in faster:
        assert all(abs(v)/duration <= limit*.12 for v, limit in zip(speeds, VELOCITIES))
        assert all(abs(a)/duration**2 <= .48 for a in accelerations)


@pytest.mark.parametrize('scale', [0., -1., 2.01, float('nan'), float('inf')])
def test_speed_scale_rejects_invalid_or_untested_ranges(scale):
    with pytest.raises(ValueError):
        retime(TARGET, TARGET, SCHEDULE, speed_scale=scale)
