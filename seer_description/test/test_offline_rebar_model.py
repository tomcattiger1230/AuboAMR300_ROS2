"""Check offline geometry against recorded MoveIt poses and a blocked shortcut."""
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts/rebar_pose_planner"))
from offline_rebar_model import ROOT, RobotModel


def test_fk_matches_recorded_moveit_source_and_front_pose():
    model = RobotModel()
    report = json.loads((ROOT / "seer_description/test/results/rebar_pose_planner_slot1_20260927.json").read_text())
    source = model.tcp(report["stages"][0]["joints_rad"])
    assert np.linalg.norm(source[:3, 3] - report["source_lift_tcp_base"]) < 1e-6
    assert np.allclose(source[:3, :3], np.diag([1, -1, -1]), atol=1e-6)
    front = model.tcp([2.597, -.357, -1.451, -1.094, -1.026, 0.])
    assert np.linalg.norm(front[:3, 3] - [-.8, -.08, 1.5]) < .00025
    assert np.allclose(front[:3, :3], [[0, 0, -1], [-1, 0, 0], [0, 1, 0]], atol=.00025)


def test_direct_joint_shortcut_detects_payload_arm_intersection():
    model = RobotModel()
    start = model.ik([.2927, 0, 1.05], np.diag([1., -1., -1.]),
                     [-2.813483, -1.50368, -1.59933, -1.66645, 1.5708, 1.89891])
    end = np.array([2.597 - 2 * np.pi, -.357, -1.451, -1.094, -1.026, 0.])
    assert model.clearance(start)[0] > 0
    assert model.clearance(end)[0] > 0
    clearance, pair = model.clearance(start + .30 * (end - start))
    assert clearance < -.01
    assert pair == "payload / upperArm_Link"


def test_exported_candidate_keeps_margin_limits_and_front_pose():
    model = RobotModel()
    directory = ROOT / "seer_description/test/results/rebar_compact_local_20260928"
    data = np.load(directory / "trajectory.npz")
    joints = data["joints_rad"]
    assert np.all(joints >= model.lower) and np.all(joints <= model.upper)
    assert np.max(np.abs(np.diff(joints, axis=0))) < np.deg2rad(.5)
    changes = np.diff(joints, axis=0)
    assert np.all(changes * (joints[-1] - joints[0]) >= -1e-12)
    recomputed = [model.clearance(q)[0] for q in joints]
    assert min(recomputed) >= .008
    assert np.allclose(recomputed, data["clearance_m"], atol=1e-10)
    end = model.tcp(joints[-1])
    assert np.linalg.norm(end[:3, 3] - [-.8, -.08, 1.5]) < 1e-6
    assert np.allclose(end[:3, 2], [-1, 0, 0], atol=1e-6)
    assert abs(end[2, 1]) > .999999
