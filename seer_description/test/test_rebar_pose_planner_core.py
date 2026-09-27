"""Geometry checks for the four-slot rebar TCP planner."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]
                       / "scripts" / "rebar_pose_planner"))
from rebar_pose_planner_core import (  # noqa: E402
    DEFAULT_TARGET, PlanInput, SLOT_X, quat_from_rpy_deg, quat_rotate,
    source_grasp_quat, source_lift_tcp, target_axes_world, target_in_base,
    wrist_from_tcp,
)


def test_all_slots_use_calibrated_lift_tcp():
    for slot, x in enumerate(SLOT_X, 1):
        assert source_lift_tcp(slot) == (x, 0.0, 0.90)
    assert wrist_from_tcp(source_lift_tcp(1), source_grasp_quat()) == (
        SLOT_X[0], 0.0, 1.06
    )


def test_front_target_points_into_tester_and_keeps_bar_vertical():
    request = PlanInput(1, "world", *DEFAULT_TARGET)
    base_quat = quat_from_rpy_deg(0.0, 0.0, -90.0)
    tcp, q = target_in_base(request, (6.08, 3.05, 0.0), base_quat)
    assert all(abs(a - b) < 1e-9 for a, b in zip(tcp, (-0.85, -0.08, 1.50)))
    assert all(abs(a - b) < 1e-9 for a, b in zip(
        quat_rotate(q, (0.0, 0.0, 1.0)), (-1.0, 0.0, 0.0)
    ))
    approach, bar = target_axes_world(request, base_quat)
    assert all(abs(a - b) < 1e-9 for a, b in zip(approach, (0.0, 1.0, 0.0)))
    assert all(abs(a - b) < 1e-9 for a, b in zip(bar, (0.0, 0.0, 1.0)))


def test_invalid_target_is_rejected():
    try:
        PlanInput(5, "world", *DEFAULT_TARGET).validate()
    except ValueError:
        pass
    else:
        raise AssertionError("invalid slot accepted")
    try:
        PlanInput(1, "world", math.nan, *DEFAULT_TARGET[1:]).validate()
    except ValueError:
        pass
    else:
        raise AssertionError("NaN accepted")
