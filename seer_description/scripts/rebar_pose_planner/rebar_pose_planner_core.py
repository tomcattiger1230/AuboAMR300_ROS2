"""Pure geometry and input checks for the rebar pose planning window."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from rebar_experiment_geometry import SLOT_X  # noqa: E402

PICKUP_LIFT_Z = 0.90
TCP_IN_WRIST = (0.0, 0.0, 0.16)
DEFAULT_TARGET = (6.0, 3.90, 1.50, 90.0, 0.0, 180.0)


@dataclass(frozen=True)
class PlanInput:
    slot: int
    frame: str
    x: float
    y: float
    z: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float

    def validate(self) -> None:
        if self.slot not in (1, 2, 3, 4):
            raise ValueError("取筋点位必须为 1–4")
        if self.frame not in ("world", "base_footprint"):
            raise ValueError("目标坐标系必须为 world 或 base_footprint")
        values = (self.x, self.y, self.z, self.roll_deg,
                  self.pitch_deg, self.yaw_deg)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("目标位姿必须为有限数值")
        if not 0.1 <= self.z <= 3.0:
            raise ValueError("目标 Z 应在 0.1–3.0 m 内")
        if any(abs(value) > 360 for value in values[3:]):
            raise ValueError("姿态角应在 -360°–360° 内")


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, vector):
    result = quat_multiply(quat_multiply(q, (*vector, 0.0)), quat_conjugate(q))
    return result[:3]


def quat_from_rpy_deg(roll, pitch, yaw):
    r, p, y = (math.radians(value) / 2.0 for value in (roll, pitch, yaw))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def source_lift_tcp(slot):
    if slot not in (1, 2, 3, 4):
        raise ValueError("取筋点位必须为 1–4")
    return (SLOT_X[slot - 1], 0.0, PICKUP_LIFT_Z)


def source_grasp_quat():
    # Local X closes on the bar; local Y follows its length in the rack.
    return (1.0, 0.0, 0.0, 0.0)


def wrist_from_tcp(tcp, wrist_quat):
    offset = quat_rotate(wrist_quat, TCP_IN_WRIST)
    return tuple(value - delta for value, delta in zip(tcp, offset))


def target_in_base(request: PlanInput, base_position, base_quat):
    """Return target TCP and wrist orientation in base_footprint."""
    request.validate()
    point = (request.x, request.y, request.z)
    orientation = quat_from_rpy_deg(
        request.roll_deg, request.pitch_deg, request.yaw_deg
    )
    if request.frame == "base_footprint":
        return point, orientation
    inverse = quat_conjugate(base_quat)
    relative = tuple(value - origin for value, origin in zip(point, base_position))
    return quat_rotate(inverse, relative), quat_multiply(inverse, orientation)


def target_axes_world(request: PlanInput, base_quat):
    """Tool approach and bar axes in world coordinates."""
    request.validate()
    q = quat_from_rpy_deg(request.roll_deg, request.pitch_deg, request.yaw_deg)
    if request.frame == "base_footprint":
        q = quat_multiply(base_quat, q)
    return quat_rotate(q, (0.0, 0.0, 1.0)), quat_rotate(q, (0.0, 1.0, 0.0))
