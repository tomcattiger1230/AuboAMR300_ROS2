"""File protocol and motion limits for the Isaac rebar tester.

The host ROS 2 bridge and Isaac's bundled Python use separate ROS installations.
An atomic JSON handoff keeps this small device controller independent of either.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path


DEFAULTS = {"upper_z": 1.68, "lower_z": 1.05, "upper_opening": 0.08, "lower_opening": 0.08}
LIMITS = {
    "upper_z": (1.50, 1.88),
    "lower_z": (0.92, 1.34),
    "upper_opening": (0.024, 0.16),
    "lower_opening": (0.024, 0.16),
}
RATES = {"upper_z": 0.12, "lower_z": 0.12, "upper_opening": 0.08, "lower_opening": 0.08}
MIN_CARRIAGE_GAP = 0.28


def rebar_grip_ready(actual, center, expected, axis):
    """Require both closed jaws and the measured vertical bar in their gap."""
    axis_length = math.sqrt(sum(value * value for value in axis))
    return (
        actual["upper_opening"] <= 0.030
        and actual["lower_opening"] <= 0.030
        and abs(actual["upper_z"] - 1.87) < 0.02
        and abs(actual["lower_z"] - 1.12) < 0.02
        and abs(center[0] - expected[0]) < 0.035
        and abs(center[1] - expected[1]) < 0.055
        and abs(center[2] - expected[2]) < 0.08
        and axis_length > 0
        and abs(axis[2] / axis_length) > 0.95
    )


def paths():
    domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    return (Path(f"/tmp/rebar_tester_command_{domain}.json"),
            Path(f"/tmp/rebar_tester_state_{domain}.json"))


def clamp(key, value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{key} must be finite")
    low, high = LIMITS[key]
    return max(low, min(high, value))


def atomic_write(path, values):
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(values, sort_keys=True), encoding="utf-8")
    os.replace(temporary, path)


def read_values(path):
    values = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(values, dict):
        raise ValueError("expected an object")
    return {key: clamp(key, values[key]) for key in DEFAULTS if key in values}


def read_state(path):
    values = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(values, dict):
        raise ValueError("expected a state object")
    state = {key: clamp(key, values[key]) for key in DEFAULTS}
    gripped = values.get("rebar_gripped", False)
    if not isinstance(gripped, bool):
        raise ValueError("rebar_gripped must be boolean")
    state["rebar_gripped"] = gripped
    return state


class IsaacRebarTesterController:
    def __init__(self, stage):
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        self.Gf = Gf
        self.Usd = Usd
        self.UsdGeom = UsdGeom
        self.stage = stage
        self.command_path, self.state_path = paths()
        self.targets = DEFAULTS.copy()
        self.actual = DEFAULTS.copy()
        self.prims = {}
        for name in ("UpperCarriage", "LowerCarriage"):
            root = f"/World/RebarTestMachine/{name}"
            for part in ("", "/JawLeft", "/JawRight", "/JawLeftTip", "/JawRightTip"):
                prim = stage.GetPrimAtPath(root + part)
                if not prim.IsValid():
                    raise RuntimeError(f"Missing rebar tester prim: {root + part}")
                self.prims[name + part] = prim.GetAttribute("xformOp:translate")
        self.last_command_mtime = None
        self.last_status_time = 0.0
        self.rebar_prim = stage.GetPrimAtPath("/World/RebarStation/Rebar")
        self.rebar_kinematic = None
        self.rebar_gripped = False
        if self.rebar_prim.IsValid():
            body = UsdPhysics.RigidBodyAPI(self.rebar_prim)
            self.rebar_kinematic = body.GetKinematicEnabledAttr()
            if not self.rebar_kinematic.IsValid():
                self.rebar_kinematic = body.CreateKinematicEnabledAttr(False)
        self.apply()
        self.write_state()

    def write_state(self):
        atomic_write(self.state_path,
                     {**self.actual, "rebar_gripped": self.rebar_gripped})

    def bar_in_grip(self):
        """Check the actual rigid-body pose before transferring support."""
        rebar_world = self.UsdGeom.Xformable(self.rebar_prim).ComputeLocalToWorldTransform(
            self.Usd.TimeCode.Default()
        )
        machine = self.stage.GetPrimAtPath("/World/RebarTestMachine")
        machine_world = self.UsdGeom.Xformable(machine).ComputeLocalToWorldTransform(
            self.Usd.TimeCode.Default()
        )
        center = rebar_world.ExtractTranslation()
        expected = machine_world.Transform(self.Gf.Vec3d(-0.08, 0, 1.50))
        axis = rebar_world.TransformDir(self.Gf.Vec3d(1, 0, 0))
        return rebar_grip_ready(self.actual, center, expected, axis)

    def update_rebar_grip(self):
        if self.rebar_kinematic is None:
            return
        closed = (self.actual["upper_opening"] <= 0.030
                  and self.actual["lower_opening"] <= 0.030)
        if not self.rebar_gripped and closed and self.bar_in_grip():
            # The jaws are visual geometry. Kinematic hold represents their
            # support so the real dynamic rebar remains at its measured pose
            # when the robot fingers release it; no pose is teleported.
            if not self.rebar_kinematic.Set(True):
                raise RuntimeError("Cannot lock rebar rigid body in tester")
            self.rebar_gripped = True
            print("Rebar tester accepted and holds the rebar", flush=True)
        elif self.rebar_gripped and not closed:
            if not self.rebar_kinematic.Set(False):
                raise RuntimeError("Cannot unlock rebar rigid body in tester")
            self.rebar_gripped = False
            print("Rebar tester released the rebar", flush=True)

    def apply(self):
        for prefix, name in (("upper", "UpperCarriage"), ("lower", "LowerCarriage")):
            self.prims[name].Set(self.Gf.Vec3d(-0.08, 0, self.actual[prefix + "_z"]))
            jaw_center = 0.03 + self.actual[prefix + "_opening"] / 2
            self.prims[name + "/JawLeft"].Set(self.Gf.Vec3d(0, jaw_center, 0))
            self.prims[name + "/JawRight"].Set(self.Gf.Vec3d(0, -jaw_center, 0))
            tip_z = -0.075 if prefix == "upper" else 0.075
            self.prims[name + "/JawLeftTip"].Set(self.Gf.Vec3d(0, jaw_center, tip_z))
            self.prims[name + "/JawRightTip"].Set(self.Gf.Vec3d(0, -jaw_center, tip_z))

    def update(self, dt, now):
        try:
            mtime = self.command_path.stat().st_mtime_ns
            if mtime != self.last_command_mtime:
                self.targets.update(read_values(self.command_path))
                self.last_command_mtime = mtime
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
            print(f"Ignoring invalid rebar tester command: {exc}", flush=True)

        changed = False
        for key, target in self.targets.items():
            step = RATES[key] * min(max(dt, 0.0), 0.1)
            actual = self.actual[key]
            next_value = actual + max(-step, min(step, target - actual))
            if key == "upper_z":
                next_value = max(next_value, self.actual["lower_z"] + MIN_CARRIAGE_GAP)
            elif key == "lower_z":
                next_value = min(next_value, self.actual["upper_z"] - MIN_CARRIAGE_GAP)
            if abs(next_value - actual) > 1e-9:
                self.actual[key] = next_value
                changed = True
        if changed:
            self.apply()
        self.update_rebar_grip()
        if now - self.last_status_time >= 0.1:
            self.write_state()
            self.last_status_time = now
