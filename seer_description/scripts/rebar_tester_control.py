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


class IsaacRebarTesterController:
    def __init__(self, stage):
        from pxr import Gf

        self.Gf = Gf
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
        self.apply()
        atomic_write(self.state_path, self.actual)

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
        if now - self.last_status_time >= 0.1:
            atomic_write(self.state_path, self.actual)
            self.last_status_time = now
