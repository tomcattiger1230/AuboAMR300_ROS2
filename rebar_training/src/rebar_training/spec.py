"""Simulator-independent, SI-unit task contract; quaternions use XYZW (Lab 3)."""

from dataclasses import dataclass
from pathlib import Path
import math
import os

ARM_JOINTS = (
    "shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint",
)
FINGER_JOINTS = ("gripper1_joint", "gripper2_joint")
JAW_NAMES = ("upper_left", "upper_right", "lower_left", "lower_right")
# Existing measured pregrasp and an IK-adjusted compact-curve branch. Moving
# the insertion TCP to x=-0.84 clears the original machine deck collider.
GRASP_JOINTS = (0.2266, -0.8704, 1.0793, 0.3789, 1.5708, -2.915)
INSERT_JOINTS = (2.6974383, -0.2541563, -1.3929375, -1.1387811, -1.1264383, 0.0)
ACTION_SIZE = 7
OBSERVATION_SIZE = 71
ISAAC_LAB_TAG = "v3.0.0-beta2.patch1"
ISAAC_SIM_VERSION = "6.0.1"


def repository_root() -> Path:
    return Path(os.environ.get("REBAR_REPO_ROOT", Path(__file__).resolve().parents[3])).resolve()


def asset_directory() -> Path:
    return Path(os.environ.get("REBAR_TRAINING_ASSETS", repository_root() / "rebar_training/artifacts/assets"))


@dataclass(frozen=True)
class TaskSpec:
    task: str = "grasp"
    control_dt: float = 1 / 30
    episode_seconds: float = 12.0
    lift_height: float = 0.12
    grasp_hold_seconds: float = 0.5
    align_hold_seconds: float = 0.3
    handoff_hold_seconds: float = 1.0
    contact_force_min: float = 0.5
    force_limit: float = 250.0
    grasp_distance: float = 0.045
    position_tolerance: float = 0.008
    axis_tolerance_rad: float = math.radians(3)
    stable_linear_speed: float = 0.04
    stable_angular_speed: float = 0.20
    release_distance: float = 0.06
    drop_height: float = 0.10
    arm_speed: float = 0.35
    target_lead: float = 0.12
    finger_travel: float = 0.0285
    # Local to each cloned environment; derived from lab grip line and parked base.
    insert_goal: tuple[float, float, float] = (-0.87, -0.08, 1.50)

    def __post_init__(self):
        if self.task not in ("grasp", "insert", "loading"):
            raise ValueError("task must be grasp, insert or loading")
        for key, value in vars(self).items():
            if isinstance(value, (float, int)) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{key} must be finite and positive")
        if self.axis_tolerance_rad >= math.pi / 2:
            raise ValueError("axis tolerance must be less than pi/2")
        if self.contact_force_min >= self.force_limit:
            raise ValueError("contact threshold must be below force limit")
        if len(self.insert_goal) != 3 or not all(math.isfinite(x) for x in self.insert_goal):
            raise ValueError("insert_goal must contain three finite coordinates")
        required_steps = self.grasp_hold_steps if self.task == "grasp" else self.align_hold_steps + self.handoff_hold_steps
        if self.episode_steps <= required_steps:
            raise ValueError("episode must exceed the required dwell time")

    def steps(self, seconds):
        return math.ceil(seconds / self.control_dt - 1e-9)

    @property
    def episode_steps(self):
        return self.steps(self.episode_seconds)

    @property
    def grasp_hold_steps(self):
        return self.steps(self.grasp_hold_seconds)

    @property
    def align_hold_steps(self):
        return self.steps(self.align_hold_seconds)

    @property
    def handoff_hold_steps(self):
        return self.steps(self.handoff_hold_seconds)
