"""Seeded, bounded reset sampling, with no ROS services or simulator restart."""

from dataclasses import dataclass

import torch

from .spec import GRASP_JOINTS, INSERT_JOINTS
from .randomization import profile


def quat_apply(q, vector):
    """Apply a unit XYZW quaternion to vectors (Isaac Lab 3 convention)."""
    t = 2 * torch.linalg.cross(q[..., :3], vector, dim=-1)
    return vector + q[..., 3:] * t + torch.linalg.cross(q[..., :3], t, dim=-1)


def quat_multiply(a, b):
    xyz = a[..., 3:] * b[..., :3] + b[..., 3:] * a[..., :3] + torch.linalg.cross(a[..., :3], b[..., :3], dim=-1)
    w = a[..., 3:] * b[..., 3:] - (a[..., :3] * b[..., :3]).sum(-1, keepdim=True)
    return torch.cat((xyz, w), -1)


@dataclass
class ResetBatch:
    joints: torch.Tensor
    station_pose: torch.Tensor
    bar_pose: torch.Tensor


class ResetSampler:
    def __init__(self, seed, device="cpu"):
        self.device = device
        self.generator = torch.Generator(device=device)
        self.seed(seed)

    def seed(self, value):
        self.generator.manual_seed(int(value))

    def sample(self, task, count, randomization='baseline'):
        if task not in ("grasp", "insert", "loading") or count < 0:
            raise ValueError("expected grasp/insert/loading and a non-negative count")
        bounds = profile(randomization)
        noise = torch.rand((count, 9), generator=self.generator, device=self.device) * 2 - 1
        pose = torch.zeros(count, 7, device=self.device)
        pose[:, :2] = noise[:, :2] * bounds.source_xy_m
        pose[:, 0] -= 1.15
        yaw = noise[:, 2] * bounds.source_yaw_rad
        pose[:, 5] = torch.sin(yaw / 2)
        pose[:, 6] = torch.cos(yaw / 2)
        bar = pose.clone()
        bar[:, 2] = 0.522  # source-cylinder height within the translated station
        nominal = INSERT_JOINTS if task == "insert" else GRASP_JOINTS
        joints = torch.zeros(count, 8, device=self.device)
        joints[:, :6] = torch.tensor(nominal, device=self.device) + noise[:, 3:9] * (0.005 if task == "insert" else bounds.arm_joints_rad)
        if task == "insert":
            joints[:, 6:] = torch.tensor([0.015, 0.019], device=self.device)
            # Insert bar pose is set from measured wrist FK after joint reset.
            bar[:, :3] = torch.tensor([-0.80, -0.08, 1.50], device=self.device)
            bar[:, 3:] = torch.tensor([0.0, -2 ** -0.5, 0.0, 2 ** -0.5], device=self.device)
        return ResetBatch(joints, pose, bar)
