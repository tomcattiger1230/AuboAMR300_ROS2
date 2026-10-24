"""Measured task waypoints; advances goals, never applies a robot action.

The route is part of the continuous task observation, including during pure
policy evaluation. This makes the demonstration supervisor's history visible
instead of asking a feed-forward actor to infer hidden approach/close stages.
"""
import math
import torch

from .reset import quat_multiply

LOADING_CONTRACT = "loading_route_v1"
CARRY_JOINTS = (2.3118482, 1.5642045, 1.4233553, -.1408492, .8297445, 0.)


class LoadingRoute:
    def __init__(self, count, device, require_release_clearance=False):
        self.device = device
        self.stage = torch.zeros(count, dtype=torch.long, device=device)
        self.dwell = torch.zeros_like(self.stage)
        self.release_dwell = torch.zeros_like(self.stage)
        self.grasp_alignment_dwell = torch.zeros_like(self.stage)
        self.grasp_alignment_error_deg = torch.zeros(count, device=device)
        self.grasp_alignment_blocked_contact = torch.zeros(count, dtype=torch.bool, device=device)
        self.require_release_clearance = require_release_clearance
        self.anchor = torch.zeros(count, 3, device=device)
        self.initial_bar = self.anchor.clone()
        self.quaternion = torch.zeros(count, 4, device=device)

    def reset(self, ids, bar_pose):
        self.stage[ids] = self.dwell[ids] = 0
        self.release_dwell[ids] = 0
        self.grasp_alignment_dwell[ids] = 0
        self.grasp_alignment_error_deg[ids] = 0
        self.grasp_alignment_blocked_contact[ids] = False
        self.anchor[ids] = 0
        self.initial_bar[ids] = bar_pose[:, :3]
        nominal = torch.tensor([math.sqrt(.5), math.sqrt(.5), 0., 0.], device=self.device).expand(len(ids), -1)
        self.quaternion[ids] = quat_multiply(bar_pose[:, 3:], nominal)

    def advance(self, state, logic, joints, goal, *, grasp_quaternion=None):
        s = state
        ready = (self.stage == 0) & (torch.linalg.vector_norm(s.bar + s.bar.new_tensor([0, 0, .10]) - s.tcp, dim=-1) < .006)
        self.grasp_alignment_blocked_contact.zero_()
        if grasp_quaternion is not None:
            q = torch.as_tensor(grasp_quaternion, device=self.device, dtype=self.quaternion.dtype)
            if (q.shape != self.quaternion.shape or not torch.isfinite(q).all()
                    or (torch.linalg.vector_norm(q, dim=-1) < .5).any()):
                raise ValueError('Invalid measured grasp quaternion')
            q = q / torch.linalg.vector_norm(q, dim=-1, keepdim=True)
            target = self.quaternion / torch.linalg.vector_norm(self.quaternion, dim=-1, keepdim=True).clamp_min(1e-8)
            dot = (q * target).sum(-1).abs().clamp(0, 1)
            self.grasp_alignment_error_deg = 2 * torch.acos(dot) * (180 / math.pi)
            empty = (s.fingers.abs() < .003).all(-1) & (s.finger_forces < .5).all(-1)
            aligned = dot >= math.cos(math.radians(2) / 2)
            self.grasp_alignment_dwell = torch.where(ready & aligned & empty,
                                                      self.grasp_alignment_dwell + 1, 0)
            # Correct pose at the existing 100 mm standoff before descent.
            # A drift during descent may retreat only with open/unloaded fingers.
            drift = (self.stage == 1) & (dot < math.cos(math.radians(3) / 2))
            self.grasp_alignment_blocked_contact.copy_(drift & ~empty)
            self.stage[drift & empty] = 0
            ready &= (self.grasp_alignment_dwell >= 9) & ~drift
        else:
            self.grasp_alignment_dwell.zero_()
        self.stage[ready] = 1
        ready = (self.stage == 1) & ~self.grasp_alignment_blocked_contact & (torch.linalg.vector_norm(s.bar + s.bar.new_tensor([0, 0, .004]) - s.tcp, dim=-1) < .005)
        self.stage[ready] = 2
        self.anchor[ready] = s.tcp[ready]
        ready = (self.stage == 2) & (logic.contact_dwell >= 8) & (s.finger_forces >= 15).all(-1)
        self.stage[ready] = 3
        self.stage[(self.stage == 3) & logic.pick_complete] = 4
        ready = (self.stage == 4) & ((joints - joints.new_tensor(CARRY_JOINTS)).abs().amax(-1) < .025)
        self.stage[ready] = 6
        stand_off = s.tcp.new_tensor(goal) + s.tcp.new_tensor([0., .06, 0.])
        ready = (self.stage == 6) & (torch.linalg.vector_norm(stand_off - s.tcp, dim=-1) < .012)
        self.stage[ready] = 7
        ready = (self.stage == 7) & (logic.phase == 2)
        self.stage[ready] = 8
        self.anchor[ready] = s.tcp[ready]
        supported=(s.jaw_forces>=.5).all(-1)
        if self.require_release_clearance:
            # Current 2.1 kg specimen and estimated 0.8 jaw friction: do not
            # begin robot release on four tiny contact impulses alone.
            supported &= s.jaw_forces.sum(-1)>=40.
        self.dwell = torch.where((self.stage == 8) & supported, self.dwell + 1, 0)
        self.stage[(self.stage == 8) & (self.dwell >= 8)] = 9
        opened=(s.fingers.abs() <= .003).all(-1)
        if self.require_release_clearance:
            clear=(self.stage==9) & opened & (s.finger_forces<.5).all(-1) & supported
            self.release_dwell=torch.where(clear,self.release_dwell+1,0)
            opened &= self.release_dwell>=3
        self.stage[(self.stage == 9) & opened] = 10

    def targets(self, state, goal):
        s = state
        desired = s.bar.clone()
        desired[:, 2] += .10
        desired[self.stage == 1, 2] = s.bar[self.stage == 1, 2] + .004
        desired[self.stage == 2] = self.anchor[self.stage == 2]
        desired[self.stage == 3] = self.initial_bar[self.stage == 3] + s.bar.new_tensor([0., 0., .16])
        desired[self.stage == 4] = s.bar.new_tensor([-.80, 0., 1.25])
        desired[self.stage == 6] = s.bar.new_tensor(goal) + s.bar.new_tensor([0., .06, 0.])
        desired[self.stage == 7] = s.tcp[self.stage == 7] + s.bar.new_tensor(goal) - s.bar[self.stage == 7]
        desired[self.stage >= 8] = self.anchor[self.stage >= 8]
        desired[self.stage >= 10, 2] += .10
        quaternion = self.quaternion.clone()
        quaternion[self.stage >= 4] = s.bar.new_tensor([math.sqrt(.5), 0., 0., math.sqrt(.5)])
        grip = torch.where((self.stage < 2) | (self.stage >= 9), -1., 1.)
        return desired, quaternion, grip
