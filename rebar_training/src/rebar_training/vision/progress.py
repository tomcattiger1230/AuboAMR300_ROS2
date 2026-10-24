"""Control phases from estimated steel pose and measured contact, never rewards."""
import math
import torch


class VisualProgress:
    def __init__(self, spec, count, device):
        self.spec = spec
        self.alignment_tolerance=None
        self.phase = torch.zeros(count, dtype=torch.long, device=device)
        self.contact_dwell = torch.zeros_like(self.phase)
        self.held = torch.zeros_like(self.phase)
        self.aligned = torch.zeros_like(self.phase)
        self.verified = torch.zeros_like(self.phase)
        self.pick_complete = torch.zeros(count, dtype=torch.bool, device=device)
        self.previous_actions = torch.zeros(count, 7, device=device)
        self.initial_height = torch.full((count,), .522, device=device)

    def reset(self, ids):
        for tensor in (self.phase, self.contact_dwell, self.held, self.aligned, self.verified):
            tensor[ids] = 0
        self.pick_complete[ids] = False
        self.previous_actions[ids] = 0

    def advance(self, state, actions, valid):
        s, p = state, self.spec
        distance = torch.linalg.vector_norm(s.tcp-s.bar, dim=-1)
        grasped = (s.finger_forces >= p.contact_force_min).all(-1) & (distance < p.grasp_distance) & valid
        stable = (torch.linalg.vector_norm(s.linear_velocity, dim=-1) <= p.stable_linear_speed)
        stable &= torch.linalg.vector_norm(s.angular_velocity, dim=-1) <= p.stable_angular_speed
        upright = s.bar_axis[:, 2].abs() >= math.cos(p.axis_tolerance_rad)
        self.contact_dwell = torch.where(grasped, self.contact_dwell+1, 0)
        lifted = s.bar[:, 2]-self.initial_height >= p.lift_height
        self.held = torch.where(grasped & stable & lifted, self.held+1, 0)
        self.pick_complete |= self.held >= p.grasp_hold_steps
        self.phase = torch.where(self.pick_complete & (self.phase == 0), 1, self.phase)
        error = torch.linalg.vector_norm(s.bar-s.bar.new_tensor(p.insert_goal), dim=-1)
        tolerance=p.position_tolerance if self.alignment_tolerance is None else min(p.position_tolerance,self.alignment_tolerance)
        aligned = (self.pick_complete & grasped & stable & upright & valid & (error <= tolerance))
        self.aligned = torch.where(aligned, self.aligned+1, 0)
        self.phase = torch.where(self.aligned >= p.align_hold_steps, 2, self.phase)
        supported = (s.jaw_forces >= p.contact_force_min).all(-1)
        released = ((s.fingers.abs() <= .003).all(-1) & (s.finger_forces < p.contact_force_min).all(-1)
                    & (distance >= p.release_distance))
        confirm = (self.phase == 2) & valid & supported & released & stable & upright & (error <= p.position_tolerance)
        self.verified = torch.where(confirm, self.verified+1, 0)
        self.previous_actions.copy_(actions.clamp(-1, 1))

    def progress(self):
        return torch.stack((self.held/self.spec.grasp_hold_steps,
            torch.where(self.pick_complete, self.aligned/self.spec.align_hold_steps, self.contact_dwell/8),
            self.verified/self.spec.handoff_hold_steps), -1).clamp(0, 1)
