"""Feedback controller for physical feasibility checks and curriculum references.

This controls only the same seven actions as the policy. It never writes a bar
pose, creates an attachment, or bypasses task termination. Expert success is
reported separately from learned-policy success.
"""
import math
import torch

from .reset import quat_multiply
from .route import CARRY_JOINTS


def limited(vector, limit):
    return vector * (limit / torch.linalg.vector_norm(vector, dim=-1, keepdim=True).clamp_min(limit))


class CartesianExpert:
    def __init__(self, env):
        self.env = env
        self.stage = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self.dwell = torch.zeros_like(self.stage)
        self.age = torch.zeros_like(self.stage)
        self.anchor = torch.zeros(env.num_envs, 3, device=env.device)
        self.quaternion = torch.zeros(env.num_envs, 4, device=env.device)
        self.initial_bar = self.anchor.clone()

    def reset(self, ids):
        e = self.env
        s = e._state()
        self.stage[ids] = self.dwell[ids] = self.age[ids] = 0
        self.anchor[ids] = s.tcp[ids]
        self.initial_bar[ids] = s.bar[ids]
        self.quaternion[ids] = e._wrist_tcp()[1][ids]
        if e.cfg.task in ('grasp', 'loading'):
            yaw = torch.atan2(s.bar_axis[ids, 1], s.bar_axis[ids, 0])
            q_yaw = torch.zeros(len(ids), 4, device=e.device)
            q_yaw[:, 2], q_yaw[:, 3] = torch.sin(yaw / 2), torch.cos(yaw / 2)
            nominal = torch.tensor([math.sqrt(.5), math.sqrt(.5), 0., 0.], device=e.device).expand(len(ids), -1)
            self.quaternion[ids] = quat_multiply(q_yaw, nominal)

    def __call__(self):
        e = self.env
        s = e._state()
        if e.cfg.task == 'loading':
            desired, quaternion, grip = e.route.targets(s, e.task_spec.insert_goal)
            actions = self.cartesian_actions(desired, quaternion, grip)
            measured = e.robot.data.joint_pos.torch[:, e.control_ids[:6]]
            velocity = 1.5 * (measured.new_tensor(CARRY_JOINTS) - measured)
            scale = velocity.abs().amax(-1, keepdim=True).clamp_min(e.task_spec.arm_speed)
            joint_path = e.route.stage == 4
            actions[joint_path, :6] = (velocity / scale)[joint_path]
            self.stage.copy_(e.route.stage)
            return actions
        self.age += 1
        desired = s.bar.clone()
        desired_quaternion = self.quaternion.clone()
        grip = torch.ones(e.num_envs, device=e.device)
        joint_path = torch.zeros(e.num_envs, dtype=torch.bool, device=e.device)
        if e.cfg.task in ('grasp', 'loading'):
            desired[:, 2] += .10
            approach = (self.stage == 0)
            ready = approach & (torch.linalg.vector_norm(desired - s.tcp, dim=-1) < .006)
            self.stage[ready], self.age[ready] = 1, 0
            approach = self.stage == 0
            down = self.stage == 1
            desired[~approach, 2] = s.bar[~approach, 2] + .004
            ready = down & (torch.linalg.vector_norm(desired - s.tcp, dim=-1) < .005)
            self.stage[ready], self.age[ready] = 2, 0
            self.anchor[ready] = s.tcp[ready]
            closing = self.stage == 2
            desired[closing] = self.anchor[closing]
            ready = closing & (e.logic.contact_dwell >= 8) & (s.finger_forces >= 15).all(-1)
            self.stage[ready], self.age[ready] = 3, 0
            lifting = self.stage >= 3
            desired[lifting] = self.initial_bar[lifting]
            desired[lifting, 2] += .16
            grip[self.stage < 2] = -1

        else:
            goal = torch.tensor(e.task_spec.insert_goal, device=e.device)
            desired = s.tcp + goal - s.bar
            fixture_closing = (e.logic.phase == 1) & (self.stage == 0)
            self.stage[fixture_closing] = 1
            self.anchor[fixture_closing] = s.tcp[fixture_closing]
            closing = self.stage == 1
            desired[self.stage >= 1] = self.anchor[self.stage >= 1]
            supported = (s.jaw_forces >= .5).all(-1)
            self.dwell = torch.where(closing & supported, self.dwell + 1, 0)
            ready = closing & (self.dwell >= 8)
            self.stage[ready], self.age[ready] = 2, 0
            ready = (self.stage == 2) & (s.fingers.abs() <= .003).all(-1)
            self.stage[ready], self.age[ready] = 3, 0
            # Straight X retreat drives this compact elbow branch into the
            # machine deck. With fingers open, retreat upward along the bar.
            desired[self.stage >= 3, 2] += .10
            grip[self.stage >= 2] = -1
        actions = self.cartesian_actions(desired, desired_quaternion, grip)

        return actions

    def cartesian_actions(self, desired, desired_quaternion, grip):
        e = self.env
        tcp, q = e._wrist_tcp()
        tcp = tcp - e.scene.env_origins
        inverse = q.clone()
        inverse[:, :3] *= -1
        delta = quat_multiply(desired_quaternion, inverse)
        rotation = 2 * delta[:, :3] * torch.where(delta[:, 3:] < 0, -1., 1.)
        speed = .08 if e.cfg.task == 'loading' else .045
        velocity = torch.cat((limited(4 * (desired - tcp), speed), limited(2 * rotation, .3)), -1)
        index = e.wrist_id - int(e.robot.is_fixed_base)
        # PhysX prepends six floating-root columns for a free mobile base.
        columns = [j + (0 if e.robot.is_fixed_base else 6) for j in e.control_ids[:6]]
        jac = e.robot.data.body_link_jacobian_w.torch[:, index, :, columns].clone()
        offset = (e._wrist_tcp()[0] - e.robot.data.body_link_pos_w.torch[:, e.wrist_id])
        jac[:, :3] += torch.linalg.cross(jac[:, 3:].transpose(1, 2), offset[:, None, :], dim=-1).transpose(1, 2)
        regularizer = torch.eye(6, device=e.device)[None] * .002 ** 2
        joint_velocity = (jac.transpose(1, 2) @ torch.linalg.solve(jac @ jac.transpose(1, 2) + regularizer, velocity[..., None])).squeeze(-1)
        arm = joint_velocity / e.task_spec.arm_speed
        arm = arm / arm.abs().amax(-1, keepdim=True).clamp_min(1.)
        actions = torch.cat((arm, grip[:, None]), -1).clamp(-1, 1)
        return actions
