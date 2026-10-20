"""Batched reward, terminal and reset logic, shared by CPU tests and the GPU env.

No simulated motion lives here: callers supply measured state after each physics
step. Contact forces must be filtered to the rebar, not total finger/fixture force.
"""

from dataclasses import dataclass
import math

import torch

from .spec import ACTION_SIZE, TaskSpec


@dataclass
class TaskState:
    tcp: torch.Tensor                    # [N,3], env-local
    bar: torch.Tensor                    # [N,3], env-local
    bar_axis: torch.Tensor               # [N,3], cylinder local +X in env frame
    linear_velocity: torch.Tensor        # [N,3], m/s
    angular_velocity: torch.Tensor       # [N,3], rad/s
    fingers: torch.Tensor                # [N,2], measured joint positions, m
    finger_forces: torch.Tensor          # [N,2], finger-to-bar force magnitudes, N
    jaw_forces: torch.Tensor             # [N,4], pad-to-bar force magnitudes, N
    unsafe_force: torch.Tensor           # [N], max net robot/bar force, N
    assisted: torch.Tensor               # [N], forbidden attachment/kinematic flag


@dataclass
class StepResult:
    reward: torch.Tensor
    success: torch.Tensor
    failure: torch.Tensor
    terminated: torch.Tensor
    truncated: torch.Tensor
    close_fixture: torch.Tensor
    terms: dict[str, torch.Tensor]


class EpisodeLogic:
    """Advance exactly once per control step, before Isaac's automatic reset."""

    def __init__(self, spec: TaskSpec, num_envs: int, device="cpu"):
        self.spec = spec
        self.num_envs = num_envs
        self.device = device
        self.elapsed = torch.zeros(num_envs, dtype=torch.long, device=device)
        self.phase = torch.zeros_like(self.elapsed)  # 0 approach, 1 physical handoff
        self.held = torch.zeros_like(self.elapsed)
        self.aligned = torch.zeros_like(self.elapsed)
        self.verified = torch.zeros_like(self.elapsed)
        self.contact_dwell = torch.zeros_like(self.elapsed)
        self.pick_complete = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.max_height = torch.zeros(num_envs, device=device)
        self.initial_height = torch.zeros(num_envs, device=device)
        self.previous_actions = torch.zeros(num_envs, ACTION_SIZE, device=device)
        self.active = torch.zeros(num_envs, dtype=torch.bool, device=device)

    def reset(self, env_ids, initial_height):
        ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        heights = torch.as_tensor(initial_height, dtype=torch.float32, device=self.device)
        if heights.shape != ids.shape or not torch.isfinite(heights).all():
            raise ValueError("reset heights must be finite and match env_ids")
        for value in (self.elapsed, self.phase, self.held, self.aligned, self.verified, self.contact_dwell):
            value[ids] = 0
        self.previous_actions[ids] = 0
        self.initial_height[ids] = heights
        self.max_height[ids] = heights
        self.pick_complete[ids] = False
        self.active[ids] = True

    def advance(self, state: TaskState, actions: torch.Tensor) -> StepResult:
        n, s = self.num_envs, self.spec
        shapes = {"tcp": 3, "bar": 3, "bar_axis": 3, "linear_velocity": 3,
                  "angular_velocity": 3, "fingers": 2, "finger_forces": 2, "jaw_forces": 4}
        for key, width in shapes.items():
            if getattr(state, key).shape != (n, width):
                raise ValueError(f"{key} must have shape {(n, width)}")
        if actions.shape != (n, ACTION_SIZE):
            raise ValueError(f"actions must have shape {(n, ACTION_SIZE)}")
        if state.unsafe_force.shape != (n,) or state.assisted.shape != (n,):
            raise ValueError("unsafe_force and assisted must have shape [N]")
        if not self.active.all():
            raise RuntimeError("reset terminated environments before advancing")

        valid = torch.isfinite(actions).all(-1) & torch.isfinite(state.unsafe_force)
        for key in shapes:
            valid &= torch.isfinite(getattr(state, key)).all(-1)
        a = torch.nan_to_num(actions, nan=0.0, posinf=0.0, neginf=0.0).clamp(-1, 1)
        axis_norm = torch.linalg.vector_norm(state.bar_axis, dim=-1)
        valid &= (axis_norm > 1e-6) & torch.isfinite(axis_norm)
        axis_z = (state.bar_axis[:, 2].abs() / axis_norm.clamp_min(1e-6)).clamp(0, 1)
        upright = axis_z >= math.cos(s.axis_tolerance_rad)
        stable = (torch.linalg.vector_norm(state.linear_velocity, dim=-1) <= s.stable_linear_speed)
        stable &= torch.linalg.vector_norm(state.angular_velocity, dim=-1) <= s.stable_angular_speed
        distance = torch.linalg.vector_norm(state.tcp - state.bar, dim=-1)
        # Bilateral bar contact + geometry, not a closed-joint command.
        grasped = (state.finger_forces >= s.contact_force_min).all(-1) & (distance < s.grasp_distance)
        lift = state.bar[:, 2] - self.initial_height
        dropped = lift < -s.drop_height
        if s.task == "loading":
            self.max_height = torch.maximum(self.max_height, state.bar[:, 2])
            supported_now = (state.jaw_forces >= s.contact_force_min).all(-1)
            dropped |= (state.bar[:, 2] < self.max_height - s.drop_height) & ~grasped & ~supported_now
        force_bad = state.unsafe_force > s.force_limit
        force_bad |= torch.cat((state.finger_forces, state.jaw_forces), -1).amax(-1) > s.force_limit
        failure = (~valid) | dropped | force_bad | state.assisted.bool()
        goal = torch.as_tensor(s.insert_goal, device=self.device)
        error = torch.linalg.vector_norm(state.bar - goal, dim=-1)
        aligned_now = (error <= s.position_tolerance) & upright & stable & grasped
        self.elapsed += 1
        self.held = torch.where(grasped & stable & (lift >= s.lift_height), self.held + 1, 0)
        self.aligned = torch.where(aligned_now, self.aligned + 1, 0)
        if s.task in ("grasp", "loading"):
            self.contact_dwell = torch.where(grasped, self.contact_dwell + 1, 0)

        if s.task == "grasp":
            self.phase = torch.where(grasped, torch.where(self.contact_dwell >= 8, 2, 1), 0)
            success = self.held >= s.grasp_hold_steps
            self.pick_complete |= success & ~failure
            reach = torch.exp(-distance / 0.10)
            task_reward = 2.0 * reach + 2.0 * grasped.float() + 4.0 * lift.clamp(0, s.lift_height) / s.lift_height
        else:
            handoff_phase = 1
            if s.task == "loading":
                self.pick_complete |= (self.held >= s.grasp_hold_steps) & ~failure
                self.aligned = torch.where(self.pick_complete, self.aligned, 0)
                handoff_phase = 2
                self.phase = torch.where(self.pick_complete & (self.phase == 0), 1, self.phase)
            self.phase = torch.where(self.aligned >= s.align_hold_steps, handoff_phase, self.phase)
            # Every pad must actually touch the bar. The bar must remain stable
            # after both robot fingers open and move away; no kinematic latch.
            supported = (state.jaw_forces >= s.contact_force_min).all(-1)
            released = ((state.fingers.abs() <= 0.003).all(-1) & (distance >= s.release_distance)
                        & (state.finger_forces < s.contact_force_min).all(-1))
            verify = (self.phase == handoff_phase) & supported & released & upright & stable & (error <= s.position_tolerance)
            self.verified = torch.where(verify, self.verified + 1, 0)
            success = self.verified >= s.handoff_hold_steps
            alignment = torch.exp(-error / 0.05) * axis_z.pow(8)
            # Stop rewarding the robot for holding on once the fixture closes.
            task_reward = 4.0 * alignment + torch.where(self.phase < handoff_phase, grasped.float(), supported.float())
            task_reward += 3.0 * verify.float()
            if s.task == "loading":
                acquire = 2 * torch.exp(-distance / .10) + 2 * grasped.float() + 4 * lift.clamp(0, s.lift_height) / s.lift_height
                task_reward = torch.where(self.pick_complete, task_reward, acquire)
                success &= self.pick_complete

        success &= ~failure  # failure wins even on the last dwell step
        effort = 0.01 * a.square().sum(-1)
        smoothness = 0.02 * (a - self.previous_actions).square().sum(-1)
        # Non-positive running cost discourages farming an intermediate pose
        # until timeout; only completed, unassisted success receives a bonus.
        # A short failed episode must not avoid the remaining running costs.
        # Bound the worst possible step cost: score>=0, seven clipped actions,
        # and an action change of at most 2 per component. Charge this remaining
        # horizon on failure; even the worst continuation then beats quitting.
        max_step_cost = 8.0 * s.control_dt + 0.01 * ACTION_SIZE + 0.02 * 4 * ACTION_SIZE + 0.01
        running = s.control_dt * (task_reward - 8.0) - effort - smoothness - 0.01
        running = torch.nan_to_num(running, nan=-max_step_cost, posinf=-max_step_cost, neginf=-max_step_cost)
        remaining = (s.episode_steps - self.elapsed).clamp_min(0)
        failure_cost = 10.0 + remaining * max_step_cost
        reward = running + 20.0 * success.float() - failure_cost * failure.float()
        terminated = success | failure
        truncated = (self.elapsed >= s.episode_steps) & ~terminated
        self.active &= ~(terminated | truncated)
        self.previous_actions.copy_(a)
        terms = {"success": success, "failure": failure, "invalid": ~valid, "dropped": dropped,
                 "over_force": force_bad, "assisted": state.assisted.bool(), "grasped": grasped,
                 "position_error_m": error if s.task != "grasp" else (s.lift_height - lift).clamp_min(0),
                 "lift_m": lift, "axis_z": axis_z,
                 "picked": self.pick_complete,
                 "action_cost": effort, "smoothness_cost": smoothness}
        return StepResult(reward, success, failure, terminated, truncated,
                          ((self.phase == 1) & (s.task == "insert")) | ((self.phase == 2) & (s.task == "loading")), terms)

    def progress(self):
        return torch.stack((self.held / self.spec.grasp_hold_steps,
                            (torch.where(self.pick_complete, self.aligned / self.spec.align_hold_steps, self.contact_dwell / 8)
                             if self.spec.task == "loading" else (self.contact_dwell / 8 if self.spec.task == "grasp" else self.aligned / self.spec.align_hold_steps)),
                            self.verified / self.spec.handoff_hold_steps), dim=-1).clamp(0, 1)


def action_targets(actions, measured, targets, lower, upper, spec):
    """Six bounded joint velocities plus one absolute symmetric gripper command."""
    a = torch.nan_to_num(actions, nan=0.0, posinf=0.0, neginf=0.0).clamp(-1, 1)
    result = targets.clone()
    arm = targets[:, :6] + a[:, :6] * spec.arm_speed * spec.control_dt
    arm = torch.maximum(torch.minimum(arm, measured[:, :6] + spec.target_lead), measured[:, :6] - spec.target_lead)
    result[:, :6] = arm
    result[:, 6:8] = (a[:, 6:7] + 1.0) * 0.5 * spec.finger_travel
    return torch.maximum(torch.minimum(result, upper), lower)
