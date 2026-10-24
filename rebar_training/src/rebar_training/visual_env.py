"""Opt-in visual control. Steel truth is confined to reward/reset/evaluation.

The actor, task route, observation motion, grasp prior and machine commands use
the separate VisualControl path. RGB is its only environmental sensor modality.
"""
from dataclasses import replace
import numpy as np
import torch

from .isaac_env import RebarEnv
from .core import TaskState
from .vision.capture import array, synchronize_mounts, optical_transform
from .vision.observation import ObserveThenReturn
from .vision.observer import CameraPacket, RobotPacket, ImageObserver
from .vision.progress import VisualProgress
from .vision.segment import ImageDetector
from .vision.servo import RobotServo
from .vision.blind import blinded_control

VISUAL_CONTRACT = 'loading_visual_v1'


class VisualControl:
    """Read only robot feedback and camera RGB; never use env._state/bar.data."""
    def __init__(self, env):
        self.env = env
        self.detector = ImageDetector(env.cfg.visual_detector, env.device)
        self.observers = [ImageObserver(self.detector) for _ in range(env.num_envs)]
        self.motion = ObserveThenReturn(env)
        self.motion.allow_return[:] = False
        self.servo = RobotServo(env)
        self.logic = VisualProgress(env.task_spec, env.num_envs, env.device)
        self.valid = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        self.route_initialized = self.valid.clone()
        self.tick = 0
        self.state = None
        self.packets = None
        self.latest_cameras = None
        self.blank_triggered = [False]*env.num_envs
        self.blank_remaining = [0]*env.num_envs
        self.paused_steps = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self.was_paused=self.valid.clone()
        from .randomization import profile
        self.image_bounds=profile(getattr(env.cfg,'reset_profile','baseline'))
        self.image_rng=np.random.default_rng(env.cfg.seed)
        self.image_gain=np.ones(env.num_envs);self.image_offset=np.zeros(env.num_envs)

    def reset(self, ids):
        self.motion.reset(ids)
        self.motion.allow_return[ids] = False
        self.logic.reset(ids)
        self.valid[ids] = False; self.route_initialized[ids] = False
        self.paused_steps[ids] = 0
        self.was_paused[ids]=False
        self.env.route.stage[ids] = self.env.route.dwell[ids] = 0
        self.env.route.anchor[ids] = self.env.route.initial_bar[ids] = 0
        self.env.route.quaternion[ids] = torch.tensor([0., 0., 0., 1.], device=self.env.device)
        for i in ids.tolist():
            self.observers[i].reset()
            self.blank_triggered[i] = False; self.blank_remaining[i] = 0
            self.image_gain[i]=1+self.image_rng.uniform(-1,1)*self.image_bounds.image_gain
            self.image_offset[i]=self.image_rng.uniform(-1,1)*self.image_bounds.image_offset

    def robot_packets(self, timestamp):
        e = self.env
        tcp, quat = e._wrist_tcp()
        tcp = array(tcp-e.scene.env_origins); quat = array(quat)
        fingers = array(e.robot.data.joint_pos)[:, e.control_ids[6:]]
        # Raw tactile load, without the simulator's identity-filtered Rebar
        # contact matrix. A real sensor cannot identify its contacting object.
        def tactile(name):
            load = array(e.contact_sensors[name].data.net_forces_w)
            return np.linalg.norm(load, axis=-1).sum(axis=1)
        force = np.stack([tactile('finger_'+str(i)) for i in range(2)], -1)
        jaws = np.stack([tactile(n) for n in ('upper_left','upper_right','lower_left','lower_right')], -1)
        return [RobotPacket(tcp[i], quat[i], fingers[i], force[i], jaws[i], timestamp) for i in range(e.num_envs)]

    def camera_packets(self, timestamp):
        e = self.env
        synchronize_mounts(e)
        sensor = e.vision_cameras['wrist']
        data = sensor.data
        # Deliberately index only RGB. No depth, segmentation or label lookup.
        rgb = array(data.output['rgb'])
        mono = np.round(rgb[..., :3] @ np.array([.299,.587,.114])).clip(0,255).astype(np.uint8)
        from .randomization import perturb_image
        for i in range(e.num_envs):
            mono[i]=perturb_image(mono[i],self.image_gain[i],self.image_offset[i],
                                 self.image_bounds.image_noise_std,self.image_rng)
        if e.cfg.visual_blank_stage >= 0:
            for i in range(e.num_envs):
                if not self.blank_triggered[i] and e.route.stage[i] >= e.cfg.visual_blank_stage:
                    self.blank_triggered[i] = True; self.blank_remaining[i] = e.cfg.visual_blank_frames
                if self.blank_remaining[i] > 0:
                    mono[i] = 0; self.blank_remaining[i] -= 1
        position = array(data.pos_w)-array(e.scene.env_origins)
        quaternion = array(data.quat_w_ros)
        K, frame = array(data.intrinsic_matrices), array(sensor.frame)
        self.latest_cameras = [CameraPacket(mono[i], K[i], np.asarray(optical_transform(position[i],quaternion[i])),
                             timestamp, int(frame[i])) for i in range(e.num_envs)]
        return self.latest_cameras

    def measured_state(self):
        e = self.env
        packet = self.robot_packets(round(self.tick*e.task_spec.control_dt*1e9))
        def tensor(values):
            return torch.as_tensor(np.asarray(values), device=e.device, dtype=torch.float32)
        # Zero placeholders are exposed only while no action policy is enabled.
        poses = [np.nan_to_num(o.position) for o in self.observers]
        axes = [np.nan_to_num(o.axis) for o in self.observers]
        return TaskState(tcp=tensor([p.tcp for p in packet]), bar=tensor(poses), bar_axis=tensor(axes),
            linear_velocity=tensor([o.linear_velocity for o in self.observers]),
            angular_velocity=tensor([o.angular_velocity for o in self.observers]),
            fingers=tensor([p.fingers for p in packet]), finger_forces=tensor([p.finger_forces for p in packet]),
            jaw_forces=tensor([p.jaw_forces for p in packet]),
            unsafe_force=tensor([max(np.max(p.finger_forces),np.max(p.jaw_forces)) for p in packet]),
            assisted=torch.zeros(e.num_envs, dtype=torch.bool, device=e.device))

    def after_physics(self):
        e = self.env
        self.tick += 1
        timestamp = round(self.tick*e.task_spec.control_dt*1e9)
        self.packets = self.robot_packets(timestamp)
        cameras = self.camera_packets(timestamp) if self.tick % 3 == 0 else [None]*e.num_envs
        for i, observer in enumerate(self.observers):
            self.valid[i] = observer.step(self.packets[i], cameras[i],
                survey_allowed=bool(self.motion.motion.ready[i]),
                grasp_allowed=bool(e.route.stage[i] >= 2), release_requested=bool(e.route.stage[i] >= 9))
            self.motion.allow_return[i] = observer.acquired
            if observer.acquired and not self.route_initialized[i]:
                pose = torch.zeros(1, 7, device=e.device)
                pose[0, :3] = torch.as_tensor(observer.initial_position, device=e.device)
                yaw = np.arctan2(observer.initial_axis[1], observer.initial_axis[0])
                pose[0, 5], pose[0, 6] = np.sin(yaw/2), np.cos(yaw/2)
                e.route.reset(torch.tensor([i], device=e.device), pose)
                self.route_initialized[i] = True
        self.state = self.measured_state()
        active = self.valid & self.motion.ready
        self.logic.advance(self.state, e.actions, active)
        old_stage = e.route.stage.clone(); old_dwell = e.route.dwell.clone(); old_anchor = e.route.anchor.clone()
        old_release = e.route.release_dwell.clone()
        e.route.advance(self.state, self.logic, e.robot.data.joint_pos.torch[:, e.control_ids[:6]], e.task_spec.insert_goal)
        e.route.stage[~active] = old_stage[~active]
        e.route.dwell[~active] = old_dwell[~active]
        e.route.anchor[~active] = old_anchor[~active]
        e.route.release_dwell[~active] = old_release[~active]
        for i in torch.where((old_stage == 2) & (e.route.stage == 3))[0].tolist():
            self.observers[i].latch_grasp(self.packets[i])
        e.close_fixture.copy_(self.logic.phase == 2)

    def action(self, actor=None):
        e = self.env
        bootstrap = self.motion.actions(self.servo)
        active = self.motion.ready & self.valid
        state = self.measured_state()
        if actor is None:
            desired = self.servo.route_actions(state, e.route)
        else:
            desired = actor(e._pack_observations(state,self.logic,e.route)['policy'])
        actions = torch.where(active[:,None], desired, bootstrap)
        paused = self.motion.ready & ~self.valid
        self.hold_on_perception_loss(actions,paused)
        if paused.any():
            actions[paused,6] = torch.where((e.route.stage[paused] >= 2) & (e.route.stage[paused] < 9), 1., -1.)
        return actions

    def hold_on_perception_loss(self,actions,paused):
        e=self.env;entering=paused & ~self.was_paused
        if entering.any():
            arm=e.cfg.robot.actuators['arm']
            # Preserve the target's gravity-supporting bias. Continuously
            # replacing targets with measured q disables position stiffness
            # under gravity and made a camera outage drop the loaded arm.
            damping_time=float(arm.damping)/float(arm.stiffness)
            velocity=e.robot.data.joint_vel.torch[:,e.control_ids[:6]]
            target=e.targets[:,:6]-damping_time*velocity
            e.targets[entering,:6]=torch.maximum(torch.minimum(target[entering],e.upper[entering,:6]),e.lower[entering,:6])
        actions[paused,:6]=0
        self.paused_steps[paused]+=1
        self.was_paused.copy_(paused)


class VisualRebarEnv(RebarEnv):
    def __init__(self, cfg, **kwargs):
        self.visual = None
        self.visual_terminal_receipts = []
        super().__init__(cfg, **kwargs)
        self.observation_contract = VISUAL_CONTRACT

    def _reset_idx(self, ids):
        super()._reset_idx(ids)
        if ids is None:
            ids = torch.arange(self.num_envs, device=self.device)
        if self.visual is None:
            self.visual = VisualControl(self)
        self.visual.reset(torch.as_tensor(ids, device=self.device, dtype=torch.long))

    def _get_observations(self):
        if self.visual is None:
            return {'policy': torch.zeros(self.num_envs,71,device=self.device)}
        with blinded_control(self):
            return self._pack_observations(self.visual.measured_state(),self.visual.logic,self.route)

    def _get_dones(self):
        # Truth branch: evaluation/reward only. No result feeds phase or clamps.
        truth = self._state()
        result = self.logic.advance(truth, self.actions)
        # Independent control branch: RGB, calibrated mount and robot feedback.
        with blinded_control(self):
            self.visual.after_physics()
        self.visual_truth_for_evaluation = truth
        # Completion comes from the independent controller, then truth scores
        # that decision. Never wait for a truth-success flag to stop the actor.
        decided = self.visual.logic.verified >= self.task_spec.handoff_hold_steps+15
        success = decided & result.success
        false_confirmation = decided & ~result.success & ~result.failure
        failure = result.failure | false_confirmation
        terminated = decided | failure
        truncated = (self.logic.elapsed >= self.task_spec.episode_steps) & ~terminated
        self.logic.active.copy_(~(terminated | truncated))
        terms = {**result.terms, 'success': success, 'failure': failure,
                 'false_visual_confirmation': false_confirmation}
        max_step_cost = 8*self.task_spec.control_dt+.01*7+.02*4*7+.01
        false_cost = 10+(self.task_spec.episode_steps-self.logic.elapsed).clamp_min(0)*max_step_cost
        reward = result.reward-20*result.success.float()+20*success.float()-false_cost*false_confirmation.float()
        self._last_result = replace(result, success=success, failure=failure, terminated=terminated, truncated=truncated,
                                    reward=reward, terms=terms)
        for i in torch.where(terminated | truncated)[0].tolist():
            observer = self.visual.observers[i]
            error = float(torch.linalg.vector_norm(self.visual.state.bar[i]-truth.bar[i]))
            self.visual_terminal_receipts.append(dict(env_id=i, control_decided=bool(decided[i]),
                false_confirmation=bool(false_confirmation[i]), estimated_position=self.visual.state.bar[i].tolist(),
                true_position_for_evaluation=truth.bar[i].tolist(), estimated_position_error_m=error,
                visual_measurements=observer.measurements, rejected_measurements=observer.rejected_measurements,
                last_perception_error=observer.last_error, paused_steps=int(self.visual.paused_steps[i]),
                route_stage=int(self.route.stage[i]), control_verified_steps=int(self.visual.logic.verified[i])))
        return self._last_result.terminated, self._last_result.truncated
