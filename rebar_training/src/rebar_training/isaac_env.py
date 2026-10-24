"""Three physical, automatically resetting DirectRLEnv tasks (Lab 3 beta2).

The insertion fixture is an autonomous simulated machine: it closes after the
alignment dwell. The policy controls only the six arm joints and robot gripper.
No demo controller, ROS bridge, grasp joint or kinematic steel hold is loaded.
"""

import math

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import DirectRLEnv
from isaaclab.sensors import ContactSensor, ContactSensorCfg

from .core import EpisodeLogic, TaskState, action_targets
from .reset import ResetSampler, quat_apply, quat_multiply
from .route import LoadingRoute, LOADING_CONTRACT
from .spec import ARM_JOINTS, FINGER_JOINTS, JAW_NAMES, OBSERVATION_SIZE, TaskSpec, asset_directory


class RebarEnv(DirectRLEnv):
    def __init__(self, cfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self.task_spec = TaskSpec(task=cfg.task, control_dt=cfg.sim.dt * cfg.decimation,
                             episode_seconds=cfg.episode_length_s,
                             position_tolerance=getattr(cfg, "position_tolerance", TaskSpec().position_tolerance),
                             insert_goal=getattr(cfg, "insert_goal", TaskSpec().insert_goal))
        self.logic = EpisodeLogic(self.task_spec, self.num_envs, self.device)
        self.route = LoadingRoute(self.num_envs, self.device,
            require_release_clearance=getattr(cfg,'visual_control',False)) if cfg.task == "loading" else None
        self.observation_contract = LOADING_CONTRACT if cfg.task == "loading" else "state_v1"
        self.sampler = ResetSampler(cfg.seed if cfg.seed is not None else 42, self.device)
        self.control_ids, names = self.robot.find_joints(list(ARM_JOINTS + FINGER_JOINTS), preserve_order=True)
        if tuple(names) != ARM_JOINTS + FINGER_JOINTS:
            raise RuntimeError(f"unexpected control joint order: {names}")
        self.jaw_ids, _ = self.fixture.find_joints([n + "_joint" for n in JAW_NAMES], preserve_order=True) if cfg.task != "grasp" else ([], [])
        self.carriage_ids = []
        if cfg.task != "grasp" and cfg.fixture_model == "full_v2":
            self.carriage_ids, _ = self.fixture.find_joints(["upper_carriage_joint", "lower_carriage_joint"], preserve_order=True)
        self.carriage_targets = torch.zeros(self.num_envs, len(self.carriage_ids), device=self.device)
        self.wrist_id = self.robot.find_bodies("wrist3_Link")[0][0]
        self.targets = torch.zeros(self.num_envs, 8, device=self.device)
        limits = self.robot.data.soft_joint_pos_limits.torch[:, self.control_ids]
        self.lower, self.upper = limits[..., 0], limits[..., 1]
        self.actions = torch.zeros(self.num_envs, 7, device=self.device)
        self.close_fixture = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.jaw_targets = torch.zeros(self.num_envs, 4, device=self.device)
        self._last_result = None
        self.completed_episodes = []  # drained by evaluation after every step
        self.capture_recorder = None

    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot)
        self.bar = RigidObject(self.cfg.bar)
        self.scene.articulations["robot"] = self.robot
        self.scene.rigid_objects["rebar"] = self.bar
        if self.cfg.task in ("grasp", "loading"):
            self.station = RigidObject(self.cfg.station)
            self.scene.rigid_objects["station"] = self.station
        if self.cfg.task != "grasp":
            if self.cfg.task == "loading":
                from pxr import UsdGeom, Gf
                import omni.usd
                cell = UsdGeom.Xform.Define(omni.usd.get_context().get_stage(), "/World/envs/env_0/Cell")
                cell.AddTranslateOp().Set(Gf.Vec3d(*getattr(self.cfg,'cell_translation',(-.38,.10,0.))))
                cell.AddRotateZOp().Set(90.0)
            self.fixture = Articulation(self.cfg.fixture)
            self.scene.articulations["fixture"] = self.fixture
            machine = sim_utils.UsdFileCfg(usd_path=str(asset_directory() / "machine.usda"))
            machine.func("/World/envs/env_0/" + ("Cell/Machine" if self.cfg.task == "loading" else "Machine"), machine)
        self.contact_sensors = {}
        self.contact_pairs = set()
        if self.cfg.record_contact_pairs:
            from omni.physx import get_physx_simulation_interface
            from pxr import PhysicsSchemaTools
            def contacts(headers, _data):
                for header in headers:
                    pair = (str(PhysicsSchemaTools.intToSdfPath(header.actor0)),
                            str(PhysicsSchemaTools.intToSdfPath(header.actor1)))
                    if pair not in self.contact_pairs:
                        print("CONTACT_PAIR", pair, flush=True)
                        self.contact_pairs.add(pair)
            self._contact_subscription = get_physx_simulation_interface().subscribe_contact_report_events(contacts)

        def contact(name, path, filtered=True):
            sensor = ContactSensor(ContactSensorCfg(
                prim_path="/World/envs/env_.*/" + path,
                update_period=0.0, history_length=self.cfg.decimation,
                filter_prim_paths_expr=["/World/envs/env_.*/Rebar/body"] if filtered else [],
            ))
            self.contact_sensors[name] = sensor
            self.scene.sensors[name] = sensor

        for i in range(2):
            contact(f"finger_{i}", f"Robot/gripper{i + 1}_link")
        if self.cfg.task != "grasp":
            for name in JAW_NAMES:
                contact(name, ("Cell/Fixture/" if self.cfg.task == "loading" else "Fixture/") + name)
        contact("bar_all", "Rebar/body", False)
        contact("arm_all", "Robot/(shoulder_Link|upperArm_Link|foreArm_Link|wrist[123]_Link|gripper_motor_link|gripper_adapter_link|mono_camera_link)", False)
        # GroundPlaneCfg's default references a cloud USD. Keep this training
        # scene local so startup does not depend on the NVIDIA asset server.
        ground_span = 2 * self.cfg.scene.env_spacing * math.ceil(math.sqrt(self.num_envs)) + 10
        ground = sim_utils.CuboidCfg(
            size=(ground_span, ground_span, 0.10),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=1.0),
        )
        ground.func("/World/ground", ground, translation=(0, 0, -0.05))
        if getattr(self.cfg, "camera_capture_config", None):
            from .vision.capture import setup_cameras
            setup_cameras(self)
        self.scene.clone_environments(copy_from_source=True)
        self.scene.filter_collisions(global_prim_paths=["/World/ground"])
        light = sim_utils.DomeLightCfg(intensity=2000.0)
        light.func("/World/Light", light)

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.sampler.seed(seed)
        return super().reset(seed=seed, options=options)

    def _pre_physics_step(self, actions):
        self.actions = actions.clone()  # preserve invalid values for failure detection
        measured = self.robot.data.joint_pos.torch[:, self.control_ids]
        self.targets = action_targets(actions, measured, self.targets, self.lower, self.upper, self.task_spec)
        if self.cfg.task != "grasp":
            closed = torch.tensor([-0.073, 0.073, -0.073, 0.073], device=self.device)
            self.jaw_targets = torch.where(self.close_fixture[:, None], closed, 0.0)

    def _apply_action(self):
        self.robot.set_joint_position_target_index(target=self.targets, joint_ids=self.control_ids)
        if self.cfg.task != "grasp":
            self.fixture.set_joint_position_target_index(target=self.jaw_targets, joint_ids=self.jaw_ids)
            if self.carriage_ids:
                self.fixture.set_joint_position_target_index(target=self.carriage_targets, joint_ids=self.carriage_ids)

    def _wrist_tcp(self):
        q = self.robot.data.body_quat_w.torch[:, self.wrist_id]
        offset = torch.zeros(self.num_envs, 3, device=self.device)
        offset[:, 2] = 0.16  # measured wrist->finger TCP in the existing grasp tests
        tcp = self.robot.data.body_pos_w.torch[:, self.wrist_id] + quat_apply(q, offset)
        return tcp, q

    def _filtered_force(self, name):
        matrix = self.contact_sensors[name].data.force_matrix_w
        if matrix is None:
            raise RuntimeError(f"missing rebar-filtered contact matrix for {name}")
        return torch.linalg.vector_norm(matrix.torch, dim=-1).sum(dim=(1, 2))

    def _state(self):
        tcp, q = self._wrist_tcp()
        identity_x = torch.zeros(self.num_envs, 3, device=self.device)
        identity_x[:, 0] = 1
        bar_axis = quat_apply(self.bar.data.root_quat_w.torch, identity_x)
        finger = torch.stack([self._filtered_force(f"finger_{i}") for i in range(2)], -1)
        jaw = (torch.stack([self._filtered_force(n) for n in JAW_NAMES], -1) if self.cfg.task != "grasp"
               else torch.zeros(self.num_envs, 4, device=self.device))
        peak_forces = []
        self._contact_peaks = {}
        for name, sensor in self.contact_sensors.items():
            history = sensor.data.net_forces_w_history.torch
            self._contact_peaks[name] = torch.linalg.vector_norm(history, dim=-1).amax(1)
            peak_forces.append(torch.linalg.vector_norm(history, dim=-1).flatten(1).amax(1))
        return TaskState(
            tcp=tcp - self.scene.env_origins,
            bar=self.bar.data.root_pos_w.torch - self.scene.env_origins,
            bar_axis=bar_axis,
            linear_velocity=self.bar.data.root_lin_vel_w.torch,
            angular_velocity=self.bar.data.root_ang_vel_w.torch,
            fingers=self.robot.data.joint_pos.torch[:, self.control_ids[6:]],
            finger_forces=finger, jaw_forces=jaw, unsafe_force=torch.stack(peak_forces, -1).amax(-1),
            # Asset audit rejects attachments / kinematic steel before launch.
            assisted=torch.zeros(self.num_envs, dtype=torch.bool, device=self.device),
        )

    def _get_dones(self):
        # DirectRLEnv calls this once after physics, then rewards, then reset.
        state = self._state()
        self._last_result = self.logic.advance(state, self.actions)
        if self.route is not None:
            self.route.advance(state, self.logic, self.robot.data.joint_pos.torch[:, self.control_ids[:6]], self.task_spec.insert_goal)
        if getattr(self, "capture_recorder", None) is not None:
            self.capture_recorder.record(state, self._last_result)
        if getattr(self.cfg, "record_diagnostics", False):
            from dataclasses import fields
            self.diagnostic_state = {field.name: getattr(state, field.name).clone() for field in fields(state)}
            self.diagnostic_peaks = {name: value.clone() for name, value in self._contact_peaks.items()}
            self.diagnostic_counters = {name: getattr(self.logic, name).clone()
                                        for name in ("held", "aligned", "verified", "pick_complete")}
            if self.route is not None:
                self.diagnostic_counters["route_stage"] = self.route.stage.clone()
                self.diagnostic_counters["route_dwell"] = self.route.dwell.clone()
        self.close_fixture.copy_(self._last_result.close_fixture)
        return self._last_result.terminated, self._last_result.truncated

    def _get_rewards(self):
        return self._last_result.reward

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        if self._last_result is not None:
            finished = ids[~self.logic.active[ids]]
            if len(finished):
                result = self._last_result
                log = self.extras.setdefault("log", {})
                log["Metrics/success_rate"] = result.success[finished].float().mean().item()
                for name in ("dropped", "over_force", "invalid", "assisted", "position_error_m"):
                    log["Metrics/" + name] = torch.nan_to_num(result.terms[name][finished].float()).mean().item()
                # Avoid unbounded Python lists during long training runs.
                self.completed_episodes = [
                    {"env_id": int(i), "success": bool(result.success[i]),
                     "failure": bool(result.failure[i]), "timeout": bool(result.truncated[i]),
                     "steps": int(self.logic.elapsed[i]),
                     "position_error_m": (float(result.terms["position_error_m"][i])
                                           if torch.isfinite(result.terms["position_error_m"][i]) else None),
                     "picked": bool(result.terms["picked"][i]),
                     "dropped": bool(result.terms["dropped"][i]), "over_force": bool(result.terms["over_force"][i])}
                    for i in finished.tolist()
                ]
        super()._reset_idx(ids)
        batch = self.sampler.sample(self.cfg.task, len(ids), getattr(self.cfg, 'reset_profile', 'baseline'))
        joint_pos = self.robot.data.default_joint_pos.torch[ids].clone()
        selected = torch.maximum(torch.minimum(batch.joints, self.upper[ids]), self.lower[ids])
        joint_pos[:, self.control_ids] = selected
        self.robot.write_joint_position_to_sim_index(position=joint_pos, env_ids=ids)
        self.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(joint_pos), env_ids=ids)
        self.targets[ids] = selected
        if self.cfg.task != "grasp":
            if self.cfg.task == "insert":
                self.targets[ids, 6:] = self.task_spec.finger_travel
            zeros = torch.zeros(len(ids), 4, device=self.device)
            self.fixture.write_joint_position_to_sim_index(position=zeros, joint_ids=self.jaw_ids, env_ids=ids)
            self.fixture.write_joint_velocity_to_sim_index(velocity=zeros, joint_ids=self.jaw_ids, env_ids=ids)
            self.jaw_targets[ids] = 0
            self.fixture.set_joint_position_target_index(target=zeros, joint_ids=self.jaw_ids, env_ids=ids)
            if self.carriage_ids:
                carriage_zeros = torch.zeros(len(ids), len(self.carriage_ids), device=self.device)
                self.carriage_targets[ids] = 0
                self.fixture.write_joint_position_to_sim_index(position=carriage_zeros, joint_ids=self.carriage_ids, env_ids=ids)
                self.fixture.write_joint_velocity_to_sim_index(velocity=carriage_zeros, joint_ids=self.carriage_ids, env_ids=ids)
                self.fixture.set_joint_position_target_index(target=carriage_zeros, joint_ids=self.carriage_ids, env_ids=ids)
        self.robot.set_joint_position_target_index(target=self.targets[ids], joint_ids=self.control_ids, env_ids=ids)
        pose = batch.bar_pose.clone()
        if self.cfg.task in ("grasp", "loading"):
            station_pose = batch.station_pose.clone()
            station_pose[:, :3] += self.scene.env_origins[ids]
            self.station.write_root_pose_to_sim_index(root_pose=station_pose, env_ids=ids)
            self.station.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(ids), 6, device=self.device), env_ids=ids)
        else:
            # Refresh FK without stepping time or advancing other episodes.
            self.scene.write_data_to_sim()
            self.sim.forward()
            self.robot.update(0.0)
            tcp, wrist_q = self._wrist_tcp()
            pose[:, :3] = tcp[ids] - self.scene.env_origins[ids]
            # Source bar local +X -> wrist +Y, perpendicular to finger closure.
            relative = torch.tensor([0, 0, 2 ** -0.5, 2 ** -0.5], device=self.device).expand(len(ids), -1)
            pose[:, 3:] = quat_multiply(wrist_q[ids], relative)
        self.logic.reset(ids, pose[:, 2])
        if self.route is not None and not getattr(self.cfg, 'visual_control', False):
            self.route.reset(ids, pose)
        pose[:, :3] += self.scene.env_origins[ids]
        self.bar.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)
        self.bar.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(ids), 6, device=self.device), env_ids=ids)
        self.actions[ids] = 0
        self.close_fixture[ids] = False
        if getattr(self.cfg, "camera_dataset_root", None):
            if self.capture_recorder is None:
                from .vision.capture import CaptureRecorder
                self.capture_recorder = CaptureRecorder(self)
            self.capture_recorder.reset(ids)

    def close(self):
        recorder = getattr(self, "capture_recorder", None)
        if recorder is not None:
            recorder.close()
            self.capture_recorder = None
        super().close()

    def _get_observations(self):
        return self._pack_observations(self._state(), self.logic, self.route)

    def _pack_observations(self, state, logic, route):
        _, wrist_q = self._wrist_tcp()
        axes = []
        for axis in ((0, 0, 1), (1, 0, 0)):
            axes.append(quat_apply(wrist_q, torch.tensor(axis, device=self.device, dtype=torch.float32).expand(self.num_envs, -1)))
        q = self.robot.data.joint_pos.torch[:, self.control_ids]
        q_scaled = 2 * (q - self.lower) / (self.upper - self.lower).clamp_min(1e-6) - 1
        goal = torch.tensor(self.task_spec.insert_goal, device=self.device).expand(self.num_envs, -1).clone()
        if self.cfg.task == "grasp":
            goal = state.bar.clone()
            goal[:, 2] = logic.initial_height + self.task_spec.lift_height
        elif self.cfg.task == "loading":
            acquire_goal = state.bar.clone()
            acquire_goal[:, 2] = logic.initial_height + self.task_spec.lift_height
            goal = torch.where(logic.pick_complete[:, None], goal, acquire_goal)
        jaws = (self.fixture.data.joint_pos.torch[:, self.jaw_ids] if self.cfg.task != "grasp"
                else torch.zeros(self.num_envs, 4, device=self.device))
        relative_goal = goal - state.bar
        progress = logic.progress()
        if route is not None:
            desired, _, _ = route.targets(state, self.task_spec.insert_goal)
            relative_goal = desired - state.tcp
            progress = torch.stack((route.stage / 10., logic.contact_dwell.float().clamp_max(8) / 8.,
                                    logic.verified / self.task_spec.handoff_hold_steps), -1)
        obs = torch.cat((q_scaled, self.robot.data.joint_vel.torch[:, self.control_ids] * 0.1,
                         state.tcp, *axes, state.bar, state.bar_axis, state.linear_velocity,
                         state.angular_velocity, relative_goal, state.finger_forces / 100,
                         state.jaw_forces / 100, jaws / 0.075,
                         torch.nn.functional.one_hot(logic.phase, 3).float(), progress,
                         logic.previous_actions, self.targets), -1)
        if obs.shape[-1] != OBSERVATION_SIZE:
            raise RuntimeError(f"observation contract changed: {obs.shape}")
        return {"policy": torch.nan_to_num(obs).clamp(-10, 10)}
