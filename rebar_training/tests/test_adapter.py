"""Exercise the actual adapter with tensor doubles; these are not physics tests."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest
torch = pytest.importorskip("torch")

from rebar_training.core import EpisodeLogic
from rebar_training.reset import ResetSampler
from rebar_training.route import LoadingRoute
from rebar_training.spec import OBSERVATION_SIZE, TaskSpec


def proxy(value):
    return SimpleNamespace(torch=value)


class Body:
    def __init__(self, n, joints=0):
        pose = torch.zeros(n, 7)
        pose[:, 6] = 1
        self.data = SimpleNamespace(
            default_joint_pos=proxy(torch.zeros(n, joints)), joint_pos=proxy(torch.zeros(n, joints)),
            joint_vel=proxy(torch.zeros(n, joints)), root_pos_w=proxy(pose[:, :3]),
            root_quat_w=proxy(pose[:, 3:]), root_lin_vel_w=proxy(torch.zeros(n, 3)),
            root_ang_vel_w=proxy(torch.zeros(n, 3)),
            body_pos_w=proxy(torch.zeros(n, 1, 3)), body_quat_w=proxy(torch.full((n, 1, 4), 0.5)),
        )
        self.targets = torch.zeros(n, joints)

    def _write(self, field, value, env_ids, joint_ids=None):
        ids = torch.as_tensor(env_ids, dtype=torch.long)
        tensor = getattr(self.data, field).torch
        if joint_ids is None:
            tensor[ids] = value
        else:
            tensor[ids[:, None], torch.tensor(joint_ids)[None, :]] = value

    def write_joint_position_to_sim_index(self, *, position, env_ids, joint_ids=None):
        self._write("joint_pos", position, env_ids, joint_ids)

    def write_joint_velocity_to_sim_index(self, *, velocity, env_ids, joint_ids=None):
        self._write("joint_vel", velocity, env_ids, joint_ids)

    def set_joint_position_target_index(self, *, target, joint_ids=None, env_ids=None):
        ids = torch.arange(len(self.targets)) if env_ids is None else env_ids
        joints = list(range(self.targets.shape[1])) if joint_ids is None else joint_ids
        self.targets[ids[:, None], torch.tensor(joints)[None, :]] = target

    def write_root_pose_to_sim_index(self, *, root_pose, env_ids):
        self._write("root_pos_w", root_pose[:, :3], env_ids)
        self._write("root_quat_w", root_pose[:, 3:], env_ids)

    def write_root_velocity_to_sim_index(self, *, root_velocity, env_ids):
        self._write("root_lin_vel_w", root_velocity[:, :3], env_ids)
        self._write("root_ang_vel_w", root_velocity[:, 3:], env_ids)

    def update(self, dt):
        pass


@pytest.fixture
def adapter_class(monkeypatch):
    class Base:
        def _reset_idx(self, ids):
            self.episode_length_buf[ids] = 0

    for name in ("isaaclab", "isaaclab.sim", "isaaclab.assets", "isaaclab.envs", "isaaclab.sensors"):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    sys.modules["isaaclab.assets"].Articulation = Body
    sys.modules["isaaclab.assets"].RigidObject = Body
    sys.modules["isaaclab.envs"].DirectRLEnv = Base
    sys.modules["isaaclab.sensors"].ContactSensor = object
    sys.modules["isaaclab.sensors"].ContactSensorCfg = object
    file = Path(__file__).parents[1] / "src/rebar_training/isaac_env.py"
    spec = importlib.util.spec_from_file_location("rebar_training._adapter_test", file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RebarEnv


def environment(cls, task):
    n = 3
    env = object.__new__(cls)
    env.cfg = SimpleNamespace(task=task)
    env.task_spec = TaskSpec(task=task)
    # gym.make assigns its registration metadata here after __init__.
    env.spec = SimpleNamespace(id="Aubo-Rebar-Direct-v0")
    env.device, env.num_envs = "cpu", n
    env.logic = EpisodeLogic(env.task_spec, n)
    env.route = LoadingRoute(n, "cpu") if task == "loading" else None
    env.sampler = ResetSampler(42)
    origins = torch.tensor([[0, 0, 0], [5, 0, 0], [10, 0, 0]], dtype=torch.float32)
    env.scene = SimpleNamespace(env_origins=origins, write_data_to_sim=lambda: None)
    env.sim = SimpleNamespace(forward=lambda: None)
    env.robot, env.bar, env.station, env.fixture = Body(n, 8), Body(n), Body(n), Body(n, 4)
    env.robot.data.body_pos_w.torch[:, 0] = origins + torch.tensor([-0.96, -0.08, 1.5])
    env.control_ids, env.jaw_ids, env.wrist_id = list(range(8)), list(range(4)), 0
    env.carriage_ids, env.carriage_targets = [], torch.zeros(n, 0)
    env.lower, env.upper = torch.full((n, 8), -6.0), torch.full((n, 8), 6.0)
    env.lower[:, 6:], env.upper[:, 6:] = 0, 0.04
    env.targets, env.actions = torch.zeros(n, 8), torch.zeros(n, 7)
    env.close_fixture, env.jaw_targets = torch.zeros(n, dtype=torch.bool), torch.zeros(n, 4)
    env._last_result, env.completed_episodes, env.extras = None, [], {}
    env.episode_length_buf = torch.zeros(n, dtype=torch.long)
    env.contact_sensors = {}
    names = ["finger_0", "finger_1", "arm_all", "bar_all"]
    if task != "grasp":
        names += ["upper_left", "upper_right", "lower_left", "lower_right"]
    for name in names:
        env.contact_sensors[name] = SimpleNamespace(data=SimpleNamespace(
            force_matrix_w=proxy(torch.ones(n, 1, 1, 3)),
            net_forces_w_history=proxy(torch.zeros(n, 4, 1, 3))))
    return env


def test_visual_packets_use_raw_tactile_load_not_identity_filtered_contacts(adapter_class,monkeypatch):
    import numpy as np
    from rebar_training.vision.blind import blinded_control
    module=visual_adapter(adapter_class,monkeypatch,return_module=True)
    env=environment(module.VisualRebarEnv,'loading')
    for sensor in env.contact_sensors.values():
        sensor.data.net_forces_w=proxy(torch.tensor([[[3.,4.,0.]]]*3))
    env.scene.rigid_objects={'rebar':env.bar,'station':env.station}
    control=object.__new__(module.VisualControl);control.env=env
    with blinded_control(env):
        packets=control.robot_packets(100)
        with pytest.raises(RuntimeError,match='Privileged truth access'):
            env._filtered_force('finger_0')
    np.testing.assert_array_equal(packets[0].finger_forces,[5.,5.])
    np.testing.assert_array_equal(packets[0].jaw_forces,[5.,5.,5.,5.])


def test_camera_outage_latches_loaded_joint_target_instead_of_following_a_falling_arm(adapter_class,monkeypatch):
    module=visual_adapter(adapter_class,monkeypatch,return_module=True)
    env=environment(module.VisualRebarEnv,'loading')
    env.cfg.robot=SimpleNamespace(actuators={'arm':SimpleNamespace(stiffness=3000.,damping=150.)})
    env.targets[:,:6]=.02;env.robot.data.joint_vel.torch[:,:6]=.1
    control=object.__new__(module.VisualControl);control.env=env
    control.was_paused=torch.zeros(3,dtype=torch.bool);control.paused_steps=torch.zeros(3,dtype=torch.long)
    action=torch.ones(3,7);paused=torch.tensor([True,False,False])
    control.hold_on_perception_loss(action,paused)
    first=env.targets[0,:6].clone()
    torch.testing.assert_close(first,torch.full((6,),.015))
    env.robot.data.joint_pos.torch[0,:6]=-.2
    control.hold_on_perception_loss(action,paused)
    torch.testing.assert_close(env.targets[0,:6],first)
    assert (action[0,:6]==0).all() and (action[1,:6]==1).all()


def test_capture_observes_terminal_state_before_automatic_reset(adapter_class):
    env = environment(adapter_class, 'grasp')
    env._reset_idx(torch.arange(3))
    env.bar.data.root_pos_w.torch[:, 2] = .7
    env.logic.held[:] = env.task_spec.grasp_hold_steps - 1
    # Put the measured TCP on the lifted bar so contact geometry is satisfied.
    tcp, _ = env._wrist_tcp()
    env.bar.data.root_pos_w.torch[:] = tcp
    env.bar.data.root_pos_w.torch[:, 2] += .2
    env.robot.data.body_pos_w.torch[:, 0, 2] += .2
    captured = []
    env.capture_recorder = SimpleNamespace(record=lambda state, result: captured.append(
        (state.bar.clone(), result.success.clone())))
    env._get_dones()
    terminal_bar, success = captured[0]
    assert success.all()
    env._reset_idx(torch.arange(3))
    assert (terminal_bar[:, 2] > env.bar.data.root_pos_w.torch[:, 2]).all()


@pytest.mark.parametrize("task", ["grasp", "insert", "loading"])
def test_adapter_partial_reset_clears_velocities_targets_and_logic(adapter_class, task):
    env = environment(adapter_class, task)
    env._reset_idx(torch.arange(3))
    env.logic.elapsed[:] = 50
    env.logic.held[:] = 4
    env.logic.phase[:] = 1
    env.actions[:] = 1
    env.jaw_targets[:] = 0.07
    env.close_fixture[:] = True
    env.robot.data.joint_vel.torch[:] = 2
    env.bar.data.root_lin_vel_w.torch[:] = 2
    other_bar = env.bar.data.root_pos_w.torch[[0, 2]].clone()
    other_q = env.robot.data.joint_pos.torch[[0, 2]].clone()
    env._reset_idx(torch.tensor([1]))
    assert env.logic.elapsed.tolist() == [50, 0, 50]
    assert env.logic.phase.tolist() == [1, 0, 1]
    assert (env.robot.data.joint_vel.torch[1] == 0).all()
    assert (env.bar.data.root_lin_vel_w.torch[1] == 0).all()
    assert (env.actions[1] == 0).all() and not env.close_fixture[1]
    if task == "insert":
        assert (env.jaw_targets[1] == 0).all()
        torch.testing.assert_close(env.robot.targets[1, 6:], torch.full((2,), env.task_spec.finger_travel))
    torch.testing.assert_close(env.bar.data.root_pos_w.torch[[0, 2]], other_bar)
    torch.testing.assert_close(env.robot.data.joint_pos.torch[[0, 2]], other_q)
    obs = env._get_observations()["policy"]
    assert obs.shape == (3, OBSERVATION_SIZE) and torch.isfinite(obs).all()


def test_reward_call_does_not_advance_logic_twice_and_failure_is_recorded(adapter_class):
    env = environment(adapter_class, "grasp")
    env._reset_idx(torch.arange(3))
    env.bar.data.root_pos_w.torch[1, 2] = 0.2
    terminated, truncated = env._get_dones()
    assert terminated.tolist() == [False, True, False]
    assert not truncated.any()
    before = env.logic.elapsed.clone()
    env._get_rewards()
    torch.testing.assert_close(env.logic.elapsed, before)
    env._reset_idx(torch.tensor([1]))
    assert env.completed_episodes[0]["failure"]
    assert env.completed_episodes[0]["dropped"]
    assert env.logic.active.all()


def test_full_machine_partial_reset_restores_both_carriages_without_touching_other_envs(adapter_class):
    env = environment(adapter_class, "loading")
    env.fixture = Body(3, 6)
    env.carriage_ids = [4, 5]
    env.carriage_targets = torch.full((3, 2), .01)
    env.fixture.data.joint_pos.torch[:] = .01
    env.fixture.data.joint_vel.torch[:] = .02
    env.fixture.targets[:] = .03
    env._reset_idx(torch.tensor([1]))
    assert (env.fixture.data.joint_pos.torch[1] == 0).all()
    assert (env.fixture.data.joint_vel.torch[1] == 0).all()
    assert (env.fixture.targets[1] == 0).all()
    assert (env.carriage_targets[1] == 0).all()
    torch.testing.assert_close(env.fixture.data.joint_pos.torch[[0, 2]], torch.full((2, 6), .01))
    torch.testing.assert_close(env.fixture.targets[[0, 2]], torch.full((2, 6), .03))


def visual_adapter(adapter_class, monkeypatch, *, return_module=False):
    pytest.importorskip('cv2');pytest.importorskip('scipy')
    monkeypatch.setitem(sys.modules,'rebar_training.isaac_env',SimpleNamespace(RebarEnv=adapter_class))
    path=Path(__file__).parents[1]/'src/rebar_training/visual_env.py'
    spec=importlib.util.spec_from_file_location('rebar_training._visual_adapter_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module if return_module else module.VisualRebarEnv


def test_visual_observation_is_invariant_to_poisoned_bar_station_and_reward_phase(adapter_class,monkeypatch):
    import copy
    cls=visual_adapter(adapter_class,monkeypatch);env=environment(cls,'loading')
    env.scene.rigid_objects={'rebar':env.bar,'station':env.station}
    state=env._state();control_logic=copy.deepcopy(env.logic)
    env.visual=SimpleNamespace(measured_state=lambda:state,logic=control_logic)
    before=env._get_observations()['policy'].clone()
    env.bar.data.root_pos_w.torch[:]=12345
    env.station.data.root_pos_w.torch[:]=-999
    env.logic.phase[:]=2;env.logic.pick_complete[:]=True;env.logic.verified[:]=500
    def forbidden_state():
        raise AssertionError('Visual actor tried to read steel truth')
    env._state=forbidden_state
    torch.testing.assert_close(env._get_observations()['policy'],before)


def test_mobile_residual_input_cannot_read_steel_truth_or_reward_and_tracks_robot_feedback(adapter_class,monkeypatch):
    import numpy as np
    from rebar_training.vision.blind import blinded_control
    from rebar_training.vision.observer import RobotPacket
    visual=visual_adapter(adapter_class,monkeypatch,return_module=True)
    monkeypatch.setitem(sys.modules,'rebar_training.visual_env',visual)
    path=Path(__file__).parents[1]/'src/rebar_training/mobile_env.py'
    spec=importlib.util.spec_from_file_location('rebar_training._mobile_adapter_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    env=environment(visual.VisualRebarEnv,'loading')
    env.scene.rigid_objects={'rebar':env.bar,'station':env.station}
    visual_state=env._state()
    packet=RobotPacket(np.array([-.8,0.,.7]),np.array([0.,0.,0.,1.]),
                       np.zeros(2),np.zeros(2),np.zeros(4),1_000_000_000)
    control=object.__new__(module.MobileControl);control.env=env
    from rebar_training.vision.recovery import TrayGraspRecovery
    control.grasp_recovery=TrayGraspRecovery()
    control.tick=30;control.stage='pickup';control.path={};control.tray_force=10.
    control.valid=torch.tensor([True]);control.motion=SimpleNamespace(ready=torch.tensor([True]))
    control.observers=[SimpleNamespace(acquired=True,last_good_ns=1_000_000_000)]
    control.measured_state=lambda:visual_state
    control.robot_packets=lambda timestamp:[packet]
    def forbidden():raise AssertionError('Residual policy read GT evaluation state')
    env._state=forbidden
    with blinded_control(env):before,active=control.learning_observation()
    assert not active  # Pregrasp orientation/standoff is executed by the safety servo.
    env.route.stage[:]=1
    with blinded_control(env):before,active=control.learning_observation()
    assert active
    env.bar.data.root_pos_w.torch[:]=12345;env.station.data.root_pos_w.torch[:]=-999
    env.logic.phase[:]=2;env.logic.verified[:]=999
    with blinded_control(env):after,_=control.learning_observation()
    np.testing.assert_array_equal(before,after)
    env.robot.data.joint_pos.torch[0,0]+=.1
    with blinded_control(env):changed,_=control.learning_observation()
    assert before[0]!=changed[0]
    control.grasp_recovery.pending=True
    with blinded_control(env):_,active=control.learning_observation()
    assert not active
