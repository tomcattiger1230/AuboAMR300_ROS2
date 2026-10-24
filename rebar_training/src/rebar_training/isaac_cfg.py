"""Isaac Lab v3.0.0-beta2.patch1 / Isaac Sim 6.0.1 configuration.

Use only the Isaac Sim PhysX backend. This module deliberately has no NumPy / ROS
dependency and is imported only by the simulator entry points.
"""

import json

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, RigidObjectCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils.configclass import configclass
from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg
from isaaclab_physx.physics import PhysxCfg

from .spec import ARM_JOINTS, FINGER_JOINTS, GRASP_JOINTS, JAW_NAMES, ACTION_SIZE, OBSERVATION_SIZE, TaskSpec, asset_directory


def fixture_model():
    return json.loads((asset_directory() / "manifest.json").read_text()).get("fixture_model", "pads_v1")


def fixture_actuators():
    actuators = {"jaws": ImplicitActuatorCfg(joint_names_expr=[name + "_joint" for name in JAW_NAMES],
                 effort_limit_sim=100, velocity_limit_sim=.02, stiffness=5000, damping=200)}
    if fixture_model() == "full_v2":
        actuators["carriages"] = ImplicitActuatorCfg(joint_names_expr=[".*_carriage_joint"],
            effort_limit_sim=10000, velocity_limit_sim=.12, stiffness=2000000, damping=20000)
    return actuators


def insert_height():
    if fixture_model() == "full_v2":
        from .full_machine import CARRIAGE_HEIGHTS
        return sum(CARRIAGE_HEIGHTS.values()) / 2
    return 1.50


def usd(name, **kwargs):
    return sim_utils.UsdFileCfg(usd_path=str(asset_directory() / (name + ".usda")), **kwargs)


@configclass
class GraspEnvCfg(DirectRLEnvCfg):
    task = "grasp"
    reset_profile = 'baseline'
    fixture_model = fixture_model()
    insert_goal = (*TaskSpec().insert_goal[:2], insert_height())
    position_tolerance = .003 if fixture_model == "full_v2" else TaskSpec().position_tolerance
    record_diagnostics = False
    record_contact_pairs = False
    camera_capture_config: str | None = None
    camera_dataset_root: str | None = None
    decimation = 4
    episode_length_s = 12.0
    action_space = ACTION_SIZE
    observation_space = OBSERVATION_SIZE
    state_space = 0
    sim = sim_utils.SimulationCfg(dt=1 / 120, render_interval=decimation,
                                  physics=PhysxCfg(enable_external_forces_every_iteration=True))
    scene = InteractiveSceneCfg(num_envs=32, env_spacing=5.0, replicate_physics=False, clone_in_fabric=False)
    robot = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=usd("robot", activate_contact_sensors=True,
                  articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                      enabled_self_collisions=True, solver_position_iteration_count=32, solver_velocity_iteration_count=8)),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0, 0, 0), rot=(0, 0, 0, 1),
            joint_pos={**dict(zip(ARM_JOINTS, GRASP_JOINTS)), **dict.fromkeys(FINGER_JOINTS, 0), ".*wheel_joint": 0}),
        actuators={
            "arm": ImplicitActuatorCfg(joint_names_expr=list(ARM_JOINTS), effort_limit_sim=200,
                                       velocity_limit_sim=0.5, stiffness=3000, damping=150),
            "fingers": ImplicitActuatorCfg(joint_names_expr=list(FINGER_JOINTS), effort_limit_sim=100,
                                           velocity_limit_sim=0.04, stiffness=5000, damping=500),
            "wheels": ImplicitActuatorCfg(joint_names_expr=[".*wheel_joint"], effort_limit_sim=0,
                                          stiffness=0, damping=0),
        },
    )
    bar = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Rebar",
        spawn=usd("rebar", activate_contact_sensors=True,
                  rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, kinematic_enabled=False,
                                                               max_depenetration_velocity=1.0)),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-1.15, 0, 0.522), rot=(0, 0, 0, 1)),
    )
    station = RigidObjectCfg(
        prim_path="/World/envs/env_.*/Station", spawn=usd("station"),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(-1.15, 0, 0), rot=(0, 0, 0, 1)),
    )
    fixture = ArticulationCfg(
        prim_path="/World/envs/env_.*/Fixture",
        spawn=usd("fixture", activate_contact_sensors=True),
        init_state=ArticulationCfg.InitialStateCfg(rot=(0, 0, 0, 1), joint_pos={".*_joint": 0}),
        actuators=fixture_actuators(),
    )


@configclass
class InsertEnvCfg(GraspEnvCfg):
    task = "insert"


@configclass
class LoadingEnvCfg(GraspEnvCfg):
    task = "loading"
    episode_length_s = 60.0
    insert_goal = (-0.30, -0.77, insert_height())
    fixture = GraspEnvCfg().fixture.replace(prim_path="/World/envs/env_.*/Cell/Fixture")


@configclass
class VisualLoadingEnvCfg(LoadingEnvCfg):
    visual_control = True
    visual_detector: str = ''
    visual_blank_stage = -1
    visual_blank_frames = 0
    episode_length_s = 120.


@configclass
class MobileVisualEnvCfg(VisualLoadingEnvCfg):
    mobile_residual_control = False
    mobile_until = 'tray'
    mobile_start_on_tray = False
    mobile_start_at_dock = False
    mobile_visual_hint_packet: str | None = None
    mobile_joint_observation = False
    mobile_initial_joints = ()
    mobile_tray_camera_outage_frames = 0
    mobile_tray_grasp_camera_outage_frames = 0
    # Separate the real camera casing from wrist1 throughout reset perturbations.
    mobile_home_wrist_offset = -.8
    cell_translation = (5.92,4.87,0.)
    episode_length_s = 600.


@configclass
class RebarPPOAlgorithmCfg(RslRlPpoAlgorithmCfg):
    class_name = "rebar_training.anchored_ppo:AnchoredPPO"
    replay_file: str | None = None
    replay_updates: int = 8
    observation_contract: str = "state_v1"


@configclass
class RebarPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    seed = 42
    num_steps_per_env = 32
    max_iterations = 1500
    save_interval = 50
    experiment_name = "aubo_rebar"
    obs_groups = {"actor": ["policy"], "critic": ["policy"]}
    actor = RslRlMLPModelCfg(
        hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.3),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True)
    algorithm = RebarPPOAlgorithmCfg(
        value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.005,
        num_learning_epochs=5, num_mini_batches=4, learning_rate=3e-4, schedule="adaptive",
        gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=1.0,
    )


@configclass
class LoadingPPORunnerCfg(RebarPPORunnerCfg):
    algorithm = RebarPPORunnerCfg().algorithm.replace(observation_contract="loading_route_v1")


@configclass
class VisualPPORunnerCfg(LoadingPPORunnerCfg):
    # This contract is for vision-adapted actors. Generic PPO collection must
    # mask the external observation bootstrap before it can train this task.
    algorithm = LoadingPPORunnerCfg().algorithm.replace(observation_contract="loading_visual_v1")
