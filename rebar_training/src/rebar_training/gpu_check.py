"""Bounded GPU smoke / checkpoint evaluation; never claims offline physics."""

import argparse
import hashlib
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
import subprocess
import uuid

from .launcher import verify_checkout
from .preflight import preflight_subprocess as preflight
from .spec import asset_directory


def make_parser(gym_names=False):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaaclab-root", type=Path, default=os.environ.get("ISAACLAB_ROOT"))
    choices = ("Aubo-Rebar-Grasp-Direct-v0", "Aubo-Rebar-Insert-Direct-v0", "Aubo-Rebar-Loading-Direct-v0") if gym_names else ("grasp", "insert", "loading")
    parser.add_argument("--task", choices=choices, required=True)
    parser.add_argument("--num_envs", type=int, default=1)
    parser.add_argument("--steps", type=int, default=400)
    parser.add_argument("--seed", type=int, default=10042)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--expert", action="store_true", help="Physical feasibility controller; not a learned policy")
    parser.add_argument("--demonstrations", type=Path, help="Save state/action demonstrations from --expert")
    parser.add_argument("--trace-stride", type=int, default=10, help="Expert trace interval, 0 disables tracing")
    parser.add_argument("--diagnostics", action="store_true", help="Record policy contact and motion traces")
    parser.add_argument("--contact-pairs", action="store_true", help="CPU PhysX contact-pair diagnostics")
    parser.add_argument("--dagger", action="store_true", help="Online student training with expert labels")
    parser.add_argument("--dagger-beta-end", type=float, default=0., help="Expert action probability during the final quarter")
    parser.add_argument("--replay-file", type=Path)
    parser.add_argument("--output-checkpoint", type=Path)
    parser.add_argument("--camera-config", type=Path, help="Opt-in capture configuration; requires --enable_cameras")
    parser.add_argument("--camera-dataset", type=Path, help="New camera dataset directory")
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--report", type=Path, required=True)
    parser.epilog = "Simulator flags such as --headless and --device are forwarded to the pinned Isaac Lab launcher."
    return parser


def main():
    # Help and argument validation also work on a CPU-only/offline machine.
    early = make_parser()
    early_args, _ = early.parse_known_args()
    if bool(early_args.camera_config) != bool(early_args.camera_dataset):
        early.error("--camera-config and --camera-dataset must be supplied together")
    if early_args.camera_config and "--enable_cameras" not in sys.argv:
        early.error("camera capture requires --enable_cameras")
    if early_args.camera_config and early_args.dagger:
        early.error("Capture is for expert/evaluation rollouts, not online DAgger updates")
    if not 0 <= early_args.dagger_beta_end <= .9:
        early.error("--dagger-beta-end must be in [0,.9]")
    if early_args.expert and early_args.checkpoint:
        early.error("choose either --expert or --checkpoint")
    if early_args.demonstrations and not early_args.expert:
        early.error("--demonstrations requires --expert")
    if early_args.dagger and (not early_args.checkpoint or not early_args.replay_file or not early_args.output_checkpoint):
        early.error("--dagger requires --checkpoint, --replay-file and --output-checkpoint")
    if early_args.isaaclab_root is None:
        early.error("set ISAACLAB_ROOT or --isaaclab-root")
    os.environ["ISAACLAB_ROOT"] = str(early_args.isaaclab_root.resolve())
    verify_checkout(early_args.isaaclab_root)
    preflight(asset_directory(), runtime=True)

    import gymnasium as gym
    import torch
    from isaaclab_tasks.utils import add_launcher_args, launch_simulation, setup_preset_cli
    from isaaclab_tasks.utils.hydra import hydra_task_config
    from .registration import register_tasks

    register_tasks()
    parser = make_parser(gym_names=True)
    # setup_preset_cli looks up the gym task name even before Hydra runs.
    argv = sys.argv[1:]
    if any("physics=" in a for a in argv):
        parser.error("only the default Isaac Sim PhysX backend is supported")
    short_task = None
    for i, token in enumerate(argv):
        if token == "--task" and i + 1 < len(argv):
            short_task = argv[i + 1]
            argv[i + 1] = f"Aubo-Rebar-{short_task.title()}-Direct-v0"
            break
        if token.startswith("--task="):
            short_task = token.split("=", 1)[1]
            argv[i] = f"--task=Aubo-Rebar-{short_task.title()}-Direct-v0"
            break
    if short_task not in ("grasp", "insert", "loading"):
        parser.error("--task must be grasp, insert or loading")
    full_task = f"Aubo-Rebar-{short_task.title()}-Direct-v0"
    sys.argv = [sys.argv[0], *argv]
    # AppLauncher also inspects the existing parser while registering flags.
    # Its first parse must already see Gym IDs instead of our short names.
    add_launcher_args(parser)
    args, remaining = setup_preset_cli(parser)
    if args.num_envs < 1 or args.steps < 1 or args.episodes < 1:
        parser.error("num_envs, steps and episodes must be positive")
    sys.argv = [sys.argv[0], *remaining]

    completed = False

    @hydra_task_config(full_task, "rsl_rl_cfg_entry_point")
    def run(cfg, agent_cfg):
        nonlocal completed
        cfg.scene.num_envs = args.num_envs
        if args.camera_config:
            cfg.camera_capture_config = str(args.camera_config.resolve())
            cfg.camera_dataset_root = str(args.camera_dataset.resolve())
        cfg.seed = args.seed
        cfg.record_diagnostics = args.expert or args.diagnostics
        cfg.record_contact_pairs = args.contact_pairs
        if args.device:
            cfg.sim.device = args.device
        with launch_simulation(cfg, args):
            env = gym.make(full_task, cfg=cfg)
            raw = env.unwrapped
            records = []
            try:
                if args.checkpoint:
                    from importlib.metadata import version
                    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg
                    from rsl_rl.runners import OnPolicyRunner
                    # Match the upstream train/play compatibility conversion.
                    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, version("rsl-rl-lib"))
                    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
                    agent_cfg.device = raw.device
                    metadata = torch.load(args.checkpoint, weights_only=True, map_location="cpu")
                    from .route_actor import configure_actor
                    configure_actor(agent_cfg, metadata)
                    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=raw.device)
                    if metadata.get("rebar_observation_contract", "state_v1") != raw.observation_contract:
                        raise RuntimeError("Checkpoint observation contract does not match this task; retrain loading route policy")
                    runner.load(str(args.checkpoint.resolve()))
                    policy = runner.get_inference_policy(device=raw.device)
                    obs = env.get_observations()
                    limit = raw.task_spec.episode_steps * (args.episodes + 1)
                else:
                    env.reset(seed=args.seed)
                    limit = args.steps
                expert = None
                trace = []
                demo_observations, demo_actions = [], []
                if args.expert:
                    from .expert import CartesianExpert
                    expert = CartesianExpert(raw)
                    expert.reset(torch.arange(raw.num_envs, device=raw.device))
                    limit = raw.task_spec.episode_steps * (args.episodes + 1)
                dagger_teacher, replay = None, None
                if args.dagger:
                    from .expert import CartesianExpert
                    from .imitation import OnlineImitation
                    dagger_teacher = CartesianExpert(raw)
                    dagger_teacher.reset(torch.arange(raw.num_envs, device=raw.device))
                    replay = OnlineImitation(policy, args.replay_file, raw.device, observation_contract=raw.observation_contract)
                    limit = args.steps
                finite = True
                steps_run = 0
                grasped_steps = torch.zeros(raw.num_envs, device=raw.device)
                initial_base = raw.robot.data.root_pos_w.torch.clone()
                local_base = initial_base - raw.scene.env_origins
                base_origin_error = float(torch.linalg.vector_norm(local_base - local_base[:1], dim=-1).max())
                fixture_origin_error = 0.0
                if short_task != "grasp":
                    local_fixture = raw.fixture.data.root_pos_w.torch - raw.scene.env_origins
                    fixture_origin_error = float(torch.linalg.vector_norm(local_fixture - local_fixture[:1], dim=-1).max())
                max_base_drift = 0.0
                max_carriage_offset = 0.0
                with torch.inference_mode(mode=not args.dagger):
                    for _ in range(limit):
                        steps_run += 1
                        if args.checkpoint:
                            with torch.no_grad():
                                actions = policy(obs)
                                if dagger_teacher:
                                    labels = dagger_teacher()
                                    replay.add(obs["policy"], labels)
                                    beta = max(args.dagger_beta_end, .9 * (1 - steps_run / (.75 * limit)))
                                    intervention = torch.rand(raw.num_envs, 1, device=raw.device) < beta
                                    actions = torch.where(intervention, labels, actions)
                            finite &= bool(torch.isfinite(actions).all() & torch.isfinite(obs["policy"]).all())
                            obs, rewards, dones, _ = env.step(actions)
                            policy.reset(dones)
                            if dagger_teacher:
                                dagger_teacher.reset(torch.where(dones)[0])
                                if steps_run % 32 == 0:
                                    replay.update()
                            finite &= bool(torch.isfinite(rewards).all())
                        else:
                            actions = expert() if expert else torch.zeros(raw.num_envs, 7, device=raw.device)
                            if args.demonstrations:
                                demo_observations.append(raw._get_observations()["policy"].clone())
                                demo_actions.append(actions.clone())
                            if not expert:
                                actions[:, 6] = -1 if short_task == "grasp" else 1
                            observation, rewards, _, _, _ = env.step(actions)
                            finite &= bool(torch.isfinite(rewards).all() & torch.isfinite(observation["policy"]).all())
                        if expert or args.diagnostics:
                            if args.trace_stride and (steps_run % args.trace_stride == 0 or raw.completed_episodes):
                                trace.append({"step": steps_run, "stage": expert.stage.tolist() if expert else None,
                                              "actions": actions.tolist(),
                                              **{k: raw.diagnostic_state[k].tolist() for k in ("tcp", "bar", "bar_axis", "linear_velocity", "angular_velocity", "finger_forces", "jaw_forces", "unsafe_force", "fingers")},
                                              **{k: v.tolist() for k, v in raw.diagnostic_counters.items()},
                                              "contact_peaks": {k: v.tolist() for k, v in raw.diagnostic_peaks.items()}})
                            if expert and raw.completed_episodes:
                                expert.reset(torch.tensor([r["env_id"] for r in raw.completed_episodes], device=raw.device))
                        records.extend(raw.completed_episodes)
                        raw.completed_episodes.clear()
                        grasped_steps += raw._last_result.terms["grasped"]
                        max_base_drift = max(max_base_drift, float(torch.linalg.vector_norm(
                            raw.robot.data.root_pos_w.torch - initial_base, dim=-1).max()))
                        if raw.carriage_ids:
                            max_carriage_offset = max(max_carriage_offset,
                                float(raw.fixture.data.joint_pos.torch[:, raw.carriage_ids].abs().max()))
                        if steps_run % 100 == 0:
                            print({"step": steps_run, "episodes": len(records),
                                   "successes": sum(r["success"] for r in records),
                                   "expert_stages": expert.stage.tolist() if expert else None,
                                   "route_stages": raw.route.stage.tolist() if raw.route is not None else None,
                                   "dagger_loss": replay.loss if replay else None}, flush=True)
                        if not args.dagger and (args.checkpoint or expert) and len(records) >= args.episodes:
                            records = records[:args.episodes]
                            break
                successes = sum(r["success"] for r in records)
                if args.dagger:
                    checkpoint = runner.alg.save()
                    checkpoint.update(iter=0, rebar_observation_contract=raw.observation_contract, infos={"source": "online_dagger", "steps": steps_run})
                    args.output_checkpoint.parent.mkdir(parents=True, exist_ok=True)
                    torch.save(checkpoint, args.output_checkpoint)
                    torch.save({"observations": replay.obs.cpu(), "actions": replay.actions.cpu(),
                                "reference_observations": replay.reference_obs.cpu(),
                                "reference_actions": replay.reference_actions.cpu(),
                                "task": short_task, "observation_contract": raw.observation_contract, "source": "DAgger expert labels on mixed-control states"},
                               args.output_checkpoint.with_suffix('.replay.pt'))
                if args.demonstrations:
                    args.demonstrations.parent.mkdir(parents=True, exist_ok=True)
                    torch.save({"observations": torch.cat(demo_observations).cpu(),
                                "actions": torch.cat(demo_actions).cpu(),
                                "task": short_task, "observation_contract": raw.observation_contract, "seed": args.seed, "source": "CartesianExpert",
                                "fixture_model": raw.cfg.fixture_model, "task_spec": asdict(raw.task_spec),
                                "asset_manifest_sha256": hashlib.sha256((asset_directory() / "manifest.json").read_bytes()).hexdigest(),
                                "success_rate": successes / len(records) if records else 0.0}, args.demonstrations)
                result = {"run_id": os.environ.get("REBAR_GPU_CHECK_WORKER"), "completed": False,
                          "mode": "online_dagger" if args.dagger else ("expert_physical_check" if expert else ("checkpoint_evaluation" if args.checkpoint else "gpu_smoke")),
                          "task": short_task, "observation_contract": raw.observation_contract, "seed": args.seed, "num_envs": args.num_envs,
                          "finite": finite, "steps_run": steps_run, "episodes": records, "episode_count": len(records),
                          "grasped_fraction_per_env": (grasped_steps / steps_run).tolist(),
                          "max_base_drift_m": max_base_drift,
                          "fixture_model": raw.cfg.fixture_model,
                          "asset_manifest_sha256": hashlib.sha256((asset_directory() / "manifest.json").read_bytes()).hexdigest(),
                          "max_carriage_offset_m": max_carriage_offset,
                          "carriage_hold_passed": max_carriage_offset < .002 if raw.carriage_ids else None,
                          "base_clone_origin_error_m": base_origin_error,
                          "fixture_clone_origin_error_m": fixture_origin_error,
                          "automatic_reset_observed": bool(records),
                          "success_rate": successes / len(records) if not args.dagger and (args.checkpoint or expert) and records else None,
                          "mixed_control_success_rate": successes / len(records) if args.dagger and records else None,
                          "trace": trace,
                          "contact_pairs": sorted(raw.contact_pairs),
                          "task_spec": asdict(raw.task_spec),
                          "checkpoint": str(args.checkpoint.resolve()) if args.checkpoint else None,
                          "smoke_passed": (finite and max_base_drift < 0.005 and base_origin_error < 0.005
                                           and fixture_origin_error < 0.005
                                           and (short_task != "insert" or bool((grasped_steps / steps_run >= 0.9).all()))
                                           and not any(r["failure"] for r in records)
                                           and (limit < raw.task_spec.episode_steps or bool(records))) if not (args.checkpoint or expert) else None}
                args.report.parent.mkdir(parents=True, exist_ok=True)
                args.report.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
                if not finite:
                    raise RuntimeError("non-finite observations/rewards during GPU check")
                if raw.carriage_ids and not result["carriage_hold_passed"]:
                    raise RuntimeError("full machine carriage moved more than 2 mm from its commanded operating height")
                if not (args.checkpoint or expert) and not result["smoke_passed"]:
                    raise RuntimeError("reset/start pose failed during GPU smoke; inspect the report before training")
                if args.checkpoint and not args.dagger and len(records) < args.episodes:
                    raise RuntimeError("evaluation ended before the requested episode count")
                result["completed"] = True
                args.report.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
            finally:
                env.close()
            completed = True

    run()
    # Also cover launch implementations which return without running the body.
    if not completed:
        raise RuntimeError("GPU check did not complete; inspect the simulator log")


def supervised_main():
    """Check a fresh completion receipt even if Kit fast-shutdown exits with 0."""
    args, _ = make_parser().parse_known_args()
    run_id = uuid.uuid4().hex
    environment = {**os.environ, "REBAR_GPU_CHECK_WORKER": run_id}
    process = subprocess.run([sys.executable, "-m", "rebar_training.gpu_check", *sys.argv[1:]], env=environment)
    try:
        result = json.loads(args.report.read_text())
    except (OSError, ValueError):
        result = {}
    if process.returncode or result.get("run_id") != run_id or not result.get("completed"):
        raise RuntimeError(f"GPU check failed or did not complete (process status {process.returncode}); inspect the simulator log and {args.report}")


if __name__ == "__main__":
    main() if os.environ.get("REBAR_GPU_CHECK_WORKER") else supervised_main()
