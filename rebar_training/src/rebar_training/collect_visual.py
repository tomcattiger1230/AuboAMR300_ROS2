"""Render observation-to-handoff teacher data. Ground truth is LABELS/TEACHER only.

This executable is not a vision-controlled policy evaluation. A separate RGB
detector and blinded controller must pass their own held-out closed-loop tests.
"""
import argparse
import json
import os
from pathlib import Path
import sys

from .launcher import verify_checkout
from .preflight import preflight_subprocess
from .spec import asset_directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', default='Aubo-Rebar-Loading-Direct-v0')
    parser.add_argument('--isaaclab-root', type=Path, default=os.environ.get('ISAACLAB_ROOT'))
    parser.add_argument('--num_envs', type=int, default=1)
    parser.add_argument('--episodes', type=int, default=2)
    parser.add_argument('--seed', type=int, default=101042)
    parser.add_argument('--observation-only', action='store_true')
    parser.add_argument('--camera-config', type=Path, required=True)
    parser.add_argument('--camera-dataset', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    early, _ = parser.parse_known_args()
    if early.isaaclab_root is None or '--enable_cameras' not in sys.argv:
        parser.error('ISAACLAB_ROOT and --enable_cameras are required')
    if early.report.exists() or early.camera_dataset.exists():
        parser.error('Use a new dataset and report path')
    verify_checkout(early.isaaclab_root)
    preflight_subprocess(asset_directory(), runtime=True)
    import gymnasium as gym
    import torch
    from isaaclab_tasks.utils import add_launcher_args, launch_simulation, setup_preset_cli
    from isaaclab_tasks.utils.hydra import hydra_task_config
    from .registration import register_tasks
    register_tasks()
    add_launcher_args(parser)
    args, remaining = setup_preset_cli(parser)
    sys.argv = [sys.argv[0], *remaining]

    @hydra_task_config('Aubo-Rebar-Loading-Direct-v0', 'rsl_rl_cfg_entry_point')
    def run(cfg, _agent_cfg):
        cfg.scene.num_envs = args.num_envs
        cfg.seed = args.seed
        cfg.episode_length_s = 120.
        cfg.camera_capture_config = str(args.camera_config.resolve())
        cfg.camera_dataset_root = str(args.camera_dataset.resolve())
        records, observations = [], []
        with launch_simulation(cfg, args):
            env = gym.make('Aubo-Rebar-Loading-Direct-v0', cfg=cfg)
            e = env.unwrapped
            try:
                env.reset(seed=args.seed)
                from .expert import CartesianExpert
                from .vision.observation import ObserveThenReturn
                from .vision.capture import array
                expert = CartesianExpert(e)
                expert.reset(torch.arange(e.num_envs, device=e.device))
                lifecycle = ObserveThenReturn(e)
                motion = lifecycle.motion
                observed = torch.zeros(e.num_envs, device=e.device, dtype=torch.bool)
                with torch.inference_mode():
                    for step in range(e.task_spec.episode_steps * (args.episodes + 1)):
                        observation_actions = lifecycle.actions(expert)
                        newly_ready = motion.ready & ~observed
                        if newly_ready.any():
                            from .vision.capture import synchronize_mounts
                            synchronize_mounts(e)
                            camera = e.vision_cameras['wrist'].data
                            for i in torch.where(newly_ready)[0].tolist():
                                item = dict(step=step, env_id=i, tcp=(e._wrist_tcp()[0]-e.scene.env_origins)[i].tolist(),
                                            camera_position=(array(camera.pos_w)[i]-array(e.scene.env_origins)[i]).tolist(),
                                            target_tcp=motion.target[i].tolist())
                                observations.append(item)
                                print('OBSERVATION_READY', item, flush=True)
                            observed |= newly_ready
                        actions = observation_actions if args.observation_only else torch.where(
                            lifecycle.ready[:, None], expert(), observation_actions)
                        env.step(actions)
                        records.extend(e.completed_episodes)
                        if e.completed_episodes:
                            ids = torch.tensor([r['env_id'] for r in e.completed_episodes], device=e.device)
                            lifecycle.reset(ids); expert.reset(ids); observed[ids] = False
                            e.completed_episodes.clear()
                        if step % 100 == 0:
                            print(dict(step=step, ready=motion.ready.tolist(), returned=lifecycle.ready.tolist(), route=e.route.stage.tolist(),
                                       tcp=(e._wrist_tcp()[0]-e.scene.env_origins).tolist()), flush=True)
                        if (args.observation_only and motion.ready.all()) or len(records) >= args.episodes:
                            break
                report = dict(completed=bool(observations), mode='ground_truth_teacher_data_collection',
                    observation_motion_uses_steel_truth=False, vision_control_evaluation=False,
                    seed=args.seed, observations=observations, episodes=records[:args.episodes],
                    success_rate=sum(r['success'] for r in records[:args.episodes])/args.episodes if records else None)
                args.report.parent.mkdir(parents=True, exist_ok=True)
                args.report.write_text(json.dumps(report, indent=2)+'\n')
            finally:
                env.close()
    run()


if __name__ == '__main__':
    main()
