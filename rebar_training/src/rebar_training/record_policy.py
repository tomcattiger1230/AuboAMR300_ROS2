"""Record one real simulated rollout of the exported, frozen RL policy."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

from .launcher import verify_checkout
from .preflight import preflight_subprocess
from .spec import asset_directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--isaaclab-root', type=Path, default=os.environ.get('ISAACLAB_ROOT'))
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--export-report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=30042)
    parser.add_argument('--eye', type=float, nargs=3, default=(2.8, 2.7, 2.6))
    parser.add_argument('--lookat', type=float, nargs=3, default=(-.55, -.30, 1.05))
    early, _ = parser.parse_known_args()
    if early.isaaclab_root is None or '--enable_cameras' not in sys.argv:
        parser.error('Set ISAACLAB_ROOT and --enable_cameras for real RTX recording')
    if early.output.exists():
        parser.error('Output already exists; preserve existing recordings')
    provenance = json.loads(early.export_report.read_text())
    digest = hashlib.sha256(early.policy.read_bytes()).hexdigest()
    if digest != provenance['files']['policy_jit.pt'] or not provenance['jit_native_parity']:
        raise RuntimeError('Exported policy provenance does not match the validated checkpoint')
    verify_checkout(early.isaaclab_root)
    preflight_subprocess(asset_directory(), runtime=True)

    import gymnasium as gym
    import imageio.v2 as imageio
    import numpy as np
    import torch
    from PIL import Image, ImageDraw, ImageFont
    from isaaclab_tasks.utils import add_launcher_args, launch_simulation, setup_preset_cli
    from isaaclab_tasks.utils.hydra import hydra_task_config
    from .registration import register_tasks

    register_tasks()
    parser.add_argument('--task', default='Aubo-Rebar-Loading-Direct-v0')
    add_launcher_args(parser)
    args, remaining = setup_preset_cli(parser)
    sys.argv = [sys.argv[0], *remaining]
    labels = {0: 'Approach rebar', 1: 'Lower gripper', 2: 'Grasp rebar', 3: 'Lift rebar',
              4: 'Carry and orient', 6: 'Approach testing machine', 7: 'Insert and align',
              8: 'Machine clamps rebar', 9: 'Release gripper', 10: 'Retract robot'}

    @hydra_task_config('Aubo-Rebar-Loading-Direct-v0', 'rsl_rl_cfg_entry_point')
    def run(cfg, _agent_cfg):
        cfg.scene.num_envs = 1
        cfg.seed = args.seed
        cfg.viewer.eye, cfg.viewer.lookat = tuple(args.eye), tuple(args.lookat)
        cfg.video_recorder.backend_source = 'renderer'
        cfg.video_recorder.window_width, cfg.video_recorder.window_height = 1280, 720
        args.output.parent.mkdir(parents=True, exist_ok=True)
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 23)
        frames, stages, episode = 0, [], None
        writer = None
        with launch_simulation(cfg, args):
            env = gym.make('Aubo-Rebar-Loading-Direct-v0', cfg=cfg, render_mode='rgb_array')
            raw = env.unwrapped
            try:
                obs, _ = env.reset(seed=args.seed)
                policy = torch.jit.load(str(args.policy), map_location=raw.device).eval()
                for _ in range(16):
                    rgb = raw.render()
                if rgb is None or np.asarray(rgb).std() < 2:
                    raise RuntimeError('Camera rendered blank frames')
                Image.fromarray(rgb).save(args.output.with_suffix('.preview.png'))
                writer = imageio.get_writer(str(args.output), fps=30, codec='libx264', quality=8,
                                             macro_block_size=None, output_params=['-movflags', '+faststart'])

                def append(frame, phase, elapsed, success=False):
                    nonlocal frames
                    image = Image.fromarray(frame).convert('RGB')
                    draw = ImageDraw.Draw(image)
                    draw.rectangle((0, 0, 1280, 82), fill=(15, 23, 38))
                    draw.text((24, 9), 'REBAR LOADING | Trained RL policy | PPO 1500 | 1x speed', font=font, fill='white')
                    draw.text((24, 44), f'{elapsed:05.2f}s  |  {phase}', font=font,
                              fill=(100, 240, 160) if success else (120, 200, 255))
                    writer.append_data(np.asarray(image))
                    frames += 1

                append(rgb, labels[0], 0.)
                with torch.inference_mode():
                    for step in range(raw.task_spec.episode_steps):
                        stage = int(raw.route.stage[0])
                        if not stages or stages[-1]['stage'] != stage:
                            stages.append({'step': step, 'stage': stage, 'label': labels.get(stage, str(stage))})
                            print('STAGE', step, stage, labels.get(stage), flush=True)
                        actions = policy(obs['policy'])
                        if not torch.isfinite(actions).all():
                            raise RuntimeError('Non-finite learned policy output')
                        obs, _, _, _, _ = env.step(actions)
                        if raw.completed_episodes:
                            episode = raw.completed_episodes[0]
                            break  # Do not show the automatic reset as part of handoff.
                        rgb = raw.render()
                        append(rgb, labels.get(stage, str(stage)), (step + 1) * raw.task_spec.control_dt)
                        if stage not in {s.get('saved') for s in stages}:
                            Image.fromarray(rgb).save(args.output.with_name(args.output.stem + f'.stage_{stage}.png'))
                            stages[-1]['saved'] = stage
                if episode is None:
                    raise RuntimeError('No completed rollout was recorded')
                success = bool(episode['success'] and episode['picked'] and not episode['failure'] and not episode['timeout'])
                for _ in range(60):
                    append(rgb, 'SUCCESS: rebar held by testing machine' if success else 'ROLLOUT DID NOT SUCCEED',
                           episode['steps'] * raw.task_spec.control_dt, success)
                writer.close()
                writer = None
                report = {'mode': 'frozen_rl_policy_video', 'seed': args.seed,
                          'source_checkpoint_sha256': provenance['source_sha256'], 'jit_policy_sha256': digest,
                          'video': str(args.output), 'fps': 30, 'frames': frames, 'playback_speed': 1,
                          'simulated_seconds': episode['steps'] * raw.task_spec.control_dt,
                          'episode': episode, 'stages': stages, 'success': success,
                          'expert_actions': False, 'training_updates': False}
                args.output.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
                print(json.dumps(report, indent=2), flush=True)
            finally:
                if writer is not None:
                    writer.close()
                env.close()
    run()


if __name__ == '__main__':
    main()
