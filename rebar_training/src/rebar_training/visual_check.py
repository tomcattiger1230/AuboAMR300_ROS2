"""Bounded RGB-only servo/policy evaluation with privileged-state tripwires."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid

from .launcher import verify_checkout
from .preflight import preflight_subprocess
from .spec import asset_directory


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task', default='Aubo-Rebar-Visual-Loading-Direct-v0')
    p.add_argument('--isaaclab-root', type=Path, default=os.environ.get('ISAACLAB_ROOT'))
    p.add_argument('--num_envs', type=int, default=1)
    p.add_argument('--episodes', type=int, default=1)
    p.add_argument('--seed', type=int, default=105042)
    p.add_argument('--detector', type=Path, required=True)
    p.add_argument('--camera-config', type=Path, required=True)
    p.add_argument('--checkpoint', type=Path)
    p.add_argument('--state-warmstart', action='store_true')
    p.add_argument('--demonstrations', type=Path)
    p.add_argument('--label-replay', type=Path, help='Label student-visited states with visual servo, without executing it')
    p.add_argument('--blank-stage', type=int, default=-1)
    p.add_argument('--blank-frames', type=int, default=0)
    p.add_argument('--video', type=Path, help='One-environment diagnostic overview plus actual control camera')
    p.add_argument('--report', type=Path, required=True)
    return p


def main():
    if os.environ.get('REBAR_STARTUP_TRACE'):
        import faulthandler,signal
        faulthandler.enable();faulthandler.register(signal.SIGUSR1,all_threads=True)
        faulthandler.dump_traceback_later(30,repeat=True)
    p = parser(); early,_ = p.parse_known_args()
    if early.report.exists():
        p.error('Use a fresh report path')
    if early.video and (early.video.exists() or early.num_envs != 1):
        p.error('Video requires one environment and a new output path')
    if early.checkpoint and early.demonstrations:
        p.error('Neural actions are not servo demonstrations; use --label-replay for independent teacher labels')
    if early.label_replay and not early.checkpoint:
        p.error('--label-replay requires a student checkpoint')
    if any(path and path.exists() for path in (early.demonstrations,early.label_replay)):
        p.error('Use fresh demonstration/replay paths')
    if early.isaaclab_root is None or '--enable_cameras' not in sys.argv:
        p.error('ISAACLAB_ROOT and --enable_cameras are required')
    verify_checkout(early.isaaclab_root); preflight_subprocess(asset_directory(), runtime=True)
    import gymnasium as gym
    import torch
    from isaaclab_tasks.utils import add_launcher_args,launch_simulation,setup_preset_cli
    from isaaclab_tasks.utils.hydra import hydra_task_config
    from .registration import register_tasks
    from .vision.blind import blinded_control
    register_tasks(); add_launcher_args(p)
    args,remaining = setup_preset_cli(p); sys.argv = [sys.argv[0],*remaining]

    @hydra_task_config('Aubo-Rebar-Visual-Loading-Direct-v0','rsl_rl_cfg_entry_point')
    def run(cfg,agent_cfg):
        cfg.scene.num_envs=args.num_envs;cfg.seed=args.seed
        cfg.visual_detector=str(args.detector.resolve())
        cfg.camera_capture_config=str(args.camera_config.resolve())
        cfg.visual_blank_stage=args.blank_stage;cfg.visual_blank_frames=args.blank_frames
        if args.video:
            cfg.viewer.eye=(-3.,2.4,2.6);cfg.viewer.lookat=(-.55,-.30,1.05)
            cfg.video_recorder.backend_source='renderer'
            cfg.video_recorder.window_width=1280;cfg.video_recorder.window_height=720
        records,receipts,trace,observations,labels=[],[],[],[],[]
        counts=dict(bootstrap=0,visual_servo=0,neural=0,paused=0)
        with launch_simulation(cfg,args):
            if os.environ.get('REBAR_STARTUP_TRACE'):faulthandler.cancel_dump_traceback_later()
            env=gym.make('Aubo-Rebar-Visual-Loading-Direct-v0',cfg=cfg,
                         render_mode='rgb_array' if args.video else None);e=env.unwrapped
            writer=None;video_frames=0
            try:
                env.reset(seed=args.seed)
                if args.video:
                    import imageio.v2 as imageio
                    import numpy as np
                    from PIL import Image,ImageDraw,ImageFont
                    args.video.parent.mkdir(parents=True,exist_ok=True)
                    writer=imageio.get_writer(str(args.video),fps=15,codec='libx264',quality=8,
                        macro_block_size=None,output_params=['-movflags','+faststart'])
                    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',21)
                    for _ in range(16):e.render()
                actor=None
                if args.checkpoint:
                    import yaml
                    from tensordict import TensorDict
                    from .route_actor import RouteActor,actor_constructor_config
                    saved=torch.load(args.checkpoint,weights_only=True,map_location=e.device)
                    contract=saved.get('rebar_observation_contract')
                    if contract!='loading_visual_v1' and not(args.state_warmstart and contract=='loading_route_v1'):
                        raise ValueError('Vision checkpoint required; state transfer must be explicitly requested')
                    actor_config=actor_constructor_config(agent_cfg.actor.to_dict(),saved)
                    actor=RouteActor(TensorDict({'policy':torch.zeros(e.num_envs,71,device=e.device)},batch_size=[e.num_envs]),
                        {'actor':['policy']},'actor',7,**actor_config).to(e.device).eval()
                    actor.load_state_dict(saved['actor_state_dict'])
                    def predict(observation):
                        return actor(TensorDict({'policy':observation},batch_size=[len(observation)]))
                else:
                    predict=None
                with torch.inference_mode():
                    for step in range(e.task_spec.episode_steps*(args.episodes+1)):
                        active=e.visual.valid & e.visual.motion.ready
                        with blinded_control(e):
                            obs=e._pack_observations(e.visual.measured_state(),e.visual.logic,e.route)['policy']
                            actions=e.visual.action(predict)
                            teacher=e.visual.servo.route_actions(e.visual.measured_state(),e.route) if args.label_replay else actions
                        if (args.demonstrations or args.label_replay) and active.any():
                            observations.append(obs[active].clone());labels.append(teacher[active].clone())
                        counts['bootstrap']+=int((~e.visual.motion.ready).sum())
                        counts['paused']+=int((e.visual.motion.ready & ~e.visual.valid).sum())
                        counts['neural' if actor else 'visual_servo']+=int(active.sum())
                        env.step(actions)
                        if writer and step%2==0 and not e.completed_episodes:
                            rgb=e.render()
                            frame=Image.fromarray(rgb).convert('RGB');draw=ImageDraw.Draw(frame)
                            draw.rectangle((0,0,1280,88),fill=(15,23,38))
                            label='Vision neural policy: PPO prior + visual adaptation' if actor else 'RGB-only visual servo feasibility'
                            draw.text((20,10),label,font=font,fill='white')
                            phase='Far observation / return' if not e.visual.motion.ready[0] else f'Loading stage {int(e.route.stage[0])}'
                            draw.text((20,45),f'{step/30:.1f}s | {phase} | steel truth blocked in control',font=font,fill=(120,200,255))
                            if e.visual.latest_cameras:
                                wrist=Image.fromarray(e.visual.latest_cameras[0].mono8).convert('RGB').resize((384,231))
                                frame.paste(wrist,(876,118));draw.text((876,91),'Actual wrist mono camera',font=font,fill='white')
                            writer.append_data(np.asarray(frame));video_frames+=1
                        if step%30==0:
                            truth=e.visual_truth_for_evaluation
                            error=torch.linalg.vector_norm(e.visual.state.bar-truth.bar,dim=-1)
                            row=dict(step=step,route=e.route.stage.tolist(),valid=e.visual.valid.tolist(),
                                acquired=[o.acquired for o in e.visual.observers],returned=e.visual.motion.ready.tolist(),
                                errors_m=error.tolist(),estimated=e.visual.state.bar.tolist(),
                                true_for_evaluation=truth.bar.tolist(),
                                last_error=[o.last_error for o in e.visual.observers],
                                control_phase=e.visual.logic.phase.tolist(),
                                picked=e.visual.logic.pick_complete.tolist(),
                                observation_motion_error_m=e.visual.motion.motion.position_error.tolist(),
                                contact_dwell=e.visual.logic.contact_dwell.tolist(),
                                fingers=e.visual.state.finger_forces.tolist(),jaws=e.visual.state.jaw_forces.tolist())
                            trace.append(row)
                            print(json.dumps(row),flush=True)
                        if step%300==0 and e.visual.latest_cameras:
                            from PIL import Image
                            folder=args.report.with_suffix('');folder.mkdir(parents=True,exist_ok=True)
                            for i,packet in enumerate(e.visual.latest_cameras):
                                Image.fromarray(packet.mono8).save(folder/f'wrist_env{i}_step{step}.png')
                        records.extend(e.completed_episodes);e.completed_episodes.clear()
                        receipts.extend(e.visual_terminal_receipts);e.visual_terminal_receipts.clear()
                        if len(records)>=args.episodes:
                            break
                records=records[:args.episodes];receipts=receipts[:args.episodes]
                success=sum(r['success'] for r in records)/len(records) if records else 0.
                data_path=args.demonstrations or args.label_replay
                if data_path and observations:
                    data_path.parent.mkdir(parents=True,exist_ok=True)
                    torch.save(dict(observations=torch.cat(observations).cpu(),actions=torch.cat(labels).cpu(),
                        task='loading',observation_contract='loading_visual_v1',
                        source='visual servo labels on student-visited states' if actor else 'RGB-only visual servo',
                        success_rate=success,seed=args.seed),data_path)
                if writer is not None:
                    writer.close();writer=None
                    if not args.video.is_file() or args.video.stat().st_size<4096 or video_frames<1:
                        raise RuntimeError('Diagnostic video did not finalize correctly')
                report=dict(run_id=os.environ.get('REBAR_VISUAL_WORKER'),completed=len(records)==args.episodes,
                    mode='vision_policy_evaluation' if actor else 'vision_servo_feasibility',
                    seed=args.seed,episodes=records,visual_receipts=receipts,success_rate=success,
                    observation_contract='loading_visual_v1',control_privileged_state_guard=True,
                    runtime_camera_outputs=['rgb'],detector_sha256=hashlib.sha256(args.detector.read_bytes()).hexdigest(),
                    checkpoint_sha256=hashlib.sha256(args.checkpoint.read_bytes()).hexdigest() if actor else None,
                    neural_transfer_from_state_policy=bool(args.state_warmstart),counts=counts,trace=trace,
                    blank_camera_test=dict(stage=args.blank_stage,frames=args.blank_frames),
                    video=str(args.video) if args.video else None,video_frames=video_frames,
                    overview_used_for_control=False,tactile_feedback='unfiltered net contact load',
                    teacher_labels_only=bool(args.label_replay),teacher_actions_executed=actor is None,
                    release_clearance_guard=e.route.require_release_clearance,
                    ground_truth_control_inputs=False,ground_truth_reward_and_evaluation=True)
                args.report.parent.mkdir(parents=True,exist_ok=True)
                args.report.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
                print('VISUAL_RESULT',success,counts,flush=True)
            finally:
                if writer is not None:writer.close()
                env.close()
    run()


def supervised():
    args,_=parser().parse_known_args();run_id=uuid.uuid4().hex
    process=subprocess.run([sys.executable,'-m','rebar_training.visual_check',*sys.argv[1:]],
        env={**os.environ,'REBAR_VISUAL_WORKER':run_id})
    try:report=json.loads(args.report.read_text())
    except (OSError,ValueError):report={}
    if process.returncode or report.get('run_id')!=run_id or not report.get('completed'):
        raise RuntimeError('Visual run failed or produced no fresh completion receipt; inspect its log')


if __name__=='__main__':
    main() if os.environ.get('REBAR_VISUAL_WORKER') else supervised()
