"""Bounded physical validation of staged, RGB-only mobile mission integration."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid

from .launcher import verify_checkout
from .spec import asset_directory
from .randomization import PROFILES, describe


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',default='Aubo-Rebar-Mobile-Visual-Direct-v0')
    p.add_argument('--isaaclab-root',type=Path,default=os.environ.get('ISAACLAB_ROOT'))
    p.add_argument('--seed',type=int,default=119042)
    p.add_argument('--episodes',type=int,default=1)
    p.add_argument('--capture-stride',type=int,default=300,help='Control steps between diagnostic RGB packets')
    p.add_argument('--capture-stage',action='append',help='Limit periodic diagnostic capture to this mission stage; repeatable')
    p.add_argument('--randomization',choices=tuple(PROFILES),default='baseline')
    p.add_argument('--residual-checkpoint',type=Path,help='Deploy the bounded manipulation residual, retaining the RGB servo and odometry')
    p.add_argument('--learn-residual',action='store_true',help='Collect on-policy masked rollouts and update residual PPO')
    p.add_argument('--rl-iterations',type=int,default=40)
    p.add_argument('--rl-rollout',type=int,default=256)
    p.add_argument('--residual-scale',type=float,default=.08)
    p.add_argument('--save-policy',type=Path,help='Atomic resumable checkpoint; required for learning')
    p.add_argument('--regression',action='store_true',help='Permit a training seed for a labeled regression; never a holdout pass')
    p.add_argument('--tray-camera-outage-frames',type=int,default=0,
                   help='Regression-only black wrist frames during the empty return from tray observation')
    p.add_argument('--tray-grasp-camera-outage-frames',type=int,default=0,
                   help='Regression-only black wrist frames during a supported partial tray grasp')
    p.add_argument('--until',choices=('tray','dock','clamp'),default='tray')
    p.add_argument('--start-on-tray',action='store_true',help='Isolated navigation/regrasp curriculum, never an end-to-end claim')
    p.add_argument('--start-at-dock',action='store_true',help='Reset-only dock/final-skill curriculum; requires --start-on-tray')
    p.add_argument('--survey-from-packet',type=Path,help='Previously recorded mono/K/TF packet supplies only a visual search hint')
    p.add_argument('--joint-observation',action='store_true')
    p.add_argument('--initial-joints',nargs=6,type=float,help='Reset-only curriculum pose; physical collision checks remain active')
    p.add_argument('--detector',type=Path,required=True)
    p.add_argument('--camera-config',type=Path,required=True)
    p.add_argument('--report',type=Path,required=True)
    p.add_argument('--video',type=Path,help='Diagnostic renderer overview plus the actual wrist control image')
    return p


def main():
    if os.environ.get('REBAR_STARTUP_TRACE'):
        import faulthandler,signal
        faulthandler.enable();faulthandler.register(signal.SIGUSR1,all_threads=True)
        faulthandler.dump_traceback_later(30,repeat=True)
    p=parser();early,_=p.parse_known_args()
    if early.report.exists():p.error('Use a fresh report path')
    if early.episodes<1:p.error('--episodes must be positive')
    if early.capture_stride<1:p.error('--capture-stride must be positive')
    if early.tray_camera_outage_frames<0 or (early.tray_camera_outage_frames and not early.regression):
        p.error('Tray camera fault injection requires --regression and a nonnegative frame count')
    if early.tray_grasp_camera_outage_frames<0 or (early.tray_grasp_camera_outage_frames and not early.regression):
        p.error('Grasp camera fault injection requires --regression and a nonnegative frame count')
    if early.learn_residual and (not early.save_policy or early.rl_iterations<1 or early.rl_rollout<32):
        p.error('Learning requires --save-policy, positive iterations and rollout >=32')
    if early.save_policy and early.save_policy.exists():p.error('Use a fresh policy output path; use --residual-checkpoint to resume')
    early.report.parent.mkdir(parents=True,exist_ok=True)
    if early.video and early.video.exists():p.error('Use a fresh video path')
    if early.start_on_tray and early.until=='tray':p.error('--start-on-tray requires --until dock or clamp')
    if early.start_at_dock and not(early.start_on_tray and early.until in ('dock','clamp')):
        p.error('--start-at-dock requires --start-on-tray --until dock or clamp')
    if early.survey_from_packet and not early.start_at_dock:p.error('Recorded survey hint is only for the explicit dock curriculum')
    if early.initial_joints and not early.start_on_tray:p.error('--initial-joints is only for the explicitly marked tray curriculum')
    if '--enable_cameras' not in sys.argv:p.error('--enable_cameras required')
    verify_checkout(early.isaaclab_root)
    subprocess.run([sys.executable,'-c',
        'from rebar_training.mobile_assets import audit_mobile_assets; import sys; print(audit_mobile_assets(sys.argv[1]))',
        str(asset_directory())],check=True)
    import gymnasium as gym
    import torch
    from isaaclab_tasks.utils import add_launcher_args,launch_simulation,setup_preset_cli
    from isaaclab_tasks.utils.hydra import hydra_task_config
    from .registration import register_tasks
    from .vision.blind import blinded_control
    register_tasks();add_launcher_args(p);args,remaining=setup_preset_cli(p)
    sys.argv=[sys.argv[0],*remaining]

    @hydra_task_config('Aubo-Rebar-Mobile-Visual-Direct-v0',None)
    def run(cfg,_agent):
        from isaaclab.actuators import ImplicitActuatorCfg
        cfg.scene.num_envs=1;cfg.seed=args.seed;cfg.mobile_until=args.until
        cfg.reset_profile=args.randomization
        if args.learn_residual or args.residual_checkpoint:
            from .mobile_residual import OBS_SIZE
            cfg.mobile_residual_control=True;cfg.observation_space=OBS_SIZE
        cfg.mobile_start_on_tray=args.start_on_tray
        cfg.mobile_start_at_dock=args.start_at_dock
        cfg.mobile_visual_hint_packet=str(args.survey_from_packet.resolve()) if args.survey_from_packet else None
        cfg.mobile_joint_observation=args.joint_observation
        cfg.mobile_initial_joints=tuple(args.initial_joints or ())
        cfg.mobile_tray_camera_outage_frames=args.tray_camera_outage_frames
        cfg.mobile_tray_grasp_camera_outage_frames=args.tray_grasp_camera_outage_frames
        cfg.visual_detector=str(args.detector.resolve());cfg.camera_capture_config=str(args.camera_config.resolve())
        if args.video:
            cfg.viewer.eye=(-3.,2.4,2.6);cfg.viewer.lookat=(-.55,-.30,1.05)
            cfg.video_recorder.backend_source='renderer'
            cfg.video_recorder.window_width=1280;cfg.video_recorder.window_height=720
        cfg.robot.actuators['wheels']=ImplicitActuatorCfg(joint_names_expr=['.*wheel_joint'],effort_limit_sim=120.,
            velocity_limit_sim=3.,stiffness=0.,damping=1000.)
        rows=[];bad_packets=0;last_bad_frame=-1
        with launch_simulation(cfg,args):
            if os.environ.get('REBAR_STARTUP_TRACE'):faulthandler.cancel_dump_traceback_later()
            env=gym.make('Aubo-Rebar-Mobile-Visual-Direct-v0',cfg=cfg,
                         render_mode='rgb_array' if args.video else None);e=env.unwrapped
            writer=None;video_frames=0;video_view=None;video_views=[]
            try:
                env.reset(seed=args.seed)
                learner=None;initial_updates=0;rl_rows=[]
                if args.learn_residual or args.residual_checkpoint:
                    from .mobile_residual import (ResidualPPO,progress_potential,residual_reward,
                                                 precision_residual_gain,PRECISION_ENVELOPE)
                    learner=ResidualPPO(e.device,args.residual_scale,args.seed,args.residual_checkpoint)
                    initial_updates=learner.updates
                    if args.learn_residual:
                        learner.training_contexts.append(dict(seed=args.seed,randomization=args.randomization,
                            until=args.until,initialized_on_tray=args.start_on_tray,initialized_at_dock=args.start_at_dock))
                    elif not args.regression and args.seed in {c['seed'] for c in learner.training_contexts}:
                        raise ValueError('Evaluation seed was used in training; select a fresh holdout seed or label --regression')
                if args.video:
                    import imageio.v2 as imageio
                    import numpy as np
                    from PIL import Image,ImageDraw,ImageFont
                    from isaacsim.core.rendering_manager import ViewportManager
                    args.video.parent.mkdir(parents=True,exist_ok=True)
                    writer=imageio.get_writer(str(args.video),fps=10,codec='libx264',quality=8,
                        macro_block_size=None,output_params=['-movflags','+faststart'])
                    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',21)
                    for _ in range(16):e.render()
                def save_packet(tag,terminal=None):
                    from PIL import Image
                    import numpy as np
                    packet=terminal[0] if terminal else e.visual.latest_cameras[0]
                    folder=args.report.with_suffix('');folder.mkdir(parents=True,exist_ok=True)
                    Image.fromarray(packet.mono8).save(folder/(tag+'.png'))
                    meta=dict(K=packet.K.tolist(),T_task_optical=packet.T_scene_optical.tolist(),
                        timestamp_ns=packet.timestamp_ns,frame=packet.frame,
                        last_error=terminal[2] if terminal else e.visual.observers[0].last_error)
                    probability=terminal[1] if terminal else e.visual.observers[0].last_probability
                    if probability is not None:
                        np.save(folder/(tag+'.probability.npy'),probability,allow_pickle=False)
                    (folder/(tag+'.json')).write_text(json.dumps(meta,indent=2)+'\n')
                with torch.no_grad():
                    limit=(e.task_spec.episode_steps+1)*args.episodes
                    if args.learn_residual:limit+=args.rl_iterations*args.rl_rollout+e.task_spec.episode_steps
                    for step in range(limit):
                        episode=len(e.mobile_receipts)
                        transition=None;active=False;command_gain=0.;requested_residual=None
                        with blinded_control(e):
                            if learner:
                                observation,active=e.visual.learning_observation()
                                residual,transition=learner.act(observation,args.learn_residual)
                                command_gain=precision_residual_gain(observation)
                                requested_residual=residual
                                residual=residual*command_gain
                                active=active and command_gain>0
                                potential=progress_potential(e.visual.learning_metric(),e.visual.observers[0].acquired)
                                actions=e.visual.action(residual=residual,residual_scale=args.residual_scale)
                            else:actions=e.visual.action()
                        env.step(actions)
                        done=len(e.mobile_receipts)>episode
                        if args.learn_residual:
                            with blinded_control(e):
                                next_observation,_=e.visual.learning_observation()
                                next_potential=progress_potential(e.visual.learning_metric(),e.visual.observers[0].acquired)
                            reward=residual_reward(potential,next_potential,done=done,
                                success=bool(done and e.mobile_receipts[-1]['success']),
                                effort=float(residual.square().sum()) if active else 0.)
                            learner.record(transition,reward,next_observation,done,active)
                            if len(learner.buffer)>=args.rl_rollout:
                                update=learner.update();rl_rows.append(update);learner.save(args.save_policy)
                                print('PPO_UPDATE '+json.dumps(update),flush=True)
                                progress=args.report.with_suffix('.training.json')
                                progress.write_text(json.dumps(dict(updates=learner.updates,
                                    new_updates=learner.updates-initial_updates,steps=step+1,
                                    policy_steps=learner.policy_steps,episodes=len(e.mobile_receipts),
                                    checkpoint=str(args.save_policy),latest_update=update),indent=2)+'\n')
                        if len(e.mobile_receipts)>episode:
                            receipt=e.mobile_receipts[-1]
                            receipt['episode']=episode
                            receipt['residual_policy']=bool(learner)
                            receipt['residual_training']=args.learn_residual
                            if e.mobile_terminal_packet:save_packet(f'terminal_episode{episode}',e.mobile_terminal_packet)
                            bad_packets=0;last_bad_frame=-1
                            progress=dict(run_id=os.environ.get('REBAR_MOBILE_MISSION_WORKER'),
                                completed=(len(e.mobile_receipts)>=args.episodes and (not args.learn_residual
                                    or learner.updates-initial_updates>=args.rl_iterations)),
                                requested_episodes=args.episodes,episodes=len(e.mobile_receipts),
                                successes=sum(r['success'] for r in e.mobile_receipts),
                                results=list(e.mobile_receipts),randomization=describe(args.randomization),
                                end_to_end=not args.start_on_tray,neural_full_mission=False)
                            temporary=args.report.with_suffix('.progress.tmp')
                            temporary.write_text(json.dumps(progress,indent=2,allow_nan=False)+'\n')
                            temporary.replace(args.report.with_suffix('.progress.json'))
                            print('EPISODE_RECEIPT '+json.dumps(receipt),flush=True)
                            if len(e.mobile_receipts)>=args.episodes and (not args.learn_residual
                                    or learner.updates-initial_updates>=args.rl_iterations):break
                            continue
                        if writer and (step+1)%3==0:
                            view=('machine' if args.start_at_dock else 'travel' if e.visual.stage in ('navigate','dock') else 'machine'
                                  if e.visual.stage in ('pickup_tray','tray_lift_high','loading') else 'pickup')
                            if view!=video_view:
                                eye,target={'pickup':((-3.,2.4,2.6),(-.55,-.30,1.05)),
                                    'travel':((10.,-7.,7.),(3.,2.2,.8)),
                                    'machine':((3.2,6.8,3.1),(6.,4.35,1.))}[view]
                                # Renderer-backed recording has its own Kit
                                # camera. sim.set_camera_view only drives active
                                # visualizers and silently leaves this one fixed.
                                ViewportManager.set_camera_view('/OmniverseKit_Persp',eye=list(eye),target=list(target))
                                e.sim.render();video_view=view
                                video_views.append(dict(step=step,view=view,eye=eye,target=target))
                            frame=Image.fromarray(e.render()).convert('RGB');draw=ImageDraw.Draw(frame)
                            draw.rectangle((0,0,1280,88),fill=(15,23,38))
                            title=('Camera-only arm residual PPO + visual servo + odometry; not standalone RL' if learner
                                   else 'RGB-only mobile servo prototype + odometry; not full-mission RL')
                            draw.text((20,10),title,font=font,fill='white')
                            draw.text((20,45),f'{step/30:.1f}s | {e.visual.stage} | steel truth blocked in control',font=font,fill=(120,200,255))
                            if e.visual.latest_cameras:
                                wrist=Image.fromarray(e.visual.latest_cameras[0].mono8).convert('RGB').resize((384,231))
                                frame.paste(wrist,(876,118));draw.rectangle((876,88,1260,118),fill=(15,23,38))
                                draw.text((876,91),'Actual wrist mono camera',font=font,fill='white')
                            writer.append_data(np.asarray(frame));video_frames+=1
                        if step%30==0 or (e.visual.stage in ('navigate','dock') and e.visual.gate.reason!='enabled'):
                            v=e.visual;o=v.observers[0]
                            row=dict(step=step,episode=episode,stage=v.stage,route=int(e.route.stage[0]),valid=bool(v.valid[0]),
                                observation_ready=bool(v.motion.motion.ready[0]),returned=bool(v.motion.ready[0]),
                                observation_error_m=v.motion.motion.position_error.tolist(),
                                estimated_bar=o.position.tolist() if o.acquired else None,
                                true_bar_for_evaluation=e.bar.data.root_pos_w.torch[0].tolist(),
                                true_bar_base_for_evaluation=v.frame().point(e.bar.data.root_pos_w.torch[0].cpu().numpy()).tolist(),
                                base=e.robot.data.root_pos_w.torch[0].tolist(),
                                finger_forces=v.packets[0].finger_forces.tolist(),
                                tray_load_N=v.tray_force,last_error=o.last_error,wheel_command=v.wheel_command,
                                tcp_base=v.state.tcp[0].tolist())
                            row['arm_joints']=e.robot.data.joint_pos.torch[0,e.control_ids[:6]].tolist()
                            row.update(transport_gate=v.gate.reason,gate_counts=dict(v.gate_counts),
                                camera_age_ms=(v.tick/30-o.last_good_ns*1e-9)*1000,
                                observation_quaternion_error=v.motion.motion.quaternion_error.tolist(),
                                finger_positions=v.packets[0].fingers.tolist())
                            row['active_reobservations']=list(v.reobservations)
                            row['perception_quality']=dict(o.quality)
                            row['tray_reacquisition_count']=len(v.tray_reobservations)
                            row['tray_grasp_abort_count']=len(v.tray_grasp_aborts)
                            row['grasp_recovery_pending']=v.grasp_recovery.pending
                            row['tcp_quaternion_xyzw']=v.packets[0].quaternion_xyzw.tolist()
                            row['grasp_goal_quaternion_xyzw']=e.route.quaternion[0].tolist()
                            row['grasp_alignment_error_deg']=float(e.route.grasp_alignment_error_deg[0])
                            row['grasp_alignment_dwell']=int(e.route.grasp_alignment_dwell[0])
                            row.update(residual_command_gain=command_gain,residual_active=bool(active),
                                residual_requested_norm=float(requested_residual.norm()) if requested_residual is not None else 0.,
                                residual_scaled_norm=float(residual.norm()) if learner and active else 0.)
                            rows.append(row);print(json.dumps(row),flush=True)
                        if (step%args.capture_stride==0 and e.visual.latest_cameras
                                and (not args.capture_stage or e.visual.stage in args.capture_stage)):
                            save_packet(f'wrist_step{step}')
                        o=e.visual.observers[0]
                        if o.acquired and o.last_error and bad_packets<8 and o.last_frame!=last_bad_frame:
                            save_packet(f'rejected_{bad_packets}_step{step}');bad_packets+=1;last_bad_frame=o.last_frame
                if writer is not None:
                    writer.close();writer=None
                    if not args.video.is_file() or args.video.stat().st_size<4096 or video_frames<1:
                        raise RuntimeError('Diagnostic video did not finalize correctly')
                if args.learn_residual and learner.buffer:
                    update=learner.update();rl_rows.append(update);learner.save(args.save_policy)
                source_root=Path(__file__).resolve().parent.parent
                source_hash=hashlib.sha256()
                for file in sorted(source_root.rglob('*.py')):
                    source_hash.update(str(file.relative_to(source_root)).encode());source_hash.update(file.read_bytes())
                completed=(len(e.mobile_receipts)>=args.episodes and (not args.learn_residual
                           or learner.updates-initial_updates>=args.rl_iterations))
                report=dict(run_id=os.environ.get('REBAR_MOBILE_MISSION_WORKER'),completed=completed,
                    seed=args.seed,regression=args.regression,
                    source_sha256=source_hash.hexdigest(),
                    detector_sha256=hashlib.sha256(args.detector.read_bytes()).hexdigest(),
                    asset_manifest_sha256=hashlib.sha256((asset_directory()/'manifest.json').read_bytes()).hexdigest(),
                    result=e.mobile_receipts[0] if e.mobile_receipts else None,trace=rows,
                    results=list(e.mobile_receipts),requested_episodes=args.episodes,
                    episodes=len(e.mobile_receipts),successes=sum(r['success'] for r in e.mobile_receipts),
                    randomization=describe(args.randomization),
                    residual_policy=bool(learner),residual_training=args.learn_residual,
                    residual_total_updates=learner.updates if learner else 0,
                    residual_trained_steps=learner.policy_steps if learner else 0,
                    residual_actor_updates=learner.actor_updates if learner else 0,
                    residual_reward_contract=learner.reward_contract if learner else None,
                    residual_discount=learner.gamma if learner else None,
                    residual_precision_envelope=PRECISION_ENVELOPE if learner else None,
                    residual_checkpoint=str(args.residual_checkpoint) if args.residual_checkpoint else None,
                    residual_checkpoint_sha256=(hashlib.sha256(args.residual_checkpoint.read_bytes()).hexdigest()
                                                if args.residual_checkpoint else None),
                    trained_checkpoint=str(args.save_policy) if args.save_policy else None,
                    ppo_updates=rl_rows,
                    overridden_policy_samples_excluded_from_gradient=True,
                    observation_inputs=['wrist mono8','measured robot/chassis feedback','raw finger/jaw/tray load',
                                        'surveyed destinations and measured phase state'],
                    residual_action_size=6 if learner else 0,low_level_action_size=7,
                    navigation_policy='surveyed CAD route with odometry servo; not learned',
                    diagnostic_tray_camera_outage_frames=args.tray_camera_outage_frames,
                    diagnostic_tray_grasp_camera_outage_frames=args.tray_grasp_camera_outage_frames,
                    initial_home_wrist_offset_rad=0. if args.start_on_tray else e.cfg.mobile_home_wrist_offset,
                    privileged_control_guard=True,steel_truth_control=False,
                    controller=('bounded residual PPO over visual servo; scripted observation/gripper/odometry'
                                if learner else 'visual servo with surveyed destinations and odometry navigation'),
                    nav2=False,neural_full_mission=False,cosmos_inference=False,
                    navigation_goals=e.visual.navigation_goals,navigation_obstacles=e.visual.navigation_obstacles,
                    video=str(args.video) if args.video else None,video_frames=video_frames,video_fps=10,
                    overview_used_for_control=False,
                    video_views=video_views,
                    initialized_at_dock=args.start_at_dock,visual_hint_packet=str(args.survey_from_packet) if args.survey_from_packet else None,
                    end_to_end=not args.start_on_tray,steel_initialized_on_tray_at_reset=args.start_on_tray)
                args.report.parent.mkdir(parents=True,exist_ok=True)
                args.report.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
            finally:
                if writer is not None:writer.close()
                env.close()
    run()


def supervised():
    args,_=parser().parse_known_args();run_id=uuid.uuid4().hex
    process=subprocess.run([sys.executable,'-m','rebar_training.mobile_mission_check',*sys.argv[1:]],
        env={**os.environ,'REBAR_MOBILE_MISSION_WORKER':run_id})
    try:report=json.loads(args.report.read_text())
    except (OSError,ValueError):report={}
    if process.returncode or report.get('run_id')!=run_id or not report.get('completed'):
        raise RuntimeError('Mobile mission has no fresh completed receipt; inspect its log')


if __name__=='__main__':
    main() if os.environ.get('REBAR_MOBILE_MISSION_WORKER') else supervised()
