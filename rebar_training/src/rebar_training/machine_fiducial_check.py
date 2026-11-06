"""Finite actual-machine-mounted fiducial arrival/transfer simulation, frozen policy."""
import argparse
from dataclasses import replace
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback
import uuid


def clean(value):
    """Absent/invalid failed measurements stay null, never fabricated zeroes."""
    import numpy as np
    if isinstance(value,np.ndarray):return clean(value.tolist())
    if isinstance(value,np.generic):return clean(value.item())
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    return value


def save(path,data):
    tmp=path.with_suffix(path.suffix+'.pending');tmp.write_text(json.dumps(clean(data),indent=2,allow_nan=False)+'\n');tmp.replace(path)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--layout',type=Path,required=True)
    p.add_argument('--camera-config',type=Path,required=True)
    p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--detector',type=Path,required=True)
    p.add_argument('--machine',default='machine_1',choices=('machine_1','machine_2','machine_3'))
    p.add_argument('--until',choices=('observe','transfer'),default='transfer')
    p.add_argument('--max-steps',type=int,default=12000)
    p.add_argument('--isaaclab-root',type=Path,default=os.environ.get('ISAACLAB_ROOT'))
    early,_=p.parse_known_args()
    if early.output.exists():p.error('Fresh output path required')
    if not 300<=early.max_steps<=18000:p.error('Finite max-steps 300..18000 required')
    if '--enable_cameras' not in sys.argv:p.error('--enable_cameras required')
    from .launcher import verify_checkout
    verify_checkout(early.isaaclab_root)
    from .workcells import WorkcellLayout,Pose
    from .policy_compatibility import FrozenStationResidual,StationPolicyAdapter,compose_residual_actions,sample_from_legacy_control,SUPPORTED_ROUTES
    layout=WorkcellLayout.load(early.layout);layout.station(early.machine,'machine')
    frozen=FrozenStationResidual(early.checkpoint);identity_before=frozen.verify_unchanged()
    from isaaclab_tasks.utils import add_launcher_args,launch_simulation,setup_preset_cli
    from isaaclab_tasks.utils.hydra import hydra_task_config
    import gymnasium as gym
    from .registration import register_tasks
    register_tasks();task='Aubo-Rebar-Machine-Fiducial-Direct-v0'
    gym.register(id=task,entry_point='rebar_training.machine_fiducial_env:MachineFiducialEnv',disable_env_checker=True,
        kwargs={'env_cfg_entry_point':'rebar_training.isaac_cfg:MobileVisualEnvCfg'})
    add_launcher_args(p);args,remaining=setup_preset_cli(p);sys.argv=[sys.argv[0],*remaining]
    args.output.mkdir(parents=True,exist_ok=False)
    run_id=uuid.uuid4().hex;started=time.time()
    report=dict(schema='mounted_fiducial_arrival_transfer_candidate_v1',run_id=run_id,
        machine_id=args.machine,source_policy_context='source_A',layout_sha256=layout.sha256,
        initialized_on_tray=True,end_to_end_source_pickup=False,requested_until=args.until,
        policy_optimizer_before=identity_before,training=False,optimizer_instantiated=False,cosmos_used=False,
        hardware_or_ros=False,simulation_started=True,full_mission_acceptance=False,errors=[],trace=[],
        source_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    save(args.output/'status.json',dict(state='starting',run_id=run_id,pid=os.getpid()))

    @hydra_task_config(task,None)
    def run(cfg,_agent):
        import cv2
        import numpy as np
        import torch
        import imageio.v2 as imageio
        from PIL import Image,ImageDraw
        from isaaclab.actuators import ImplicitActuatorCfg
        cfg.scene.num_envs=1;cfg.seed=6110601;cfg.scene.env_spacing=15.
        station=layout.station(args.machine)
        cfg.viewer.eye=station.pose_map.point((-2.8,2.8,3.1))
        cfg.viewer.lookat=station.pose_map.point((.1,.4,1.1))
        cfg.video_recorder.eye=cfg.viewer.eye;cfg.video_recorder.lookat=cfg.viewer.lookat
        cfg.episode_length_s=args.max_steps/30+10;cfg.mobile_until='clamp'
        cfg.workcell_layout=str(args.layout.resolve());cfg.workcell_source='source_A';cfg.workcell_machine=args.machine
        cfg.workcell_initial_offset=.15;cfg.workcell_ideal_machine_survey=False;cfg.workcell_transport_recenter=False
        cfg.mobile_start_on_tray=False;cfg.mobile_start_at_dock=False;cfg.mobile_visual_hint_packet=None
        cfg.visual_detector=str(args.detector.resolve());cfg.camera_capture_config=str(args.camera_config.resolve())
        cfg.mobile_residual_control=True;cfg.observation_space=76
        cfg.robot.actuators['wheels']=ImplicitActuatorCfg(joint_names_expr=['.*wheel_joint'],effort_limit_sim=120.,
            velocity_limit_sim=3.,stiffness=0.,damping=1000.)
        rows=[];calls=0;video_frames=0;latest_mono=None;fid_files=[];env=None;writer=None
        pose_write_attempts=[];original_pose_writers=[]
        with launch_simulation(cfg,args):
            from .vision.blind import blinded_control
            from .vision.capture import array
            from .mobile_residual import FIELDS
            def capture(image,row):
                nonlocal latest_mono
                latest_mono=image.copy()
                tag=f"fid_{row['tick']:06d}_{row['frame']:06d}"
                cv2.imwrite(str(args.output/(tag+'.png')),image)
                row['image_file']=tag+'.png';row['image_sha256']=hashlib.sha256((args.output/(tag+'.png')).read_bytes()).hexdigest()
                save(args.output/(tag+'.json'),row);fid_files.append(tag+'.json')
            try:
                env=gym.make(task,cfg=cfg,render_mode='rgb_array');e=env.unwrapped;env.reset(seed=cfg.seed)
                v=e.visual;v.fid_callback=capture;e.trial_started=True
                for asset_name,asset in [('robot',e.robot),('steel',e.bar)]:
                    for method in ('write_root_pose_to_sim_index','write_root_pose_to_sim',
                                   'write_root_state_to_sim_index','write_root_state_to_sim'):
                        if not hasattr(asset,method):continue
                        original_pose_writers.append((asset,method,getattr(asset,method)))
                        def reject_pose_write(*a,_asset=asset_name,_method=method,**kw):
                            pose_write_attempts.append(dict(asset=_asset,method=_method))
                            raise RuntimeError('Root/steel pose write forbidden after trial reset')
                        setattr(asset,method,reject_pose_write)
                report['mounts']=e.marker_mount_records
                report['initial_base_pose']=array(e.robot.data.root_pos_w)[0].tolist()
                report['declared_initialization']='steel supported on real tray at reset; source pickup excluded'
                report['coarse_navigation_bias_base_xy_yaw']=[.020,0.,.010]
                report['navigation_goals']=v.navigation_goals;report['navigation_obstacles']=v.navigation_obstacles
                from isaacsim.core.rendering_manager import ViewportManager
                station=layout.station(args.machine)
                eye=station.pose_map.point((-2.8,2.8,3.1));target=station.pose_map.point((.1,.4,1.1))
                ViewportManager.set_camera_view('/OmniverseKit_Persp',eye=list(eye),target=list(target));e.sim.render()
                writer=imageio.get_writer(str(args.output/'arrival_transfer.mp4'),fps=10,codec='libx264',quality=7,
                    macro_block_size=1,pixelformat='yuv420p')
                with (args.output/'policy_inputs.jsonl').open('x') as policy_log,torch.no_grad():
                    for step in range(args.max_steps):
                        v=e.visual
                        with blinded_control(e):
                            actions=v.action()
                            if v.fid_phase is None and v.fid_verified and v.stage in SUPPORTED_ROUTES and bool(v.valid[0]):
                                if v.tick/30-v.observers[0].last_good_ns*1e-9<=.2:
                                    fixture=v.fixture_odom;profile=layout.profiles[layout.station(args.machine).profile_id]
                                    shift=np.eye(4);shift[:3,3]=-np.array(profile.fixture_goal)
                                    station_pose=fixture@shift
                                    refined=Pose(*station_pose[:3,3],float(np.arctan2(station_pose[1,0],station_pose[0,0])))
                                    adapter=StationPolicyAdapter(layout,'source_A',args.machine,refined_machine_pose=refined)
                                    sample=sample_from_legacy_control(v,inference_now_s=v.tick/30)
                                    packet=adapter.encode(sample)
                                    # Preserve the frozen 76-D encoding; the measured
                                    # 3-D goal must be identical to the local servo goal.
                                    values=list(packet.values);offset=0
                                    for name,width in FIELDS:
                                        if name=='machine_goal_minus_tcp':values[offset:offset+width]=[(g-t)/5 for g,t in zip(v.goal(),sample.tcp_base)]
                                        offset+=width
                                    packet=replace(packet,values=tuple(values),actual_machine_goal_base=tuple(v.goal()),
                                                   policy_context_goal_base=tuple(v.goal()))
                                    decision=frozen.infer(packet);actions=compose_residual_actions(actions,decision)
                                    calls+=int(decision['active'])
                                    policy_log.write(json.dumps(clean(dict(step=step,phase=v.stage,route=int(e.route.stage[0]),
                                        values=packet.values,decision=decision)),allow_nan=False)+'\n')
                        env.step(actions)
                        if (step+1)%3==0:
                            image=Image.fromarray(np.asarray(e.render(),dtype=np.uint8)[:,:,:3]).convert('RGB')
                            draw=ImageDraw.Draw(image);draw.rectangle((0,0,image.width,64),fill=(15,23,38))
                            draw.text((15,10),f'{step/30:.1f}s | {v.stage} | machine-mounted board | frozen policy, no training',fill='white')
                            draw.text((15,35),'Stage test starts with steel on tray; no source pickup. Overview is recording only.',fill=(120,200,255))
                            if latest_mono is not None:
                                inset=Image.fromarray(latest_mono).convert('RGB');inset.thumbnail((320,210))
                                image.paste(inset,(image.width-inset.width-10,80))
                            writer.append_data(np.asarray(image));video_frames+=1
                        if step%30==0:
                            observer=v.observers[0]
                            active_motion=v.fid_motion if v.fid_phase in ('observe','return') else v.motion
                            row=dict(step=step,sim_time_s=v.tick/30,stage=v.stage,fid_phase=v.fid_phase,
                                route=int(e.route.stage[0]),valid=bool(v.valid[0]),tray_load_n=v.tray_force,
                                base=array(e.robot.data.root_pos_w)[0].tolist(),fid_verified=v.fid_verified,
                                fid_corrections=v.fid_corrections,last_marker_error=v.fid_last_error,
                                last_steel_error=observer.last_error,policy_calls=calls,failure=v.failure,
                                observed_steel=observer.position.tolist() if observer.acquired else None,
                                arm_q=array(e.robot.data.joint_pos)[0,e.control_ids[:6]].tolist(),
                                observation_position_error_m=float(active_motion.motion.position_error[0]),
                                observation_quaternion_error=float(active_motion.motion.quaternion_error[0]))
                            Image.fromarray(np.asarray(e.render(),dtype=np.uint8)[:,:,:3]).save(args.output/'latest_overview.png')
                            if v.latest_cameras:
                                packet=v.latest_cameras[0];tag=f'steel_{step:06d}'
                                Image.fromarray(packet.mono8).save(args.output/(tag+'.png'))
                                save(args.output/(tag+'.json'),dict(timestamp_ns=packet.timestamp_ns,frame=packet.frame,
                                    K=packet.K.tolist(),T_base_optical=packet.T_scene_optical.tolist(),
                                    stage=v.stage,error=observer.last_error,quality=observer.quality))
                                if observer.last_probability is not None:
                                    np.save(args.output/(tag+'.probability.npy'),observer.last_probability,allow_pickle=False)
                            rows.append(clean(row));print(json.dumps(clean(row),allow_nan=False),flush=True)
                            save(args.output/'status.json',dict(state='running',run_id=run_id,pid=os.getpid(),latest=row))
                        if e.mobile_receipts:break
                        if args.until=='observe' and v.fid_observation_done:break
                report.update(completed_steps=step+1,native_results=list(e.mobile_receipts),
                    fid_events=v.fid_events,fid_rows_files=fid_files,fid_verified=v.fid_verified,
                    fid_corrections=v.fid_corrections,fid_last_error=v.fid_last_error,
                    policy_calls=calls,trace=rows,video_frames=video_frames,video_fps=10,
                    overview_used_for_control=False,automatic_terminal_reset_suppressed=e.automatic_terminal_reset_suppressed,
                    root_or_steel_pose_writes_during_trial=False)
                report['requested_stage_completed']=(bool(v.fid_observation_done) if args.until=='observe' else
                    bool(len(e.mobile_receipts)==1 and e.mobile_receipts[0]['success'] and v.fid_verified))
                import omni.usd
                omni.usd.get_context().get_stage().GetRootLayer().Export(str(args.output/'mounted_scene.usda'))
            except Exception as exc:
                report['errors'].append(type(exc).__name__+': '+str(exc));(args.output/'error.txt').write_text(traceback.format_exc())
            finally:
                if writer is not None:writer.close()
                if env is not None:
                    v=env.unwrapped.visual
                    report.update(fid_events=v.fid_events,fid_verified=v.fid_verified,
                        fid_corrections=v.fid_corrections,fid_last_error=v.fid_last_error,
                        native_results=list(env.unwrapped.mobile_receipts),
                        peak_contact_loads_N={name:dict(zip(e.contact_sensors[name].body_names,loads[0].tolist()))
                            for name,loads in e._contact_peaks.items()})
                report['forbidden_pose_write_attempts']=pose_write_attempts
                report.update(policy_optimizer_after=frozen.verify_unchanged(),elapsed_wall_s=time.time()-started,
                    video_frames=video_frames,video_fps=10,fid_rows_files=fid_files,trace=rows,policy_calls=calls)
                save(args.output/'report.json',report)
                save(args.output/'status.json',dict(state='failed' if report['errors'] or not report.get('requested_stage_completed') or
                    report.get('native_results') and not report['native_results'][-1].get('success') else 'completed_stage_probe',run_id=run_id,
                    fid_verified=report.get('fid_verified',False),native_success=bool(report.get('native_results') and
                        report['native_results'][-1].get('success')),errors=report['errors']))
                for asset,method,original in original_pose_writers:setattr(asset,method,original)
                if env is not None:env.close()
    run()


if __name__=='__main__':main()
