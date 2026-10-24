"""Frozen, single-episode physical workcell candidate; no training entry point."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import uuid


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--isaaclab-root',type=Path,default=os.environ.get('ISAACLAB_ROOT'))
    p.add_argument('--layout',type=Path,required=True)
    p.add_argument('--source',required=True)
    p.add_argument('--machine',required=True)
    p.add_argument('--seed',type=int,required=True)
    p.add_argument('--detector',type=Path,required=True)
    p.add_argument('--camera-config',type=Path,required=True)
    p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--report',type=Path,required=True)
    p.add_argument('--max-steps',type=int,default=18030)
    p.add_argument('--initial-offset',type=float,default=.15)
    p.add_argument('--ideal-machine-survey',action='store_true',help='Explicit ideal localization probe: target pose is exact by scene construction; not visual refinement')
    p.add_argument('--randomization',choices=('baseline','expanded'),default='baseline')
    p.add_argument('--capture-stride',type=int,default=90)
    return p


def atomic(path,data):
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n');tmp.replace(path)


def main():
    p=parser();early,_=p.parse_known_args()
    if early.report.exists() or early.report.with_suffix('').exists():p.error('Fresh output and raw directory required')
    if not 1<=early.max_steps<=24000:p.error('Bounded --max-steps 1..24000 required')
    if early.capture_stride<1:p.error('Positive capture stride required')
    if not early.ideal_machine_survey:p.error('--ideal-machine-survey is required; camera-based machine localization is not implemented')
    if '--enable_cameras' not in sys.argv:p.error('--enable_cameras required')
    from .launcher import verify_checkout
    from .workcells import WorkcellLayout
    from .policy_compatibility import FrozenStationResidual,StationPolicyAdapter,compose_residual_actions,sample_from_legacy_control,SUPPORTED_ROUTES
    layout=WorkcellLayout.load(early.layout);layout.station(early.source,'source');layout.station(early.machine,'machine')
    verify_checkout(early.isaaclab_root)
    frozen=FrozenStationResidual(early.checkpoint);identity_before=frozen.verify_unchanged()
    import torch
    import gymnasium as gym
    import numpy as np
    from isaaclab_tasks.utils import add_launcher_args,launch_simulation,setup_preset_cli
    from isaaclab_tasks.utils.hydra import hydra_task_config
    from .registration import register_tasks
    register_tasks()
    task='Aubo-Rebar-Workcell-Frozen-Direct-v0'
    gym.register(id=task,entry_point='rebar_training.workcell_sim_env:WorkcellVisualEnv',disable_env_checker=True,
                 kwargs={'env_cfg_entry_point':'rebar_training.isaac_cfg:MobileVisualEnvCfg'})
    add_launcher_args(p);args,remaining=setup_preset_cli(p);sys.argv=[sys.argv[0],*remaining]
    from .vision.blind import blinded_control
    from .vision.capture import array
    raw=args.report.with_suffix('');raw.mkdir(parents=True,exist_ok=False)
    run_id=uuid.uuid4().hex
    atomic(args.report.with_suffix('.status.json'),dict(state='starting',run_id=run_id,source=args.source,machine=args.machine,
        pid=os.getpid(),training_started=False,policy_optimizer_sha256=frozen.file_sha,updated_unix=time.time()))

    @hydra_task_config(task,None)
    def run(cfg,_agent):
        from isaaclab.actuators import ImplicitActuatorCfg
        cfg.scene.num_envs=1;cfg.seed=args.seed;cfg.mobile_until='clamp';cfg.reset_profile=args.randomization
        cfg.mobile_residual_control=True;cfg.observation_space=76
        cfg.mobile_start_on_tray=False;cfg.mobile_start_at_dock=False;cfg.mobile_visual_hint_packet=None
        cfg.workcell_layout=str(args.layout.resolve());cfg.workcell_source=args.source;cfg.workcell_machine=args.machine
        cfg.workcell_initial_offset=args.initial_offset;cfg.workcell_ideal_machine_survey=True
        cfg.visual_detector=str(args.detector.resolve());cfg.camera_capture_config=str(args.camera_config.resolve())
        cfg.scene.env_spacing=15. # Sufficient physical floor for all three surveyed machines.
        cfg.robot.actuators['wheels']=ImplicitActuatorCfg(joint_names_expr=['.*wheel_joint'],effort_limit_sim=120.,velocity_limit_sim=3.,stiffness=0.,damping=1000.)
        cfg.viewer.eye=(-3.,-5.,8.);cfg.viewer.lookat=(5.,2.,0.)
        adapter=StationPolicyAdapter(layout,args.source,args.machine)
        rows=[];decisions=0;error=None;terminal=[];navigation_goals=None;navigation_obstacles=None
        robot_tilt_max=0.;started=time.time()
        def save_packet(e,tag,terminal_packet=None):
            from PIL import Image
            if terminal_packet is None:
                if not e.visual.latest_cameras:return
                packet=e.visual.latest_cameras[0];prob=e.visual.observers[0].last_probability;reason=e.visual.observers[0].last_error
            else:packet,prob,reason=terminal_packet
            Image.fromarray(packet.mono8).save(raw/(tag+'.png'))
            if prob is not None:np.save(raw/(tag+'.probability.npy'),prob,allow_pickle=False)
            atomic(raw/(tag+'.json'),dict(timestamp_ns=packet.timestamp_ns,frame=packet.frame,K=packet.K.tolist(),
                T_base_optical=packet.T_scene_optical.tolist(),last_error=reason))
        def publish():
            unchanged=frozen.verify_unchanged()
            report=dict(schema='workcell_physical_candidate_v1',run_id=run_id,source_id=args.source,machine_id=args.machine,
                layout_sha256=layout.sha256,seed=args.seed,randomization=args.randomization,initial_offset_m=args.initial_offset,
                completed=bool(terminal),results=terminal,error=error,bounded_steps=args.max_steps,trace=rows,
                frozen_identity_before=identity_before,frozen_identity_after=unchanged,residual_training=False,new_learning_updates=0,
                optimizer_instantiated=False,simulation_started=True,hardware_or_ros_started=False,policy_calls=decisions,
                end_to_end=True,source_navigation_in_physics=True,navigation_backend='static CAD + measured odometry',
                machine_visual_identity_verified=False,machine_pose_source='exact registered pose by scene construction; explicit ideal localization probe',
                ideal_machine_survey=True,visual_machine_pose_refinement=False,
                multi_workcell_task_executor_verified=False,physical_acceptance=False,
                scope='physical candidate controller probe; task-level acceptance requires independent review and machine observation binding',
                navigation_goals=navigation_goals,navigation_obstacles=navigation_obstacles,raw_directory=str(raw),
                candidate_env_sha256=hashlib.sha256(Path(__file__).with_name('workcell_sim_env.py').read_bytes()).hexdigest(),elapsed_wall_s=time.time()-started)
            atomic(args.report,report)
            atomic(args.report.with_suffix('.status.json'),dict(state='failed' if error or terminal and not terminal[-1]['success'] else 'completed' if terminal else 'bounded_probe_complete',
                run_id=run_id,report=str(args.report),error=error,updated_unix=time.time(),training_started=False))
        with launch_simulation(cfg,args):
            env=None
            try:
                env=gym.make(task,cfg=cfg);e=env.unwrapped
                env.reset(seed=args.seed)
                navigation_goals=e.visual.navigation_goals;navigation_obstacles=e.visual.navigation_obstacles
                atomic(raw/'scene_navigation.json',dict(layout=layout.to_dict(),goals=navigation_goals,obstacles=navigation_obstacles,
                    initial_base=array(e.robot.data.root_pos_w)[0].tolist(),initial_base_quaternion=array(e.robot.data.root_quat_w)[0].tolist(),
                    initial_bar_for_evaluation=array(e.bar.data.root_pos_w)[0].tolist(),machine_goal=e.machine_goal_map,
                    scene_rigid_objects=list(e.scene.rigid_objects),scene_articulations=list(e.scene.articulations)))
                with (raw/'policy_inputs.jsonl').open('x') as policy_log,torch.no_grad():
                    for step in range(args.max_steps):
                        v=e.visual;decision=None
                        with blinded_control(e):
                            nominal=v.action();actions=nominal
                            # Bootstrap and bounded visual-loss hold stay under the original controller.
                            if (not v.initial_navigation and v.stage in SUPPORTED_ROUTES and bool(v.valid[0])
                                    and v.tick/30-v.observers[0].last_good_ns*1e-9<=.2):
                                sample=sample_from_legacy_control(v,inference_now_s=v.tick/30)
                                packet=adapter.encode(sample);decision=frozen.infer(packet)
                                actions=compose_residual_actions(nominal,decision);decisions+=int(decision['active'])
                                policy_log.write(json.dumps(dict(step=step,phase=v.stage,route=int(e.route.stage[0]),
                                    values=packet.values,decision=decision,nominal=nominal.cpu().tolist(),actions=actions.cpu().tolist()),allow_nan=False)+'\n')
                        env.step(actions)
                        if e.mobile_receipts:
                            terminal=list(e.mobile_receipts);save_packet(e,'terminal',e.mobile_terminal_packet);break
                        if v.source_return_handoffs and v.source_return_handoffs[-1]['tick']==v.tick:save_packet(e,v.source_return_handoffs[-1]['packet_tag'])
                        if (step+1)%args.capture_stride==0:save_packet(e,'wrist_'+str(step+1))
                        if step%30==0:
                            observer=v.observers[0]
                            row=dict(step=step,sim_time_s=v.tick/30,stage=v.stage,source_navigation=v.initial_navigation,
                                route=int(e.route.stage[0]),valid=bool(v.valid[0]),base=array(e.robot.data.root_pos_w)[0].tolist(),
                                base_quaternion=array(e.robot.data.root_quat_w)[0].tolist(),wheel_command=v.wheel_command,
                                robot_twist=array(e.robot.data.root_lin_vel_w)[0].tolist(),last_error=observer.last_error,
                                estimated_bar=observer.position.tolist() if observer.acquired else None,
                                camera_age_ms=(v.tick/30-observer.last_good_ns*1e-9)*1000,
                                arm_joints=array(e.robot.data.joint_pos)[0,e.control_ids[:6]].tolist(),
                                finger_positions=v.packets[0].fingers.tolist(),finger_forces=v.packets[0].finger_forces.tolist(),
                                tray_load_n=float(v.tray_force),policy_calls=decisions)
                            rows.append(row);print(json.dumps(row,allow_nan=False),flush=True)
                            atomic(args.report.with_suffix('.status.json'),dict(state='running',run_id=run_id,pid=os.getpid(),
                                source=args.source,machine=args.machine,latest=row,updated_unix=time.time(),training_started=False))
                if not terminal:save_packet(e,'bounded_end')
            except Exception as ex:
                error=type(ex).__name__+': '+str(ex)
                import traceback
                (raw/'error.txt').write_text(traceback.format_exc())
            finally:
                # Kit can terminate Python during shutdown; persist proofs first.
                publish()
                if env is not None:env.close()
            if error:raise RuntimeError(error)
    run()


if __name__=='__main__':main()
