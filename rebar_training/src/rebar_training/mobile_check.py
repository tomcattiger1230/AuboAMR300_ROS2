"""Bounded physical tray transport curriculum probe, without steel attachments.

This is deliberately not an end-to-end or camera-only mission claim. The bar is
initialized on the real rack at reset; its subsequent motion is purely physical.
Robot odometry drives a small route, steel truth is used only for evaluation.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import uuid

from .launcher import verify_checkout
from .mobile_assets import audit_mobile_assets
from .spec import ARM_JOINTS,asset_directory


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--isaaclab-root',type=Path,default=os.environ.get('ISAACLAB_ROOT'))
    p.add_argument('--report',type=Path,required=True)
    return p


def main():
    p=parser()
    early,_=p.parse_known_args()
    if early.report.exists():p.error('Use a fresh report path')
    verify_checkout(early.isaaclab_root)
    # Keep pxr imports out of Kit's parent startup process.
    import subprocess
    subprocess.run([sys.executable,'-c',
        'from rebar_training.mobile_assets import audit_mobile_assets; import sys; print(audit_mobile_assets(sys.argv[1]))',
        str(asset_directory())],check=True)
    import numpy as np
    import torch
    import gymnasium as gym
    from isaaclab_tasks.utils import add_launcher_args,launch_simulation,setup_preset_cli
    from isaaclab_tasks.utils.hydra import hydra_task_config
    from isaaclab.actuators import ImplicitActuatorCfg
    from .reset import quat_apply
    from .mobile import PrecisionDock,wheel_speeds
    from .registration import register_tasks
    register_tasks();add_launcher_args(p)
    args,remaining=setup_preset_cli(p);sys.argv=[sys.argv[0],*remaining]

    def transport_class():
        from .isaac_env import RebarEnv
        from isaaclab.sensors import ContactSensor,ContactSensorCfg
        class TransportEnv(RebarEnv):
            def _setup_scene(self):
                super()._setup_scene()
                self.tray_load=ContactSensor(ContactSensorCfg(prim_path='/World/envs/env_.*/Robot/tray_link',
                    update_period=0.,history_length=4))
                self.scene.sensors['tray_load']=self.tray_load

            def _reset_idx(self,ids):
                super()._reset_idx(ids)
                ids=torch.arange(self.num_envs,device=self.device) if ids is None else torch.as_tensor(ids,device=self.device)
                # Curriculum initialization only. Never called during the probe.
                pose=torch.tensor([.2927,0.,.715,0.,0.,2**-.5,2**-.5],device=self.device).repeat(len(ids),1)
                pose[:,:3]+=self.scene.env_origins[ids]
                self.bar.write_root_pose_to_sim_index(root_pose=pose,env_ids=ids)
                self.bar.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(ids),6,device=self.device),env_ids=ids)
                q=self.robot.data.default_joint_pos.torch[ids].clone()
                q[:,self.control_ids[:6]]=q.new_tensor((0.,-.35,.6,0.,.35,0.))
                self.robot.write_joint_position_to_sim_index(position=q,env_ids=ids)
                self.targets[ids]=q[:,self.control_ids]

            def _pre_physics_step(self,actions):
                self.actions=actions.clone()

            def _apply_action(self):
                self.robot.set_joint_position_target_index(target=self.targets,joint_ids=self.control_ids)
                self.robot.set_joint_velocity_target_index(target=self.wheel_target,joint_ids=self.wheel_ids)

            def _get_dones(self):
                zeros=torch.zeros(self.num_envs,dtype=torch.bool,device=self.device)
                return zeros,zeros

            def _get_rewards(self):return torch.zeros(self.num_envs,device=self.device)

            def _get_observations(self):return {'policy':torch.zeros(self.num_envs,71,device=self.device)}

        return TransportEnv

    @hydra_task_config('Aubo-Rebar-Grasp-Direct-v0','rsl_rl_cfg_entry_point')
    def run(cfg,_agent):
        cfg.scene.num_envs=1;cfg.episode_length_s=300.;cfg.seed=116042
        cfg.robot.actuators['wheels']=ImplicitActuatorCfg(joint_names_expr=['.*wheel_joint'],
            effort_limit_sim=120.,velocity_limit_sim=3.,stiffness=0.,damping=1000.)
        cfg.robot.init_state.joint_pos.update(dict(zip(ARM_JOINTS,(0.,-.35,.6,0.,.35,0.))))
        rows=[];goals=[(.5,0.,0.),(.5,.5,np.pi/2),(.5,.5,0.)]
        with launch_simulation(cfg,args):
            e=transport_class()(cfg)
            try:
                e.wheel_ids,_=e.robot.find_joints(['left_wheel_joint','right_wheel_joint'],preserve_order=True)
                if e.robot.body_names[0]!='base_footprint':
                    raise RuntimeError('Odometry root is not the chassis: '+e.robot.body_names[0])
                e.wheel_target=torch.zeros(1,2,device=e.device)
                e.reset();stage=0;dock=PrecisionDock(goals[0],.015,.02)
                failed=None;settled_reference=None;max_drift=0.;tray_min=float('inf')
                for step in range(9000):
                    # Robot-only odometry for driving. Steel data below is
                    # strictly an evaluator and never steers the wheels.
                    base=e.robot.data.root_pos_w.torch[0]-e.scene.env_origins[0]
                    q=e.robot.data.root_quat_w.torch[0];qv=q.tolist()
                    yaw=np.arctan2(2*(qv[3]*qv[2]+qv[0]*qv[1]),1-2*(qv[1]**2+qv[2]**2))
                    measured=e.robot.data.root_lin_vel_w.torch[0].tolist()
                    angular=e.robot.data.root_ang_vel_w.torch[0,2].item()
                    command=(0.,0.) if step<90 or stage==len(goals) else dock.step(
                        (*base[:2].tolist(),yaw),(float(np.linalg.norm(measured[:2])),angular),1/30)
                    e.wheel_target[0]=e.wheel_target.new_tensor(wheel_speeds(*command))
                    e.step(torch.zeros(1,7,device=e.device))
                    q=e.robot.data.root_quat_w.torch[0]
                    inverse=q.clone();inverse[:3]*=-1
                    bar=e.bar.data.root_pos_w.torch[0]-e.robot.data.root_pos_w.torch[0]
                    relative=quat_apply(inverse[None],bar[None])[0]
                    axis=quat_apply(inverse[None],quat_apply(e.bar.data.root_quat_w.torch,
                        torch.tensor([[1.,0.,0.]],device=e.device)))[0]
                    load=torch.linalg.vector_norm(e.tray_load.data.net_forces_w.torch,dim=-1).sum().item()
                    if step==90:settled_reference=relative.clone()
                    if step>=90:
                        drift=float(torch.linalg.vector_norm(relative-settled_reference));max_drift=max(max_drift,drift)
                        tray_min=min(tray_min,load)
                        if abs(relative[0]-.2927)>.025 or abs(relative[1])>.035 or abs(relative[2]-.715)>.025 or abs(axis[1])<.95:
                            failed='steel left the physical tray'
                        if e._state().unsafe_force.item()>250:failed='contact force exceeded 250 N'
                    if step%30==0:
                        row=dict(step=step,route_stage=stage,base=base.tolist(),yaw=float(yaw),
                            steel_base_for_evaluation=relative.tolist(),tray_load_N=load,command=command)
                        rows.append(row);print(json.dumps(row),flush=True)
                    if failed:break
                    if step>=90 and stage<len(goals) and dock.ready:
                        stage+=1
                        if stage<len(goals):dock=PrecisionDock(goals[stage],.015,.02)
                        else:finish_step=step
                    if stage==len(goals) and step-finish_step>=60:break
                e.wheel_target.zero_();e.step(torch.zeros(1,7,device=e.device))
                result=dict(run_id=os.environ.get('REBAR_MOBILE_WORKER'),completed=True,
                    success=failed is None and stage==len(goals),failure=failed,
                    mode='isolated_physical_tray_transport_curriculum',camera_only=False,end_to_end=False,
                    steel_initialized_on_tray_at_reset=True,steel_pose_writes_after_reset=False,
                    steel_truth_for_control=False,robot_odometry_source='simulated chassis feedback',
                    navigation='bounded odometry servo; Nav2 not used in this probe',
                    max_steel_drift_m=max_drift,min_tray_contact_N=tray_min if np.isfinite(tray_min) else None,
                    completed_waypoints=stage,trace=rows)
                args.report.parent.mkdir(parents=True,exist_ok=True)
                args.report.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
                print('MOBILE_PROBE',result['success'],failed,flush=True)
            finally:e.close()
    run()


def supervised():
    import subprocess
    args,_=parser().parse_known_args();run_id=uuid.uuid4().hex
    process=subprocess.run([sys.executable,'-m','rebar_training.mobile_check',*sys.argv[1:]],
        env={**os.environ,'REBAR_MOBILE_WORKER':run_id})
    try:report=json.loads(args.report.read_text())
    except (OSError,ValueError):report={}
    if process.returncode or report.get('run_id')!=run_id or not report.get('completed'):
        raise RuntimeError('Mobile probe failed or has no fresh receipt; inspect its log')


if __name__=='__main__':
    main() if os.environ.get('REBAR_MOBILE_WORKER') else supervised()
