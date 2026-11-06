"""Bounded Isaac Sim wrist-image fiducial probe. No Cosmos, policy or training.

The initial marker placement is a declared scene construction. Thereafter the
estimator sees rendered Mono8, robot FK and fixed extrinsics only. Simulator
marker/camera truth is kept in a separate evaluation record.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import traceback


def save(path, value):
    tmp = path.with_suffix(path.suffix+'.pending')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n'); tmp.replace(path)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--isaaclab-root', type=Path, default=os.environ.get('ISAACLAB_ROOT'))
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--camera-config', type=Path, required=True)
    p.add_argument('--mode', choices=('render', 'correct'), default='render')
    p.add_argument('--board', action='store_true', help='Three rigid markers with surveyed 140mm centre spacing')
    p.add_argument('--planar', action='store_true', help='Explicit level-floor + surveyed fixture-height constraint; requires --board')
    p.add_argument('--max-steps', type=int, default=1200)
    early, _ = p.parse_known_args()
    if early.output.exists():
        p.error('New output directory required')
    if not 120 <= early.max_steps <= 2400:
        p.error('Bounded steps 120..2400 required')
    if early.planar and not early.board:p.error('--planar requires --board')
    from .launcher import verify_checkout
    verify_checkout(early.isaaclab_root)
    from isaaclab_tasks.utils import add_launcher_args, launch_simulation, setup_preset_cli
    from isaaclab_tasks.utils.hydra import hydra_task_config
    from .registration import register_tasks
    register_tasks(); add_launcher_args(p)
    args, remaining = setup_preset_cli(p); sys.argv = [sys.argv[0], *remaining]
    args.output.mkdir(parents=True, exist_ok=False)
    base = Path(os.environ['REBAR_REPO_ROOT'])
    policy = base/'rebar_training/artifacts/mobile_curriculum_20261020/source_return_v2_frozen_acceptance_20261022/frozen_policy.pt'
    before = digest(policy)
    if before != '393b47f8385a9163e3c9f88b53375923327aa3c80239a3af105e700cf1f93a82':
        raise ValueError('Frozen checkpoint mismatch')
    report = dict(scope='isolated_wrist_fiducial_simulation_probe', mode=args.mode,
                  policy_optimizer_sha_before=before, policy_optimizer_loaded=False,
                  cosmos_used=False, training=False, hardware_or_ros=False, simulation=True,
                  end_to_end_steel_transfer=False, physical_acceptance=False,
                  marker_fixture_relation='explicit synthetic rigid anchor for sensor/local docking probe',
                  script_sha256=digest(Path(__file__)), samples=[], errors=[], started_unix=time.time())
    save(args.output/'status.json', dict(state='starting', pid=os.getpid(), mode=args.mode))

    @hydra_task_config('Aubo-Rebar-Grasp-Direct-v0', 'rsl_rl_cfg_entry_point')
    def run(cfg, _agent):
        import cv2
        import numpy as np
        import torch
        from scipy.spatial.transform import Rotation
        from isaaclab.actuators import ImplicitActuatorCfg
        cfg.scene.num_envs=1; cfg.seed=61106; cfg.episode_length_s=200.
        cfg.camera_capture_config=str(args.camera_config.resolve())
        cfg.robot.actuators['wheels']=ImplicitActuatorCfg(joint_names_expr=['.*wheel_joint'],
            effort_limit_sim=120., velocity_limit_sim=3., stiffness=0., damping=1000.)
        cfg.robot.init_state.joint_pos.update(dict(zip(
            ('shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint'),
            (0.,-.35,.6,0.,.35,-.8))))
        with launch_simulation(cfg, args):
            from .isaac_env import RebarEnv
            from .vision.capture import array, synchronize_mounts, optical_transform
            from .fiducial_localization import (CameraCalibration, MarkerAnchor, WristMarkerEstimator,
                WristMarkerBoardEstimator, PlanarWristBoardEstimator, RelocalizationSession, inverse, rotation_distance)
            from pxr import Gf, Sdf, UsdGeom, UsdShade
            import omni.usd

            class ProbeEnv(RebarEnv):
                def _pre_physics_step(self, actions): self.actions=actions.clone()
                def _apply_action(self):
                    self.robot.set_joint_position_target_index(target=self.targets, joint_ids=self.control_ids)
                    self.robot.set_joint_velocity_target_index(target=self.wheel_target, joint_ids=self.wheel_ids)
                def _get_dones(self):
                    z=torch.zeros(self.num_envs,dtype=torch.bool,device=self.device);return z,z
                def _get_rewards(self): return torch.zeros(self.num_envs,device=self.device)
                def _get_observations(self):return {'policy':torch.zeros(self.num_envs,71,device=self.device)}

            e=None
            try:
                e=ProbeEnv(cfg)
                e.wheel_ids,_=e.robot.find_joints(['left_wheel_joint','right_wheel_joint'],preserve_order=True)
                e.wheel_target=torch.zeros(1,2,device=e.device);e.reset()
                # Explicit probe reset pose; the standard task reset samples a
                # pickup pose and therefore overrides cfg.init_state joints.
                joints=e.robot.data.joint_pos.torch.clone()
                joints[:,e.control_ids[:6]]=joints.new_tensor((0.,-.35,.6,0.,.35,-.8))
                e.robot.write_joint_position_to_sim_index(position=joints)
                e.targets[:]=joints[:,e.control_ids]
                zeros=torch.zeros(1,7,device=e.device)
                for _ in range(90):e.step(zeros)
                camera=e.vision_cameras['wrist'];mount=e.vision_mounts['wrist']
                body=e.robot.find_bodies(mount['body'])[0][0]
                def base_pose():
                    return np.asarray(optical_transform(array(e.robot.data.root_pos_w)[0],array(e.robot.data.root_quat_w)[0]))
                def base_camera_fk():
                    world_body=np.asarray(optical_transform(array(e.robot.data.body_pos_w)[0,body],array(e.robot.data.body_quat_w)[0,body]))
                    return inverse(base_pose())@world_body
                mount_t=np.asarray(optical_transform(mount['position'],mount['quaternion_xyzw']))
                synchronize_mounts(e);e.sim.render();camera.update(1/30,force_recompute=True)
                initial_world_camera=base_pose()@base_camera_fk()@mount_t
                # Surveyed fixture origin and marker are fixed after this setup.
                camera_marker=np.eye(4)
                camera_marker[:3,:3]=Rotation.from_euler('xyz',[np.pi-.25,.15,.03]).as_matrix()
                camera_marker[:3,3]=[.02,-.07 if args.board else 0.,.5]
                world_marker=initial_world_camera@camera_marker
                world_fixture=np.eye(4);world_fixture[:3,3]=world_marker[:3,3]
                fixture_marker=inverse(world_fixture)@world_marker
                stage=omni.usd.get_context().get_stage()
                marker=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100),11,560)
                # Usd geometry uses code cells, avoiding filtering/texture-loading ambiguity.
                # Top-left row maps to marker +Y; black outer square is exactly 160mm.
                board=UsdGeom.Xform.Define(stage,'/World/FiducialBoard')
                board.AddTransformOp().Set(Gf.Matrix4d(world_marker.T.tolist()))
                def material(path, color):
                    mat=UsdShade.Material.Define(stage,path);shader=UsdShade.Shader.Define(stage,path+'/Shader')
                    shader.CreateIdAttr('UsdPreviewSurface')
                    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
                    shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(1.)
                    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');return mat
                black=material('/World/MarkerBlack',(0.,0.,0.));white=material('/World/MarkerWhite',(1.,1.,1.))
                def quad(name,x0,y0,x1,y1,z,mat):
                    mesh=UsdGeom.Mesh.Define(stage,'/World/FiducialBoard/'+name)
                    mesh.CreatePointsAttr([Gf.Vec3f(x0,y0,z),Gf.Vec3f(x1,y0,z),Gf.Vec3f(x1,y1,z),Gf.Vec3f(x0,y1,z)])
                    mesh.CreateFaceVertexCountsAttr([4]);mesh.CreateFaceVertexIndicesAttr([0,1,2,3])
                    mesh.CreateDoubleSidedAttr(True);mesh.CreateSubdivisionSchemeAttr('none')
                    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
                quad('WhiteMargin',-.24 if args.board else -.10,-.10,.24 if args.board else .10,.10,-.0001,white)
                # Seven code/border cells per side in DICT_5X5 + 1 border.
                marker_specs=[(11,-.14,.10),(21,0.,.10),(31,.14,.10)] if args.board else [(11,0.,.16)]
                for marker_id,centre,length in marker_specs:
                    marker=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100),marker_id,560)
                    bits=marker.reshape(7,80,7,80).mean(axis=(1,3))
                    for row in range(7):
                        for col in range(7):
                            x0=centre-length/2+col*length/7;y1=length/2-row*length/7
                            quad(f'Cell_{marker_id}_{row}_{col}',x0,y1-length/7,x0+length/7,y1,0.,white if bits[row,col]>127 else black)
                K=array(camera.data.intrinsic_matrices)[0]
                image_shape=array(camera.data.output['rgb']).shape
                c=CameraCalibration('sim_wrist','sim_K','usd_fixed_mount',(image_shape[2],image_shape[1]),K,(0.,)*5,mount_t,
                    'control_base',mount['body'],'wrist_optical','sim_clock')
                anchors=[]
                for marker_id,centre,length in marker_specs:
                    local=np.eye(4);local[0,3]=centre
                    anchors.append(MarkerAnchor('machine_1','DICT_5X5_100',marker_id,length,fixture_marker@local,'scene_survey'))
                estimator=WristMarkerBoardEstimator(c,anchors) if args.board else WristMarkerEstimator(c,anchors[0])
                report['construction']=dict(world_fixture=world_fixture.tolist(),world_marker=world_marker.tolist(),
                    initial_world_camera=initial_world_camera.tolist(),mount_body=mount['body'],fixed_mount=mount_t.tolist(),
                    initial_optical_from_board_for_construction=camera_marker.tolist(),
                    K=K.tolist(),image_size=list(c.image_size),markers=[dict(id=i,centre_x_m=x,length_m=l) for i,x,l in marker_specs],
                    marker_placement_by_scene_construction=True,
                    board_not_yet_attached_to_original_testing_machine=True,
                    physics_backend='Isaac Sim PhysX',image_source='RTX Camera rgb -> Mono8',
                    estimator_inputs='image, calibrated K, robot link FK, fixed mount, fixture-marker survey only')
                step=0
                initial_base=base_pose()
                desired_fixture_base=inverse(world_fixture)@initial_base
                if args.planar:
                    estimator=PlanarWristBoardEstimator(c,anchors,fixture_from_base_z_m=float(desired_fixture_base[2,3]))
                    report['additional_planar_prior']=dict(level_floor=True,
                        fixed_surveyed_fixture_from_base_height_m=float(desired_fixture_base[2,3]),
                        current_robot_xy_or_yaw_not_used_as_pose_prior=True,
                        not_equivalent_to_unconstrained_6d_estimator=True)
                phase='observe';session=None;dock=None;correction_complete=False
                if args.mode=='correct':
                    # One declared reset perturbation, before the controlled trial.
                    # No root-pose writes are permitted during correction.
                    offset=np.eye(4);offset[:3,:3]=Rotation.from_euler('z',.03).as_matrix()
                    offset[0,3]=.04
                    perturbed=initial_base@offset
                    reset=torch.tensor([*perturbed[:3,3],*Rotation.from_matrix(perturbed[:3,:3]).as_quat()],device=e.device,dtype=torch.float32)[None]
                    e.robot.write_root_pose_to_sim_index(root_pose=reset)
                    e.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1,6,device=e.device))
                    for _ in range(90):e.step(zeros)
                    report['reset_perturbation']=dict(base_relative_xy_yaw=[.04,0.,.03],before_trial_only=True,
                        desired_fixture_from_base=desired_fixture_base.tolist(),empty_base_probe=True,
                        steel_transport_or_arm_clearance_accepted=False,settling_steps_after_reset=90)
                from .mobile import PrecisionDock,wheel_speeds
                for step in range(args.max_steps):
                    e.step(zeros);synchronize_mounts(e)
                    e.sim.render();camera.update(1/30,force_recompute=True)
                    if step%3:continue
                    data=camera.data;rgb=array(data.output['rgb'])[0,:,:,:3]
                    mono=np.round(rgb@np.array([.299,.587,.114])).clip(0,255).astype(np.uint8)
                    stamp=int(e.common_step_counter)/30
                    image_name=f'wrist_{step:04d}.png';cv2.imwrite(str(args.output/image_name),mono)
                    measured=(float(np.linalg.norm(array(e.robot.data.root_lin_vel_w)[0,:2])),float(array(e.robot.data.root_ang_vel_w)[0,2]))
                    row=dict(step=step,image=image_name,image_sha256=digest(args.output/image_name),stamp_s=stamp,
                             phase=phase,measured_twist=measured,
                             robot_input_base_from_mount=base_camera_fk().tolist(),
                             joint_velocity_rad_s=array(e.robot.data.joint_vel)[0,e.control_ids[:6]].tolist(),
                             camera_frame=array(camera.frame).tolist(),
                             camera_world_pose_for_evaluation=optical_transform(array(data.pos_w)[0],array(data.quat_w_ros)[0]))
                    try:
                        obs=estimator.estimate(mono,machine_id='machine_1',frame_id=f'sim_frame_{step}',
                            image_stamp_s=stamp,fk_stamp_s=stamp,now_s=stamp,
                            base_from_flange=base_camera_fk(),camera_id=c.camera_id,calibration_id=c.calibration_id,
                            hand_eye_id=c.hand_eye_id,fk_base_frame_id=c.base_frame_id,fk_flange_frame_id=c.flange_frame_id,
                            optical_frame_id=c.optical_frame_id,clock_id=c.clock_id)
                        actual=base_pose();truth=inverse(actual)@world_fixture
                        estimate=np.asarray(obs['base_from_fixture'])
                        row.update(accepted=True,observation=obs,evaluation_only=dict(
                            base_from_fixture_truth=truth.tolist(),
                            translation_error_m=float(np.linalg.norm(estimate[:3,3]-truth[:3,3])),
                            rotation_error_rad=rotation_distance(estimate,truth)))
                        if args.mode=='correct':
                            pose=np.asarray(obs['fixture_from_base'])
                            planar=(float(pose[0,3]),float(pose[1,3]),float(np.arctan2(pose[1,0],pose[0,0])))
                            if phase in ('observe','verify'):
                                e.wheel_target.zero_()
                                if max(abs(v) for v in measured)<.005:
                                    if session is None:
                                        session=RelocalizationSession(estimator,session_id='session_'+phase,issued_s=stamp,
                                            fixture_from_base_goal=desired_fixture_base)
                                    gate=session.observe(mono,measured_base_twist=measured,base_stopped_confirmed=True,
                                        machine_id='machine_1',frame_id=f'sim_frame_{step}',image_stamp_s=stamp,fk_stamp_s=stamp,
                                        now_s=stamp,base_from_flange=base_camera_fk(),camera_id=c.camera_id,calibration_id=c.calibration_id,
                                        hand_eye_id=c.hand_eye_id,fk_base_frame_id=c.base_frame_id,fk_flange_frame_id=c.flange_frame_id,
                                        optical_frame_id=c.optical_frame_id,clock_id=c.clock_id)
                                    row['session_state']=gate['state']
                                    if gate['state']=='precision_docking_required':
                                        if phase=='verify':raise ValueError('Post-correction fresh observation did not confirm docking')
                                        goal=(float(desired_fixture_base[0,3]),float(desired_fixture_base[1,3]),
                                              float(np.arctan2(desired_fixture_base[1,0],desired_fixture_base[0,0])))
                                        dock=PrecisionDock(goal);phase='correct';session=None
                                    elif gate['state']=='docked_geometry_candidate':
                                        if phase=='verify':correction_complete=True;phase='complete'
                                        else:raise ValueError('Declared 40mm perturbation was not resolved as a correction')
                            elif phase=='correct':
                                command=dock.step(planar,measured,.1)
                                e.wheel_target[0]=e.wheel_target.new_tensor(wheel_speeds(*command))
                                row['wheel_command_linear_angular']=command
                                if dock.ready:e.wheel_target.zero_();phase='verify';session=None
                    except ValueError as exc:
                        row.update(accepted=False,error=str(exc));e.wheel_target.zero_()
                        if args.mode=='correct':
                            report['errors'].append(str(exc));report['samples'].append(row);break
                    report['samples'].append(row)
                    if step%30==0:print(json.dumps(dict(step=step,accepted=row['accepted'],error=row.get('error'))),flush=True)
                    save(args.output/'status.json',dict(state='running',pid=os.getpid(),step=step,accepted=sum(v['accepted'] for v in report['samples'])))
                    if args.mode=='render' and step>=117:break
                    if correction_complete:break
                e.wheel_target.zero_()
                report['completed_steps']=step+1
                report['robot_body_names']=e.robot.body_names
                report['accepted_samples']=sum(v['accepted'] for v in report['samples'])
                report['rendered_probe_passed']=report['accepted_samples']>=3 and all(
                    v['evaluation_only']['translation_error_m']<.015 and v['evaluation_only']['rotation_error_rad']<np.deg2rad(3)
                    for v in report['samples'] if v['accepted'])
                report['base_motion_commanded']=args.mode=='correct' and dock is not None
                report['correction_phase_completed']=correction_complete
                report['root_pose_writes_during_correction']=False
                report['final_base_pose_for_evaluation']=base_pose().tolist()
                report['desired_base_pose_for_evaluation']=initial_base.tolist()
                report['final_base_translation_error_m']=float(np.linalg.norm(base_pose()[:2,3]-initial_base[:2,3]))
                report['final_base_rotation_error_rad']=rotation_distance(base_pose(),initial_base)
                report['correction_controller_pose_source']='fiducial pose from current RTX Mono8 + robot FK; no marker truth'
                stage.GetRootLayer().Export(str(args.output/'scene.usda'))
            except Exception as exc:
                report['errors'].append(repr(exc));(args.output/'error.txt').write_text(traceback.format_exc())
            finally:
                report['policy_optimizer_sha_after']=digest(policy)
                report['policy_optimizer_unchanged']=before==report['policy_optimizer_sha_after']
                report['elapsed_wall_s']=time.time()-report['started_unix']
                save(args.output/'report.json',report)
                save(args.output/'status.json',dict(state='failed' if report['errors'] else 'completed',
                    rendered_probe_passed=report.get('rendered_probe_passed',False),errors=report['errors']))
                if e is not None:e.close()
    run()


if __name__=='__main__':main()
