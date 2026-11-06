"""Opt-in machine-mounted fiducial arrival/transfer candidate; no learning.

Initial steel-on-tray placement is reset-only and explicitly not source pickup.
All subsequent steel movement uses the original physical visual skills.
"""
import math
import numpy as np
import torch
from scipy.spatial.transform import Rotation

from .workcell_sim_env import WorkcellVisualEnv,WorkcellControl
from .vision.observation import ObservationMotion,JointObservationMotion,ObserveThenReturn
from .vision.observer import ImageObserver
from .vision.capture import array,optical_transform,synchronize_mounts
from .fiducial_localization import CameraCalibration,RelocalizationSession,inverse,fixture_loading_geometry
from .machine_fiducial import (CharucoFixtureEstimator,attach_board,station_from_board,CAMERA_POSITION_STATION,
                               CAMERA_AIM_BELOW_CENTRE_M)
from .mobile import SettledPrecisionDock,TransportFeedback,TransportGate,wheel_speeds


class MachineFiducialControl(WorkcellControl):
    def reset(self,ids):
        super().reset(ids)
        self.fid_phase=None;self.fid_verified=False;self.fixture_odom=None
        self.fid_events=[];self.fid_rows=[];self.fid_callback=None;self.fid_last_frame=-1
        self.fid_session=None;self.fid_corrections=0;self.fid_observation_done=False
        self.fid_last_error='';self.fid_pose_rows=[];self.fid_estimator=None

    def begin_tray_regrasp(self,observed):
        if not self.fid_verified:
            self.request_machine_observation()
        else:super().begin_tray_regrasp(observed)

    def request_machine_observation(self):
        # Return along the measured clear tray path first. Directly blending
        # two independent IK branches folded the camera into the forearm.
        self.motion.allow_return[:]=True
        self.fid_phase='stow_tray';self.fid_phase_start=self.tick
        self.fid_motion_allowed=False
        self.fid_events.append(dict(tick=self.tick,event='return_tray_view_before_board'))
        self.enter('tray_view_return');self.env.wheel_target.zero_()

    def begin_machine_observation(self):
        e=self.env;station=e.layout.station(e.cfg.workcell_machine,'machine');frame=self.frame()
        camera=frame.point(station.pose_map.point(CAMERA_POSITION_STATION))
        aim=station_from_board()[:3,3].copy();aim[2]-=CAMERA_AIM_BELOW_CENTRE_M
        target=frame.point(station.pose_map.point(tuple(aim)))
        # Robot-only FK/IK; the actual outbound path is replayed on return.
        motion=JointObservationMotion(e,camera,target,(0.,0.,1.))
        self.fid_motion=ObserveThenReturn(e,motion,return_speed_scale=.5)
        self.fid_motion.allow_return[:]=False
        self.fid_phase='observe';self.fid_phase_start=self.tick;self.fid_session=None
        self.fid_pose_rows=[];self.fid_last_frame=-1
        self.fid_motion_allowed=False
        self.fid_events.append(dict(tick=self.tick,event='begin_machine_observation',machine_id=e.cfg.workcell_machine,
            camera_base=list(map(float,camera)),coarse_target_base=list(map(float,target)),
            settled_base_pose=self.frame().world_from_base.tolist(),correction_count=self.fid_corrections))
        self.enter('machine_observe');e.wheel_target.zero_();e.close_fixture[:]=False

    def _snapshot(self):
        e=self.env;synchronize_mounts(e);sensor=e.fiducial_camera
        mount=e.vision_mounts['wrist'];body=e.robot.find_bodies(mount['body'])[0][0]
        from .reset import quat_apply,quat_multiply
        q=e.robot.data.body_quat_w.torch[:,body];p=e.robot.data.body_pos_w.torch[:,body]
        sensor.set_world_poses(p+quat_apply(q,p.new_tensor(mount['position']).reshape(1,3)),
            quat_multiply(q,q.new_tensor(mount['quaternion_xyzw']).reshape(1,4)),convention='ros')
        # Expose at the measured wrist pose before pairing RGB with FK.
        # Reading cached pixels immediately after moving the camera prim pairs
        # the preceding render with the current FK, despite fresh metadata.
        e.sim.render();sensor.update(0.,force_recompute=True);data=sensor.data
        frame=int(array(sensor.frame)[0])
        if frame==self.fid_last_frame:return None
        self.fid_last_frame=frame
        rgb=array(data.output['rgb'])[0,:,:,:3]
        mono=np.round(rgb@np.array([.299,.587,.114])).clip(0,255).astype(np.uint8)
        mount=e.vision_mounts['wrist'];body=e.robot.find_bodies(mount['body'])[0][0]
        world_body=np.asarray(optical_transform(array(e.robot.data.body_pos_w)[0,body],array(e.robot.data.body_quat_w)[0,body]))
        base_body=self.frame().base_from_world@world_body
        fixed=np.asarray(optical_transform(mount['position'],mount['quaternion_xyzw']))
        k=array(data.intrinsic_matrices)[0];now=self.tick/30
        if self.fid_estimator is None:
            c=CameraCalibration('wrist_mono','mounted_charuco_K','usd_fixed_wrist_mount',(mono.shape[1],mono.shape[0]),
                k,(0.,)*5,fixed,'control_base',mount['body'],'wrist_optical','sim_clock')
            self.fid_estimator=CharucoFixtureEstimator(c,e.layout,e.cfg.workcell_machine)
        c=self.fid_estimator.calibration
        kwargs=dict(machine_id=e.cfg.workcell_machine,frame_id=f'fid_{frame}',image_stamp_s=now,fk_stamp_s=now,now_s=now,
            base_from_flange=base_body,camera_id=c.camera_id,calibration_id=c.calibration_id,hand_eye_id=c.hand_eye_id,
            fk_base_frame_id=c.base_frame_id,fk_flange_frame_id=c.flange_frame_id,
            optical_frame_id=c.optical_frame_id,clock_id=c.clock_id)
        return mono,kwargs,dict(tick=self.tick,stage=self.stage,frame=frame,stamp_s=now,
            K=k.tolist(),base_from_mount=base_body.tolist(),fixed_mount=fixed.tolist(),
            base_pose_odom=self.frame().world_from_base.tolist(),camera_shape=list(mono.shape),
            robot_joint_velocity_rad_s=array(e.robot.data.joint_vel)[0,e.control_ids[:6]].tolist(),
            camera_world_pose_for_evaluation=optical_transform(array(data.pos_w)[0],array(data.quat_w_ros)[0]))

    def _stationary_guard(self):
        e=self.env
        self.tray_force=float(torch.linalg.vector_norm(e.tray_load.data.net_forces_w.torch,dim=-1).sum())
        self.packets=self.robot_packets(round(self.tick/30*1e9))
        p=self.packets[0]
        if self.tray_force<5 or np.any(np.abs(p.fingers)>=.003) or np.any(p.finger_forces>=.5):
            raise ValueError('Fiducial observation requires supported tray and open unloaded fingers')
        twist=(float(np.linalg.norm(array(e.robot.data.root_lin_vel_w)[0,:2])),float(array(e.robot.data.root_ang_vel_w)[0,2]))
        return twist

    def _emit(self,image,row):
        self.fid_rows.append(row)
        if self.fid_callback is not None:self.fid_callback(image,row)

    def _accepted_observation(self,gate):
        poses=[np.asarray(v['base_from_fixture']) for v in self.fid_session.frames]
        mean=np.eye(4);mean[:3,3]=np.mean([p[:3,3] for p in poses],axis=0)
        mean[:3,:3]=Rotation.from_matrix(np.stack([p[:3,:3] for p in poses])).mean().as_matrix()
        world_fixture=self.frame().world_from_base@mean
        self.pending_fixture_odom=world_fixture
        self.pending_gate=gate['state']
        self.fid_events.append(dict(tick=self.tick,event='fresh_board_pose_stable',state=gate['state'],
            evidence_frame_ids=gate['evidence_frame_ids'],delta_base_xy_yaw=gate['delta_base_xy_yaw'],
            base_from_fixture=mean.tolist(),fixture_odom=world_fixture.tolist()))
        self.fid_observation_done=True
        self.fid_motion.allow_return[:]=True;self.fid_phase='return';self.fid_phase_start=self.tick
        self.enter('machine_return')

    def after_physics(self):
        if self.fid_phase is None:
            if self.fid_verified:
                # A static registered fixture observation may be held while
                # stationary; base motion invalidates the transfer lease.
                delta=inverse(self.accepted_base_odom)@self.frame().world_from_base
                if np.linalg.norm(delta[:3,3])>.005 or Rotation.from_matrix(delta[:3,:3]).magnitude()>.01:
                    self.failure='Base moved after final fiducial confirmation';self.env.wheel_target.zero_();return
            return super().after_physics()
        e=self.env;self.tick+=1;self.stage_age+=1;e.close_fixture[:]=False
        try:
            self.fid_motion_allowed=False
            twist=self._stationary_guard()
            if self.tick-self.fid_phase_start>60*30:
                raise ValueError('Mounted fiducial phase deadline: '+self.fid_phase)
            loads=torch.linalg.vector_norm(e.contact_sensors['arm_all'].data.net_forces_w.torch,dim=-1)
            if float(loads.max())>=.5:
                raise ValueError('Empty-wrist observation contacted the arm or machine')
            if self.fid_phase in ('stow_tray','observe','return','refresh'):
                e.wheel_target.zero_();self.wheel_command=(0.,0.)
                if max(map(abs,twist))>=.005:return
                self.fid_motion_allowed=True
            if self.fid_phase=='stow_tray':
                if self.motion.ready[0]:self.begin_machine_observation()
            elif self.fid_phase=='observe':
                if self.tick%3==0:
                    snap=self._snapshot()
                    if snap is None:return
                    image,kwargs,row=snap
                    if not self.fid_motion.motion.ready[0]:
                        row.update(accepted=False,reason='observation motion not settled');self._emit(image,row);return
                    if max(abs(v) for v in row['robot_joint_velocity_rad_s'])>.005:
                        row.update(accepted=False,reason='wrist still settling');self._emit(image,row);return
                    try:
                        if self.fid_session is None:
                            profile=e.layout.profiles[e.layout.station(e.cfg.workcell_machine).profile_id]
                            goal=np.eye(4);a=profile.navigation_offset
                            goal[:3,:3]=Rotation.from_euler('z',a.yaw).as_matrix()
                            goal[:3,3]=np.array(a.values()[:3])-np.array(profile.fixture_goal)
                            self.fid_session=RelocalizationSession(self.fid_estimator,
                                session_id='machine_observation_'+str(self.fid_corrections),issued_s=self.tick/30,
                                fixture_from_base_goal=goal)
                        gate=self.fid_session.observe(image,measured_base_twist=twist,base_stopped_confirmed=True,**kwargs)
                        row.update(accepted=True,gate=gate)
                        self.fid_last_error=''
                        self._emit(image,row)
                        if gate['state']!='collecting':self._accepted_observation(gate)
                    except ValueError as exc:
                        row.update(accepted=False,reason=str(exc));self._emit(image,row);self.fid_last_error=str(exc)
                        # A failed measured session is a real hold, no silent
                        # resampling until a favourable three-frame sequence.
                        if self.fid_session is not None and self.fid_session.invalidated:
                            raise ValueError('Fiducial gate rejected: '+str(exc))
            elif self.fid_phase=='return':
                if self.fid_motion.ready[0]:
                    self.fid_phase='refresh';self.fid_phase_start=self.tick;self.refresh_good=0
                    self.motion=ObserveThenReturn(e,JointObservationMotion(e,(-.45,0,1.45),(.2927,0,.715),(0.,1.,0.)))
                    self.motion.allow_return[:]=False
                    self.observers=[ImageObserver(self.detector,plane_height=.715,require_full_support=True)]
                    self.enter('tray_refresh')
            elif self.fid_phase in ('refresh','correct'):
                camera=self.camera_packets(round(self.tick/30*1e9))[0] if self.tick%3==0 else None
                observer=self.observers[0]
                self.valid[0]=observer.step(self.packets[0],camera,survey_allowed=bool(self.motion.motion.ready[0]))
                if camera is not None:self.refresh_good=self.refresh_good+1 if self.valid[0] and not observer.last_error else 0
                self.state=self.measured_state()
                if self.fid_phase=='refresh' and self.refresh_good>=3:
                    if self.pending_gate=='precision_docking_required':
                        if self.fid_corrections>=2:raise ValueError('Fiducial correction count exhausted')
                        profile=e.layout.profiles[e.layout.station(e.cfg.workcell_machine).profile_id]
                        f_goal=np.eye(4);a=profile.navigation_offset;f_goal[:3,:3]=Rotation.from_euler('z',a.yaw).as_matrix()
                        f_goal[:3,3]=np.array(a.values()[:3])-np.array(profile.fixture_goal)
                        target=self.pending_fixture_odom@f_goal
                        from .navigation_map import chassis_clear
                        pose=self.frame().world_from_base;yaw=float(np.arctan2(pose[1,0],pose[0,0]))
                        target_yaw=float(np.arctan2(target[1,0],target[0,0]))
                        for blend in np.linspace(0,1,21):
                            point=(1-blend)*pose[:2,3]+blend*target[:2,3]
                            if not chassis_clear((*point,(1-blend)*yaw+blend*target_yaw),self.navigation_obstacles):
                                raise ValueError('Fiducial correction path violates machine CAD')
                        self.fid_navigation=SettledPrecisionDock((*target[:2,3],target_yaw),max_linear=.03,max_angular=.08)
                        self.gate=TransportGate(max_linear=.03,max_angular=.08)
                        self.fid_phase='correct';self.fid_phase_start=self.tick;self.fid_corrections+=1
                        self.fid_events.append(dict(tick=self.tick,event='start_guarded_correction',
                            target_pose_odom=target.tolist(),arm_returned=True,tray_refreshed=True))
                        self.enter('fiducial_correct')
                    else:
                        self.fixture_odom=self.pending_fixture_odom;self.fid_verified=True
                        self.accepted_base_odom=self.frame().world_from_base.copy()
                        self.fid_events.append(dict(tick=self.tick,event='final_pose_confirmed_after_return',
                            corrections=self.fid_corrections,fresh_tray_frames=self.refresh_good))
                        self.fid_phase=None
                        super().begin_tray_regrasp(observer)
                elif self.fid_phase=='correct':
                    pose=self.frame().world_from_base;yaw=float(np.arctan2(pose[1,0],pose[0,0]))
                    command=self.fid_navigation.step((*pose[:2,3],yaw),twist,1/30)
                    tray=(self.tray_force>=5 and self.valid[0] and np.linalg.norm(observer.position-[.2927,0,.715])<.025
                          and abs(observer.axis[1])>.95)
                    now=self.tick/30;self.gate.command_twist(*command,now)
                    self.wheel_command=self.gate.tick(TransportFeedback(now,observer.last_good_ns*1e-9,now,bool(tray),
                        bool(self.motion.motion.ready[0]),bool(np.all(self.packets[0].fingers<.003))))
                    e.wheel_target[0]=e.wheel_target.new_tensor(wheel_speeds(*self.wheel_command))
                    if self.fid_navigation.ready:
                        e.wheel_target.zero_();self.fid_events.append(dict(tick=self.tick,event='correction_stopped',
                            measured_pose_odom=pose.tolist(),twist=list(twist)))
                        self.request_machine_observation()
        except Exception as exc:
            self.failure=type(exc).__name__+': '+str(exc);e.wheel_target.zero_();self.wheel_command=(0.,0.)

    def action(self,*args,**kwargs):
        if self.fid_phase in ('stow_tray','observe','return'):
            if not self.fid_motion_allowed:
                result=self.env.actions.new_zeros((1,7));result[:,6]=-1;return result
            return (self.motion if self.fid_phase=='stow_tray' else self.fid_motion).actions(self.servo)
        if self.fid_phase in ('refresh','correct'):
            return self.motion.actions(self.servo)
        return super().action(*args,**kwargs)

    def learning_observation(self):
        if self.fid_phase is not None:return [0.]*76,False
        return super().learning_observation()

    def goal(self):
        if self.fixture_odom is None:return super().goal()
        return tuple(self.frame().point(self.fixture_odom[:3,3]))

    def loading_geometry(self):
        if not self.fid_verified or self.fixture_odom is None:
            raise ValueError('Loading is blocked until fresh mounted-board confirmation')
        return fixture_loading_geometry(self.env.layout,self.env.cfg.workcell_machine,
                                       self.frame().base_from_world@self.fixture_odom)


class MachineFiducialEnv(WorkcellVisualEnv):
    def __init__(self,*args,**kwargs):
        self.trial_started=False;self.automatic_terminal_reset_suppressed=False
        super().__init__(*args,**kwargs)

    def _setup_scene(self):
        super()._setup_scene()
        from isaaclab.sensors import Camera,CameraCfg
        import omni.usd
        stage=omni.usd.get_context().get_stage();self.marker_mount_records=[]
        for station in self.layout.stations.values():
            if self.layout.profiles[station.profile_id].kind!='machine':continue
            prefix='/World/envs/env_0/Cell' if station.id==self.cfg.workcell_machine else '/World/envs/env_0/OtherCell_'+station.id
            self.marker_mount_records.append(attach_board(stage,prefix+'/Machine',self.layout,station.id))
        # Two render products from the same physical wrist optical prim. The
        # legacy steel input remains 1024x615; only fiducial RGB is 2048x1230.
        path='/World/envs/env_.*/Robot/mono_camera_link/camera_optical_frame/ros2_camera'
        self.fiducial_camera=Camera(CameraCfg(prim_path=path,spawn=None,width=2048,height=1230,update_period=0.,
            update_latest_camera_pose=True,data_types=['rgb']))
        self.scene.sensors['machine_fiducial_rgb']=self.fiducial_camera

    def _reset_idx(self,ids):
        if self.trial_started and self.mobile_receipts:
            # One-shot candidate: retain the actual terminal scene. The CLI
            # immediately stops; no hidden new bar/root reset at completion.
            self.automatic_terminal_reset_suppressed=True;return
        if self.visual is None:self.visual=MachineFiducialControl(self)
        super()._reset_idx(ids)
        ids=torch.arange(self.num_envs,device=self.device) if ids is None else torch.as_tensor(ids,device=self.device)
        goal=self.layout.navigation_goal(self.cfg.workcell_machine)
        pose=torch.zeros(1,7,device=self.device);pose[0,:3]=pose.new_tensor(goal.point((-.15,0,0)))
        pose[0,3:]=pose.new_tensor([0.,0.,math.sin(goal.yaw/2),math.cos(goal.yaw/2)])
        self.robot.write_root_pose_to_sim_index(root_pose=pose,env_ids=ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1,6,device=self.device),env_ids=ids)
        q=self.robot.data.joint_pos.torch[ids].clone();q[:,self.control_ids[:6]]=q.new_tensor((0.,-.35,.6,0.,.35,-.8))
        self.robot.write_joint_position_to_sim_index(position=q,env_ids=ids);self.targets[ids]=q[:,self.control_ids]
        from .reset import quat_apply,quat_multiply
        bar=pose.new_tensor([[.2927,0.,.715,0.,0.,2**-.5,2**-.5]])
        bar[:,:3]=pose[:,:3]+quat_apply(pose[:,3:],bar[:,:3]);bar[:,3:]=quat_multiply(pose[:,3:],bar[:,3:])
        self.bar.write_root_pose_to_sim_index(root_pose=bar,env_ids=ids)
        self.bar.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1,6,device=self.device),env_ids=ids)
        self.scene.write_data_to_sim();self.sim.forward();self.robot.update(0.)
        v=self.visual;v.initial_navigation=False;v.tray_verified=True
        # Declared coarse navigation bias exercises a measured correction;
        # it is not a physical pose write during the trial.
        coarse=goal.compose(type(goal)(.020,0.,0.,.010))
        v.navigation_goals=[(coarse.x,coarse.y,coarse.yaw)];v.navigation_index=0
        # The Cartesian servo can fold the wrist through the upper arm from
        # this compact arrival posture. Choose an actual USD IK joint branch,
        # retain physical contact guards, and replay its measured path.
        v.motion=ObserveThenReturn(self,JointObservationMotion(self,(-.45,0,1.45),(.2927,0,.715),(0.,1.,0.)))
        v.motion.allow_return[:]=False;v.observers=[ImageObserver(v.detector,plane_height=.715,require_full_support=True)]
        v.valid[:]=False;v.enter('observe_tray');self.wheel_target.zero_()

    def _get_dones(self):
        before=len(self.mobile_receipts)
        result=super()._get_dones()
        if len(self.mobile_receipts)>before:
            from copy import deepcopy
            self.mobile_receipts[-1].update(
                end_to_end=False,initialization='steel supported on physical tray at reset',
                mounted_marker_verified=self.visual.fid_verified,
                fiducial_events=deepcopy(self.visual.fid_events),
                fiducial_corrections=self.visual.fid_corrections,
                fiducial_last_error=self.visual.fid_last_error,
                fixture_odom_estimate=self.visual.fixture_odom.tolist() if self.visual.fixture_odom is not None else None)
        return result
