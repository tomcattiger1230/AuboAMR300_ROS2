"""Experimental camera-only mobile mission; physical validation is staged.

All steel estimates come from RGB. Chassis feedback defines the moving task
frame, and surveyed tray/machine geometry supplies destinations, never steel
poses. This controller is visual servo plus odometry, not a learned full mission.
"""
from dataclasses import replace
import math
import numpy as np
import torch
from isaaclab.sensors import ContactSensor,ContactSensorCfg

from .isaac_env import RebarEnv
from .visual_env import VisualRebarEnv,VisualControl
from .expert import CartesianExpert
from .reset import quat_apply,quat_multiply
from .mobile import TransportGate,TransportFeedback,PrecisionDock,NavigationProgress,wheel_speeds
from .navigation_map import validated_detour
from .spec import asset_directory
from .vision.capture import array
from .vision.frames import BaseFrame
from .vision.observer import ImageObserver
from .vision.observation import ObservationMotion,ObserveThenReturn,JointObservationMotion,PICKUP_SEARCH_VIEWS
from .vision.recovery import TrayGraspRecovery
from .vision.servo import RobotServo
from .vision.blind import blinded_control
from .vision.recovery import tray_reobservation_allowed
from .vision.source_return import source_return_handoff


class BaseServo(RobotServo):
    def cartesian_actions(self,desired,quaternion,grip):
        e=self.env
        q=e.robot.data.root_quat_w.torch
        p=e.robot.data.root_pos_w.torch-e.scene.env_origins
        return CartesianExpert.cartesian_actions(self,p+quat_apply(q,desired),quat_multiply(q,quaternion),grip)


class MobileControl(VisualControl):
    def __init__(self,env):
        if env.num_envs!=1:raise ValueError('Mobile mission currently requires one environment')
        super().__init__(env)
        self.servo=BaseServo(env)
        # Reserve part of the unchanged 3 mm scoring budget for visual error.
        self.logic.alignment_tolerance=.0008
        self.mission_trace=[]
        self.navigation_goals,self.navigation_obstacles=validated_detour(asset_directory(),env.cfg.cell_translation)

    def reset(self,ids):
        super().reset(ids)
        self.motion=ObserveThenReturn(self.env);self.motion.allow_return[:]=False
        self.observers=[ImageObserver(self.detector,fixed_support_prior=True)]
        self.stage='pickup';self.stage_age=0;self.stage_dwell=0
        self.failure=None;self.finished=False;self.tray_verified=False
        self.path={}
        self.gate=TransportGate();self.navigation_index=0
        self.navigation=None;self.mission_trace=[]
        self.navigation_progress=NavigationProgress()
        self.tray_force=0.;self.wheel_command=(0.,0.)
        self.gate_counts={}
        self.reobservations=[];self.view_switch_tick=self.tick
        self.tray_reobservations=[];self.pending_tray_regrasp=False
        self.tray_outage_triggered=False;self.tray_outage_remaining=0
        self.grasp_recovery=TrayGraspRecovery();self.tray_grasp_aborts=[]
        self.grasp_outage_triggered=False
        self.initial_views=[]
        self.source_return_handoffs=[]
        self.initial_view_ready_tick=None
        self.view_search=False;self.view_good_images=0
        self.logic.initial_height[:]=.522
        self.env.wheel_target.zero_()
        self.previous_frame=self.frame().world_from_base

    def frame(self):
        e=self.env
        return BaseFrame(array(e.robot.data.root_pos_w)[0]-array(e.scene.env_origins)[0],
                         array(e.robot.data.root_quat_w)[0])

    def robot_packets(self,timestamp):
        return [self.frame().robot_packet(super().robot_packets(timestamp)[0])]

    def camera_packets(self,timestamp):
        packets=super().camera_packets(timestamp)
        self.latest_cameras=[self.frame().camera_packet(packets[0])]
        return self.latest_cameras

    def enter(self,stage):
        self.mission_trace.append(dict(tick=self.tick,from_stage=self.stage,to_stage=stage))
        self.stage=stage;self.stage_age=self.stage_dwell=0

    def begin_tray_regrasp(self,observed):
        e=self.env
        self.logic.reset(torch.tensor([0],device=e.device))
        self.logic.initial_height[:]=.715;self.route_initialized[:]=False;e.route.stage[:]=0
        self.observers=[ImageObserver(self.detector,plane_height=.715,prefer_full_support=True,
                                     track_support_texture=True)]
        self.observers[0].survey_prior=(observed.position.copy(),observed.axis.copy())
        if self.pending_tray_regrasp:self.tray_reobservations[-1]['reacquired_tick']=self.tick
        self.pending_tray_regrasp=False
        self.enter('pickup_tray')

    def reobserve_tray(self,observed):
        """Observe again only after the measured empty-wrist return has ended."""
        e=self.env
        self.tray_reobservations.append(dict(tick=self.tick,reason=observed.last_error,
            last_good_ns=observed.last_good_ns,return_speed_scale=.5))
        # The known empty-hand outward path is recorded again, then retraced
        # at half speed. The original steel remains supported by real contact.
        motion_class=JointObservationMotion if e.cfg.mobile_joint_observation else ObservationMotion
        self.motion=ObserveThenReturn(e,motion_class(e,(-.45,0,1.45),(.2927,0,.715),(0.,1.,0.)),
                                      return_speed_scale=.5)
        self.motion.allow_return[:]=False
        self.observers=[ImageObserver(self.detector,plane_height=.715,require_full_support=True)]
        self.observers[0].survey_prior=(observed.position.copy(),observed.axis.copy())
        self.valid[:]=False;self.route_initialized[:]=False
        self.pending_tray_regrasp=True
        self.enter('observe_tray')

    def goal(self):
        return tuple(self.frame().point([6.,4.,self.env.task_spec.insert_goal[2]]))

    def move_bar(self,target,quaternion,grip=1.):
        s=self.measured_state()
        target=s.tcp+s.tcp.new_tensor(target)-s.bar
        q=s.tcp.new_tensor(quaternion).reshape(1,4)
        return self.servo.cartesian_actions(target,q,s.tcp.new_tensor([grip]))

    def settled_at(self,target,quaternion,tolerance=.004):
        robot=self.packets[0];o=self.observers[0]
        good=self.valid[0] and np.linalg.norm(o.position-target)<tolerance
        good=good and abs(robot.quaternion_xyzw@quaternion)>math.cos(math.radians(1.5))
        self.stage_dwell=self.stage_dwell+1 if good else 0
        return self.stage_dwell>=9

    def after_physics(self):
        e=self.env;self.tick+=1;self.stage_age+=1
        timestamp=round(self.tick*e.task_spec.control_dt*1e9)
        self.packets=self.robot_packets(timestamp)
        camera=self.camera_packets(timestamp)[0] if self.tick%3==0 else None
        o=self.observers[0]
        if camera is not None:
            if (self.env.cfg.mobile_tray_grasp_camera_outage_frames and not self.grasp_outage_triggered
                    and self.stage=='pickup_tray' and e.route.stage[0]==2 and not o.anchored
                    and np.all(self.packets[0].finger_forces>=15)):
                self.grasp_outage_triggered=True
                self.tray_outage_remaining=self.env.cfg.mobile_tray_grasp_camera_outage_frames
            if (self.env.cfg.mobile_tray_camera_outage_frames and not self.tray_outage_triggered
                    and self.stage=='pickup_tray' and self.stage_age>=300 and o.acquired):
                self.tray_outage_triggered=True
                self.tray_outage_remaining=self.env.cfg.mobile_tray_camera_outage_frames
            if self.tray_outage_remaining:
                camera=replace(camera,mono8=np.zeros_like(camera.mono8))
                self.latest_cameras[0]=camera;self.tray_outage_remaining-=1
        current_frame=self.frame()
        delta=current_frame.base_from_world@self.previous_frame
        transfer=(self.stage=='loading' and e.route.stage[0]>=9 and o.anchored
                  and np.all(self.packets[0].jaw_forces>=.5)
                  and np.sum(self.packets[0].jaw_forces)>=o.machine_load_min)
        if ((self.stage=='pickup' and not o.anchored) or (self.stage=='loading' and (o.released or transfer))):
            o.reframe(delta)
            if self.stage=='pickup' and self.route_initialized[0]:
                for field in ('initial_bar','anchor'):
                    value=getattr(e.route,field)
                    value[0]=value.new_tensor(delta[:3,:3]@array(value)[0]+delta[:3,3])
                from .vision.observation import matrix_quaternion
                rotation=e.route.quaternion.new_tensor(matrix_quaternion(delta[:3,:3])).reshape(1,4)
                e.route.quaternion.copy_(quat_multiply(rotation,e.route.quaternion))
        self.previous_frame=current_frame.world_from_base
        if self.stage=='pickup' and self.motion.motion.ready[0] and self.initial_view_ready_tick is None:
            self.initial_view_ready_tick=self.tick
        ready_wait=(self.initial_view_ready_tick is not None and self.tick-self.initial_view_ready_tick>=30)
        unreachable_wait=(bool(self.initial_views) and self.tick-self.view_switch_tick>=300)
        if (self.stage=='pickup' and not o.acquired and (ready_wait or unreachable_wait)
                and len(self.initial_views)<len(PICKUP_SEARCH_VIEWS)):
            # A closer bar can crop a cap in the original oblique view. Expand
            # the surveyed field of view; no steel pose/GT or partial-end grasp
            # is used. Keep the measured outbound history for safe retracing.
            view,reference=PICKUP_SEARCH_VIEWS[len(self.initial_views)]
            # Roll the image so the rod uses the 1024-pixel horizontal field.
            # A small lateral camera offset avoids the symmetric wrist IK
            # singularity of a pure height increase at y=0.
            self.motion.motion=ObservationMotion(e,view,(-1.15,0.,.522),reference)
            self.view_switch_tick=self.tick
            self.initial_view_ready_tick=None
            self.initial_views.append(dict(tick=self.tick,camera_base=view,reference=reference,reason=o.last_error))
        picking=self.stage in ('pickup','pickup_tray','loading')
        self.valid[0]=o.step(self.packets[0],camera,
            survey_allowed=bool(self.motion.motion.ready[0]),
            grasp_allowed=picking and bool(e.route.stage[0]>=2),
            release_requested=self.stage=='loading' and bool(e.route.stage[0]>=9))
        if (self.stage=='pickup' and e.route.stage[0]==0 and not o.anchored
                and not self.motion.ready[0] and self.motion.motion.ready[0]
                and source_return_handoff(valid=bool(self.valid[0]),last_good_ns=o.last_good_ns,
                    last_error=o.last_error,camera=camera,center=o.initial_position if o.acquired else np.full(3,np.nan),
                    tcp=self.packets[0].tcp,fingers=self.packets[0].fingers,
                    finger_forces=self.packets[0].finger_forces,
                    visible_edges=o.quality.get('visible_edges',0),residual_px=o.quality.get('residual_px',np.inf),
                    return_index=self.motion.index[0],history_length=len(self.motion.history[0]))):
            self.motion.ready[0]=True
            self.source_return_handoffs.append(dict(tick=self.tick,last_good_ns=o.last_good_ns,
                tcp_base=self.packets[0].tcp.tolist(),return_index=self.motion.index[0],
                history_length=len(self.motion.history[0]),stage=self.stage,route=int(e.route.stage[0]),
                source_center=o.initial_position.tolist(),source_clearance_m=float(self.packets[0].tcp[2]-o.initial_position[2]),
                fingers=self.packets[0].fingers.tolist(),finger_forces=self.packets[0].finger_forces.tolist(),
                visible_edges=o.quality.get('visible_edges',0),residual_px=o.quality.get('residual_px'),
                camera_timestamp_ns=camera.timestamp_ns,camera_frame=camera.frame,
                packet_tag='source_return_handoff_tick'+str(self.tick)))
        if self.view_search and camera is not None:
            self.view_good_images=self.view_good_images+1 if self.valid[0] and not o.last_error else 0
            if self.view_good_images>=3:
                # Stop searching at the first useful measured view rather
                # than completing an unnecessary sweep near the forearm.
                p,q=e.control_tcp_task()
                self.motion.motion.target=p.clone();self.motion.motion.quaternion=q.clone()
                self.motion.motion.ready[:]=False;self.motion.motion.dwell[:]=0
                self.view_search=False
                self.reobservations[-1].update(accepted_tick=self.tick,tcp_base=p[0].tolist())
        self.tray_force=float(torch.linalg.vector_norm(e.tray_load.data.net_forces_w.torch,dim=-1).sum())
        self.state=self.measured_state()
        self.wheel_command=(0.,0.)
        was_pending=self.grasp_recovery.pending
        recovery=self.grasp_recovery.step(stage=self.stage,route=int(e.route.stage[0]),
            returned=bool(self.motion.ready[0]),valid=bool(self.valid[0]),anchored=o.anchored,
            timestamp_ns=timestamp,last_good_ns=o.last_good_ns,fingers=self.packets[0].fingers,
            finger_forces=self.packets[0].finger_forces,tray_force=self.tray_force,
            tcp=self.packets[0].tcp,reobservations=len(self.tray_reobservations))
        if recovery:
            if not was_pending and recovery=='open':
                self.tray_grasp_aborts.append(dict(tick=self.tick,reason=o.last_error,
                    tcp_base=self.packets[0].tcp.tolist(),finger_forces=self.packets[0].finger_forces.tolist(),
                    tray_load_N=self.tray_force,last_good_ns=o.last_good_ns,
                    arm_joints=e.robot.data.joint_pos.torch[0,e.control_ids[:6]].tolist()))
            if recovery=='failed':self.failure=self.grasp_recovery.error
            elif recovery=='reobserve':
                self.tray_grasp_aborts[-1].update(open_unloaded_tick=self.tick,
                    open_fingers=self.packets[0].fingers.tolist(),open_forces=self.packets[0].finger_forces.tolist(),
                    supported_tray_load_N=self.tray_force,returned=bool(self.motion.ready[0]),
                    open_arm_joints=e.robot.data.joint_pos.torch[0,e.control_ids[:6]].tolist())
                self.reobserve_tray(o)
                self.tray_reobservations[-1]['trigger']='supported_partial_grasp_abort'
            e.close_fixture[0]=False;e.wheel_target.zero_()
            return
        if tray_reobservation_allowed(stage=self.stage,route=int(e.route.stage[0]),
                returned=bool(self.motion.ready[0]),valid=bool(self.valid[0]),anchored=o.anchored,
                timestamp_ns=timestamp,last_good_ns=o.last_good_ns,fingers=self.packets[0].fingers,
                finger_forces=self.packets[0].finger_forces,tray_force=self.tray_force):
            if len(self.tray_reobservations)>=2:self.failure='tray visual reacquisition attempts exhausted'
            else:
                self.reobserve_tray(o)
                e.close_fixture[0]=False;e.wheel_target.zero_()
                return
        self.motion.allow_return[0]=o.acquired and self.stage in ('pickup','pickup_tray')
        if picking:
            if o.acquired and not self.route_initialized[0]:
                yaw=np.arctan2(o.initial_axis[1],o.initial_axis[0])
                pose=self.state.tcp.new_tensor([[*o.initial_position,0.,0.,np.sin(yaw/2),np.cos(yaw/2)]])
                e.route.reset(torch.tensor([0],device=e.device),pose);self.route_initialized[0]=True
            active=self.valid & self.motion.ready
            self.logic.spec=replace(e.task_spec,insert_goal=self.goal())
            self.logic.advance(self.state,e.actions,active)
            old=e.route.stage.clone();dwell=e.route.dwell.clone();anchor=e.route.anchor.clone();release=e.route.release_dwell.clone()
            grasp_q=(self.state.tcp.new_tensor(self.packets[0].quaternion_xyzw).reshape(1,4)
                     if self.stage in ('pickup','pickup_tray') and bool(active[0]) else None)
            e.route.advance(self.state,self.logic,e.robot.data.joint_pos.torch[:,e.control_ids[:6]],self.goal(),
                            grasp_quaternion=grasp_q)
            if e.route.grasp_alignment_blocked_contact[0]:
                self.failure='grasp orientation drift with pre-contact load'
            e.route.stage[~active]=old[~active];e.route.dwell[~active]=dwell[~active];e.route.anchor[~active]=anchor[~active]
            e.route.release_dwell[~active]=release[~active]
            if old[0]==2 and e.route.stage[0]==3:o.latch_grasp(self.packets[0])
            if self.stage=='pickup' and e.route.stage[0]>=4:
                self.source_above=o.position.copy();self.source_above[2]=.90
                # The outer reset bound can put a 0.9 m vertical lift outside
                # the arm workspace. The bar is already >=120 mm above the
                # source support; retract inward while raising it. Use the
                # observed position, never the reset sampler or steel truth.
                self.source_above[0]=max(self.source_above[0],-1.10)
                self.enter('tray_raise')
            elif self.stage=='pickup_tray' and e.route.stage[0]>=4:
                # A 600 mm rod cannot rotate upright immediately above the
                # 715 mm rack. Gain end clearance before the joint carry path.
                self.tray_lift_target=o.position.copy();self.tray_lift_target[2]=1.20
                self.tray_lift_quaternion=self.packets[0].quaternion_xyzw.copy()
                self.enter('tray_lift_high')
            elif self.stage=='loading' and self.logic.verified[0]>=45:
                self.finished=True
            e.close_fixture[0]=self.stage=='loading' and self.logic.phase[0]==2
        else:e.close_fixture[0]=False
        q=self.state.tcp.new_tensor([0.,1.,0.,0.]).cpu().numpy()
        path={'tray_raise':(getattr(self,'source_above',np.array([-1.15,0,.90])),e.route.quaternion[0].cpu().numpy(),'tray_rotate'),
              'tray_rotate':(getattr(self,'source_above',np.array([-1.15,0,.90])),q,'tray_via_side'),
              'tray_via_side':(np.array([-.90,-.45,.90]),q,'tray_via_front'),
              'tray_via_front':(np.array([.2927,-.45,.90]),q,'tray_above'),
              'tray_above':(np.array([.2927,0.,.90]),q,'tray_down'),
              'tray_down':(np.array([.2927,0.,.717]),q,'tray_release')}
        if hasattr(self,'tray_lift_target'):
            path['tray_lift_high']=(self.tray_lift_target,self.tray_lift_quaternion,'loading')
        self.path=path
        if self.stage in path:
            target,orientation,next_stage=path[self.stage]
            if self.settled_at(target,orientation):
                if next_stage=='tray_release':self.release_tcp=self.state.tcp[0].clone()
                self.enter(next_stage)
        elif self.stage=='tray_release':
            p=self.packets[0]
            supported=self.valid[0] and self.tray_force>=5 and np.all(p.fingers<.003) and np.all(p.finger_forces<.5)
            self.stage_dwell=self.stage_dwell+1 if supported else 0
            if self.stage_dwell>=15:
                # A measured support transition updates only the estimator.
                # Steel physics is never attached, frozen or repositioned.
                o.anchored=o.released=False;o.plane_height=.715
                o.position[2]=.715;o.local_correction[:]=0
                self.enter('tray_retreat')
        elif self.stage=='tray_retreat':
            target=self.release_tcp.clone();target[2]=.90
            if torch.linalg.vector_norm(self.state.tcp[0]-target)<.006 and self.tray_force>=5:
                self.tray_verified=True
                if e.cfg.mobile_until=='tray':self.finished=True
                else:
                    motion_class=JointObservationMotion if e.cfg.mobile_joint_observation else ObservationMotion
                    self.motion=ObserveThenReturn(e,motion_class(e,(-.45,0,1.45),(.2927,0,.715),(0.,1.,0.)))
                    self.motion.allow_return[:]=False
                    self.observers=[ImageObserver(self.detector,plane_height=.715,require_full_support=True)]
                    self.enter('observe_tray')
        elif self.stage=='observe_tray' and o.acquired:
            if self.pending_tray_regrasp or (e.cfg.mobile_start_at_dock and e.cfg.mobile_until=='clamp'):
                # This explicit reset-only skill already starts at the tester.
                # A lateral reset offset must not provoke an in-place 90 deg
                # differential-drive correction in the tight machine dock.
                # The arm uses the measured base pose to transform its goal.
                # Continuous missions still execute their complete docking.
                self.begin_tray_regrasp(o)
            elif e.cfg.mobile_start_at_dock:
                self.navigation_index=3;self.navigation=PrecisionDock(self.navigation_goals[-1]);self.enter('dock')
            else:
                self.navigation=PrecisionDock(self.navigation_goals[0],.015,.02,.15,.2)
                self.enter('navigate')
        elif self.stage in ('navigate','dock'):
            if (not self.valid[0] and timestamp-o.last_good_ns>500_000_000
                    and self.tick-self.view_switch_tick>=120 and len(self.reobservations)<8):
                views=[(-.45,-.03,1.45),(-.45,.03,1.45),(-.45,0.,1.48)]
                view=views[len(self.reobservations)%len(views)]
                # Chassis is already stopped by the stale-image interlock.
                # Move only the empty wrist to expose the cap silhouette.
                self.motion.motion=ObservationMotion(e,view,(.2927,0,.715),(0.,1.,0.))
                self.view_switch_tick=self.tick
                self.view_search=True;self.view_good_images=0
                self.reobservations.append(dict(tick=self.tick,camera_base=view,reason=o.last_error))
            base=self.frame().world_from_base
            pose=(*base[:2,3],float(np.arctan2(base[1,0],base[0,0])))
            speed=array(e.robot.data.root_lin_vel_w)[0];angular=array(e.robot.data.root_ang_vel_w)[0,2]
            command=self.navigation.step(pose,(float(np.linalg.norm(speed[:2])),float(angular)),1/30)
            now=self.tick/30
            tray_valid=(self.tray_force>=5 and np.linalg.norm(o.position-[.2927,0,.715])<.025
                        and abs(o.axis[1])>.95)
            self.gate.command_twist(*command,now)
            self.wheel_command=self.gate.tick(TransportFeedback(now,o.last_good_ns*1e-9,now,bool(tray_valid),
                bool(not self.view_search and self.motion.motion.ready[0] and self.motion.motion.position_error[0]<.025
                     and self.motion.motion.quaternion_error[0]<.0005),bool(np.all(self.packets[0].fingers<.003))))
            self.gate_counts[self.gate.reason]=self.gate_counts.get(self.gate.reason,0)+1
            blocked=self.navigation_progress.step(now,pose,self.wheel_command)
            if blocked:
                self.failure=blocked;self.wheel_command=(0.,0.)
            if self.navigation.ready and self.valid[0] and tray_valid and not self.view_search and self.motion.motion.ready[0]:
                self.navigation_index+=1
                goals=self.navigation_goals+[self.navigation_goals[-1]]
                if self.navigation_index<3:
                    self.navigation=PrecisionDock(goals[self.navigation_index],.015,.02,.15,.2)
                elif self.navigation_index==3:
                    self.navigation=PrecisionDock(goals[3]);self.enter('dock')
                else:
                    self.wheel_command=(0.,0.)
                    if e.cfg.mobile_until=='dock':
                        self.finished=True;self.enter('docked');return
                    self.begin_tray_regrasp(o)
        if self.stage_age>(240 if self.stage in ('navigate','dock') else 120)*30:
            self.failure='stage timeout: '+self.stage
        e.wheel_target[0]=e.wheel_target.new_tensor(wheel_speeds(*self.wheel_command))

    def learning_observation(self):
        """Only camera estimates, surveyed goals and measured robot feedback."""
        from .mobile_residual import PHASES,pack_observation
        e=self.env;o=self.observers[0];s=self.measured_state()
        packet=self.robot_packets(round(self.tick*e.task_spec.control_dt*1e9))[0]
        q=array(e.robot.data.joint_pos)[0,e.control_ids[:6]]
        dq=array(e.robot.data.joint_vel)[0,e.control_ids[:6]]
        phase=np.zeros(len(PHASES));phase[PHASES.index(self.stage)]=1
        route=np.zeros(11);route[int(e.route.stage[0])]=1
        age=(self.tick*e.task_spec.control_dt-o.last_good_ns*1e-9) if o.acquired else 1.
        base=self.frame().world_from_base[:2,3]
        velocity=array(e.robot.data.root_lin_vel_w)[0]
        angular=array(e.robot.data.root_ang_vel_w)[0,2]
        fields=dict(q=q/np.pi,dq=dq/.5,tcp=packet.tcp/2,tcp_quaternion=packet.quaternion_xyzw,
            bar_minus_tcp=array(s.bar-s.tcp)[0],bar_axis=array(s.bar_axis)[0],
            bar_velocity=array(s.linear_velocity)[0]/.1,bar_angular_velocity=array(s.angular_velocity)[0]/.5,
            fingers=packet.fingers/.0285,finger_load=packet.finger_forces/50,
            jaw_load=packet.jaw_forces/50,tray_load=[self.tray_force/25],
            machine_goal_minus_tcp=(np.asarray(self.goal())-packet.tcp)/5,
            image_age=[min(age,1.)],valid=[float(self.valid[0])],phase=phase,route=route,
            base_xy=base/10,base_velocity=[np.linalg.norm(velocity[:2])/.15,angular/.2])
        alignment_hold=self.stage in ('pickup','pickup_tray') and int(e.route.stage[0])==0
        active=not alignment_hold and not self.grasp_recovery.pending and bool(self.valid[0]) and (self.stage in self.path or self.stage in ('tray_release','tray_retreat')
            or self.stage in ('pickup','pickup_tray','loading') and bool(self.motion.ready[0]))
        return pack_observation(fields),active

    def learning_metric(self):
        """Training potential; this never controls phase transitions or physics."""
        e=self.env;s=self.measured_state();key=(self.stage,int(e.route.stage[0]))
        if self.stage in ('pickup','pickup_tray','loading'):
            if e.route.stage[0]==4:
                from .route import CARRY_JOINTS
                q=e.robot.data.joint_pos.torch[0,e.control_ids[:6]]
                return key,float(torch.linalg.vector_norm(q-q.new_tensor(CARRY_JOINTS)))*.05
            desired,_,_=e.route.targets(s,self.goal())
            return key,float(torch.linalg.vector_norm(desired-s.tcp))
        if self.stage in self.path:
            return key,float(np.linalg.norm(np.asarray(self.path[self.stage][0])-array(s.bar)[0]))
        if self.stage=='tray_retreat':
            target=self.release_tcp.clone();target[2]=.9
            return key,float(torch.linalg.vector_norm(target-s.tcp[0]))
        return key,0.

    def action(self,actor=None,*,residual=None,residual_scale=.08):
        e=self.env
        if self.grasp_recovery.pending:
            # Hold all arm joints while opening; no blind retreat or residual.
            result=e.actions.new_zeros((1,7));result[:,6]=-1.
            return result
        if actor is not None:raise ValueError('Use the explicitly contracted bounded residual interface')
        if self.stage in ('pickup','pickup_tray','loading'):
            bootstrap=self.motion.actions(self.servo)
            action=self.servo.route_actions(self.measured_state(),e.route,self.goal())
            result=torch.where((self.valid & self.motion.ready)[:,None],action,bootstrap)
        elif self.stage in ('observe_tray','navigate','dock'):
            result=self.motion.actions(self.servo)
        elif self.stage in self.path:
            target,q,_=self.path[self.stage];result=self.move_bar(target,q)
        else:
            target=self.release_tcp.clone().reshape(1,3)
            if self.stage=='tray_retreat':target[:,2]=.90
            result=self.servo.cartesian_actions(target,target.new_tensor([[0.,1.,0.,0.]]),target.new_tensor([-1.]))
        if residual is not None:
            if not 0<residual_scale<=.15 or residual.shape!=(1,6) or not torch.isfinite(residual).all():
                raise ValueError('Invalid bounded manipulation residual')
            _,active=self.learning_observation()
            if active:result[:,:6]=(result[:,:6]+residual_scale*residual.clamp(-1,1)).clamp(-1,1)
        pause=(not self.valid[0] and (self.stage not in ('pickup','pickup_tray','observe_tray','navigate','dock')
                                    or self.motion.ready[0]))
        self.hold_on_perception_loss(result,torch.tensor([pause],device=e.device,dtype=torch.bool))
        if pause:
            if self.stage in ('pickup','pickup_tray','loading'):
                result[:,6]=1 if 2<=e.route.stage[0]<9 else -1
        return result


class MobileVisualEnv(VisualRebarEnv):
    def __init__(self,cfg,**kwargs):
        self.mobile_receipts=[]
        super().__init__(cfg,**kwargs)
        self.wheel_ids,_=self.robot.find_joints(['left_wheel_joint','right_wheel_joint'],preserve_order=True)
        self.wheel_target=torch.zeros(self.num_envs,2,device=self.device)
        self.mobile_elapsed=0;self.mobile_reward=torch.zeros(self.num_envs,device=self.device)
        from .mobile_residual import CONTRACT
        self.observation_contract=CONTRACT if cfg.mobile_residual_control else 'mobile_visual_servo_v1'
        if self.robot.body_names[0]!='base_footprint':raise RuntimeError('Mobile chassis must be the articulation root')

    def _setup_scene(self):
        super()._setup_scene()
        self.tray_load=ContactSensor(ContactSensorCfg(prim_path='/World/envs/env_.*/Robot/tray_link',
                                                     update_period=0.,history_length=4))
        self.scene.sensors['tray_load']=self.tray_load

    def control_tcp_task(self):
        tcp,q=self._wrist_tcp();rootq=self.robot.data.root_quat_w.torch
        inverse=rootq.clone();inverse[:,:3]*=-1
        return quat_apply(inverse,tcp-self.robot.data.root_pos_w.torch),quat_multiply(inverse,q)

    def _get_observations(self):
        if not self.cfg.mobile_residual_control:return super()._get_observations()
        from .mobile_residual import OBS_SIZE
        if self.visual is None:return {'policy':torch.zeros(self.num_envs,OBS_SIZE,device=self.device)}
        with blinded_control(self):observation,_=self.visual.learning_observation()
        return {'policy':torch.tensor([observation],device=self.device,dtype=torch.float32)}

    def _reset_idx(self,ids):
        self._last_result=None;RebarEnv._reset_idx(self,ids)
        ids=torch.arange(self.num_envs,device=self.device) if ids is None else torch.as_tensor(ids,device=self.device)
        pose=torch.zeros(len(ids),7,device=self.device);pose[:,:3]=self.scene.env_origins[ids];pose[:,6]=1
        if self.cfg.mobile_start_at_dock:pose[:,:2]+=pose.new_tensor([6.3,4.77])
        from .randomization import profile
        bounds=profile(self.cfg.reset_profile)
        if bounds.base_xy_m or bounds.base_yaw_rad:
            noise=torch.rand((len(ids),3),generator=self.sampler.generator,device=self.device)*2-1
            pose[:,:2]+=noise[:,:2]*bounds.base_xy_m
            yaw=noise[:,2]*bounds.base_yaw_rad
            pose[:,5]=torch.sin(yaw/2);pose[:,6]=torch.cos(yaw/2)
        self.robot.write_root_pose_to_sim_index(root_pose=pose,env_ids=ids)
        self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(len(ids),6,device=self.device),env_ids=ids)
        if not self.cfg.mobile_start_on_tray:
            # Reset-only pose change; preserve every sampled perturbation and
            # all self-collision/force guards. The former home wrist angle lets
            # the physical camera casing penetrate wrist1 at some reset draws.
            q=self.robot.data.joint_pos.torch[ids].clone()
            q[:,self.control_ids[5]]+=self.cfg.mobile_home_wrist_offset
            if ((q[:,self.control_ids]<self.lower[ids])|(q[:,self.control_ids]>self.upper[ids])).any():
                raise ValueError('Mobile camera-clearance home pose outside joint limits')
            self.robot.write_joint_position_to_sim_index(position=q,env_ids=ids)
            self.targets[ids]=q[:,self.control_ids]
            self.robot.set_joint_position_target_index(target=self.targets[ids],joint_ids=self.control_ids,env_ids=ids)
        self.scene.write_data_to_sim();self.sim.forward();self.robot.update(0.)
        if self.visual is None:self.visual=MobileControl(self)
        self.visual.reset(ids);self.mobile_elapsed=0
        if self.cfg.mobile_start_on_tray:
            # Explicit curriculum initialization. Continuous mission runs never
            # enter this branch; their steel must be physically placed first.
            q=self.robot.data.joint_pos.torch[ids].clone()
            seed=self.cfg.mobile_initial_joints or (0.,-.35,.6,0.,.35,0.)
            selected=q.new_tensor(seed)
            if not torch.isfinite(selected).all() or ((selected<self.lower[0,:6])|(selected>self.upper[0,:6])).any():
                raise ValueError('Curriculum arm pose outside actual USD joint limits')
            q[:,self.control_ids[:6]]=selected
            self.robot.write_joint_position_to_sim_index(position=q,env_ids=ids)
            self.targets[ids]=q[:,self.control_ids]
            bar=torch.tensor([[.2927,0.,.715,0.,0.,2**-.5,2**-.5]],device=self.device)
            bar[:,:3]=pose[:,:3]+quat_apply(pose[:,3:],bar[:,:3])
            bar[:,3:]=quat_multiply(pose[:,3:],bar[:,3:])
            self.bar.write_root_pose_to_sim_index(root_pose=bar,env_ids=ids)
            self.bar.write_root_velocity_to_sim_index(root_velocity=torch.zeros(1,6,device=self.device),env_ids=ids)
            self.scene.write_data_to_sim();self.sim.forward();self.robot.update(0.)
            v=self.visual
            motion_class=JointObservationMotion if self.cfg.mobile_joint_observation else ObservationMotion
            v.motion=ObserveThenReturn(self,motion_class(self,(-.45,0,1.45),(.2927,0,.715),(0.,1.,0.)))
            v.motion.allow_return[:]=False
            v.observers=[ImageObserver(v.detector,plane_height=.715,require_full_support=True)]
            if self.cfg.mobile_visual_hint_packet:
                from .vision.support_packet import recorded_support_pose
                position,axis,_=recorded_support_pose(self.cfg.mobile_visual_hint_packet,v.detector,.715)
                v.observers[0].survey_prior=(position,axis)
            v.enter('observe_tray')

    def _apply_action(self):
        super()._apply_action()
        self.robot.set_joint_velocity_target_index(target=self.wheel_target,joint_ids=self.wheel_ids)

    def _get_dones(self):
        self.mobile_elapsed+=1
        truth=self._state()
        with blinded_control(self):self.visual.after_physics()
        v=self.visual
        target=truth.bar.new_tensor([6.,4.,self.task_spec.insert_goal[2]])
        error=float(torch.linalg.vector_norm(truth.bar[0]-target))
        actual_handoff=(error<=.003 and abs(float(truth.bar_axis[0,2]))>=math.cos(math.radians(3))
            and bool((truth.jaw_forces[0]>=.5).all()) and bool((truth.fingers[0]<.003).all())
            and float(torch.linalg.vector_norm(truth.tcp[0]-truth.bar[0]))>=.06
            and float(torch.linalg.vector_norm(truth.linear_velocity[0]))<=.04
            and float(torch.linalg.vector_norm(truth.angular_velocity[0]))<=.2)
        frame=v.frame();tray_position=frame.point(array(truth.bar)[0])
        tray_axis=frame.base_from_world[:3,:3]@array(truth.bar_axis)[0]
        actual_tray=(np.linalg.norm(tray_position-[.2927,0,.715])<.015 and abs(tray_axis[1])>.95
            and bool((truth.fingers[0]<.003).all()) and bool((truth.finger_forces[0]<.5).all())
            and float(torch.linalg.vector_norm(truth.tcp[0]-truth.bar[0]))>=.06 and v.tray_force>=5)
        failed=v.failure or ('force exceeded 250 N' if truth.unsafe_force[0]>250 else None)
        if truth.bar[0,2]<.2:failed='steel fell below the support system'
        timeout=self.mobile_elapsed>=self.task_spec.episode_steps
        done=v.finished or bool(failed) or timeout
        if done:
            if self.cfg.mobile_until=='tray':accepted=v.tray_verified and actual_tray
            elif self.cfg.mobile_until=='dock':
                base=frame.world_from_base
                yaw=float(np.arctan2(base[1,0],base[0,0]))
                accepted=actual_tray and np.linalg.norm(base[:2,3]-[6.3,4.77])<.015 and abs(yaw)<.015
            else:accepted=actual_handoff
            success=v.finished and not failed and accepted
            if v.finished and not failed and not accepted:
                failed='independent '+self.cfg.mobile_until+' acceptance failed'
            self.mobile_receipts.append(dict(completed=True,success=bool(success),failure=failed,timeout=bool(timeout),
                until=self.cfg.mobile_until,stage=v.stage,steps=self.mobile_elapsed,tray_verified=v.tray_verified,
                true_position_for_evaluation=truth.bar[0].tolist(),machine_error_m=error,
                mission_transitions=list(v.mission_trace),camera_only=True,neural_policy=False,
                rebar_pose_writes_after_reset=False,actual_handoff=bool(actual_handoff)))
            self.mobile_receipts[-1]['reset_profile']=self.cfg.reset_profile
            self.mobile_receipts[-1]['image_gain']=float(v.image_gain[0])
            self.mobile_receipts[-1]['image_offset']=float(v.image_offset[0])
            self.mobile_receipts[-1].update(actual_tray=bool(actual_tray),true_tray_position_for_evaluation=tray_position.tolist())
            self.mobile_receipts[-1]['transport_interlock_counts']=dict(v.gate_counts)
            self.mobile_receipts[-1]['active_reobservations']=list(v.reobservations)
            self.mobile_receipts[-1]['initial_observation_views']=list(v.initial_views)
            self.mobile_receipts[-1]['support_texture_measurements']=v.observers[0].texture_measurements
            self.mobile_receipts[-1]['tray_visual_reacquisitions']=list(v.tray_reobservations)
            self.mobile_receipts[-1]['diagnostic_tray_outage_triggered']=v.tray_outage_triggered
            self.mobile_receipts[-1]['tray_grasp_aborts']=list(v.tray_grasp_aborts)
            self.mobile_receipts[-1]['diagnostic_grasp_outage_triggered']=v.grasp_outage_triggered
            self.mobile_receipts[-1]['unsafe_force_N']=float(truth.unsafe_force[0])
            self.mobile_receipts[-1]['peak_contact_loads_N']={
                name:dict(zip(self.contact_sensors[name].body_names,loads[0].tolist()))
                for name,loads in self._contact_peaks.items()}
            self.mobile_receipts[-1]['finger_positions']=v.packets[0].fingers.tolist()
            self.mobile_receipts[-1]['raw_finger_forces_N']=v.packets[0].finger_forces.tolist()
            self.mobile_receipts[-1]['source_return_handoffs']=v.source_return_handoffs
            self.mobile_receipts[-1]['tcp_quaternion_xyzw']=v.packets[0].quaternion_xyzw.tolist()
            self.mobile_receipts[-1]['grasp_goal_quaternion_xyzw']=self.route.quaternion[0].tolist()
            self.mobile_receipts[-1]['grasp_alignment_error_deg']=float(self.route.grasp_alignment_error_deg[0])
            self.mobile_receipts[-1]['jaw_forces_N']=truth.jaw_forces[0].tolist()
            self.mobile_receipts[-1]['bar_axis_for_evaluation']=truth.bar_axis[0].tolist()
            self.mobile_receipts[-1]['tcp_clearance_m']=float(torch.linalg.vector_norm(truth.tcp[0]-truth.bar[0]))
            self.mobile_receipts[-1]['steel_linear_speed_m_s']=float(torch.linalg.vector_norm(truth.linear_velocity[0]))
            self.mobile_receipts[-1]['steel_angular_speed_rad_s']=float(torch.linalg.vector_norm(truth.angular_velocity[0]))
            self.mobile_receipts[-1].update(last_perception_error=v.observers[0].last_error,
                estimated_bar=v.state.bar[0].tolist(),base_pose=self.robot.data.root_pos_w.torch[0].tolist(),
                base_quaternion=self.robot.data.root_quat_w.torch[0].tolist(),
                route_stage=int(self.route.stage[0]),paused_steps=int(v.paused_steps[0]),
                terminal_tcp_base=v.state.tcp[0].tolist())
            self.mobile_terminal_packet=(v.latest_cameras[0],v.observers[0].last_probability,
                                         v.observers[0].last_error) if v.latest_cameras else None
            self.mobile_receipts[-1]['arm_joints']=self.robot.data.joint_pos.torch[0,self.control_ids[:6]].tolist()
            sensor=self.contact_sensors['arm_all']
            loads=torch.linalg.vector_norm(sensor.data.net_forces_w.torch[0],dim=-1)
            self.mobile_receipts[-1]['arm_contact_loads_N']=dict(zip(sensor.body_names,loads.tolist()))
        self.mobile_reward[:]=float(v.finished)-float(bool(failed))
        return torch.tensor([done and not timeout],device=self.device),torch.tensor([timeout],device=self.device)

    def _get_rewards(self):return self.mobile_reward
