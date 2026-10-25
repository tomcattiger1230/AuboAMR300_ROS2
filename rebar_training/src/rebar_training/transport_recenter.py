"""Bounded recovery decisions from measured tray/camera/chassis feedback only.

This module cannot move a robot or read simulator steel state. The caller must
execute its existing physical grasp/place skills and provide their receipts.
"""
import math
import numpy as np


class TransportRecentering:
    warning_m=.008
    corridor_warning_m=.004
    centered_m=.006
    hard_limit_m=.025
    max_attempts=2
    max_recovery_s=110.

    def __init__(self):
        self.phase='idle';self.error=None;self.attempts=[]
        self.warning_frames=0;self.last_image_ns=-1;self.stopped_since=None
        self.verified_frames=0;self.verified_image_ns=-1
        self.deferred_warnings=0

    @property
    def active(self):return self.phase!='idle'

    def fail(self,reason):
        self.error=reason;self.phase='failed'
        if self.attempts:self.attempts[-1]['error']=reason
        return 'failed'

    @staticmethod
    def feedback(*,now_ns,image_ns,valid,position,axis,load,fingers,forces,twist,full_caps):
        p,a,f,force,t=map(lambda x:np.asarray(x,float),(position,axis,fingers,forces,twist))
        finite=(p.shape==(3,) and a.shape==(3,) and f.shape==(2,) and force.shape==(2,) and t.shape==(2,)
            and all(np.isfinite(x).all() for x in (p,a,f,force,t)) and math.isfinite(load))
        fresh=bool(finite and valid and full_caps and 0<=now_ns-image_ns<=200_000_000)
        offset=float(np.linalg.norm(p-[.2927,0.,.715])) if finite else math.inf
        supported=bool(fresh and 5<=load<=250 and offset<.025 and abs(a[1])>.95)
        empty=bool(finite and np.all(np.abs(f)<.003) and np.all((force>=0)&(force<.5)))
        stopped=bool(finite and np.max(np.abs(t))<.008)
        return dict(fresh=fresh,offset=offset,supported=supported,empty=empty,stopped=stopped)

    def navigation(self,*,tick,clearance,arm_stowed,prepare_corridor=False,**values):
        now=values['now_ns'];image=values['image_ns'];f=self.feedback(**values)
        new_image=image!=self.last_image_ns
        if new_image:self.last_image_ns=image
        if self.phase=='failed':return 'failed'
        if self.phase=='idle':
            # Stop on one measured warning; only subsequent independent
            # images and stopped dwell can permit an arm movement.
            threshold=self.corridor_warning_m if prepare_corridor else self.warning_m
            self.warning_frames=1 if new_image and f['supported'] and f['empty'] and f['offset']>=threshold else 0
            if not self.warning_frames:return None
            if not clearance:
                # A warning is not a second hard stop threshold. Keep the
                # original 25mm/load/camera transport interlock in charge in
                # a narrow corridor; never stretch the arm there.
                self.deferred_warnings+=1;return None
            if len(self.attempts)>=self.max_attempts:return self.fail('transport recenter attempts exhausted')
            if not arm_stowed:return self.fail('transport recenter requested with unstowed arm')
            self.attempts.append(dict(requested_tick=int(tick),started_ns=int(now),requested_position=[float(x) for x in values['position']],
                requested_offset_m=f['offset'],request_mode='before_narrow_corridor' if prepare_corridor else 'measured_slip',state='braking'))
            self.phase='braking';self.stopped_since=None
        elif self.phase=='braking' and new_image:
            self.warning_frames=self.warning_frames+1 if f['supported'] else 0
        if self.phase!='braking':return 'stop'
        if now-self.attempts[-1]['started_ns']>5_000_000_000:return self.fail('transport recenter braking/observation timeout')
        if not (f['supported'] and f['empty'] and clearance and arm_stowed):
            return self.fail('transport recenter lost measured support or workspace before grasp')
        self.stopped_since=(self.stopped_since if self.stopped_since is not None else now) if f['stopped'] else None
        if self.stopped_since is None or now-self.stopped_since<500_000_000 or self.warning_frames<3:return 'stop'
        self.phase='manipulating';self.attempts[-1].update(parked_tick=int(tick),parked_twist=[float(x) for x in values['twist']],
            stopped_dwell_s=(now-self.stopped_since)*1e-9,pregrasp_full_frames=self.warning_frames,state='manipulating')
        return 'recenter'

    def grasped(self,*,tick,position,fingers,forces):
        p,f,force=map(lambda x:np.asarray(x,float),(position,fingers,forces))
        if self.phase!='manipulating' or p.shape!=(3,) or f.shape!=(2,) or force.shape!=(2,) or not (
                all(np.isfinite(x).all() for x in (p,f,force)) and np.all((force>=15)&(force<=250)) and np.all(np.abs(f)<=.04)):
            return self.fail('transport recenter requires measured bilateral grasp')
        self.attempts[-1]['grasp']=dict(tick=int(tick),position=[float(x) for x in position],
            fingers=[float(x) for x in fingers],forces=[float(x) for x in forces])

    def released(self,*,tick,load,fingers,forces):
        f,force=map(lambda x:np.asarray(x,float),(fingers,forces))
        if self.phase!='manipulating' or 'grasp' not in self.attempts[-1] or not (
                f.shape==(2,) and force.shape==(2,) and np.isfinite(f).all() and np.isfinite(force).all()
                and 5<=load<=250 and np.all(np.abs(f)<.003) and np.all((force>=0)&(force<.5))):
            return self.fail('transport recenter release lacks support/open unloaded fingers')
        self.phase='verifying';self.verified_frames=0;self.verified_image_ns=-1
        self.attempts[-1]['state']='verifying'
        self.attempts[-1]['release']=dict(tick=int(tick),load=float(load),fingers=[float(x) for x in fingers],forces=[float(x) for x in forces])

    def verify(self,*,tick,**values):
        if self.phase!='verifying':return False
        f=self.feedback(**values)
        if values['now_ns']-self.attempts[-1]['started_ns']>self.max_recovery_s*1e9:
            self.fail('transport recenter overall deadline');return False
        if values['image_ns']!=self.verified_image_ns:
            self.verified_image_ns=values['image_ns']
            self.verified_frames=self.verified_frames+1 if f['supported'] and f['empty'] and f['stopped'] and f['offset']<self.centered_m else 0
        if self.verified_frames<3:return False
        self.attempts[-1].update(verified_tick=int(tick),verified_position=[float(x) for x in values['position']],verified_offset_m=f['offset'],
            verified_load=float(values['load']),verified_twist=[float(x) for x in values['twist']],verified_fresh_frames=self.verified_frames,
            elapsed_s=(values['now_ns']-self.attempts[-1]['started_ns'])*1e-9,state='complete')
        self.phase='idle';self.warning_frames=0;return True


def recenter_workspace_clear(pose,obstacles,radius=1.50):
    """A conservative arm/600mm specimen disk, checked against full-height CAD."""
    if len(pose)<2 or not all(math.isfinite(x) for x in pose) or radius<1.50:return False
    for obstacle in obstacles:
        lo,hi=obstacle['minimum'],obstacle['maximum']
        dx=max(lo[0]-pose[0],0.,pose[0]-hi[0]);dy=max(lo[1]-pose[1],0.,pose[1]-hi[1])
        if math.hypot(dx,dy)<radius:return False
    return True


def approaching_recenter_boundary(pose,goal,obstacles,lookahead=.50):
    """Service the load before losing an arm-safe workspace, using known CAD."""
    delta=np.asarray(goal[:2])-np.asarray(pose[:2]);distance=float(np.linalg.norm(delta))
    if distance<1e-6 or not recenter_workspace_clear(pose,obstacles):return False
    future=np.asarray(pose[:2])+delta/distance*min(lookahead,distance)
    return not recenter_workspace_clear((*future,pose[2]),obstacles)
