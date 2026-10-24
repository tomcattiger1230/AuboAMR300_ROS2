"""Measured-odometry docking and fail-closed transport command arbitration.

Nav2 supplies coarse map commands. This boundary grants wheel motion only while
the camera, physical tray load, arm stow and command lease remain valid. It has
no simulator object handles and cannot read steel truth.
"""
from dataclasses import dataclass
from collections import deque
import math


def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))


def wheel_speeds(linear,angular,radius=.1,track=.6):
    if radius<=0 or track<=0 or not all(math.isfinite(x) for x in (linear,angular,radius,track)):
        raise ValueError('Finite twist and positive wheel geometry required')
    return ((linear-.5*track*angular)/radius,(linear+.5*track*angular)/radius)


@dataclass(frozen=True)
class TransportFeedback:
    now: float
    image_stamp: float
    tray_stamp: float
    tray_supported: bool
    arm_stowed: bool
    fingers_open: bool
    emergency_stop: bool=False


class TransportGate:
    def __init__(self,max_linear=.15,max_angular=.2,command_timeout=.5,sensor_timeout=.2):
        self.max_linear,self.max_angular=max_linear,max_angular
        self.command_timeout,self.sensor_timeout=command_timeout,sensor_timeout
        self.command=(0.,0.);self.command_stamp=-math.inf
        self.previous=(0.,0.);self.last_tick=None;self.reason='no command'

    def command_twist(self,linear,angular,stamp):
        if not all(math.isfinite(v) for v in (linear,angular,stamp)):
            self.command=(0.,0.);self.command_stamp=-math.inf
            raise ValueError('Non-finite navigation command')
        if stamp<self.command_stamp:
            raise ValueError('Out-of-order navigation command')
        self.command=(linear,angular);self.command_stamp=stamp

    def tick(self,feedback):
        f=feedback
        if not math.isfinite(f.now) or (self.last_tick is not None and f.now<=self.last_tick):
            self.previous=(0.,0.)
            raise ValueError('Transport clock must advance monotonically')
        dt=min(.1,f.now-self.last_tick) if self.last_tick is not None else 1/30
        self.last_tick=f.now
        # Producers use integer sensor nanoseconds. Converting those to
        # seconds and comparing floats made a same-time frame look 1 ulp
        # into the future, repeatedly stopping an otherwise valid drive.
        def fresh(stamp,limit):
            return math.isfinite(stamp) and 0<=round(f.now*1e9)-round(stamp*1e9)<=round(limit*1e9)
        conditions=(
            (f.emergency_stop,'emergency stop'),
            (not fresh(self.command_stamp,self.command_timeout),'expired command'),
            (not fresh(f.image_stamp,self.sensor_timeout),'stale camera'),
            (not fresh(f.tray_stamp,self.sensor_timeout),'stale tray load'),
            (not f.tray_supported,'steel not supported by tray'),
            (not f.arm_stowed,'arm outside transport pose'),
            (not f.fingers_open,'gripper still closed'))
        self.reason=next((reason for stop,reason in conditions if stop),'enabled')
        if self.reason!='enabled':
            self.previous=(0.,0.)
            return self.previous
        wanted=(max(-self.max_linear,min(self.max_linear,self.command[0])),
                max(-self.max_angular,min(self.max_angular,self.command[1])))
        # Limit acceleration while supported; fault stops bypass this ramp.
        self.previous=tuple(old+max(-a*dt,min(a*dt,new-old))
                            for old,new,a in zip(self.previous,wanted,(.08,.2)))
        return self.previous


class PrecisionDock:
    """Final odometry servo after coarse Nav2 navigation, with settled dwell."""
    def __init__(self,goal,position_tolerance=.008,yaw_tolerance=.008,max_linear=.04,max_angular=.12):
        self.goal=tuple(goal);self.position_tolerance=position_tolerance
        self.yaw_tolerance=yaw_tolerance;self.stable_seconds=0.;self.ready=False
        self.max_linear,self.max_angular=max_linear,max_angular

    def step(self,pose,measured_twist,dt):
        if dt<=0 or not all(math.isfinite(x) for x in (*pose,*measured_twist,dt)):
            raise ValueError('Finite measured odometry and positive dt required')
        dx,dy=self.goal[0]-pose[0],self.goal[1]-pose[1]
        distance=math.hypot(dx,dy);yaw=wrap(self.goal[2]-pose[2])
        good=distance<self.position_tolerance and abs(yaw)<self.yaw_tolerance
        stopped=max(abs(x) for x in measured_twist)<.008
        self.stable_seconds=self.stable_seconds+dt if good and stopped else 0.
        self.ready=self.stable_seconds>=.5
        if good:return (0.,0.)
        if distance<self.position_tolerance:
            return (0.,max(-.10,min(.10,1.5*yaw)))
        heading=wrap(math.atan2(dy,dx)-pose[2])
        direction=1.
        if abs(heading)>math.pi/2:
            heading=wrap(heading-math.copysign(math.pi,heading));direction=-1.
        linear=direction*min(self.max_linear,.7*distance)*max(0.,math.cos(heading))
        if abs(heading)>.35:linear=0.
        return linear,max(-self.max_angular,min(self.max_angular,2*heading))


class NavigationProgress:
    """Detect sustained blocked driving; camera-interlocked stops reset it."""
    def __init__(self,seconds=2.):
        self.seconds=seconds;self.history=deque();self.mode=None

    def step(self,now,pose,command):
        mode='linear' if abs(command[0])>=.04 else 'angular' if abs(command[1])>=.05 else None
        if mode is None or mode!=self.mode:
            self.history.clear();self.mode=mode
        if mode is None:return None
        self.history.append((now,tuple(pose)))
        while len(self.history)>1 and self.history[1][0]<=now-self.seconds:self.history.popleft()
        if now-self.history[0][0]<self.seconds:return None
        old=self.history[0][1]
        progress=math.hypot(pose[0]-old[0],pose[1]-old[1]) if mode=='linear' else abs(wrap(pose[2]-old[2]))
        if progress<(.025 if mode=='linear' else .03):return 'navigation blocked: '+mode+' command without odometry progress'
        return None
