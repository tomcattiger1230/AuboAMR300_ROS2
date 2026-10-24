from dataclasses import replace
import math
import pytest
from rebar_training.mobile import TransportFeedback,TransportGate,PrecisionDock,wheel_speeds
from rebar_training.mobile import NavigationProgress


def test_blocked_navigation_stops_after_sustained_command_but_interlocks_reset_window():
    watchdog=NavigationProgress()
    for i in range(61):
        result=watchdog.step(i/30,(0.,0.,0.),(.15,0.))
    assert result and 'blocked' in result
    assert watchdog.step(2.1,(0.,0.,0.),(0.,0.)) is None
    assert watchdog.step(3.,(0.,0.,0.),(.15,0.)) is None
    for i in range(1,70):
        assert watchdog.step(3+i/30,(i*.003,0.,0.),(.15,0.)) is None


def test_blocked_turn_detects_wrapped_yaw_progress():
    watchdog=NavigationProgress()
    for i in range(100):
        yaw=math.atan2(math.sin(3.+i*.004),math.cos(3.+i*.004))
        assert watchdog.step(i/30,(0.,0.,yaw),(0.,.2)) is None
    watchdog=NavigationProgress()
    for i in range(61):result=watchdog.step(i/30,(0.,0.,0.),(0.,.2))
    assert result and 'angular' in result


def test_transport_stops_immediately_for_camera_outage_and_needs_fresh_feedback():
    gate=TransportGate();gate.command_twist(.15,0.,0.)
    good=TransportFeedback(.01,0.,0.,True,True,True)
    assert gate.tick(good)[0]>0
    assert gate.tick(replace(good,now=.3))==(0.,0.)
    assert gate.reason=='stale camera'
    gate.command_twist(.15,0.,.31)
    assert gate.tick(replace(good,now=.32,image_stamp=.31,tray_stamp=.31))[0]>0
    assert gate.tick(replace(good,now=.33,image_stamp=.33,tray_stamp=.33,tray_supported=False))==(0.,0.)


def test_same_sensor_nanosecond_is_not_rejected_by_float_seconds_roundoff():
    gate=TransportGate();gate.command_twist(.15,0.,.3)
    assert gate.tick(TransportFeedback(.3,.1+.2,.3,True,True,True))[0]>0
    # Real future timestamps still fail closed at nanosecond resolution.
    assert gate.tick(TransportFeedback(.31,.310001,.31,True,True,True))==(0.,0.)
    assert gate.reason=='stale camera'


@pytest.mark.parametrize('field',['arm_stowed','fingers_open','tray_supported'])
def test_transport_interlock_cannot_be_bypassed_by_nav2(field):
    gate=TransportGate();gate.command_twist(1.,1.,0.)
    assert gate.tick(replace(TransportFeedback(.01,0.,0.,True,True,True),**{field:False}))==(0.,0.)


def test_precision_dock_converges_in_a_kinematic_odometry_test():
    # Tests the feedback mathematics only, never a claimed physical/Nav2 run.
    dock=PrecisionDock((.15,.01,.04));pose=[0.,0.,0.];twist=(0.,0.);dt=1/30
    for _ in range(3000):
        twist=dock.step(pose,twist,dt)
        pose[0]+=twist[0]*math.cos(pose[2])*dt
        pose[1]+=twist[0]*math.sin(pose[2])*dt
        pose[2]+=twist[1]*dt
        if dock.ready:break
    assert dock.ready and math.dist(pose[:2],dock.goal[:2])<.008
    assert abs(pose[2]-.04)<.008
    assert wheel_speeds(.1,0.)==pytest.approx((1.,1.))
    assert wheel_speeds(0.,.2)==pytest.approx((-.6,.6))
