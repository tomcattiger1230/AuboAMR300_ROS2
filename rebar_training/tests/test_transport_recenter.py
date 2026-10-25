import math
import pytest
from rebar_training.transport_recenter import TransportRecentering,recenter_workspace_clear


def feedback(ns,**changes):
    return dict(dict(now_ns=ns,image_ns=ns,valid=True,position=[.2927,-.009,.715],axis=[0.,1.,0.],load=21.,
        fingers=[0.,0.],forces=[0.,0.],twist=[0.,0.],full_caps=True),**changes)


def parked(c):
    for k in range(9):
        ns=(k+1)*100_000_000
        result=c.navigation(tick=k*3,clearance=True,arm_stowed=True,**feedback(ns))
        if result=='recenter':return ns
    raise AssertionError('did not acquire stationary recenter')


def complete(c,offset=.003):
    ns=parked(c);c.grasped(tick=30,position=[.2927,-.009,.73],fingers=[.017,.017],forces=[50.,51.])
    c.released(tick=300,load=21.,fingers=[0.,0.],forces=[0.,0.])
    for k in range(3):
        result=c.verify(tick=400+k*3,**feedback(ns+30_000_000_000+k*100_000_000,position=[.2927,offset,.715]))
    return result


def test_measured_stop_grasp_unload_and_fresh_centered_frames_are_required():
    c=TransportRecentering();assert complete(c)
    a=c.attempts[0];assert a['state']=='complete' and a['verified_offset_m']<.006
    assert a['stopped_dwell_s']>=.5 and c.phase=='idle'
    assert complete(c)
    for k in range(3):result=c.navigation(tick=1000+k,clearance=True,arm_stowed=True,**feedback(100_000_000_000+k*100_000_000))
    assert result=='failed' and 'exhausted' in c.error


def test_driving_never_grants_arm_motion_and_repeated_image_is_not_evidence():
    c=TransportRecentering()
    for k in range(20):
        result=c.navigation(tick=k,clearance=True,arm_stowed=True,**feedback(1_000_000_000+k*33_333_333,twist=[.15,0.]))
        assert result in (None,'stop')
    c=TransportRecentering()
    for k in range(6):
        assert c.navigation(tick=k,clearance=True,arm_stowed=True,**feedback(1_000_000_000+k*20_000_000,image_ns=1_000_000_000))=='stop'
    assert c.phase=='braking' and c.warning_frames==1


@pytest.mark.parametrize('change',[dict(valid=False),dict(full_caps=False),dict(load=0.),dict(load=math.nan),
    dict(position=[math.nan,0.,.715]),dict(position=[.2927,-.026,.715]),dict(axis=[0.,.7,.7]),
    dict(fingers=[.017,.017]),dict(forces=[1.,1.]),dict(image_ns=-1)])
def test_missing_or_unsafe_measurements_never_request_manipulation(change):
    c=TransportRecentering()
    for k in range(5):
        assert c.navigation(tick=k,clearance=True,arm_stowed=True,**feedback((k+1)*100_000_000,**change)) is None
    assert not c.attempts


def test_lost_support_after_request_and_missing_workspace_fail_closed():
    c=TransportRecentering()
    for k in range(3):x=c.navigation(tick=k,clearance=False,arm_stowed=True,**feedback((k+1)*100_000_000))
    assert x=='failed' and 'workspace' in c.error
    c=TransportRecentering()
    for k in range(3):c.navigation(tick=k,clearance=True,arm_stowed=True,**feedback((k+1)*100_000_000,twist=[.1,0.]))
    assert c.navigation(tick=4,clearance=True,arm_stowed=True,**feedback(400_000_000,load=0.))=='failed'


def test_unverified_centering_or_timeout_never_resumes_navigation():
    c=TransportRecentering();assert not complete(c,offset=.009)
    assert not c.verify(tick=5000,**feedback(200_000_000_000,position=[.2927,0.,.715]))
    assert c.phase=='failed'


@pytest.mark.parametrize('forces',[[0.,0.],[50.],[50.,math.nan],[251.,30.]])
def test_missing_bilateral_grasp_never_counts_recenter(forces):
    c=TransportRecentering();parked(c)
    assert c.grasped(tick=30,position=[.2927,0.,.74],fingers=[.017,.017],forces=forces)=='failed'


def test_full_height_cad_recovery_disk_rejects_near_obstacle():
    o=[dict(minimum=[1.,-.1,1.2],maximum=[1.1,.1,1.5])]
    assert not recenter_workspace_clear((0.,0.,0.),o)
    assert recenter_workspace_clear((-1.,0.,0.),o)
    assert not recenter_workspace_clear((-1.,0.,0.),o,radius=.1)
