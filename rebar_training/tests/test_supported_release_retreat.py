from types import SimpleNamespace
import numpy as np
import pytest
from rebar_training.vision.observer import ImageObserver,RobotPacket,CameraPacket
import rebar_training.vision.observer as module


def robot(time=1_100_000_000,**changes):
    d=dict(tcp=np.array([.2927,0.,.724]),quaternion_xyzw=np.array([0.,1.,0.,0.]),fingers=np.zeros(2),
        finger_forces=np.zeros(2),jaw_forces=np.zeros(4),timestamp_ns=time);d.update(changes);return RobotPacket(**d)


def observer():
    o=ImageObserver(SimpleNamespace(probabilities=lambda image:np.ones(image.shape)),plane_height=.715,
        prefer_full_support=True,track_support_texture=True)
    o.acquired=True;o.position=np.array([.2927,0.,.715]);o.axis=np.array([0.,1.,0.]);o.local_center=np.zeros(3)
    o.last_good_ns=o.last_tick_ns=1_000_000_000;o.last_error=''
    return o


def test_partial_cap_after_unload_keeps_real_side_edge_freshness_without_texture(monkeypatch):
    def no_caps(*a,**k):raise ValueError('caps occluded by retreating empty wrist')
    def side_edges(image,probability,K,T,p,a,**kwargs):
        return p.copy(),a.copy(),dict(axial_position_measured=False,full_length=False,residual_px=.48,visible_edges=100)
    def bad_texture(*a,**k):raise ValueError('Axial texture registration lacks consistent image evidence')
    monkeypatch.setattr(module,'resting_pose',no_caps);monkeypatch.setattr(module,'constrained_pose',side_edges)
    camera=CameraPacket(np.zeros((16,16),np.uint8),np.eye(3),np.eye(4),1_300_000_000,1)
    old=observer();old.texture.update=bad_texture
    assert not old.step(robot(1_300_000_000),camera)
    fixed=observer();fixed.texture.update=bad_texture;before=fixed.last_good_ns
    fixed.supported_release_retreat(robot(),21.)
    assert fixed.last_good_ns==before and fixed.acquired
    assert fixed.step(robot(1_300_000_000),camera)
    assert fixed.last_good_ns==camera.timestamp_ns and fixed.last_error==''
    assert fixed.quality['axial_position_measured'] is False
    # No new frame still expires under the original 200ms freshness limit.
    assert not fixed.step(robot(1_600_000_000))


@pytest.mark.parametrize('change,load',[
    ({'time':1_300_000_000},21.),({'fingers':np.array([.017,.017])},21.),
    ({'finger_forces':np.array([1.,0.])},21.),({},0.),({},float('nan')),
    ({'finger_forces':np.array([-.1,0.])},21.),({'fingers':np.array([float('nan'),0.])},21.)])
def test_grasped_unloaded_or_stale_release_cannot_switch_observation_mode(change,load):
    o=observer()
    with pytest.raises(ValueError):o.supported_release_retreat(robot(**change),load)
    assert o.track_support_texture and o.prefer_full_support
