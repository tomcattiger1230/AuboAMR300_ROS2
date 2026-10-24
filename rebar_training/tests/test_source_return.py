from types import SimpleNamespace
import numpy as np
import pytest
from rebar_training.vision.source_return import source_return_handoff


def inputs():
    # Last quarter of the measured return, source still has fresh side edges.
    K=np.array([[870.,0,512],[0,870.,307.5],[0,0,1.]])
    camera=SimpleNamespace(timestamp_ns=123,K=K,T_scene_optical=np.eye(4),mono8=np.zeros((615,1024)))
    return dict(valid=True,last_good_ns=123,last_error='',camera=camera,
        center=np.array([0.,.25,1.]),tcp=np.array([0.,0.,1.2]),
        fingers=np.zeros(2),finger_forces=np.zeros(2),visible_edges=66,
        residual_px=1.,return_index=20,history_length=100)


def test_handoff_before_fresh_source_exits_the_image():
    assert source_return_handoff(**inputs())
    x=inputs();x['center']=np.array([0.,0.,1.]);assert not source_return_handoff(**x)


@pytest.mark.parametrize('change',[
    dict(valid=False),dict(last_good_ns=122),dict(last_error='Insufficient bilateral image edges'),
    dict(fingers=np.array([.01,0.])),dict(finger_forces=np.array([.5,0.])),
    dict(tcp=np.array([0.,0.,1.05])),dict(visible_edges=17),dict(residual_px=4.1),
    dict(return_index=90),dict(return_index=0),dict(center=np.array([0.,0.,np.nan]))])
def test_source_handoff_cannot_bypass_vision_load_height_or_return_branch(change):
    x=inputs();x.update(change);assert not source_return_handoff(**x)
