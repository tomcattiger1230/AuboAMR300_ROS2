import json
import numpy as np
import pytest
from rebar_training.workcell_sim_check import atomic
from rebar_training.mobile import TransportGate,TransportFeedback
from rebar_training.transport_recenter import TransportRecentering
from test_transport_recenter import feedback,parked


def test_real_numpy_contact_samples_produce_complete_atomic_receipt(tmp_path):
    c=TransportRecentering();ns=parked(c)
    c.grasped(tick=np.int64(30),position=np.array([.2927,-.009,.73],np.float32),
        fingers=np.array([.017,.017],np.float32),forces=np.array([50.,51.],np.float32))
    c.released(tick=np.int64(300),load=np.float32(21),fingers=np.zeros(2,np.float32),forces=np.zeros(2,np.float32))
    for k in range(3):assert c.verify(tick=400+k*3,**feedback(ns+30_000_000_000+k*100_000_000,position=[.2927,.003,.715]))==(k==2)
    # The recovery receipt itself is primitive JSON; the publisher also handles
    # diagnostic NumPy scalars and arrays, retaining strict NaN rejection.
    json.dumps(c.attempts,allow_nan=False)
    p=tmp_path/'result.json';atomic(p,dict(attempts=c.attempts,load=np.float32(21),accepted=np.bool_(True),matrix=np.eye(2,dtype=np.float32)))
    d=json.loads(p.read_text());assert d['accepted'] is True and d['attempts'][0]['state']=='complete'
    before=p.read_bytes()
    with pytest.raises(ValueError):atomic(p,dict(load=np.float32(np.nan)))
    assert p.read_bytes()==before


def test_narrow_corridor_warning_cannot_weaken_original_hard_stop():
    c=TransportRecentering();gate=TransportGate()
    for k,offset in enumerate((.008,.012,.020,.026),1):
        values=feedback(k*100_000_000,position=[.2927,offset,.715]);safe=c.feedback(**values)
        assert c.navigation(tick=k,clearance=False,arm_stowed=True,**values) is None
        t=k/10;gate.command_twist(.15,0.,t)
        command=gate.tick(TransportFeedback(t,t,t,safe['supported'],True,True))
        if offset<.025:assert command[0]>0
        else:assert command==(0.,0.) and gate.reason=='steel not supported by tray'
    assert not c.attempts and not c.active


def test_recovery_packet_survives_observer_replacement_and_renderer_reuse():
    import ast
    from pathlib import Path
    from dataclasses import replace
    from types import SimpleNamespace
    from rebar_training.vision.observer import CameraPacket
    src=Path(__file__).parents[1]/'src/rebar_training/workcell_sim_env.py';tree=ast.parse(src.read_text())
    cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='WorkcellControl')
    method=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='recenter_packet')
    namespace={'replace':replace};exec(compile(ast.Module(body=[method],type_ignores=[]),str(src),'exec'),namespace)
    camera=CameraPacket(np.full((8,8),50,np.uint8),np.eye(3),np.eye(4),100,1)
    observer=SimpleNamespace(last_good_ns=100,position=np.array([.2927,0.,.715]),quality={'full_length':True},
        last_probability=np.ones((8,8),np.float32),last_error='')
    c=SimpleNamespace(observers=[observer],transport_recenter=SimpleNamespace(attempts=[{}]),tick=3,
        latest_cameras=[camera],recenter_packets=[],recenter_raw_packets=[])
    namespace['recenter_packet'](c,'parked');c.observers=[SimpleNamespace(last_probability=None)]
    camera.mono8[:]=0;camera.K[:]=0;observer.last_probability[:]=0
    tag,saved,probability,error=c.recenter_raw_packets[0]
    assert tag=='transport_recenter_1_parked' and saved.timestamp_ns==100 and error==''
    assert saved.mono8.min()==50 and np.array_equal(saved.K,np.eye(3)) and probability.min()==1
