from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import json
import socket

import pytest

from rebar_training.workcells import WorkcellLayout
from rebar_training.workcell_mission import MissionTask
from rebar_training.policy_compatibility import FrozenStationResidual
from rebar_training.mission_execution import NavigationStatus, TaskExecutor, SeerNavigationPort
from rebar_training.offline_execution_check import make_offline_executor, run_wiring, fixture_status_decoder

ROOT=Path(__file__).parents[1]
CHECKPOINT=ROOT/'artifacts/mobile_curriculum_20261020/source_return_v2_frozen_acceptance_20261022/frozen_policy.pt'


@pytest.fixture(scope='module')
def policy():
    if not CHECKPOINT.exists():pytest.skip('Frozen artifact unavailable')
    return FrozenStationResidual(CHECKPOINT)


@pytest.fixture
def layout():return WorkcellLayout.load(ROOT/'config/multi_workcell_offline.json')


@pytest.mark.parametrize('source',['source_A','source_B'])
@pytest.mark.parametrize('machine',['machine_1','machine_2','machine_3'])
def test_entire_port_wiring_uses_actual_frozen_inference_without_any_socket(layout,policy,monkeypatch,source,machine):
    def forbidden(*a,**k):raise AssertionError('Network access attempted')
    monkeypatch.setattr(socket,'socket',forbidden)
    r=run_wiring(layout,MissionTask('task',source,machine,layout.steel_id),policy)
    assert r['passed'] and len(r['events'])==12 and len(r['skill_starts'])==4
    requests=[c for c in r['transport_calls'] if c['operation']==3051]
    assert len(requests)==2 and requests[0]['payload']['id']==source+'_dock' and requests[1]['payload']['id']==machine+'_dock'
    assert requests[0]['payload']['task_id']!=requests[1]['payload']['task_id']
    assert len(r['frozen_decisions'])==2 and all(x['decision']['active'] for x in r['frozen_decisions'])
    for item in r['recorded_actions']:
        a=item['actions'][0]
        assert a[6]==1 and max(abs(x) for x in a[:6])<=.080001 and not item['applied_to_device']
    policy.verify_unchanged()


def test_ack_is_not_completion_and_duplicate_ticks_do_not_repeat_submissions(layout,policy):
    e,t,d=make_offline_executor(layout,MissionTask('task','source_A','machine_1',layout.steel_id),policy)
    first=e.tick(1)
    assert e.flow.index==0 and e.flow.state=='waiting'
    assert e.tick(1)==first and len(t.calls)==1
    e.tick(1.02)
    assert e.flow.index==0  # raw state 2, even though ACK was ret_code=0
    e.tick(1.04)
    assert e.flow.index==1
    assert len([c for c in t.calls if c['operation']==3051])==1


@pytest.mark.parametrize('fault_name',['lost_tray','stale_camera','wrong_target','wrong_request','paused','failed','unknown_state','wrong_map_pose'])
def test_continuous_faults_cancel_only_the_current_navigation_without_relaunch(layout,policy,fault_name):
    def fault(c,e,t):
        if c['kind']=='navigate_machine' and t.active and t.active['task_id']==c['command_id']:
            if fault_name=='lost_tray':e['tray_load_n']=0
            if fault_name=='stale_camera':e['image_stamp_s']-=1
            if fault_name=='wrong_map_pose' and t.polls>=1:e['base_pose_map']=[-20,-20,0,0]
    def decoder(nav,details):
        s=fixture_status_decoder(nav,details)
        if '3.0' in s.request_id:
            if fault_name=='wrong_target':return replace(s,target_id='another_station')
            if fault_name=='wrong_request':return replace(s,request_id='other_job')
            if fault_name=='paused':return replace(s,state=3)
            if fault_name=='failed':return replace(s,state=5)
            if fault_name=='unknown_state':return replace(s,state=99)
        return s
    e,t,d=make_offline_executor(layout,MissionTask('task','source_A','machine_1',layout.steel_id),policy,fault=fault,decoder=decoder)
    now=1
    for _ in range(60):
        result=e.tick(now)
        if e.flow.state=='blocked':break
        now+=.21 if e.active and e.active['kind']=='observe_source' else .02
    assert e.flow.state=='blocked' and e.flow.index==3
    assert result['preserve_grip'] and result['stop_base']
    assert len([c for c in t.calls if c['operation']==3051])==2
    expected_cancel=0 if fault_name in ('wrong_target','wrong_request','wrong_map_pose','failed') else 1
    assert len([c for c in t.calls if c['operation']==3003])==expected_cancel
    for _ in range(3):now+=.02;e.tick(now)
    assert len([c for c in t.calls if c['operation']==3051])==2
    assert not any('load_machine' in c.get('kind','') for c in e.events)


def test_uncertain_ack_failure_is_cancelled_without_repeating_a_request(layout,policy):
    e,t,d=make_offline_executor(layout,MissionTask('task','source_A','machine_1',layout.steel_id),policy)
    original=t.request
    def failing(port,op,payload):
        if op==3051:
            original(port,op,payload);return {'ret_code':7}
        return original(port,op,payload)
    t.request=failing
    assert e.tick(1)['kind']=='hold'
    e.tick(1.1)
    assert e.flow.index==0 and len([c for c in t.calls if c['operation']==3051])==1
    assert len([c for c in t.calls if c['operation']==3003])==1


def test_cancel_preserves_grip_and_cancels_only_once(layout,policy):
    e,t,d=make_offline_executor(layout,MissionTask('task','source_A','machine_1',layout.steel_id),policy)
    e.tick(1);out=e.cancel(1.01)
    assert out['preserve_grip'] and out['stop_base'] and e.flow.state=='cancelled'
    e.tick(1.02);e.tick(1.03)
    assert len([c for c in t.calls if c['operation']==3003])==1


def test_live_transports_are_rejected_before_any_request(layout):
    class NeverLive:
        offline_only=False
        def request(self,*a):raise AssertionError('Real client reached')
    with pytest.raises(ValueError,match='offline'):
        SeerNavigationPort(layout,NeverLive(),lambda *_:None,fixture_status_decoder)


def test_skill_input_frame_mismatch_stops_before_inference(layout,policy,monkeypatch):
    e,t,d=make_offline_executor(layout,MissionTask('task','source_A','machine_1',layout.steel_id),policy)
    original=d.poll
    def wrong_frame(command,now):
        result=original(command,now)
        if result.sample:result.sample=replace(result.sample,base_pose_map=(90,90,0,0))
        return result
    d.poll=wrong_frame
    calls=[];infer=policy.infer
    monkeypatch.setattr(policy,'infer',lambda packet:(calls.append(packet),infer(packet))[1])
    now=1
    for _ in range(30):
        e.tick(now)
        if e.flow.state=='blocked':break
        now+=.21 if e.active and e.active['kind']=='observe_source' else .02
    assert e.flow.state=='blocked' and e.flow.index==2 and not calls
    assert d.holds and d.holds[-1]['preserve_grip'] and not d.actions


def test_seer_client_adapter_matches_existing_apiresult_contract_without_constructing_a_client(layout):
    from types import SimpleNamespace
    from rebar_training.mission_execution import SeerClientTransportAdapter
    class OfflineClient:
        offline_only=True
        def __init__(self):self.calls=[]
        def request(self,port,operation,payload):
            self.calls.append((port,operation,payload));return SimpleNamespace(ok=True,response={'ret_code':0})
    client=OfflineClient();adapter=SeerClientTransportAdapter(client)
    assert adapter.request(19206,3051,{'source_id':'A','id':'B','task_id':'test'})=={'ret_code':0}
    assert client.calls==[(19206,3051,{'source_id':'A','id':'B','task_id':'test'})]
    client.offline_only=False
    with pytest.raises(ValueError,match='offline'):
        SeerNavigationPort(layout,SeerClientTransportAdapter(client),None,None)


def test_unknown_navigation_owner_is_not_globally_cancelled(layout,policy):
    def other(nav,details):return NavigationStatus('somebody_elses_job','another_target',2)
    e,t,d=make_offline_executor(layout,MissionTask('task','source_A','machine_1',layout.steel_id),policy,decoder=other)
    e.tick(1);e.tick(1.02)
    assert e.flow.state=='blocked'
    assert not [c for c in t.calls if c['operation']==3003]
    assert e.navigation.cancellations[-1]['cancel_state']=='ownership_unconfirmed'


def test_rejected_session_is_never_started_or_held():
    from rebar_training.legacy_skill_driver import LegacyControlDriver
    class NotOfflineSession:
        offline_only=False
        def start(self):raise AssertionError('Rejected session started')
        def hold(self,*a,**kw):raise AssertionError('Rejected session invoked during cleanup')
    class Factory:
        offline_only=True
        def __call__(self,*a):return NotOfflineSession()
    driver=LegacyControlDriver(Factory())
    with pytest.raises(ValueError,match='offline'):
        driver.start({'command_id':'x'}, {})
    driver.hold('x','rejected',preserve_grip=True,stop_base=True)
    assert driver.session is None


def test_explicit_status_mapping_has_no_missing_identity_defaults():
    from rebar_training.mission_execution import SeerStatusFields
    decode=SeerStatusFields(('job','request'),('job','target'))
    assert decode({'task_status':4},{'job':{'request':'ours','target':'M3'}})==NavigationStatus('ours','M3',4)
    with pytest.raises(ValueError):decode({'task_status':4},{})
    with pytest.raises(ValueError):decode({'task_status':True},{'job':{'request':'ours','target':'M3'}})


def test_offline_execution_cli_help_imports_no_simulator_or_actor():
    import subprocess,sys
    code="""
import sys
from rebar_training.offline_execution_check import main
try:main(['--help'])
except SystemExit as ex:assert ex.code==0
assert not any(n=='torch' or n.startswith(('isaac','gymnasium','rclpy','omni')) for n in sys.modules)
"""
    p=subprocess.run([sys.executable,'-c',code],capture_output=True,text=True,timeout=15)
    assert p.returncode==0,p.stderr


def test_source_manipulation_while_chassis_moves_is_held_before_actor(layout,policy,monkeypatch):
    e,t,d=make_offline_executor(layout,MissionTask('task','source_A','machine_1',layout.steel_id),policy)
    original=d.poll
    def moving(command,now):
        result=original(command,now)
        if result.sample:result.sample=replace(result.sample,base_velocity_map=(.02,0,0))
        return result
    d.poll=moving
    calls=[];infer=policy.infer
    monkeypatch.setattr(policy,'infer',lambda packet:(calls.append(packet),infer(packet))[1])
    now=1
    for _ in range(30):
        e.tick(now)
        if e.flow.state=='blocked':break
        now+=.21 if e.active and e.active['kind']=='observe_source' else .02
    assert e.flow.state=='blocked' and e.flow.index==2 and not calls and not d.actions


def test_explicit_observation_retry_wires_a_fresh_session_without_repeating_navigation(layout,policy):
    e,t,d=make_offline_executor(layout,MissionTask('task','source_A','machine_1',layout.steel_id),policy)
    original=d.poll
    def bad_observation(c,now):
        result=original(c,now)
        if c['kind']=='observe_source' and result.completion:
            result.completion['evidence']['full_bar_visible']=False
        return result
    d.poll=bad_observation;now=1
    for _ in range(20):
        e.tick(now)
        if e.flow.state=='blocked':break
        now+=.21 if e.active and e.active['kind']=='observe_source' else .02
    assert e.flow.index==1 and e.flow.state=='blocked'
    old=e.active['command_id'];now+=.01;new=e.retry_observation(now)
    assert new['command_id']!=old and e.flow.state=='waiting'
    d.poll=original
    for _ in range(40):
        now+=.21 if e.active and e.active['kind']=='observe_source' else .02
        e.tick(now)
        if e.flow.state in ('completed','blocked'):break
    assert e.flow.state=='completed'
    assert len([c for c in t.calls if c['operation']==3051])==2
    assert len(set(d.starts))==len(d.starts)
