"""Synthetic receipt mutations and the actual inference condition, no simulator."""
import ast
from pathlib import Path
from types import SimpleNamespace
import pytest
from test_navigation_arrival_review import receipt
from rebar_training.workcell_sim_review import review


def report():
    layout,r=receipt();r.update(transport_recenter_enabled=True,transport_recenter_nominal_only=True,recenter_policy_calls=0,
        recenter_workspace_height_m=2.,recenter_workspace_obstacles=[dict(minimum=[10.,10.,1.],maximum=[11.,11.,2.])])
    r['results'][0].update(transport_recenter_packets=[dict(packet_tag=p,phase=p,camera_timestamp_ns=1,quality=dict(full_length=True,axial_position_measured=True)) for p in ('parked','grasp','release','verified')],navigation_ticks_by_stage=dict(navigate=3000,dock=80),transport_recenter_attempts=[dict(
        state='complete',requested_offset_m=.009,requested_position=[.2927,.009,.715],verified_offset_m=.003,verified_position=[.2927,.003,.715],
        requested_pose=[0.,0.,0.],parked_pose=[0.,0.,0.],workspace_radius_m=1.5,stopped_dwell_s=.533,parked_twist=[0.,0.],pregrasp_full_frames=3,
        requested_tick=1,parked_tick=20,grasp=dict(tick=30,forces=[50.,50.]),release=dict(tick=80,load=21.,fingers=[0.,0.],forces=[0.,0.]),
        verified_tick=100,elapsed_s=50.,verified_fresh_frames=3,verified_load=21.,verified_twist=[0.,0.])])
    r['recenter_packet_files']={p:dict(timestamp_ns=1,image_sha256='a'*64,probability_sha256='b'*64,packet_sha256='c'*64) for p in ('parked','grasp','release','verified')}
    return layout,r


@pytest.mark.parametrize('mutation',['grasp','release','frames','deadline','position','workspace','navigation_budget','policy'])
def test_independent_review_rejects_unsupported_recovery_claim(mutation):
    layout,r=report();assert review(r,layout)['physical_candidate_passed']
    a=r['results'][0]['transport_recenter_attempts'][0]
    if mutation=='grasp':a['grasp']['forces']=[0.,50.]
    elif mutation=='release':a['release']['forces']=[1.,0.]
    elif mutation=='frames':a['verified_fresh_frames']=2
    elif mutation=='deadline':a['elapsed_s']=111.
    elif mutation=='position':a['verified_position'][1]=.01
    elif mutation=='workspace':a['parked_pose']=[10.,10.,0.]
    elif mutation=='navigation_budget':r['results'][0]['navigation_ticks_by_stage']['navigate']=7201
    else:r['recenter_policy_calls']=1
    assert not review(r,layout)['physical_candidate_passed']


def test_actual_inference_condition_bypasses_arbitrary_transit_recenter_pose():
    src=Path(__file__).parents[1]/'src/rebar_training/workcell_sim_check.py';tree=ast.parse(src.read_text())
    condition=next(n.test for n in ast.walk(tree) if isinstance(n,ast.If)
        and 'v.transport_resume is None' in ast.unparse(n.test))
    code=compile(ast.Expression(condition),str(src),'eval')
    # No pose rebasing or adapter call is even reached while recentering.
    assert not eval(code,{'v':SimpleNamespace(transport_resume={'index':1})})
    v=SimpleNamespace(transport_resume=None,initial_navigation=False,stage='pickup_tray',valid=[True],tick=30,
        observers=[SimpleNamespace(last_good_ns=1_000_000_000)])
    assert eval(code,{'v':v,'SUPPORTED_ROUTES':{'pickup_tray':(0,1,2,3)}})
