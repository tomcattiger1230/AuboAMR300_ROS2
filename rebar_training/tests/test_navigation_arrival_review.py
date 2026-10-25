"""Synthetic receipt mutations must fail the independent measured-arrival review."""
from pathlib import Path
import pytest
from rebar_training.workcells import WorkcellLayout,FROZEN_POLICY_SHA256
from rebar_training.workcell_sim_review import review


def receipt():
    layout=WorkcellLayout.load(Path(__file__).parents[1]/'config/multi_workcell_offline.json')
    goal=layout.machine_geometry('machine_1')['goal'];s=layout.navigation_goal('source_A')
    stages=('tray_raise','tray_rotate','tray_via_side','tray_via_front','tray_above','tray_down','tray_release',
        'tray_retreat','observe_tray','navigate','dock','pickup_tray','tray_lift_high','loading')
    r=dict(success=True,failure=None,timeout=False,source_id='source_A',machine_id='machine_1',end_to_end=True,tray_verified=True,
        rebar_pose_writes_after_reset=False,camera_only=True,true_position_for_evaluation=list(goal),machine_error_m=0.,machine_goal_map=list(goal),
        bar_axis_for_evaluation=[0.,0.,1.],jaw_forces_N=[1.]*4,finger_positions=[0.,0.],raw_finger_forces_N=[0.,0.],tcp_clearance_m=.08,
        steel_linear_speed_m_s=0.,steel_angular_speed_rad_s=0.,unsafe_force_N=1.,arm_contact_loads_N={'arm':0.},peak_contact_loads_N={'arm_all':{'arm':0.}},
        mission_transitions=[dict(to_stage=x) for x in stages],source_navigation=dict(base_pose=[s.x,s.y,s.yaw],linear_speed=0.,yaw_rate=0.),actual_handoff=True,
        navigation_controller_contract='settled_position_then_yaw_v1',navigation_events=[])
    goals=[[0.,0.,0.],[1.,0.,0.]]
    for i,g in enumerate(goals+[goals[-1]]):
        pt,yt=(.015,.02) if i<2 else (.008,.008)
        r['navigation_events'].append(dict(index=i,goal=g,pose=g.copy(),position_tolerance_m=pt,yaw_tolerance_rad=yt,linear_speed=0.,yaw_rate=0.,stopped_dwell_s=.5))
    frozen=dict(sha256=FROZEN_POLICY_SHA256,file_and_tensor_payload_unchanged=True,model_optimizer_tensors=52,counters=dict(updates=466,actor_updates=170,policy_steps=36602))
    return layout,dict(schema='workcell_physical_candidate_v1',layout_sha256=layout.sha256,simulation_started=True,hardware_or_ros_started=False,
        residual_training=False,new_learning_updates=0,optimizer_instantiated=False,frozen_identity_before=frozen,frozen_identity_after=frozen,
        error=None,completed=True,results=[r],source_id='source_A',machine_id='machine_1',navigation_goals=goals)


@pytest.mark.parametrize('mutation',['missing','position','yaw','moving','dwell','threshold','index'])
def test_navigation_review_rejects_missing_unsettled_or_relaxed_arrival(mutation):
    layout,report=receipt();assert review(report,layout)['physical_candidate_passed']
    events=report['results'][0]['navigation_events'];event=events[-1]
    if mutation=='missing':events.pop()
    elif mutation=='position':event['pose'][0]+=.009
    elif mutation=='yaw':event['pose'][2]=.009
    elif mutation=='moving':event['linear_speed']=.009
    elif mutation=='dwell':event['stopped_dwell_s']=.49
    elif mutation=='threshold':event['position_tolerance_m']=.02
    else:event['index']=0
    checked=review(report,layout);assert not checked['physical_candidate_passed'] and checked['errors']
    assert not checked['task_interface_accepted']
