#!/usr/bin/env python3
"""Original-image cap replay and preemption audit; never a physical retest.

Scoring truth is used only to audit the completed failed run, never passed to
the recovery controller. Saved images are 6 seconds apart; counterfactual arm
or vehicle motion is deliberately not inferred from the old trajectory.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image
from rebar_training.vision.cylinder import resting_pose
from rebar_training.transport_recenter import TransportRecentering,recenter_workspace_clear


def replay(path):
    raw=path.read_bytes();assert hashlib.sha256(raw).hexdigest()=='c8ff4c0212a67d0660138b93622f87dd17edd43d03777c7151664899a0c59bf2'
    report=json.loads(raw);folder=path.with_suffix('');trace={x['step']:x for x in report['trace']}
    center=np.array([.2927,0.,.715]);axis=np.array([0.,1.,0.]);rows=[];guard=TransportRecentering();preemption=None
    # Original CAD includes chassis-height obstacles. Full-height workspace is
    # checked separately by v9 CAD preflight before any native launch.
    for tick in range(3960,10981,180):
        p=folder/('wrist_'+str(tick));packet=json.loads(p.with_suffix('.json').read_text())
        image=np.array(Image.open(p.with_suffix('.png')));probability=np.load(p.with_suffix('.probability.npy'))
        center,axis,quality=resting_pose(image,probability,np.array(packet['K']),np.array(packet['T_base_optical']),
            plane_height=.715,prior_center=center,prior_axis=axis)
        row=trace[tick];position_error=float(np.linalg.norm(center-np.asarray(row['estimated_bar'])))
        item=dict(tick=tick,center=center.tolist(),axis=axis.tolist(),quality=quality,logged_estimate=row['estimated_bar'],
            replay_vs_log_m=position_error,original_guard=row['transport_guard'])
        if preemption is None:
            clear=recenter_workspace_clear(row['base'],report['navigation_obstacles'])
            action=guard.navigation(tick=tick,clearance=clear,arm_stowed=True,
                now_ns=round(row['sim_time_s']*1e9),image_ns=packet['timestamp_ns'],valid=row['valid'],
                position=center,axis=axis,load=row['tray_load_n'],fingers=row['finger_positions'],forces=row['finger_forces'],
                twist=[float(np.linalg.norm(row['robot_twist'][:2])),row['base_angular_velocity_map'][2]],
                full_caps=quality['full_length'] and quality['axial_position_measured'])
            if action:
                preemption=dict(tick=tick,action=action,position=center.tolist(),offset_m=float(np.linalg.norm(center-[.2927,0.,.715])),
                    native_full_height_clearance_pending=True,scope='old saved-frame detection only; original moving feedback cannot prove a counterfactual stop or grasp')
        rows.append(item)
    p=folder/'terminal';packet=json.loads(p.with_suffix('.json').read_text())
    terminal,_,quality=resting_pose(np.array(Image.open(p.with_suffix('.png'))),np.load(p.with_suffix('.probability.npy')),
        np.array(packet['K']),np.array(packet['T_base_optical']),plane_height=.715,prior_center=center,prior_axis=axis)
    receipt=report['results'][0];truth=np.asarray(receipt['true_tray_position_for_evaluation'])
    first_blocked=next(x for x in report['trace'] if x['stage']=='navigate' and x['transport_guard']=='steel not supported by tray')
    assert preemption and preemption['action']=='stop' and preemption['tick']<first_blocked['step']
    return dict(schema='original_workcell_tray_slip_replay_v1',report_sha256=hashlib.sha256(raw).hexdigest(),
        known_failure=receipt['failure'],actual_tray_accepted=receipt['actual_tray'],
        terminal_truth_for_audit_only=truth.tolist(),terminal_replayed_visual=terminal.tolist(),
        terminal_visual_truth_error_m=float(np.linalg.norm(terminal-truth)),
        physical_axial_offset_m=float(truth[1]),terminal_quality=quality,
        replayed_full_cap_frames=len(rows)+1,max_replay_vs_logged_estimate_m=max(x['replay_vs_log_m'] for x in rows),
        preemption=preemption,original_guard_first_step=first_blocked['step'],
        sparse_saved_frame_warning_lead_s=(first_blocked['step']-preemption['tick'])/30,
        conclusion='Real steel slid axially on the tray. Vision and scoring truth agree; the original guard correctly stopped. No bounded physical recenter path existed in v8.',
        cause_limit='Existing logs do not isolate contact-solver creep, friction or drive excitation contributions. Assets/friction are kept unchanged.',
        physical_recovery_success_proven=False,simulation_started=False,training_started=False,new_learning_updates=0,rows=rows)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--report',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=replay(a.report);a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'}))
