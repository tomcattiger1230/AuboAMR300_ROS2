#!/usr/bin/env python3
"""Replay recorded 1 Hz command branches, then separate synthetic stress tests.

This does not replay Isaac physics or claim counterfactual physical success.
No simulator, actor, checkpoint loader or optimizer is imported.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from rebar_training.mobile import PrecisionDock,SettledPrecisionDock,TransportGate,TransportFeedback,wrap
from rebar_training.workcell_physics import plan_chassis_path
from rebar_training.workcells import WorkcellLayout
from rebar_training.navigation_map import chassis_clear


def integrate(goals,start,controller=SettledPrecisionDock,drift=8e-5,lag=.3,deadline=240):
    pose=list(start);twist=[0.,0.];gate=TransportGate();index=0;events=[]
    def make():
        return controller(goals[index],.015,.02,.15,.2) if index<len(goals)-1 else controller(goals[index])
    dock=make()
    for k in range(round(deadline*30)):
        t=(k+1)/30;command=dock.step(pose,twist,1/30);gate.command_twist(*command,t)
        out=gate.tick(TransportFeedback(t,t,t,True,True,True))
        twist=[a+(b-a)*lag for a,b in zip(twist,out)]
        pose[0]+=twist[0]*math.cos(pose[2])/30
        pose[1]+=twist[0]*math.sin(pose[2])/30+drift/30;pose[2]+=twist[1]/30
        if dock.ready:
            events.append(dict(index=index,seconds=t,position_error_m=math.dist(pose[:2],dock.goal[:2]),
                yaw_error_rad=abs(wrap(dock.goal[2]-pose[2])),phase_changes=getattr(dock,'phase_changes',None)))
            index+=1
            if index==len(goals):break
            dock=make()
    return dict(ready=index==len(goals),seconds=t,events=events,final_pose=pose,drift_m_s=drift,lag_fraction=lag)


def replay(report_path,layout_path):
    raw=report_path.read_bytes();report=json.loads(raw)
    expected='921fa620ad56151c7e05b32d008ee2284e9f22c3030011d499b791d10e47d58f'
    assert hashlib.sha256(raw).hexdigest()==expected,'Original failed report identity changed'
    goal=report['navigation_goals'][3];rows=[];starts=[]
    for row in report['trace']:
        if row['stage']!='navigate' or row['step']<6750:continue
        q=row['base_quaternion'];pose=[*row['base'][:2],2*math.atan2(q[2],q[3])]
        distance=math.dist(pose[:2],goal[:2]);error=wrap(goal[2]-pose[2])
        command=PrecisionDock(goal,.015,.02,.15,.2).step(pose,(0.,0.),1/30)
        rows.append(dict(step=row['step'],distance_m=distance,yaw=pose[2],yaw_error=error,
            legacy_branch='goal_yaw' if distance<.015 else 'position_heading',raw_angular=command[1],
            logged_angular=row['wheel_command'][1],valid=row['valid']))
        if distance<.015:starts.append((distance,pose,row['step']))
    layout=WorkcellLayout.load(layout_path)
    obstacles=json.loads((report_path.with_suffix('')/'scene_navigation.json').read_text())['obstacles']
    routes=[]
    for source in ('source_A','source_B'):
        for machine in ('machine_1','machine_2','machine_3'):
            s=layout.navigation_goal(source);g=layout.navigation_goal(machine)
            goals=plan_chassis_path((s.x,s.y,s.yaw),(g.x,g.y,g.yaw),obstacles)
            previous=(s.x,s.y,s.yaw);cad_samples=0
            for i,point in enumerate(goals):
                distance=math.dist(previous[:2],point[:2]);heading=math.atan2(point[1]-previous[1],point[0]-previous[0])
                for k in range(math.ceil(distance/.025)+1):
                    f=k/max(1,math.ceil(distance/.025))
                    assert chassis_clear((previous[0]+f*(point[0]-previous[0]),previous[1]+f*(point[1]-previous[1]),heading),obstacles)
                    cad_samples+=1
                if i<len(goals)-1:
                    for k in range(361):
                        assert chassis_clear((*point[:2],math.radians(k)),obstacles);cad_samples+=1
                previous=point
            # The environment includes a separate final precision docking dwell.
            stress=[integrate(goals+[goals[-1]],(s.x,s.y,s.yaw),drift=d,lag=l)
                    for d,l in ((0.,.3),(8e-5,.3),(-8e-5,.15))]
            routes.append(dict(source=source,machine=machine,goals=goals,independent_cad_samples=cad_samples,
                independent_cad_passed=True,synthetic_stress=stress))
    _,start,start_step=max(starts)
    # Two identical goals retain coarse tolerances in the single-waypoint stress.
    stress=[]
    for drift in (0.,8e-6,2e-5,4e-5,8e-5):
        stress.append(dict(drift_m_s=drift,legacy=integrate([goal,goal],start,PrecisionDock,drift),
            candidate=integrate([goal,goal],start,SettledPrecisionDock,drift)))
    return dict(schema='workcell_navigation_offline_replay_v2',source_report_sha256=expected,
        scope='measured 1Hz command branches; synthetic kinematics separately labelled; no physical pass claimed',
        active_goal_inferred_not_logged=goal,start_sample_step=start_step,
        measured_log=dict(samples=len(rows),sample_interval_s=1.,branch_switches=sum(a['legacy_branch']!=b['legacy_branch'] for a,b in zip(rows,rows[1:])),
            turns_away_from_goal_yaw=sum(r['raw_angular']*r['yaw_error']<0 for r in rows),all_images_valid=all(r['valid'] for r in rows),
            distance_min_m=min(r['distance_m'] for r in rows),distance_max_m=max(r['distance_m'] for r in rows),rows=rows),
        synthetic_boundary_stress=stress,synthetic_routes=routes,
        all_candidate_stress_converged=all(x['candidate']['ready'] for x in stress) and all(x['ready'] for r in routes for x in r['synthetic_stress']),
        original_navigation_timeout_s=240,simulation_started=False,training_started=False,new_learning_updates=0)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--report',type=Path,required=True)
    p.add_argument('--layout',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=replay(a.report,a.layout);a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(output=str(a.output),samples=result['measured_log']['samples'],switches=result['measured_log']['branch_switches'],
        all_candidate_stress_converged=result['all_candidate_stress_converged'],route_max_seconds=max(x['seconds'] for r in result['synthetic_routes'] for x in r['synthetic_stress']))))
    raise SystemExit(0 if result['all_candidate_stress_converged'] else 1)
