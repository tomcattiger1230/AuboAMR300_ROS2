"""Independent physical candidate review, separate from task-interface acceptance."""
import math
from .workcells import FROZEN_POLICY_SHA256,wrap


def review(report,layout):
    errors=[]
    def require(condition,message):
        if not condition:errors.append(message)
    require(report.get('schema')=='workcell_physical_candidate_v1','report schema')
    require(report.get('layout_sha256')==layout.sha256,'layout identity')
    require(report.get('simulation_started') is True and report.get('hardware_or_ros_started') is False,'simulation scope')
    require(report.get('residual_training') is False and report.get('new_learning_updates')==0 and report.get('optimizer_instantiated') is False,'frozen-only scope')
    for key in ('frozen_identity_before','frozen_identity_after'):
        d=report.get(key,{})
        require(d.get('sha256')==FROZEN_POLICY_SHA256 and d.get('file_and_tensor_payload_unchanged') is True,'frozen SHA/payload '+key)
        require(d.get('model_optimizer_tensors')==52 and d.get('counters')==dict(updates=466,actor_updates=170,policy_steps=36602),'frozen tensors/counters '+key)
    require(report.get('error') is None,'runtime error: '+str(report.get('error')))
    require(report.get('completed') is True,'no completed physical episode')
    results=report.get('results',[]);require(len(results)==1,'exactly one episode required')
    if len(results)!=1:return dict(physical_candidate_passed=False,errors=errors,task_interface_accepted=False)
    r=results[0]
    require(r.get('success') is True and r.get('failure') is None and r.get('timeout') is False,'native unsuccessful episode')
    require(r.get('source_id')==report['source_id'] and r.get('machine_id')==report['machine_id'],'episode station binding')
    require(r.get('end_to_end',report.get('end_to_end')) is True and r.get('tray_verified') is True,'continuous source-to-tray chain')
    require(r.get('rebar_pose_writes_after_reset') is False and r.get('camera_only') is True,'truth/pose-write guard')
    try:
        goal=layout.machine_geometry(report['machine_id'])['goal']
        actual=r['true_position_for_evaluation'];error=math.dist(actual,goal)
        require(error<=.003 and abs(error-r['machine_error_m'])<1e-7,'independent target clamp position')
        require(r['machine_goal_map']==list(goal),'requested target scoring identity')
        axis=r['bar_axis_for_evaluation'];require(abs(axis[2])>=math.cos(math.radians(3)),'steel vertical alignment')
        jaw=r['jaw_forces_N'];require(len(jaw)==4 and min(jaw)>=.5,'four physical jaw contacts')
        fingers=r['finger_positions'];forces=r['raw_finger_forces_N']
        require(len(fingers)==2 and max(map(abs,fingers))<.003 and len(forces)==2 and min(forces)>=0 and max(forces)<.5,'open unloaded fingers')
        require(r['tcp_clearance_m']>=.06,'unloaded retreat')
        require(r['steel_linear_speed_m_s']<=.04 and r['steel_angular_speed_rad_s']<=.2,'supported steel stability')
        require(r['unsafe_force_N']<=250,'unsafe force')
        require(max(r['arm_contact_loads_N'].values())<.5 and max(r['peak_contact_loads_N']['arm_all'].values())<.5,'unexpected arm contact')
        transitions=r['mission_transitions'];names=[t['to_stage'] for t in transitions]
        required=('tray_raise','tray_rotate','tray_via_side','tray_via_front','tray_above','tray_down','tray_release','tray_retreat','observe_tray','navigate','dock','pickup_tray','tray_lift_high','loading')
        index=-1
        for name in required:
            index=names.index(name,index+1)
        a=r['source_navigation'];dock=layout.navigation_goal(report['source_id'])
        require(a is not None and math.dist(a['base_pose'][:2],(dock.x,dock.y))<=.01 and abs(wrap(a['base_pose'][2]-dock.yaw))<=.01,'source navigation arrival')
        require(a['linear_speed']<.008 and abs(a['yaw_rate'])<.008,'source stopped arrival')
        if r.get('navigation_controller_contract')=='settled_position_then_yaw_v1':
            events=r['navigation_events'];goals=report['navigation_goals']+[report['navigation_goals'][-1]]
            require(len(events)==len(goals),'all navigation waypoints and precision dock recorded')
            for i,(event,goal) in enumerate(zip(events,goals)):
                pt,yt=(.015,.02) if i<len(goals)-1 else (.008,.008)
                require(event['index']==i and event['goal']==goal,'navigation waypoint identity')
                require(event['position_tolerance_m']==pt and event['yaw_tolerance_rad']==yt,'unchanged navigation thresholds')
                require(math.dist(event['pose'][:2],goal[:2])<pt and abs(wrap(goal[2]-event['pose'][2]))<yt,'measured navigation arrival')
                require(event['linear_speed']<.008 and abs(event['yaw_rate'])<.008 and event['stopped_dwell_s']>=.5,'stopped navigation dwell')
        require(r.get('actual_handoff') is True,'native physical handoff')
    except (KeyError,ValueError,TypeError,IndexError) as ex:errors.append('incomplete physical evidence: '+str(ex))
    return dict(physical_candidate_passed=not errors,errors=errors,source_id=report['source_id'],machine_id=report['machine_id'],
                task_interface_accepted=False,machine_visual_identity_verified=report.get('machine_visual_identity_verified') is True,
                scope='independent physical controller probe; full MissionFlow/machine-image identity acceptance remains separate')
