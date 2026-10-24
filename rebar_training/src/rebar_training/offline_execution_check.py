"""Offline full-port wiring + frozen-input equivalence checks. Never a simulator.

All navigation/skill telemetry below is explicitly synthetic. The real actor is
read-only. Recorded native reports are used only for a partial measured-field
audit; missing 76-D sensor fields are not filled and called recorded evidence.
"""
import argparse
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys
import time

from .workcells import WorkcellLayout, Pose
from .workcell_mission import MissionTask
from .workcell_check import fixture_result
from .policy_compatibility import PolicySample, StationPolicyAdapter, FrozenStationResidual, REFERENCE_SOURCE, REFERENCE_MACHINE, SUPPORTED_ROUTES
from .mission_execution import NavigationStatus, SeerNavigationPort, SkillExecutionPort, SkillTick, TaskExecutor


def synthetic_sample(layout, source, machine, phase, route, now, *, base_pose=None):
    source_phase=phase in ('pickup','tray_raise','tray_rotate','tray_via_side','tray_via_front','tray_above',
                          'tray_down','tray_release','tray_retreat')
    base=base_pose or layout.navigation_goal(source if source_phase else machine)
    goal=layout.machine_geometry(machine,refined_pose=layout.stations[machine].pose_map,base_pose=base)['goal']
    bar=(-1.15,0,.522) if source_phase else tuple(x+d for x,d in zip(goal,(.005,0,0)))
    tcp=tuple(x+d for x,d in zip(bar,(0,0,.004)))
    return PolicySample(phase,route,now,now,now,base.values(),
        (2.1,-.2,1.2,.1,1.57,-1.2),(0,0,0,0,0,0),tcp,(math.sqrt(.5),0,0,math.sqrt(.5)),
        bar,(1,0,0) if source_phase else (0,0,1),(0,0,0),(0,0,0),(.015,.015),(20,20),
        (0,0,0,0),0,(0,0,0),0,True,True,False)


class ScriptedSeerTransport:
    """Records requests; replies are sample-schema fixtures, not device captures."""
    offline_only=True
    def __init__(self):
        self.calls=[];self.active=None;self.polls=0;self.current_station='offline_start'
    def request(self, port, operation, payload):
        self.calls.append(dict(port=port,operation=operation,payload=deepcopy(payload)))
        if (port,operation)==(19206,3051):
            self.active=deepcopy(payload);self.polls=0;return dict(ret_code=0)
        if (port,operation)==(19204,1020):
            self.polls+=1;return dict(ret_code=0,task_status=2 if self.polls==1 else 4)
        if (port,operation)==(19204,1110):
            if self.polls>=2:self.current_station=self.active['id']
            return dict(ret_code=0,fixture_request_id=self.active['task_id'],fixture_target_id=self.active['id'])
        if (port,operation)==(19206,3003):
            return dict(ret_code=0)
        raise ValueError('Unexpected SEER operation in offline test')


def fixture_status_decoder(navigation, details):
    return NavigationStatus(details['fixture_request_id'],details['fixture_target_id'],navigation['task_status'])


class ScriptedFeedback:
    offline_only=True
    def __init__(self, flow_holder, layout, transport, fault=None):
        self.holder=flow_holder;self.layout=layout;self.transport=transport;self.fault=fault
    def __call__(self, command, now):
        flow=self.holder['executor'].flow
        e=fixture_result(flow,command,now)['evidence']
        e.update(current_navigation_id=self.transport.current_station)
        if command['kind'].startswith('navigate_') and self.transport.polls==0:
            e['base_pose_map']=[-2,-1,0,0]  # Scripted pre-arrival sample, not integrated motion.
        driver=self.holder.get('driver')
        if (command['kind'] in ('pickup_to_tray','load_machine') and driver and driver.session
                and driver.session.samples<=1):
            # Pre-terminal sensor sample for the frozen-inference tick.
            sample=synthetic_sample(self.layout,command['task_source_id'],command['task_machine_id'],
                   'pickup' if command['kind']=='pickup_to_tray' else 'loading',
                   3 if command['kind']=='pickup_to_tray' else 7,now)
            e.update(finger_positions_m=sample.fingers_m,finger_forces_n=sample.finger_forces_n,
                     jaw_forces_n=sample.jaw_forces_n,tray_load_n=sample.tray_load_n)
        if self.fault:self.fault(command,e,self.transport)
        return dict(map_id=self.layout.map_id,layout_sha256=self.layout.sha256,stamp_s=now,evidence=e)


class ScriptedSession:
    offline_only=True
    clock_offset_s=0.
    def __init__(self, holder, layout, command, context):
        self.holder=holder;self.layout=layout;self.command=command;self.context=context
        self.samples=0;self.controller=None;self.targets=[]
    def start(self):
        c=self.command
        if c['kind']=='observe_source':
            base=self.layout.navigation_goal(c['station_id'])
            observer=self.context['source_observer_factory'](self.layout,c,detector=object(),
                    base_pose_map=base.values(),robot_stamp_s=c['issued_s'],now=c['issued_s'],measured_twist=[0,0])
            assert observer.survey_prior is None and not observer.acquired
    def poll(self, now):
        import torch
        from types import SimpleNamespace
        from .legacy_observation_probe import legacy_control_fixture
        from .route import LoadingRoute
        from .vision.servo import RobotServo
        c=self.command;flow=self.holder['executor'].flow
        if c['kind'] not in ('pickup_to_tray','load_machine') or self.samples:
            return fixture_result(flow,c,now)
        sample=synthetic_sample(self.layout,c['task_source_id'],c['task_machine_id'],
                               'pickup' if c['kind']=='pickup_to_tray' else 'loading',
                               3 if c['kind']=='pickup_to_tray' else 7,now)
        control=legacy_control_fixture(sample,self.layout)
        tensor=lambda value:torch.tensor(value,dtype=torch.float32)
        data=control.env.robot.data
        data.joint_pos=SimpleNamespace(torch=tensor([sample.q]));data.joint_vel=SimpleNamespace(torch=tensor([sample.dq]))
        state=SimpleNamespace(tcp=tensor([sample.tcp_base]),bar=tensor([sample.bar_base]),bar_axis=tensor([sample.bar_axis_base]),
                 linear_velocity=tensor([sample.bar_velocity_base]),angular_velocity=tensor([sample.bar_angular_velocity_base]),
                 fingers=tensor([sample.fingers_m]),finger_forces=tensor([sample.finger_forces_n]),jaw_forces=tensor([sample.jaw_forces_n]))
        route=LoadingRoute(1,'cpu');route.reset(torch.tensor([0]),tensor([[*sample.bar_base,0,0,0,1]]));route.stage[:]=sample.route
        control.env.route=route;control.env.task_spec.arm_speed=.35
        actual_goal=self.layout.machine_geometry(c['task_machine_id'],refined_pose=self.layout.stations[c['task_machine_id']].pose_map,
                                                   base_pose=Pose.parse(sample.base_pose_map))['goal']
        control.env.task_spec.insert_goal=actual_goal;control.measured_state=lambda:state
        targets=self.targets
        class CaptureServo(RobotServo):
            def cartesian_actions(self, desired, quaternion, grip):
                targets.append(dict(desired_base=desired.tolist(),quaternion_xyzw=quaternion.tolist(),grip=grip.tolist()))
                result=torch.zeros(1,7);result[:,6]=grip;return result
        servo=CaptureServo(control.env);control.servo=servo;control.action=lambda:servo.route_actions(state,route,actual_goal)
        self.controller=control;self.samples+=1
        return None
    def visual_evidence(self, now):
        return fixture_result(self.holder['executor'].flow,self.command,now)['evidence']
    def record_actions(self, actions):pass
    def close(self):pass
    def hold(self, reason, **flags):pass


class ScriptedSessionFactory:
    offline_only=True
    def __init__(self,holder,layout):self.holder=holder;self.layout=layout
    def __call__(self,command,context):return ScriptedSession(self.holder,self.layout,command,context)


from .legacy_skill_driver import LegacyControlDriver

class ScriptedSkillDriver(LegacyControlDriver):
    def __init__(self, holder, layout):
        super().__init__(ScriptedSessionFactory(holder,layout))
        self.starts=[];self.holds=[];self.actions=[];self.contexts=[];self.nominal_targets=[]
    def start(self, command, context):
        self.starts.append(command['command_id']);self.contexts.append(context)
        super().start(command,context)
    def record_actions(self, command_id, actions):
        self.actions.append(dict(command_id=command_id,actions=actions.tolist(),applied_to_device=False))
        self.nominal_targets.append(self.session.targets[-1]);super().record_actions(command_id,actions)
    def hold(self, command_id, reason, **flags):
        self.holds.append(dict(command_id=command_id,reason=reason,**flags));super().hold(command_id,reason,**flags)


def make_offline_executor(layout, task, policy, *, fault=None, decoder=fixture_status_decoder):
    holder={};transport=ScriptedSeerTransport();feedback=ScriptedFeedback(holder,layout,transport,fault)
    nav=SeerNavigationPort(layout,transport,feedback,decoder)
    driver=ScriptedSkillDriver(holder,layout);skills=SkillExecutionPort(layout,driver,policy,feedback)
    executor=TaskExecutor(layout,task,nav,skills);holder['executor']=executor;holder['driver']=driver
    return executor,transport,driver


def run_wiring(layout, task, policy, *, fault=None, decoder=fixture_status_decoder):
    executor,transport,driver=make_offline_executor(layout,task,policy,fault=fault,decoder=decoder)
    now=1.;ticks=[]
    for _ in range(80):
        outcome=executor.tick(now);ticks.append(dict(now=now,outcome=outcome))
        if executor.flow.state in ('completed','blocked','cancelled'):break
        now+=.21 if executor.active and executor.active['kind']=='observe_source' else .02
    return dict(task_id=task.task_id,source_id=task.source_id,machine_id=task.machine_id,state=executor.flow.state,
                reason=executor.flow.reason,passed=executor.flow.state=='completed',events=executor.events,
                navigation_cancellations=executor.navigation.cancellations,hold_errors=executor.hold_errors,
                transport_calls=transport.calls,skill_starts=driver.starts,recorded_actions=driver.actions,
                frozen_decisions=executor.skills.decisions,nominal_targets=driver.nominal_targets,ticks=ticks,origin='synthetic_fixture',
                physical_acceptance=False,actuator_dispatch=False)


def fault_replay_checks(layout, policy):
    source=next(s.id for s in layout.stations.values() if layout.profiles[s.profile_id].kind=='source')
    machine=next(s.id for s in layout.stations.values() if layout.profiles[s.profile_id].kind=='machine')
    rows=[]
    for name in ('lost_tray','stale_manipulation_image','foreign_navigation_owner','failed_navigation'):
        def mutate(c,e,t,name=name):
            if name=='lost_tray' and c['kind']=='navigate_machine' and t.active and t.active['task_id']==c['command_id']:
                e['tray_load_n']=0
            if name=='stale_manipulation_image' and c['kind']=='pickup_to_tray':e['image_stamp_s']-=1
        def decode(nav,details,name=name):
            status=fixture_status_decoder(nav,details)
            if name=='foreign_navigation_owner':return NavigationStatus('foreign_job',status.target_id,2)
            if name=='failed_navigation':return NavigationStatus(status.request_id,status.target_id,5)
            return status
        flow=run_wiring(layout,MissionTask('fault_'+name,source,machine,layout.steel_id),policy,fault=mutate,decoder=decode)
        submits=[x for x in flow['transport_calls'] if x['operation']==3051]
        cancels=[x for x in flow['transport_calls'] if x['operation']==3003]
        no_retry=len(submits)==len({x['payload']['task_id'] for x in submits})
        safe_cancel=not cancels if name=='foreign_navigation_owner' else True
        rows.append(dict(name=name,passed=flow['state']=='blocked' and no_retry and safe_cancel,
                         state=flow['state'],reason=flow['reason'],automatic_resubmit=False,
                         navigation_cancellations=flow['navigation_cancellations'],
                         global_cancel_sent=bool(cancels),preserve_grip_requested=True,physical_stop_verified=False))
    return rows


def reference_layout(layout):
    data=layout.to_dict()
    source=next(s for s in data['stations'] if layout.profiles[s['profile_id']].kind=='source')
    machine=next(s for s in data['stations'] if layout.profiles[s['profile_id']].kind=='machine')
    source['pose_map_xyz_yaw']=REFERENCE_SOURCE.values();machine['pose_map_xyz_yaw']=REFERENCE_MACHINE.values()
    for profile in data['profiles']:
        if profile['id']==source['profile_id']:profile['navigation_offset_xyz_yaw']=[1.15,0,0,0]
        if profile['id']==machine['profile_id']:profile['navigation_offset_xyz_yaw']=[.3,.77,0,0]
    return WorkcellLayout(data),source['id'],machine['id']


def legacy_identity_check(layout, policy):
    import torch
    from .legacy_observation_probe import legacy_observation, LEGACY_SOURCE_SHA
    from .mobile_residual import precision_residual_gain
    reference,source,machine=reference_layout(layout);adapter=StationPolicyAdapter(reference,source,machine)
    pairs=[];maximum=0.;mask_equal=True
    for phase,routes in SUPPORTED_ROUTES.items():
        for route in routes:
            for ready,recovery,valid in ((True,False,True),(False,False,True),(True,True,True),(True,False,False)):
                sample=replace(synthetic_sample(reference,source,machine,phase,route,10.),
                               motion_ready=ready,recovery_pending=recovery,vision_valid=valid,
                               dq=(.001,-.002,.003,0,0,0),bar_velocity_base=(.001,.002,.003),
                               bar_angular_velocity_base=(.0001,.0002,.0003),base_velocity_map=(.001,-.002,0),base_yaw_rate=.001)
                old,active=legacy_observation(sample,reference);new=adapter.encode(sample)
                maximum=max(maximum,max(abs(a-b) for a,b in zip(old,new.values)))
                mask_equal=mask_equal and bool(active)==new.active;pairs.append((old,new,active))
    a=torch.tensor([x[0] for x in pairs],dtype=torch.float32);b=torch.tensor([x[1].values for x in pairs],dtype=torch.float32)
    with torch.inference_mode():old_raw=policy.model(a);new_raw=policy.model(b)
    action_difference=(old_raw-new_raw).abs().max().item();scaled_difference=0.
    for i,(old,new,active) in enumerate(pairs):
        if active:
            old_delta=.08*precision_residual_gain(old)*old_raw[i]
            new_delta=.08*precision_residual_gain(new.values)*new_raw[i]
            scaled_difference=max(scaled_difference,(old_delta-new_delta).abs().max().item())
    return dict(passed=maximum<=3e-7 and action_difference<=2e-6 and scaled_difference<=2e-6 and mask_equal,
                original_encoder_source_sha256=LEGACY_SOURCE_SHA,comparisons=len(pairs),max_input_abs_difference=maximum,
                max_actor_abs_difference=action_difference,max_applied_residual_difference=scaled_difference,
                activation_masks_identical=mask_equal,original_encoder_function_executed_in_isolation=True,
                simulator_module_imported=False,scope='synthetic measurements through the actual legacy input function')


def equivalence_check(layout, policy):
    """Same local measurement, independently relocated stations, unchanged actor."""
    import torch
    from .workcell_mission import PICKUP_STAGES,LOADING_STAGES
    reference,reference_source,reference_machine=reference_layout(layout)
    ref_adapter=StationPolicyAdapter(reference,reference_source,reference_machine)
    phases=(*PICKUP_STAGES[:-1],*LOADING_STAGES);pairs=[];maximum_input=0.;contexts_differ=0
    sources=[s.id for s in layout.stations.values() if layout.profiles[s.profile_id].kind=='source']
    machines=[s.id for s in layout.stations.values() if layout.profiles[s.profile_id].kind=='machine']
    for source in sources:
        for machine in machines:
            adapter=StationPolicyAdapter(layout,source,machine)
            for phase in phases:
                for route in SUPPORTED_ROUTES[phase]:
                    base_sample=synthetic_sample(reference,reference_source,reference_machine,phase,route,10.)
                    actual_base=layout.navigation_goal(source if phase in PICKUP_STAGES else machine)
                    sample=replace(base_sample,base_pose_map=actual_base.values())
                    old=ref_adapter.encode(base_sample);new=adapter.encode(sample)
                    diff=max(abs(a-b) for a,b in zip(old.values,new.values));maximum_input=max(maximum_input,diff)
                    if old.active!=new.active:raise AssertionError('Policy activation mask changed')
                    if math.dist(new.actual_machine_goal_base,new.policy_context_goal_base)>.001:contexts_differ+=1
                    pairs.append((old,new))
    a=torch.tensor([p[0].values for p in pairs],dtype=torch.float32)
    b=torch.tensor([p[1].values for p in pairs],dtype=torch.float32)
    with torch.inference_mode():out_a=policy.model(a);out_b=policy.model(b)
    max_action=(out_a-out_b).abs().max().item()
    return dict(passed=maximum_input<2e-6 and max_action<2e-6,pairs=len(pairs),
                max_input_abs_difference=maximum_input,max_actor_abs_difference=max_action,
                source_context_different_from_real_destination_cases=contexts_differ,
                unchanged_76d_contract=True,gripper_controlled_by_policy=False,
                scope='synthetic local-state coordinate equivalence, not physical generalization')


def native_partial_audit(path):
    raw=Path(path).read_bytes();data=json.loads(raw)
    permitted=('stage','route','valid','base','tcp_base','arm_joints','estimated_bar','finger_positions','finger_forces','camera_age_ms')
    rows=[{k:r[k] for k in permitted if k in r} for r in data['trace']]
    # This whitelist deliberately excludes every *_for_evaluation steel field.
    mismatches=[dict(stage=r['stage'],route=r['route']) for r in rows
                if r.get('stage') in SUPPORTED_ROUTES and r.get('route') not in SUPPORTED_ROUTES[r['stage']]]
    return dict(unsupported_legacy_phase_route_pairs=mismatches,source_sha256=hashlib.sha256(raw).hexdigest(),measured_rows=len(rows),
                complete_76d_replay_available=False,missing_fields=['dq','bar velocity vectors','base yaw in each row','jaw loads in each row'],
                ground_truth_fields_used=False,permitted_fields=permitted,
                valid_measured_rows=sum(r.get('valid') is True for r in rows),
                scope='partial measured-field audit only; no missing sensor values fabricated')


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--layout',type=Path,required=True)
    p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--report',type=Path,required=True)
    p.add_argument('--native-report',type=Path);args=p.parse_args(argv)
    if args.report.exists():p.error('Use a fresh report path')
    layout=WorkcellLayout.load(args.layout);policy=FrozenStationResidual(args.checkpoint)
    sources=[s.id for s in layout.stations.values() if layout.profiles[s.profile_id].kind=='source']
    machines=[s.id for s in layout.stations.values() if layout.profiles[s.profile_id].kind=='machine']
    flows=[run_wiring(layout,MissionTask('wired_'+a+'_'+b,a,b,layout.steel_id),policy) for a in sources for b in machines]
    equivalence=equivalence_check(layout,policy);legacy_identity=legacy_identity_check(layout,policy)
    faults=fault_replay_checks(layout,policy)
    report=dict(schema='station_policy_execution_offline_v1',passed=all(x['passed'] for x in flows) and equivalence['passed'] and legacy_identity['passed'] and all(x['passed'] for x in faults),
                flows=flows,counterexample_checks=faults,equivalence=equivalence,legacy_identity=legacy_identity,frozen_identity=policy.verify_unchanged(),
                input_adapter_contract='station_relative_legacy_context_v1',navigation_interface_wired=True,
                task_execution_interface_wired=True,transport_mode='injected offline fixtures',
                seer_response_codec='fixture schema only; actual device field binding not validated',
                native_partial_audit=native_partial_audit(args.native_report) if args.native_report else None,
                policy_inference_executed=True,optimizer_instantiated=False,model_updated=False,
                legacy_controller_interfaces_read=True,local_servo_geometry_math_executed=True,
                new_layout_deployment_authorized=False,
                actual_navigation_executed=False,physical_acceptance=False,simulation_started=False,training_started=False,
                hardware_or_ros_started=False,created_unix=time.time())
    args.report.parent.mkdir(parents=True,exist_ok=True)
    with args.report.open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(dict(passed=report['passed'],flows_passed=sum(x['passed'] for x in flows),equivalence=equivalence,
                          report=str(args.report.resolve()),simulation_started=False)))
    return 0 if report['passed'] else 2


if __name__=='__main__':sys.exit(main())
