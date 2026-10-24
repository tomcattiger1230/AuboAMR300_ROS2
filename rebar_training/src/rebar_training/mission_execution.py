"""Offline-wired task executor and injected navigation/skill ports.

No sockets, ROS, simulator construction or actuator dispatch. Transport/driver
objects must explicitly be offline; a production connection needs a separate
composition root and validated SEER response-field mapping.
"""
from copy import deepcopy
from dataclasses import dataclass
import math

from .workcells import Pose, finite
from .workcell_mission import MissionFlow, _fresh, _true, _empty_fingers, _tray, _visual
from .policy_compatibility import StationPolicyAdapter, compose_residual_actions
from .station_skill_adapter import StationLoadingAdapter, _skill_visual


@dataclass(frozen=True)
class NavigationStatus:
    request_id: str
    target_id: str
    state: int


@dataclass(frozen=True)
class SeerStatusFields:
    """Explicit configurable field paths; actual device captures must establish them."""
    request_id_path: tuple
    target_id_path: tuple
    state_path: tuple = ('task_status',)

    def __post_init__(self):
        for path in (self.request_id_path,self.target_id_path,self.state_path):
            if not isinstance(path,tuple) or not path or any(not isinstance(k,str) or not k for k in path):
                raise ValueError('Nonempty explicit SEER field paths are required')

    def __call__(self, navigation, details):
        def read(data,path):
            for key in path:
                if not isinstance(data,dict) or key not in data:raise ValueError('Missing SEER field path: '+'.'.join(path))
                data=data[key]
            return data
        request=read(details,self.request_id_path);target=read(details,self.target_id_path);state=read(navigation,self.state_path)
        if not isinstance(request,str) or not request or not isinstance(target,str) or not target or type(state) is not int:
            raise ValueError('Malformed task identity or navigation state')
        return NavigationStatus(request,target,state)


def _offline(port):
    if getattr(port, 'offline_only', None) is not True:
        raise ValueError('This executor accepts offline ports only; live dispatch is not enabled')


class SeerClientTransportAdapter:
    """Normalize the existing AgvClient ApiResult interface without constructing it.

    A normal client remains offline_only=False and is rejected by TaskExecutor's
    ports. Only an explicitly offline client double may be exercised here.
    """
    def __init__(self, client):
        self.client=client;self.offline_only=getattr(client,'offline_only',None) is True

    def request(self, port, operation, payload):
        reply=self.client.request(port,operation,deepcopy(payload))
        if getattr(reply,'ok',None) is not True or not isinstance(reply.response,dict):
            raise RuntimeError('AgvClient failed or returned an invalid response')
        return deepcopy(reply.response)


def _feedback(provider, layout, command, now):
    f = provider(command, now)
    if f.get('map_id')!=layout.map_id or f.get('layout_sha256')!=layout.sha256:
        raise ValueError('Feedback map/layout mismatch')
    _fresh(f.get('stamp_s'), now, command['issued_s'])
    e=f['evidence']
    if e.get('origin') not in ('synthetic_fixture','recorded_measurement'):
        raise ValueError('Unsupported offline feedback origin')
    if e.get('emergency_stop') is not False:
        raise ValueError('Emergency stop/unknown stop state')
    return deepcopy(e)


def result_for(command, now, evidence, status='succeeded'):
    return dict(**{k:command[k] for k in ('task_id','command_id','step','station_id','map_id','layout_sha256')},
                stamp_s=now,status=status,evidence=deepcopy(evidence))


class SeerNavigationPort:
    """3051 submit + 1020/1110 status + 3003 cancel, via injected offline transport.

    ACK only means accepted. The supplied decoder must bind status to the exact
    request and target. Raw field names from an actual device are deliberately
    not guessed from the GUI's aggregate task_status value.
    """
    offline_only=True

    def __init__(self, layout, transport, feedback, decode_status):
        _offline(transport)
        _offline(feedback)
        self.layout=layout;self.transport=transport;self.feedback=feedback;self.decode=decode_status
        self.active=None;self.submissions=[];self.cancellations=[]

    def _call(self, port, message, payload):
        reply=self.transport.request(port,message,deepcopy(payload))
        if not isinstance(reply,dict) or type(reply.get('ret_code')) is not int or reply['ret_code']!=0:
            raise RuntimeError('SEER request failed or missing explicit ret_code=0')
        return reply

    def _guard(self, command, now):
        e=_feedback(self.feedback,self.layout,command,now)
        _true(e,'localization_valid','path_checked','arm_stowed');_empty_fingers(e)
        _fresh(e.get('pose_stamp_s'),now,command['issued_s'])
        Pose.parse(e.get('base_pose_map'));finite(e.get('measured_twist'),2,'measured twist')
        if command['args']['loaded']:
            _tray(e,self.layout,now,command['issued_s'])
        return e

    def start(self, command, now):
        if not command['kind'].startswith('navigate_') or command.get('actuator_dispatch') is not False:
            raise ValueError('Expected an offline navigation intent')
        if self.active is not None:
            raise ValueError('Navigation already has an outstanding request')
        if now>command['deadline_s']:
            raise ValueError('Navigation intent expired')
        e=self._guard(command,now)
        source=e.get('current_navigation_id')
        if not isinstance(source,str) or not source.strip():
            raise ValueError('Measured current navigation station is required')
        payload=dict(source_id=source,id=command['args']['navigation_id'],task_id=command['command_id'])
        # Bind before sending: an ambiguous ACK failure must not cause resubmit.
        self.active=deepcopy(command);self.submissions.append(deepcopy(payload))
        self._call(19206,3051,payload)

    def poll(self, command, now):
        if self.active is None or command['command_id']!=self.active['command_id']:
            raise ValueError('Navigation poll does not match active request')
        if now>command['deadline_s']:
            raise ValueError('Navigation deadline expired')
        e=self._guard(command,now)  # Continuous, not just arrival-time load checks.
        navigation=self._call(19204,1020,{})
        details=self._call(19204,1110,{})
        status=self.decode(navigation,details)
        if (not isinstance(status,NavigationStatus) or status.request_id!=command['command_id']
                or status.target_id!=command['args']['navigation_id'] or type(status.state) is not int):
            raise ValueError('Navigation status is not bound to the active task/target')
        if status.state in (0,1,2):
            return None
        if status.state==4:
            e['navigation_id']=status.target_id
            # MissionFlow independently checks measured pose and stopped twist.
            return result_for(command,now,e)
        raise ValueError('Navigation failed/paused/cancelled/unknown: '+str(status.state))

    def finish(self, command):
        if self.active and self.active['command_id']==command['command_id']:
            self.active=None

    def hold(self, reason):
        if self.active is not None:
            command=self.active;self.active=None
            record=dict(command_id=command['command_id'],reason=str(reason),performed=False)
            self.cancellations.append(record)
            # The existing SEER cancel operation is global (no request ID).
            # Reconfirm ownership instead of cancelling another client's job.
            try:
                navigation=self._call(19204,1020,{})
                details=self._call(19204,1110,{})
                status=self.decode(navigation,details)
                if (not isinstance(status,NavigationStatus) or status.request_id!=command['command_id']
                        or status.target_id!=command['args']['navigation_id']):
                    record['cancel_state']='ownership_unconfirmed';return
                if status.state in (4,5,6):
                    record['cancel_state']='already_terminal';return
                self._call(19206,3003,{});record['performed']=True;record['cancel_state']='owned_request_cancelled'
            except Exception as ex:
                record['cancel_state']='ownership_or_cancel_unconfirmed';record['error']=str(ex)
                raise


@dataclass
class SkillTick:
    command_id: str
    sample: object = None
    nominal_actions: object = None
    visual_evidence: dict | None = None
    completion: dict | None = None


class SkillExecutionPort:
    """Connect a legacy controller/replay driver, canonical input and frozen actor.

    The driver receives concrete station adapters; it owns acquisition/skill
    completion. Policy inference cannot fabricate a successful skill receipt.
    Mixed actions are recorded by an offline driver, never sent to a device.
    """
    offline_only=True

    def __init__(self, layout, driver, frozen_policy, feedback):
        _offline(driver)
        _offline(feedback)
        self.layout=layout;self.driver=driver;self.policy=frozen_policy;self.feedback=feedback
        self.active=None;self.policy_adapter=None;self.loading=None;self.decisions=[]

    def start(self, command, now):
        if self.active is not None or command['kind'].startswith('navigate_'):
            raise ValueError('Invalid/duplicate skill start')
        if command.get('actuator_dispatch') is not False:
            raise ValueError('Live dispatch is disabled')
        _feedback(self.feedback,self.layout,command,now)
        self.active=deepcopy(command)
        context=dict(policy_contract='station_relative_legacy_context_v1',actuator_dispatch=False)
        from .station_skill_adapter import source_observer_for_command
        if command['kind']=='observe_source':
            context['source_observer_factory']=source_observer_for_command
        elif command['kind'] in ('pickup_to_tray','load_machine'):
            self.policy_adapter=StationPolicyAdapter(self.layout,command['task_source_id'],command['task_machine_id'],
                            refined_machine_pose=Pose.parse(command['args']['station_pose_map'])
                            if command['kind']=='load_machine' else None)
            context['policy_adapter']=self.policy_adapter
            if command['kind']=='load_machine':
                self.loading=StationLoadingAdapter(self.layout,command);context['loading_adapter']=self.loading
        self.driver.start(deepcopy(command),context)

    def poll(self, command, now):
        if self.active is None or self.active['command_id']!=command['command_id']:
            raise ValueError('Skill poll identity mismatch')
        e=_feedback(self.feedback,self.layout,command,now)
        if now>command['deadline_s']:
            raise ValueError('Skill deadline expired')
        tick=self.driver.poll(deepcopy(command),now)
        if tick is None:
            return None
        if not isinstance(tick,SkillTick) or tick.command_id!=command['command_id']:
            raise ValueError('Skill tick identity mismatch')
        if tick.sample is not None:
            if self.policy_adapter is None or tick.sample.phase not in command['args']['legacy_stages']:
                raise ValueError('Wrong legacy phase for the task skill')
            if abs(tick.sample.now-now)>1e-9:
                raise ValueError('Policy sample clock differs from executor clock')
            _visual(e,self.layout,now,command['issued_s'])
            if tick.sample.vision_valid is not True:
                raise ValueError('Manipulation vision lost')
            # Bind all sensor clocks/frames to the same feedback receipt.
            if abs(e['image_stamp_s']-tick.sample.image_stamp_s)>1e-9:
                raise ValueError('Policy/image receipt mismatch')
            _fresh(tick.sample.robot_stamp_s,now,command['issued_s'])
            if abs(e['pose_stamp_s']-tick.sample.robot_stamp_s)>1e-9:
                raise ValueError('Policy/robot receipt mismatch')
            measured_pose=Pose.parse(e['base_pose_map']);sample_pose=Pose.parse(tick.sample.base_pose_map)
            if any(abs(a-b)>1e-8 for a,b in zip(measured_pose.values(),sample_pose.values())):
                raise ValueError('Policy/base-frame receipt mismatch')
            _true(e,'base_stopped')
            if math.hypot(*tick.sample.base_velocity_map[:2])>=.008 or abs(tick.sample.base_yaw_rate)>=.008:
                raise ValueError('Manipulation requires a stopped chassis')
            if e.get('visual_id')!=command['args']['visual_id']:
                raise ValueError('Skill is observing another station')
            if self.loading:
                _skill_visual(e,self.layout,command['station_id'],now,command['issued_s'])
                speed=math.hypot(*tick.sample.base_velocity_map[:2])
                self.loading.geometry(tick.sample.base_pose_map,tick.sample.robot_stamp_s,now,
                                      [speed,tick.sample.base_yaw_rate])
            packet=self.policy_adapter.encode(tick.sample);decision=self.policy.infer(packet)
            actions=compose_residual_actions(tick.nominal_actions,decision)
            self.driver.record_actions(command['command_id'],actions)
            self.decisions.append(dict(command_id=command['command_id'],phase=packet.phase,route=packet.route,
                                       canonical_base_map=packet.canonical_base_map,
                                       actual_machine_goal_base=packet.actual_machine_goal_base,
                                       policy_context_goal_base=packet.policy_context_goal_base,
                                       source_destination_context_rebased=packet.source_destination_context_rebased,
                                       decision=decision,actuator_dispatch=False))
        if tick.completion is not None:
            return deepcopy(tick.completion)
        return None

    def finish(self, command):
        self.driver.finish(command['command_id']);self.active=None;self.policy_adapter=None;self.loading=None

    def hold(self, reason):
        if self.active is not None:
            command=self.active;self.active=None
            self.driver.hold(command['command_id'],str(reason),preserve_grip=True,stop_base=True)


class TaskExecutor:
    """Poll-driven orchestration; one start per command, guarded completion only."""
    def __init__(self, layout, task, navigation, skills, *, evidence_origin='synthetic_fixture'):
        _offline(navigation);_offline(skills)
        self.flow=MissionFlow(layout,task,evidence_origin=evidence_origin)
        self.navigation=navigation;self.skills=skills;self.active=None;self.port=None
        self.events=[];self.hold_errors=[];self.last_now=-math.inf;self.stopped=False;self.last_outcome=None

    def _stop(self, reason):
        if self.stopped:return
        self.stopped=True
        for port in (self.navigation,self.skills):
            try:port.hold(reason)
            except Exception as ex:self.hold_errors.append(str(ex))

    def tick(self, now):
        value,=finite([now],1,'now')
        if value==self.last_now and self.last_outcome is not None:
            return deepcopy(self.last_outcome)
        outcome=self._tick(now)
        self.last_outcome=deepcopy(outcome)
        return outcome

    def _tick(self, now):
        value,=finite([now],1,'now')
        if value<self.last_now:
            self._stop('Executor clock moved backwards');self.flow.block('Executor clock moved backwards',self.last_now)
            raise ValueError('Executor clock moved backwards')
        self.last_now=value
        if self.flow.state in ('blocked','cancelled'):
            self._stop(self.flow.reason);return self.flow.command(now)
        if self.flow.state=='completed':
            return self.flow.command(now)
        try:
            command=self.flow.command(now)
            if command['kind']=='hold':
                self._stop(command['reason']);return command
            if self.active is None:
                command.update(task_source_id=self.flow.plan['source_id'],task_machine_id=self.flow.plan['machine_id'])
                if 'policy' in command['args']:
                    command['args']['policy'].update(offline_inference_authorized=True,
                        input_adapter_contract='station_relative_legacy_context_v1',
                        compatibility='offline_shadow_context_adapter')
                self.active=command
                self.port=self.navigation if command['kind'].startswith('navigate_') else self.skills
                self.port.start(command,now)
                self.events.append(dict(type='started',command_id=command['command_id'],kind=command['kind'],time=now))
                return dict(state='waiting',command_id=command['command_id'])
            result=self.port.poll(self.active,now)
            if result is None:return dict(state='waiting',command_id=self.active['command_id'])
            outcome=self.flow.accept(result,now)
            if self.flow.state=='blocked':
                self._stop(self.flow.reason)
            else:
                self.port.finish(self.active)
                self.events.append(dict(type='completed',command_id=self.active['command_id'],kind=self.active['kind'],time=now))
                self.active=None;self.port=None
            return outcome
        except Exception as ex:
            self._stop(str(ex));return self.flow.block(str(ex),now)

    def retry_observation(self, now):
        value,=finite([now],1,'now')
        if value<self.last_now:raise ValueError('Retry clock moved backwards')
        if self.hold_errors:raise ValueError('Unconfirmed port stop requires review before retry')
        command=self.flow.retry_observation(now)
        self.active=None;self.port=None;self.stopped=False;self.last_now=value;self.last_outcome=None
        self.events.append(dict(type='observation_retry_requested',command_id=command['command_id'],time=now))
        return command

    def cancel(self, now):
        value,=finite([now],1,'now')
        if value<self.last_now:
            raise ValueError('Cancel clock moved backwards')
        self._stop('Task cancelled')
        outcome=self.flow.cancel(now);self.last_now=value;self.last_outcome=deepcopy(outcome)
        return outcome
