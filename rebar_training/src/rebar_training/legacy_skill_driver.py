"""Session adapter into existing local controllers, without constructing a runtime.

The injected session factory supplies offline/replay controller snapshots and
completion receipts. Observations are captured from the original controller;
loading nominal actions go through the station geometry adapter. Neither this
adapter nor its caller applies actions to a simulator or a robot.
"""
from .mission_execution import SkillTick, _offline
from .policy_compatibility import sample_from_legacy_control


class LegacyControlDriver:
    offline_only=True

    def __init__(self, session_factory):
        _offline(session_factory)
        self.factory=session_factory;self.session=None;self.command=None;self.context=None

    def start(self, command, context):
        if self.session is not None:raise ValueError('Legacy session already active')
        self.command=command;self.context=context
        candidate=self.factory(command,context);_offline(candidate)
        self.session=candidate;self.session.start()

    def poll(self, command, now):
        if self.session is None or command['command_id']!=self.command['command_id']:
            raise ValueError('Legacy session command identity mismatch')
        completion=self.session.poll(now)
        if completion is not None or command['kind'] not in ('pickup_to_tray','load_machine'):
            return SkillTick(command['command_id'],completion=completion)
        control=self.session.controller
        sample=sample_from_legacy_control(control,inference_now_s=now,clock_offset_s=self.session.clock_offset_s)
        visual=self.session.visual_evidence(now)
        if command['kind']=='load_machine':
            speed=(sample.base_velocity_map[0]**2+sample.base_velocity_map[1]**2)**.5
            nominal=self.context['loading_adapter'].servo_actions(control.servo,control.measured_state(),control.env.route,
                    base_pose_map=sample.base_pose_map,robot_stamp_s=sample.robot_stamp_s,now=now,
                    measured_twist=[speed,sample.base_yaw_rate],visual_evidence=visual)
        else:
            # The existing source skill remains base-local, and source docking
            # preserves its original observation workspace. No residual here;
            # SkillExecutionPort adds the unchanged actor's bounded correction.
            nominal=control.action()
        return SkillTick(command['command_id'],sample,nominal,visual,completion)

    def record_actions(self, command_id, actions):
        if self.session is None or command_id!=self.command['command_id']:
            raise ValueError('Legacy action recording identity mismatch')
        self.session.record_actions(actions)

    def finish(self, command_id):
        if self.session is None or command_id!=self.command['command_id']:
            raise ValueError('Legacy finish identity mismatch')
        self.session.close();self.session=None;self.command=None;self.context=None

    def hold(self, command_id, reason, *, preserve_grip, stop_base):
        if self.session is not None:
            if command_id!=self.command['command_id']:raise ValueError('Legacy hold identity mismatch')
            self.session.hold(reason,preserve_grip=preserve_grip,stop_base=stop_base)
            self.session=None;self.command=None;self.context=None
