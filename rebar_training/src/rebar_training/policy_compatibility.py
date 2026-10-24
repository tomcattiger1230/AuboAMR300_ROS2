"""Versioned station-relative input adapter and read-only frozen residual inference.

The actor still consumes mobile_visual_manipulation_residual_v1, 76 floats. The
new outer adapter has its own identity: source skills use the original surveyed
destination as *policy context*, not as a physical motion target. Loading uses
the actual confirmed fixture in the robot base frame. Never route policy-context
coordinates to navigation or the Cartesian servo.
"""
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path

from .workcells import Pose, finite, FROZEN_POLICY_SHA256, wrap
from .workcell_mission import PICKUP_STAGES, LOADING_STAGES, _fresh

ADAPTER_CONTRACT = 'station_relative_legacy_context_v1'
REFERENCE_SOURCE = Pose(-1.15, 0., 0., 0.)
REFERENCE_MACHINE = Pose(6., 4., 0., 0.)
# Original MobileControl switches pickup->tray_raise at route 4, and
# pickup_tray->tray_lift_high at route 4; LoadingRoute never uses route 5.
SUPPORTED_ROUTES = {**{phase:(4,) for phase in PICKUP_STAGES[1:-1]},
                    'pickup':(0,1,2,3),'pickup_tray':(0,1,2,3),
                    'tray_lift_high':(4,),'loading':(4,6,7,8,9,10)}


@dataclass(frozen=True)
class PolicySample:
    """Sensor/FK/vision measurements only. Position/axis/velocities are in base."""
    phase: str
    route: int
    now: float
    robot_stamp_s: float
    image_stamp_s: float
    base_pose_map: tuple
    q: tuple
    dq: tuple
    tcp_base: tuple
    tcp_quaternion_xyzw: tuple
    bar_base: tuple
    bar_axis_base: tuple
    bar_velocity_base: tuple
    bar_angular_velocity_base: tuple
    fingers_m: tuple
    finger_forces_n: tuple
    jaw_forces_n: tuple
    tray_load_n: float
    base_velocity_map: tuple
    base_yaw_rate: float
    vision_valid: bool
    motion_ready: bool
    recovery_pending: bool


@dataclass(frozen=True)
class CompatibleInput:
    contract: str
    values: tuple
    active: bool
    phase: str
    route: int
    canonical_base_map: tuple
    policy_context_goal_base: tuple
    actual_machine_goal_base: tuple
    source_destination_context_rebased: bool


class StationPolicyAdapter:
    def __init__(self, layout, source_id, machine_id, *, refined_machine_pose=None):
        self.layout = layout;self.source = layout.station(source_id, 'source')
        self.machine = layout.station(machine_id, 'machine')
        self.machine_pose = refined_machine_pose or self.machine.pose_map
        if not isinstance(self.machine_pose, Pose):
            raise ValueError('A planar confirmed machine pose is required')
        if (math.dist(self.machine_pose.values()[:3],self.machine.pose_map.values()[:3])>.25
                or abs(wrap(self.machine_pose.yaw-self.machine.pose_map.yaw))>math.radians(15)):
            raise ValueError('Machine refinement is outside the registered station')
        source = layout.profiles[self.source.profile_id];machine = layout.profiles[self.machine.profile_id]
        if (abs(layout.diameter_m-.024)>1e-9 or abs(layout.length_m-.6)>1e-9
                or abs(source.support_height_m-.522)>1e-9
                or machine.fixture_goal!=(0.,0.,1.508) or machine.bar_axis!=(0.,0.,1.)
                or machine.standoff_offset!=(0.,.06,0.) or machine.retreat_offset!=(0.,0.,.1)):
            raise ValueError('Geometry differs from the frozen skill contract')

    def encode(self, sample):
        from .mobile_residual import PHASES, pack_observation
        s = sample
        if s.phase not in PHASES or type(s.route) is not int or not 0 <= s.route <= 10:
            raise ValueError('Use legacy phase names and route 0..10; never task-step indexes')
        if s.route not in SUPPORTED_ROUTES.get(s.phase,()):
            raise ValueError('Legacy phase/route combination is not reachable')
        for name in ('vision_valid', 'motion_ready', 'recovery_pending'):
            if type(getattr(s, name)) is not bool:
                raise ValueError('Measured flags must be booleans')
        now, = finite([s.now], 1, 'now');_fresh(s.robot_stamp_s, now, 0);_fresh(s.image_stamp_s, now, 0)
        base = Pose.parse(s.base_pose_map)
        station_id=self.source.id if s.phase in PICKUP_STAGES else self.machine.id
        dock=self.layout.navigation_goal(station_id)
        if math.dist(base.values()[:3],dock.values()[:3])>.1 or abs(wrap(base.yaw-dock.yaw))>.1:
            raise ValueError('Manipulation input is outside the station docking envelope')
        source_phase = s.phase in PICKUP_STAGES and s.phase != 'observe_tray'
        loading_phase = s.phase in LOADING_STAGES
        if not source_phase and not loading_phase:
            raise ValueError('Residual inference is not used for observation, navigation or docking')
        local_station = self.source.pose_map if source_phase else self.machine_pose
        reference = REFERENCE_SOURCE if source_phase else REFERENCE_MACHINE
        canonical_xyz = reference.point(local_station.inverse_point(base.values()[:3]))
        canonical = Pose(*canonical_xyz, base.yaw-local_station.yaw+reference.yaw)
        actual = self.layout.machine_geometry(self.machine.id, refined_pose=self.machine_pose, base_pose=base)['goal']
        # This is a deliberate, documented policy-context adaptation for source
        # skills. Actual navigation and servo targets remain in the actual map.
        context = (canonical.inverse_point(REFERENCE_MACHINE.point((0, 0, 1.508)))
                   if source_phase else actual)
        q = finite(s.q, 6, 'q');dq = finite(s.dq, 6, 'dq');tcp = finite(s.tcp_base, 3, 'tcp')
        quat = finite(s.tcp_quaternion_xyzw, 4, 'tcp quaternion')
        axis = finite(s.bar_axis_base, 3, 'bar axis')
        if abs(sum(x*x for x in quat)-1)>1e-5 or abs(sum(x*x for x in axis)-1)>1e-5:
            raise ValueError('Measured orientations must be normalised')
        bar = finite(s.bar_base, 3, 'bar');v = finite(s.bar_velocity_base, 3, 'bar velocity')
        av = finite(s.bar_angular_velocity_base, 3, 'bar angular velocity')
        fingers = finite(s.fingers_m, 2, 'fingers');finger_force = finite(s.finger_forces_n, 2, 'finger force')
        jaw = finite(s.jaw_forces_n, 4, 'jaw force');tray, = finite([s.tray_load_n], 1, 'tray load')
        if min((*finger_force, *jaw, tray))<0:
            raise ValueError('Negative contact magnitudes')
        speed = finite(s.base_velocity_map, 3, 'base velocity');yaw_rate, = finite([s.base_yaw_rate], 1, 'yaw rate')
        phase = [float(name==s.phase) for name in PHASES];route = [float(i==s.route) for i in range(11)]
        fields = dict(q=[x/math.pi for x in q], dq=[x/.5 for x in dq], tcp=[x/2 for x in tcp],
                      tcp_quaternion=quat, bar_minus_tcp=[a-b for a,b in zip(bar,tcp)], bar_axis=axis,
                      bar_velocity=[x/.1 for x in v], bar_angular_velocity=[x/.5 for x in av],
                      fingers=[x/.0285 for x in fingers], finger_load=[x/50 for x in finger_force],
                      jaw_load=[x/50 for x in jaw], tray_load=[tray/25],
                      machine_goal_minus_tcp=[(a-b)/5 for a,b in zip(context,tcp)],
                      image_age=[min(now-s.image_stamp_s,1.)], valid=[float(s.vision_valid)],
                      phase=phase, route=route, base_xy=[canonical.x/10,canonical.y/10],
                      base_velocity=[math.hypot(*speed[:2])/.15,yaw_rate/.2])
        alignment_hold = s.phase in ('pickup','pickup_tray') and s.route==0
        path_phase = s.phase in PICKUP_STAGES[1:-1]
        active = (s.vision_valid and not s.recovery_pending and not alignment_hold
                  and (path_phase or s.phase in ('pickup','pickup_tray','loading') and s.motion_ready))
        # tray_lift_high is an original Cartesian-path phase too.
        active = active or (s.phase=='tray_lift_high' and s.vision_valid and not s.recovery_pending)
        return CompatibleInput(ADAPTER_CONTRACT, tuple(pack_observation(fields)), bool(active), s.phase, s.route,
                               canonical.values(), tuple(context), tuple(actual), source_phase)


def _tensor_fingerprint(value):
    import torch
    h=hashlib.sha256();count=0
    def visit(v):
        nonlocal count
        if isinstance(v, torch.Tensor):
            if not torch.isfinite(v).all():
                raise ValueError('Nonfinite frozen tensor')
            count+=1;h.update(str(v.dtype).encode());h.update(str(tuple(v.shape)).encode())
            h.update(v.detach().cpu().contiguous().numpy().tobytes())
        elif isinstance(v, dict):
            for key in sorted(v,key=lambda x:(type(x).__name__,str(x))):
                h.update((type(key).__name__+repr(key)).encode());visit(v[key])
        elif isinstance(v,(list,tuple)):
            for item in v:visit(item)
        else:h.update(repr(v).encode())
    visit(value)
    return h.hexdigest(),count


class FrozenStationResidual:
    """Inference only. No optimizer object, learner, rollout, update or save API."""
    def __init__(self, checkpoint):
        import torch
        from .mobile_residual import ResidualActorCritic, CONTRACT, FIELDS, REWARD_CONTRACT, DISCOUNT
        self.path=Path(checkpoint);self.file_sha=hashlib.sha256(self.path.read_bytes()).hexdigest()
        if self.file_sha!=FROZEN_POLICY_SHA256:
            raise ValueError('Frozen policy/optimizer file SHA mismatch')
        saved=torch.load(self.path,map_location='cpu',weights_only=True)
        if (saved.get('observation_contract')!=CONTRACT or saved.get('fields')!=list(FIELDS)
                or saved.get('residual_scale')!=.08 or saved.get('reward_contract')!=REWARD_CONTRACT
                or saved.get('gamma')!=DISCOUNT):
            raise ValueError('Frozen deployment contract mismatch')
        self.payload_sha,self.tensor_count=_tensor_fingerprint(saved)
        self.counters={k:saved[k] for k in ('updates','actor_updates','policy_steps')}
        if self.counters!=dict(updates=466,actor_updates=170,policy_steps=36602):
            raise ValueError('Frozen training counters differ')
        with torch.random.fork_rng(devices=[]), torch.device('cpu'):
            self.model=ResidualActorCritic().to(device='cpu',dtype=torch.float32)
        self.model.load_state_dict(saved['model'],strict=True);self.model.eval();self.model.requires_grad_(False)
        self.model_sha,_=_tensor_fingerprint(self.model.state_dict())
        # Keep no mutable optimizer state or training object in this wrapper.
        del saved

    def infer(self, packet):
        import torch
        from .mobile_residual import precision_residual_gain, OBS_SIZE
        if self.model.training or any(p.requires_grad for p in self.model.parameters()):
            raise ValueError('Frozen actor must remain eval-only with gradients disabled')
        if packet.contract!=ADAPTER_CONTRACT or len(packet.values)!=OBS_SIZE:
            raise ValueError('Compatible input contract mismatch')
        if not all(math.isfinite(float(x)) for x in packet.values):
            raise ValueError('Nonfinite actor input')
        if not packet.active:
            return dict(active=False, raw=[0.]*6, gain=0., scaled_delta=[0.]*6)
        with torch.inference_mode():
            raw=self.model(torch.tensor([packet.values],dtype=torch.float32,device='cpu'))[0]
        if not torch.isfinite(raw).all():
            raise ValueError('Nonfinite frozen actor output')
        gain=precision_residual_gain(packet.values)
        return dict(active=True,raw=raw.tolist(),gain=gain,scaled_delta=(.08*gain*raw).tolist())

    def verify_unchanged(self):
        import torch
        if hashlib.sha256(self.path.read_bytes()).hexdigest()!=self.file_sha:
            raise ValueError('Frozen checkpoint changed')
        if _tensor_fingerprint(self.model.state_dict())[0]!=self.model_sha:
            raise ValueError('In-memory frozen model changed')
        saved=torch.load(self.path,map_location='cpu',weights_only=True)
        if _tensor_fingerprint(saved)[0]!=self.payload_sha:
            raise ValueError('Model/optimizer payload changed')
        return dict(sha256=self.file_sha,model_optimizer_tensors=self.tensor_count,counters=self.counters,
                    file_and_tensor_payload_unchanged=True,optimizer_instantiated=False,training_started=False)


def compose_residual_actions(nominal, decision):
    """Return a copy of the existing 7-D servo command; never alter the gripper."""
    import torch
    if not isinstance(nominal,torch.Tensor) or nominal.shape!=(1,7) or not torch.isfinite(nominal).all():
        raise ValueError('Expected finite 1x7 nominal actions')
    if type(decision.get('active')) is not bool:
        raise ValueError('Residual active flag must be boolean')
    result=nominal.clone()
    if decision['active']:
        delta=torch.as_tensor(decision['scaled_delta'],dtype=result.dtype,device=result.device)
        if delta.shape!=(6,) or not torch.isfinite(delta).all() or delta.abs().max()>.080001:
            raise ValueError('Residual exceeds the original .08 envelope')
        result[:,:6]=(result[:,:6]+delta).clamp(-1,1)
    return result


def sample_from_legacy_control(control, *, inference_now_s, clock_offset_s=0.):
    """Read existing controller sensor/FK/vision interfaces, with a fixed clock offset.

    No steel truth/provider or evaluation receipt is accessed. The offset is a
    session clock calibration, never recomputed per frame to hide stale sensors.
    This function does not call reset, after_physics, env.step or control.action.
    """
    import numpy as np
    from .vision.capture import array
    offset,=finite([clock_offset_s],1,'clock offset')
    now,=finite([inference_now_s],1,'inference clock')
    e=control.env;observer=control.observers[0]
    timestamp=round(control.tick*e.task_spec.control_dt*1e9)
    packet=control.robot_packets(timestamp)[0];state=control.measured_state()
    frame=control.frame().world_from_base
    if np.asarray(frame).shape!=(4,4) or not np.isfinite(frame).all():
        raise ValueError('Invalid measured base frame')
    R=np.asarray(frame)[:3,:3]
    if (not np.allclose(R.T@R,np.eye(3),atol=1e-5) or np.linalg.det(R)<.99999
            or not np.allclose(np.asarray(frame)[3],(0,0,0,1),atol=1e-8)
            or max(abs(R[2,0]),abs(R[2,1]),abs(R[0,2]),abs(R[1,2]),abs(R[2,2]-1))>1e-5):
        raise ValueError('Legacy policy bridge requires a rigid planar chassis frame')
    def row(value):
        a=array(value)
        if a.shape[0]!=1:raise ValueError('Policy bridge requires one robot')
        return tuple(float(x) for x in a[0])
    joint=array(e.robot.data.joint_pos);velocity=array(e.robot.data.joint_vel)
    if joint.shape[0]!=1 or velocity.shape!=joint.shape:raise ValueError('Invalid robot joint feedback')
    base=Pose(*map(float,frame[:3,3]),float(np.arctan2(frame[1,0],frame[0,0])))
    yaw_rate=float(array(e.robot.data.root_ang_vel_w)[0,2])
    image_stamp=observer.last_good_ns*1e-9+offset if observer.acquired else -math.inf
    return PolicySample(control.stage,int(e.route.stage[0]),now,packet.timestamp_ns*1e-9+offset,image_stamp,
        base.values(),tuple(map(float,joint[0,e.control_ids[:6]])),tuple(map(float,velocity[0,e.control_ids[:6]])),
        tuple(map(float,packet.tcp)),tuple(map(float,packet.quaternion_xyzw)),row(state.bar),row(state.bar_axis),
        row(state.linear_velocity),row(state.angular_velocity),tuple(map(float,packet.fingers)),
        tuple(map(float,packet.finger_forces)),tuple(map(float,packet.jaw_forces)),float(control.tray_force),
        row(e.robot.data.root_lin_vel_w),yaw_rate,bool(control.valid[0]),bool(control.motion.ready[0]),
        bool(control.grasp_recovery.pending))
