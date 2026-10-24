"""Offline probe of the original encoder function, without importing its simulator module."""
import ast
from functools import lru_cache
import hashlib
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from .workcells import Pose

ROOT=Path(__file__).parents[2]
LEGACY_SOURCE_SHA='d2985ce3200eea0529bd3963c933683faefed41d836e7293b0da0fe60a82c3af'

@lru_cache(maxsize=1)
def _compiled_method():
    source=Path(__file__).with_name('mobile_env.py').read_bytes()
    if hashlib.sha256(source).hexdigest()!=LEGACY_SOURCE_SHA:
        raise ValueError('Legacy input source changed; review compatibility reference before probing')
    tree=ast.parse(source)
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MobileControl')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='learning_observation')
    namespace=dict(np=np,array=np.asarray,__package__='rebar_training')
    exec(compile(ast.Module(body=[method],type_ignores=[]),'<unchanged learning_observation>','exec'),namespace)
    return namespace['learning_observation']

def legacy_control_fixture(sample, layout):
    base=Pose.parse(sample.base_pose_map);c,s=math.cos(base.yaw),math.sin(base.yaw)
    T=np.array([[c,-s,0,base.x],[s,c,0,base.y],[0,0,1,base.z],[0,0,0,1]],dtype=float)
    f32=lambda x:np.asarray(x,dtype=np.float32)
    data=SimpleNamespace(joint_pos=f32([sample.q]),joint_vel=f32([sample.dq]),
                         root_lin_vel_w=f32([sample.base_velocity_map]),root_ang_vel_w=f32([[0,0,sample.base_yaw_rate]]))
    env=SimpleNamespace(robot=SimpleNamespace(data=data),control_ids=list(range(6)),
                        task_spec=SimpleNamespace(control_dt=1/30),route=SimpleNamespace(stage=np.array([sample.route])))
    packet=SimpleNamespace(timestamp_ns=round(sample.robot_stamp_s*1e9),tcp=np.asarray(sample.tcp_base),quaternion_xyzw=np.asarray(sample.tcp_quaternion_xyzw),
                           fingers=f32(sample.fingers_m),finger_forces=f32(sample.finger_forces_n),jaw_forces=f32(sample.jaw_forces_n))
    state=SimpleNamespace(tcp=f32([sample.tcp_base]),bar=f32([sample.bar_base]),bar_axis=f32([sample.bar_axis_base]),
                          linear_velocity=f32([sample.bar_velocity_base]),angular_velocity=f32([sample.bar_angular_velocity_base]))
    machine=next(s.id for s in layout.stations.values() if layout.profiles[s.profile_id].kind=='machine')
    goal=layout.machine_geometry(machine,refined_pose=layout.stations[machine].pose_map,base_pose=base)['goal']
    control=SimpleNamespace(env=env,observers=[SimpleNamespace(acquired=True,last_good_ns=round(sample.image_stamp_s*1e9))],
             measured_state=lambda:state,robot_packets=lambda timestamp:[packet],tick=sample.now*30,
             frame=lambda:SimpleNamespace(world_from_base=T),stage=sample.phase,valid=np.array([sample.vision_valid]),
             grasp_recovery=SimpleNamespace(pending=sample.recovery_pending),motion=SimpleNamespace(ready=np.array([sample.motion_ready])),
             path={k:None for k in ('tray_raise','tray_rotate','tray_via_side','tray_via_front','tray_above','tray_down','tray_lift_high')},
             tray_force=sample.tray_load_n,goal=lambda:goal)
    return control


def legacy_observation(sample, layout):
    return _compiled_method()(legacy_control_fixture(sample,layout))


