import math
from types import SimpleNamespace as S
import numpy as np
from aubo_control_gui.view_math import quaternion_from_rpy, rpy_from_quaternion, rotate_world, interpolate_trajectory


def test_orientation_round_trip_and_world_rotation():
    rpy=[.2,-.3,.8]
    q=quaternion_from_rpy(*rpy)
    assert np.allclose(rpy_from_quaternion(q),rpy)
    # Left-multiplication around world Z increments yaw, not local roll.
    assert np.allclose(rpy_from_quaternion(rotate_world(q,2,.4)),[.2,-.3,1.2])
    assert math.isclose(np.linalg.norm(rotate_world(q,0,.7)),1.)


def test_preview_interpolation_keeps_unplanned_joints_and_original_points():
    def point(t,x):return S(time_from_start=S(sec=t,nanosec=0),positions=[x],velocities=[])
    trajectory=S(joint_trajectory=S(joint_names=['arm'],points=[point(0,0),point(2,1)]))
    initial={'arm':0.,'finger':.04}
    assert interpolate_trajectory(trajectory,1,initial)=={'arm':.5,'finger':.04}
    assert interpolate_trajectory(trajectory,3,initial)=={'arm':1.,'finger':.04}
    assert trajectory.joint_trajectory.points[0].positions==[0]
    assert initial['arm']==0.
