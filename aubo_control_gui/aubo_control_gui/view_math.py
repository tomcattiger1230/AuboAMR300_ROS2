"""Coordinate transforms shared by the mouse gizmo and trajectory preview."""
import math
import numpy as np


def quaternion_multiply(a, b):
    x,y,z,w = a; X,Y,Z,W = b
    q = np.array([w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
                  w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z])
    return (q/np.linalg.norm(q)).tolist()


def rotate_world(q, axis, angle):
    delta = [0.,0.,0.,math.cos(angle/2)]
    delta[axis] = math.sin(angle/2)
    return quaternion_multiply(delta, q)


def interpolate_trajectory(trajectory, seconds, initial):
    jt = trajectory.joint_trajectory
    values = dict(initial)
    times = [p.time_from_start.sec+p.time_from_start.nanosec*1e-9 for p in jt.points]
    if not times:
        return values
    index = int(np.searchsorted(times, seconds, side='right'))
    if index == 0:
        positions = jt.points[0].positions
    elif index >= len(times):
        positions = jt.points[-1].positions
    else:
        a,b = jt.points[index-1],jt.points[index]
        t = (seconds-times[index-1])/max(1e-9,times[index]-times[index-1])
        # Interpolate stored points for display; execution sends the original trajectory.
        dt = times[index]-times[index-1]
        if len(a.velocities)==len(a.positions) and len(b.velocities)==len(b.positions):
            positions = [(2*t**3-3*t**2+1)*x+(t**3-2*t**2+t)*dt*vx+
                         (-2*t**3+3*t**2)*y+(t**3-t**2)*dt*vy
                         for x,y,vx,vy in zip(a.positions,b.positions,a.velocities,b.velocities)]
        else:
            positions = [(1-t)*x+t*y for x,y in zip(a.positions,b.positions)]
    values.update(zip(jt.joint_names, positions))
    return values


def quaternion_from_rpy(r, p, y):
    cr,sr,cp,sp,cy,sy=math.cos(r/2),math.sin(r/2),math.cos(p/2),math.sin(p/2),math.cos(y/2),math.sin(y/2)
    return [sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]


def rpy_from_quaternion(q):
    x,y,z,w=q
    return [math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(max(-1,min(1,2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))]
