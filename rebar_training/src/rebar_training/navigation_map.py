"""Known-CAD chassis route validation for the isolated mobile test cell.

This is not a dynamic-obstacle/Nav2 planner. It checks the real console and
machine collider bounds, which the old hard-coded left detour intersected.
"""
from pathlib import Path
import math
import numpy as np


def machine_obstacles(assets,cell_translation=(5.92,4.87,0.),chassis_height=.60):
    from pxr import Usd,UsdGeom,UsdPhysics,Gf
    stage=Usd.Stage.CreateInMemory();cell=UsdGeom.Xform.Define(stage,'/Cell')
    cell.AddTranslateOp().Set(Gf.Vec3d(*cell_translation));cell.AddRotateZOp().Set(90.)
    for name in ('machine','fixture'):
        node=stage.DefinePrim('/Cell/'+name.title(),'Xform')
        node.GetReferences().AddReference(str((Path(assets)/(name+'.usda')).resolve()))
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy','guide'])
    result=[]
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or prim.GetAttribute('physics:collisionEnabled').Get() is False:continue
        box=cache.ComputeWorldBound(prim).ComputeAlignedBox()
        low,high=np.asarray(box.GetMin()),np.asarray(box.GetMax())
        if low[2]<chassis_height and high[2]>.02:
            result.append(dict(path=str(prim.GetPath()),minimum=low.tolist(),maximum=high.tolist()))
    if not result:raise RuntimeError('Machine map contains no chassis-height collision geometry')
    return result


def chassis_clear(pose,obstacles,half_size=(.5,.4),padding=.02):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    rotation=np.array([[c,-s],[s,c]])
    corners=np.array([[-1,-1],[-1,1],[1,1],[1,-1]],float)*(np.asarray(half_size)+padding)
    corners=corners@rotation.T+[x,y]
    axes=[np.array([1.,0]),np.array([0.,1]),rotation[:,0],rotation[:,1]]
    for obstacle in obstacles:
        lo,hi=np.asarray(obstacle['minimum'])[:2],np.asarray(obstacle['maximum'])[:2]
        rectangle=np.array([[lo[0],lo[1]],[lo[0],hi[1]],[hi[0],hi[1]],[hi[0],lo[1]]])
        separated=False
        for axis in axes:
            a,b=corners@axis,rectangle@axis
            if a.max()<=b.min() or b.max()<=a.min():separated=True;break
        if not separated:return False
    return True


def validated_detour(assets,cell_translation=(5.92,4.87,0.),dock=(6.3,4.77,0.)):
    obstacles=machine_obstacles(assets,cell_translation)
    # 1.10 m leaves turn clearance for the chassis and stowed arm/tray.
    left=min(o['minimum'][0] for o in obstacles)-1.10
    goals=[(left,0.,0.),(left,dock[1],math.pi/2),tuple(dock)]
    for a,b,yaw in (((0.,0.),goals[0][:2],0.),(goals[0][:2],goals[1][:2],math.pi/2),
                    (goals[1][:2],goals[2][:2],0.)):
        distance=np.linalg.norm(np.asarray(b)-a)
        for xy in np.linspace(a,b,max(2,math.ceil(distance/.025))):
            if not chassis_clear((*xy,yaw),obstacles):
                raise RuntimeError('CAD chassis collision on proposed navigation segment')
    for corner in goals[:2]:
        for yaw in np.linspace(-math.pi,math.pi,73):
            if not chassis_clear((*corner[:2],yaw),obstacles):
                raise RuntimeError('CAD chassis collision during a corner turn')
    if not chassis_clear(dock,obstacles):raise RuntimeError('Docking chassis pose intersects machine CAD')
    return goals,obstacles
