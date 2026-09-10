#!/usr/bin/env python3
"""Offline triangle-mesh audit: no ROS nodes, controller or simulation motion.

Requires numpy, scipy, trimesh, python-fcl, matplotlib and pxr.
"""
import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Rectangle
from pxr import Usd, UsdGeom, UsdPhysics, Gf

ARM=['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint']

def origin(element):
    m=np.eye(4)
    if element is not None:
        m[:3,3]=np.fromstring(element.get('xyz','0 0 0'),sep=' ')
        m[:3,:3]=Rotation.from_euler('xyz',np.fromstring(element.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return m

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',type=Path,default=Path.cwd())
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();repo=args.repo;args.output.mkdir(parents=True,exist_ok=True)
    urdf=ET.parse(repo/'seer_description/urdf/composite_robot_stick_mono.urdf').getroot()
    links={e.get('name'):e for e in urdf.findall('link')}
    targets=json.loads((repo/'aubo_control_gui/config/student_key_positions.json').read_text())
    def transform(angles):
        poses={'base_footprint':np.eye(4)};pending=list(urdf.findall('joint'))
        while pending:
            progressed=False
            for j in list(pending):
                parent=j.find('parent').get('link')
                if parent not in poses:continue
                motion=np.eye(4);value=angles.get(j.get('name'),0.)
                axis=j.find('axis');axis=np.fromstring(axis.get('xyz','0 0 1'),sep=' ') if axis is not None else np.array([0.,0.,1.])
                if j.get('type') in ['revolute','continuous']:motion[:3,:3]=Rotation.from_rotvec(axis*value).as_matrix()
                elif j.get('type')=='prismatic':motion[:3,3]=axis*value
                poses[j.find('child').get('link')]=poses[parent]@origin(j.find('origin'))@motion
                pending.remove(j);progressed=True
            if not progressed:raise ValueError('Unresolved URDF joint tree')
        return poses
    def mesh(link,kind='visual'):
        e=links[link].find(kind);geo=e.find('geometry');m=geo.find('mesh')
        if m is not None:
            path=repo/m.get('filename').replace('package://','')
            result=trimesh.load(path,force='mesh');result.apply_scale(np.fromstring(m.get('scale','1 1 1'),sep=' '))
        elif geo.find('box') is not None:result=trimesh.creation.box(np.fromstring(geo.find('box').get('size'),sep=' '))
        elif geo.find('cylinder') is not None:
            c=geo.find('cylinder');result=trimesh.creation.cylinder(radius=float(c.get('radius')),height=float(c.get('length')))
        else:raise ValueError(link)
        result.apply_transform(origin(e.find('origin')))
        return result
    report={'frame':'base_footprint','method':'FCL BVH triangle-mesh intersection, no convex hull for the body visual',
        'model_hashes':{},'local_geometry':{},'usd_mesh_agreement':{},'targets':[]}
    for f in ['gripper_stick.stl','robot_body.stl','motor.stl','adapter.stl']:
        report['model_hashes'][f]=hashlib.sha256((repo/'seer_description/meshes'/f).read_bytes()).hexdigest()
    for link in ['base_link','gripper1_link','gripper2_link','gripper_motor_link','gripper_adapter_link']:
        report['local_geometry'][link]={k:mesh(link,k).bounds.tolist() for k in ['visual','collision']}
    neutral=transform({});body=mesh('base_link');body.apply_transform(neutral['base_link'])
    body_collision=mesh('base_link','collision');body_collision.apply_transform(neutral['base_link'])
    managers={}
    for name,m in [('visual',body),('box',body_collision)]:
        manager=trimesh.collision.CollisionManager();manager.add_object('base',m);managers[name]=manager
    stage=Usd.Stage.Open(str(repo/'seer_description/urdf/seer_aubo_stick_mono.usda'));cache=UsdGeom.XformCache()
    for name in ['gripper1_link','gripper2_link']:
        prim=stage.GetPrimAtPath('/World/seer_aubo_composite/'+name)
        inverse=cache.GetLocalToWorldTransform(prim).GetInverse();rows=[]
        for p in Usd.PrimRange(prim,Usd.TraverseInstanceProxies()):
            if not p.IsA(UsdGeom.Mesh):continue
            matrix=cache.GetLocalToWorldTransform(p)*inverse
            points=np.array([matrix.Transform(Gf.Vec3d(v)) for v in UsdGeom.Mesh(p).GetPointsAttr().Get()])
            bound=np.array([points.min(0),points.max(0)])
            rows.append({'prim':str(p.GetPath()),'collision':p.HasAPI(UsdPhysics.CollisionAPI),
                         'bounds':bound.tolist(),'urdf_bound_max_error_m':float(np.max(np.abs(bound-mesh(name).bounds)))})
        report['usd_mesh_agreement'][name]=rows
    fig,axes=plt.subplots(2,2,figsize=(12,9),constrained_layout=True)
    for index in range(1,5):
        name=f'存储位 {index}';entry={'name':name,'gripper_states':{}};report['targets'].append(entry)
        for label,value in [('open',0.),('closed',.04)]:
            angles=dict(zip(ARM,np.radians(targets['joint_targets'][name])));angles.update(gripper1_joint=value,gripper2_joint=value)
            poses=transform(angles);rows={}
            for finger in ['gripper1_link','gripper2_link']:
                m=mesh(finger);m.apply_transform(poses[finger]);row={'bounds':m.bounds.tolist()}
                for kind,manager in managers.items():
                    intersects,contacts=manager.in_collision_single(m,return_data=True)
                    row[kind]={'intersects':bool(intersects),'contact_count':len(contacts),
                        'sample_contact_points':[c.point.tolist() for c in contacts[:8]]}
                rows[finger]=row
                if label=='open':
                    ax=axes.flat[index-1]
                    ax.add_collection(PolyCollection(m.triangles[:,:,[0,2]],facecolors='#d84a43',edgecolors='none',alpha=.30))
            entry['gripper_states'][label]=rows
        ax=axes.flat[index-1]
        ax.add_collection(PolyCollection(body.triangles[:,:,[0,2]],facecolors='#547fa5',edgecolors='none',alpha=.025,zorder=0))
        ax.add_patch(Rectangle((-.5,.1),1.,.5,fill=False,linestyle='--',edgecolor='black',linewidth=1.5))
        ax.set(xlim=(-.65,.65),ylim=(0,1.15),xlabel='base X (m)',ylabel='base Z (m)',title=f'Storage {index}: fingers (red), body mesh (blue), collision box (dashed)')
        ax.set_aspect('equal');ax.grid(alpha=.2)
        print(name,json.dumps(entry['gripper_states'],ensure_ascii=False),flush=True)
    report['body_visual_watertight']=body.is_watertight
    fig.savefig(args.output/'storage_collision_sideviews.png',dpi=160)
    (args.output/'storage_geometry_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')

if __name__=='__main__':main()
