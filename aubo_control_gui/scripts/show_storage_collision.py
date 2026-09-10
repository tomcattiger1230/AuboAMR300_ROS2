#!/usr/bin/env python3
"""Display a target robot as RViz markers, without publishing motion commands."""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from moveit_msgs.srv import GetPositionFK,GetStateValidity
from aubo_control_gui.motion_client import ARM,GRIPPER


def multiply(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return (w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z)

def rpy(values):
    r,p,y=[v/2 for v in values];cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return (sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy)

class TargetDisplay(Node):
    def __init__(self):
        super().__init__('student_target_collision_display')
        self.declare_parameter('target_name','存储位 1')
        self.declare_parameter('gripper_closed',False)
        share=Path(get_package_share_directory('aubo_control_gui'))
        self.data=json.loads((share/'config/student_key_positions.json').read_text())
        desc=Path(get_package_share_directory('seer_description'))/'urdf/composite_robot_stick_mono.urdf'
        self.links=[e for e in ET.parse(desc).getroot().findall('link') if e.find('visual') is not None]
        self.fk=self.create_client(GetPositionFK,'compute_fk')
        self.validity=self.create_client(GetStateValidity,'check_state_validity')
        self.pub=self.create_publisher(MarkerArray,'/diagnostics/student_target',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.key=None;self.pending=False;self.markers=None
        self.create_timer(.5,self.tick)
    @staticmethod
    def marker(index,kind):
        m=Marker();m.header.frame_id='base_footprint';m.ns='student_target';m.id=index;m.type=kind;m.action=Marker.ADD;m.pose.orientation.w=1.;return m
    def tick(self):
        key=(self.get_parameter('target_name').value,self.get_parameter('gripper_closed').value)
        if self.pending:return
        if key==self.key:
            if self.markers is not None:self.pub.publish(self.markers)
            return
        if not self.fk.service_is_ready() or not self.validity.service_is_ready():return
        if key[0] not in self.data['joint_targets']:
            self.get_logger().error('Unknown target name');self.key=key;self.markers=None;self.pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]));return
        self.pending=True;self.key=key
        req=GetPositionFK.Request();req.header.frame_id='base_footprint';req.fk_link_names=[e.get('name') for e in self.links]
        req.robot_state.joint_state.name=list(ARM+GRIPPER)
        req.robot_state.joint_state.position=[math.radians(v) for v in self.data['joint_targets'][key[0]]]+[.04 if key[1] else 0.]*2
        req.robot_state.is_diff=True
        future=self.fk.call_async(req)
        def done(f):
            try:
                result=f.result()
                if result.error_code.val!=1:raise RuntimeError('FK failed')
                poses=dict(zip(result.fk_link_names,result.pose_stamped));markers=[]
                for link in self.links:
                    name=link.get('name');base=poses[name].pose;q=(base.orientation.x,base.orientation.y,base.orientation.z,base.orientation.w)
                    for visual in link.findall('visual'):
                        geom=visual.find('geometry');mesh=geom.find('mesh');box=geom.find('box');cyl=geom.find('cylinder');sphere=geom.find('sphere')
                        if mesh is not None:kind=Marker.MESH_RESOURCE
                        elif box is not None:kind=Marker.CUBE
                        elif cyl is not None:kind=Marker.CYLINDER
                        elif sphere is not None:kind=Marker.SPHERE
                        else:continue
                        m=self.marker(len(markers),kind);o=visual.find('origin')
                        xyz=[float(v) for v in (o.get('xyz','0 0 0') if o is not None else '0 0 0').split()]
                        angles=[float(v) for v in (o.get('rpy','0 0 0') if o is not None else '0 0 0').split()]
                        offset=multiply(multiply(q,(*xyz,0.)),(-q[0],-q[1],-q[2],q[3]))
                        m.pose.position.x=base.position.x+offset[0];m.pose.position.y=base.position.y+offset[1];m.pose.position.z=base.position.z+offset[2]
                        rot=multiply(q,rpy(angles));m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=rot
                        if mesh is not None:m.mesh_resource=mesh.get('filename');dims=list(map(float,mesh.get('scale','1 1 1').split()))
                        elif box is not None:dims=list(map(float,box.get('size').split()))
                        elif cyl is not None:dims=[2*float(cyl.get('radius'))]*2+[float(cyl.get('length'))]
                        else:dims=[2*float(sphere.get('radius'))]*3
                        m.scale.x,m.scale.y,m.scale.z=dims
                        if name in ['gripper1_link','gripper2_link']:color=(1.,.1,.1,.95)
                        elif name=='base_link':color=(.35,.6,.85,.35)
                        else:color=(.9,.65,.2,.5)
                        m.color.r,m.color.g,m.color.b,m.color.a=color;markers.append(m)
                v=GetStateValidity.Request();v.robot_state=req.robot_state;v.group_name='arm'
                future=self.validity.call_async(v)
                def checked(f):
                    try:
                        res=f.result()
                        for contact in res.contacts:
                            m=self.marker(len(markers),Marker.SPHERE);m.header=contact.header;m.pose.position=contact.position
                            m.scale.x=m.scale.y=m.scale.z=.035;m.color.r=1.;m.color.g=1.;m.color.a=1.;markers.append(m)
                        m=self.marker(len(markers),Marker.TEXT_VIEW_FACING);m.pose.position.z=1.45;m.scale.z=.025
                        m.color.r=m.color.g=m.color.b=m.color.a=1.
                        label='Storage '+key[0].split()[-1] if key[0].startswith('存储') else 'Placement'
                        m.text=f"{label}\nTARGET ONLY\nNOT EXECUTED"
                        markers.append(m)
                        self.pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
                        self.markers=MarkerArray(markers=markers);self.pub.publish(self.markers)
                        self.get_logger().info(f'{key[0]} target display: valid={res.valid}, contacts={len(res.contacts)}')
                    except Exception as exc:self.get_logger().error(str(exc));self.key=None
                    finally:self.pending=False
                future.add_done_callback(checked)
            except Exception as exc:
                self.get_logger().error(str(exc));self.pending=False;self.key=None
        future.add_done_callback(done)

def main():
    rclpy.init();node=TargetDisplay()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
