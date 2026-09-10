"""Qt Quick 3D preview of the i16 arm and the simulation gripper."""
from __future__ import annotations
import math
import sys
import xml.etree.ElementTree as ET
import numpy as np
from PySide6.QtCore import QUrl, Signal
from PySide6.QtGui import QMatrix3x3, QQuaternion
from .resources import get_package_share_directory
from .tool_preview import tool_visuals, origin as urdf_origin
from .view_math import rotate_world
from pathlib import Path
from PySide6.QtQuickWidgets import QQuickWidget

def _rpy(r,p,y):
    cx,sx,cy,sy,cz,sz=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx],[sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx],[-sy,cy*sx,cy*cx]])
def _axis_angle(axis,angle):
    axis=np.asarray(axis,float); axis/=np.linalg.norm(axis); x,y,z=axis; c,s=math.cos(angle),math.sin(angle); C=1-c
    return np.array([[c+x*x*C,x*y*C-z*s,x*z*C+y*s],[y*x*C+z*s,c+y*y*C,y*z*C-x*s],[z*x*C-y*s,z*y*C+x*s,c+z*z*C]])
def _transform(xyz=(0,0,0),rot=None):
    m=np.eye(4); m[:3,:3]=np.eye(3) if rot is None else rot; m[:3,3]=xyz; return m

class UrdfRobotView(QQuickWidget):
    target_changed = Signal(object)
    """Loads arm meshes and attaches URDF-derived tool visuals to wrist3."""
    def __init__(self,urdf_path,qml_path,parent=None):
        super().__init__(parent); self.setMinimumSize(560,520); self.setResizeMode(QQuickWidget.SizeRootObjectToView)
        self.joints=[]; self.angles=[0.0]*6
        self.target_pose = None
        root=ET.parse(urdf_path).getroot()
        for joint in root.findall("joint"):
            if joint.get("type") not in ("revolute","continuous"): continue
            origin,axis=joint.find("origin"),joint.find("axis")
            xyz=tuple(map(float,(origin.get("xyz","0 0 0") if origin is not None else "0 0 0").split()))
            rpy=tuple(map(float,(origin.get("rpy","0 0 0") if origin is not None else "0 0 0").split()))
            av=tuple(map(float,(axis.get("xyz","0 0 1") if axis is not None else "0 0 1").split()))
            self.joints.append((joint.get("name","joint"),xyz,rpy,av))
        self.setSource(QUrl.fromLocalFile(str(qml_path)))
        if self.status()!=QQuickWidget.Ready: raise RuntimeError("Qt Quick 3D 视图加载失败: "+"; ".join(str(e) for e in self.errors()))
        mesh_dir=urdf_path.parent.parent/"meshes"/"aubo_i16"/"visual"
        if sys.platform == "darwin":
            self.rootObject().setProperty("meshExtension", "glb")
        self.rootObject().setProperty("meshRoot",QUrl.fromLocalFile(str(mesh_dir)))
        description = Path(get_package_share_directory("seer_description"))
        composed = ET.parse(description / "urdf/composite_robot_stick_mono.urdf").getroot()
        fixed = {j.find('child').get('link'): j for j in composed.findall('joint') if j.get('type') == 'fixed'}
        def mount(link):
            if link == 'base_footprint': return np.eye(4)
            joint = fixed[link]
            return mount(joint.find('parent').get('link')) @ urdf_origin(joint.find('origin'))
        self.planning_to_scene = _transform((0,-45,0), _rpy(-math.pi/2,0,0)) @ np.diag([100,100,100,1]) @ np.linalg.inv(mount('aubo_base_link'))
        basis = [self.planning_to_scene[:3,i].tolist() for i in range(3)]
        self.rootObject().setProperty('targetBasis', (np.array(basis)/100).tolist())
        self.rootObject().targetDrag.connect(self._drag_target)
        self.tool_urdf = description / "urdf/composite_robot_stick_mono.urdf"
        self.rootObject().setProperty("toolMeshRoot", QUrl.fromLocalFile(str(qml_path.parent.parent / "meshes/gripper")))
        self._load_tools()

    def _load_tools(self, camera_mount=None):
        visuals = tool_visuals(self.tool_urdf, camera_mount)
        self.camera_center = np.array(next(v['position'] for v in visuals if v['link'] == 'camera_link'))
        for visual in visuals:
            visual['quaternion'] = QQuaternion.fromRotationMatrix(QMatrix3x3(np.array(visual['rotation']).flatten().tolist()))
        self.rootObject().setProperty("toolVisuals", visuals)
    def set_gripper_positions(self, positions):
        if len(positions) != 2 or not all(math.isfinite(v) for v in positions):
            return
        root = self.rootObject()
        if root:
            root.setProperty("finger1", float(positions[0]))
            root.setProperty("finger2", float(positions[1]))
    def set_joint_positions(self,angles):
        self.angles=list(angles)[:6]; root=self.rootObject()
        if root:
            for i,value in enumerate(self.angles,1): root.setProperty(f"j{i}",math.degrees(value))
    def _points(self):
        pose=np.eye(4); points=[pose[:3,3].copy()]
        for (_,xyz,rpy,axis),angle in zip(self.joints,self.angles):
            pose=pose@_transform(xyz,_rpy(*rpy))@_transform(rot=_axis_angle(axis,angle)); points.append(pose[:3,3].copy())
        return points

    def set_target_pose(self, pose, emit=False):
        self.target_pose = list(pose)
        point = self.planning_to_scene @ np.array([*pose[:3],1.])
        from PySide6.QtGui import QVector3D
        self.rootObject().setProperty('targetScene', QVector3D(*point[:3]))
        q = QQuaternion(pose[6],pose[3],pose[4],pose[5])
        basis = []
        for axis in (QVector3D(1,0,0),QVector3D(0,1,0),QVector3D(0,0,1)):
            v = q.rotatedVector(axis)
            basis.append((self.planning_to_scene[:3,:3] @ np.array([v.x(),v.y(),v.z()])/100).tolist())
        self.rootObject().setProperty('targetOrientation', basis)
        self.rootObject().setProperty('targetEnabled', True)
        if emit: self.target_changed.emit(self.target_pose)

    def _drag_target(self, axis, amount, rotation):
        if self.target_pose is None: return
        pose = list(self.target_pose)
        if rotation:
            pose[3:] = rotate_world(pose[3:],axis,amount)
        else:
            pose[axis] = max(-5.,min(5.,pose[axis]+amount))
        self.set_target_pose(pose, emit=True)

    def set_ghost(self, joints=None, fingers=None):
        root = self.rootObject()
        root.setProperty('ghostVisible', joints is not None)
        if joints is not None:
            root.setProperty('ghostJoints', [math.degrees(v) for v in joints])
            root.setProperty('ghostFingers', list(fingers or [root.property('finger1'),root.property('finger2')]))

    def set_target_status(self, text):
        self.rootObject().setProperty('targetStatus', text)

    def focus_camera(self):
        # Same URDF chain and scene basis as the displayed arm; view-only operation.
        pose = np.eye(4)
        for (_, xyz, rpy, axis), angle in zip(self.joints, self.angles):
            pose = pose @ _transform(xyz, _rpy(*rpy)) @ _transform(rot=_axis_angle(axis, angle))
        point = pose @ np.array([*self.camera_center, 1.])
        scene = _transform((0,-45,0), _rpy(-math.pi/2,0,0)) @ np.diag([100,100,100,1]) @ point
        from PySide6.QtGui import QVector3D
        self.rootObject().focusCamera(QVector3D(*scene[:3]))

    def reset_view(self):
        self.rootObject().resetView()

    def set_camera_mount(self, transform):
        self._load_tools(transform)
