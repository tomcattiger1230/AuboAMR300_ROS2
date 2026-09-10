"""Qt Quick 3D viewer for the real AUBO i16 COLLADA visual meshes."""
from __future__ import annotations
import math
import sys
import xml.etree.ElementTree as ET
import numpy as np
from PySide6.QtCore import QUrl
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
    """Loads link0-link6.DAE and drives their URDF transform hierarchy."""
    def __init__(self,urdf_path,qml_path,parent=None):
        super().__init__(parent); self.setMinimumSize(560,520); self.setResizeMode(QQuickWidget.SizeRootObjectToView)
        self.joints=[]; self.angles=[0.0]*6
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
    def set_joint_positions(self,angles):
        self.angles=list(angles)[:6]; root=self.rootObject()
        if root:
            for i,value in enumerate(self.angles,1): root.setProperty(f"j{i}",math.degrees(value))
    def _points(self):
        pose=np.eye(4); points=[pose[:3,3].copy()]
        for (_,xyz,rpy,axis),angle in zip(self.joints,self.angles):
            pose=pose@_transform(xyz,_rpy(*rpy))@_transform(rot=_axis_angle(axis,angle)); points.append(pose[:3,3].copy())
        return points
