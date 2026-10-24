"""Camera/proprioception frame conversion from measured chassis odometry."""
from dataclasses import replace
import numpy as np
from .capture import optical_transform
from .observation import matrix_quaternion


class BaseFrame:
    def __init__(self,position,quaternion_xyzw):
        self.world_from_base=np.asarray(optical_transform(position,quaternion_xyzw))
        self.base_from_world=np.eye(4)
        R=self.world_from_base[:3,:3]
        self.base_from_world[:3,:3]=R.T
        self.base_from_world[:3,3]=-R.T@self.world_from_base[:3,3]

    def point(self,world_point):
        return self.base_from_world[:3,:3]@np.asarray(world_point)+self.base_from_world[:3,3]

    def robot_packet(self,packet):
        wrist_rotation=np.asarray(optical_transform([0,0,0],packet.quaternion_xyzw))[:3,:3]
        return replace(packet,tcp=self.point(packet.tcp),quaternion_xyzw=matrix_quaternion(
            self.base_from_world[:3,:3]@wrist_rotation))

    def camera_packet(self,packet):
        return replace(packet,T_scene_optical=self.base_from_world@packet.T_scene_optical)
