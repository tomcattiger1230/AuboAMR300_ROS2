"""Offline IK from the actual USD joint frames; candidates need collision tests."""
import math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .spec import ARM_JOINTS
from .vision.capture import optical_transform


class UsdArm:
    def __init__(self,path):
        from pxr import Usd,UsdPhysics
        stage=Usd.Stage.Open(str(path));self.joints={}
        self.bounds={}
        for prim in stage.Traverse():
            if not prim.IsA(UsdPhysics.Joint):continue
            joint=UsdPhysics.Joint(prim)
            a,b=joint.GetBody0Rel().GetTargets(),joint.GetBody1Rel().GetTargets()
            if not a or not b:continue
            def frame(pos,quat):
                return np.asarray(optical_transform(list(pos.Get()),[*quat.Get().GetImaginary(),quat.Get().GetReal()]))
            axis=prim.GetAttribute('physics:axis').Get() if prim.IsA(UsdPhysics.RevoluteJoint) else None
            self.joints[str(b[0])]=(str(a[0]),prim.GetName(),frame(joint.GetLocalPos0Attr(),joint.GetLocalRot0Attr()),
                np.linalg.inv(frame(joint.GetLocalPos1Attr(),joint.GetLocalRot1Attr())),axis)
            if prim.GetName() in ARM_JOINTS:
                self.bounds[prim.GetName()]=tuple(math.radians(prim.GetAttribute('physics:'+side+'Limit').Get())
                                                 for side in ('lower','upper'))
        self.lower=np.array([self.bounds[n][0] for n in ARM_JOINTS])
        self.upper=np.array([self.bounds[n][1] for n in ARM_JOINTS])

    def wrist(self,q):
        values=dict(zip(ARM_JOINTS,q))
        def pose(body):
            if body=='/Robot/base_footprint':return np.eye(4)
            parent,name,F0,F1,axis=self.joints[body]
            R=np.eye(4)
            if axis:
                vector=np.eye(3)['XYZ'.index(axis)]*values.get(name,0.)
                R[:3,:3]=Rotation.from_rotvec(vector).as_matrix()
            return pose(parent)@F0@R@F1
        return pose('/Robot/wrist3_Link')

    def tcp(self,q):
        T=self.wrist(q).copy();T[:3,3]+=T[:3,:3]@[0,0,.16]
        return T

    def solve(self,target,seeds):
        candidates=[]
        def residual(q):
            actual=self.tcp(q)
            return np.r_[actual[:3,3]-target[:3,3],
                         .2*Rotation.from_matrix(target[:3,:3]@actual[:3,:3].T).as_rotvec()]
        for seed in seeds:
            fit=least_squares(residual,np.clip(seed,self.lower+1e-6,self.upper-1e-6),
                bounds=(self.lower,self.upper),max_nfev=150,ftol=1e-10,gtol=1e-10,xtol=1e-10)
            if np.linalg.norm(residual(fit.x))<1e-5:
                if not any(np.linalg.norm(np.sin((fit.x-q)/2))<.01 for q in candidates):candidates.append(fit.x)
        return candidates
