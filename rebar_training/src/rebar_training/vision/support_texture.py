"""Axial support motion from RGB texture and calibrated camera motion.

Full-cap vision initializes cylinder-surface landmarks. Partial views track
them and may add landmarks only after a geometrically verified measurement.
No simulator state, semantic labels or depth input is available to this class.
"""
import cv2
import numpy as np
from scipy.optimize import least_squares
from .geometry import calibration,project


def align_axis(old,new):
    old=np.asarray(old,float);new=np.asarray(new,float)
    old=old/np.linalg.norm(old);new=new/np.linalg.norm(new)
    v=np.cross(old,new);c=float(old@new)
    if c<.95:raise ValueError('Texture anchor axis changed beyond bounded support motion')
    V=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3)+V+V@V/(1+c)


def cylinder_points(pixels,K,T,center,axis,radius=.012):
    rays=np.linalg.solve(K,np.c_[pixels,np.ones(len(pixels))].T).T@T[:3,:3].T
    offset=T[:3,3]-center
    radial_rays=rays-(rays@axis)[:,None]*axis
    radial_offset=offset-axis*(axis@offset)
    A=(radial_rays**2).sum(-1);B=2*(radial_rays@radial_offset)
    C=radial_offset@radial_offset-radius**2;disc=B*B-4*A*C
    distance=(-B-np.sqrt(np.maximum(disc,0)))/(2*A.clip(1e-12))
    points=T[:3,3]+distance[:,None]*rays
    valid=(disc>0)&(distance>0)&(np.abs((points-center)@axis)<.27)
    return points,valid


class SupportTexture:
    def __init__(self):
        self.image=None;self.pixels=None;self.offsets=None;self.anchor_axis=None
        self.replenishments=0

    def reset(self,image,probability,K,T,center,axis):
        K,T=calibration(K,T);center=np.asarray(center,float);axis=np.asarray(axis,float)
        axis=axis/np.linalg.norm(axis)
        mask=cv2.erode((probability>.8).astype(np.uint8),np.ones((5,5),np.uint8))
        features=cv2.goodFeaturesToTrack(image,maxCorners=180,qualityLevel=.012,minDistance=5,
                                       mask=mask,blockSize=5)
        if features is None:raise ValueError('No resolvable steel texture landmarks')
        pixels=features.reshape(-1,2)
        points,valid=cylinder_points(pixels,K,T,center,axis)
        if valid.sum()<8:raise ValueError('Insufficient visible cylinder texture landmarks')
        self.image=image.copy();self.pixels=pixels[valid].copy()
        self.offsets=points[valid]-center;self.anchor_axis=axis.copy()
        self.replenishments=0

    def reframe(self,new_from_old):
        if self.image is None:return
        R=np.asarray(new_from_old)[:3,:3]
        self.offsets=self.offsets@R.T;self.anchor_axis=R@self.anchor_axis

    def update(self,image,probability,K,T,center,axis):
        if self.image is None:raise ValueError('Texture requires an observed full-cap anchor')
        K,T=calibration(K,T);center=np.asarray(center,float);axis=np.asarray(axis,float)
        axis=axis/np.linalg.norm(axis)
        offsets=self.offsets@align_axis(self.anchor_axis,axis).T
        points=center+offsets;predicted,depth=project(points,K,T)
        h,w=image.shape
        inside=(depth>.03)&(predicted[:,0]>5)&(predicted[:,0]<w-6)&(predicted[:,1]>5)&(predicted[:,1]<h-6)
        ids=np.flatnonzero(inside)
        if len(ids)<8:raise ValueError('Too few texture anchors remain inside the camera')
        previous=self.pixels[ids].astype(np.float32).reshape(-1,1,2)
        guess=predicted[ids].astype(np.float32).reshape(-1,1,2)
        options=dict(winSize=(21,21),maxLevel=3,
                     criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,40,.01))
        tracked,ok,_=cv2.calcOpticalFlowPyrLK(self.image,image,previous,guess,
                                           flags=cv2.OPTFLOW_USE_INITIAL_FLOW,**options)
        if tracked is None or ok is None:raise ValueError('Steel texture forward tracking failed')
        back,back_ok,_=cv2.calcOpticalFlowPyrLK(image,self.image,tracked,None,**options)
        if back is None or back_ok is None:raise ValueError('Steel texture backward tracking failed')
        tracked=tracked.reshape(-1,2);back=back.reshape(-1,2)
        valid=(ok.ravel()>0)&(back_ok.ravel()>0)&(np.linalg.norm(back-previous.reshape(-1,2),axis=1)<.7)
        valid&=(tracked[:,0]>4)&(tracked[:,0]<w-5)&(tracked[:,1]>4)&(tracked[:,1]<h-5)
        ip=np.rint(tracked).astype(int);ip[:,0]=ip[:,0].clip(0,w-1);ip[:,1]=ip[:,1].clip(0,h-1)
        valid&=probability[ip[:,1],ip[:,0]]>.65
        ids=ids[valid];measured=tracked[valid]
        if len(ids)<8:raise ValueError('Steel texture correspondence failed forward/backward consistency')
        basis=points[ids]
        expected,_=project(basis,K,T);shifted,_=project(basis+.001*axis,K,T)
        J=(shifted-expected)/.001
        if np.median(np.linalg.norm(J,axis=1))<40:
            raise ValueError('Steel axial texture motion is not observable in this view')
        estimates=((measured-expected)*J).sum(-1)/(J*J).sum(-1).clip(1e-6)
        initial=float(np.median(estimates))
        if abs(initial)>.010:raise ValueError('Axial texture shift exceeds supported motion bound')
        def residual(x,keep=None):
            projected,_=project(basis+x[0]*axis,K,T)
            errors=projected-measured
            return errors.ravel() if keep is None else errors[keep].ravel()
        fit=least_squares(residual,[initial],bounds=([-.010],[.010]),loss='soft_l1',f_scale=.7,
                          diff_step=1e-3,max_nfev=20)
        errors=residual(fit.x).reshape(-1,2);inliers=np.linalg.norm(errors,axis=1)<1.2
        if inliers.sum()<8 or inliers.mean()<.5:
            raise ValueError('Axial texture registration lacks consistent image evidence')
        fit=least_squares(lambda x:residual(x,inliers),fit.x,bounds=([-.010],[.010]),max_nfev=20)
        rms=float(np.sqrt(np.mean(residual(fit.x,inliers)**2)))
        if rms>.8:raise ValueError('Axial texture reprojection residual exceeds limit')
        kept=ids[inliers]
        corrected=center+float(fit.x[0])*axis
        # Surviving landmarks retain their original surface coordinates. New
        # points inherit an image-measured pose, never a failed/stale prior.
        self.offsets=offsets[kept];self.anchor_axis=axis.copy()
        self.pixels=measured[inliers].copy();self.image=image.copy()
        if len(kept)<60:
            mask=cv2.erode((probability>.8).astype(np.uint8),np.ones((7,7),np.uint8))
            for pixel in self.pixels:
                cv2.circle(mask,tuple(np.rint(pixel).astype(int)),7,0,-1)
            features=cv2.goodFeaturesToTrack(image,maxCorners=120,qualityLevel=.012,
                                            minDistance=7,mask=mask,blockSize=5)
            if features is not None:
                pixels=features.reshape(-1,2)
                added,valid=cylinder_points(pixels,K,T,corrected,axis)
                if valid.sum():
                    self.offsets=np.vstack((self.offsets,added[valid]-corrected))
                    self.pixels=np.vstack((self.pixels,pixels[valid]))
                    self.replenishments+=1
        return corrected,dict(axial_position_measured=True,
            source='calibrated_mono_support_texture',texture_landmarks=int(inliers.sum()),
            texture_residual_px=rms,texture_axial_delta_m=float(fit.x[0]),
            texture_replenishments=self.replenishments)
