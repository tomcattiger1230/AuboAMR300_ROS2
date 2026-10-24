import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from rebar_training.vision.observation import look_at
from rebar_training.vision.geometry import project
from rebar_training.vision.cylinder import cylinder_hull
from rebar_training.vision.support_texture import SupportTexture


def scene(center,camera,texture=True):
    K=np.array([[650.,0,320],[0,650.,240],[0,0,1]])
    T=np.eye(4);T[:3,3]=camera;T[:3,:3]=Rotation.from_quat(look_at(camera,[0,0,.715],(0,0,1))).as_matrix()
    axis=np.array([1.,0,0]);image=np.full((480,640),220,np.uint8)
    hull=cylinder_hull(np.asarray(center),axis,K,T)
    mask=np.zeros_like(image);cv2.fillConvexPoly(mask,np.rint(hull).astype(np.int32),1)
    image[mask>0]=150
    if texture:
        rng=np.random.default_rng(345)
        coordinates=rng.uniform([-.26,-.6],[.26,.6],(160,2))
        offsets=np.c_[coordinates[:,0],-.012*np.cos(coordinates[:,1]),.012*np.sin(coordinates[:,1])]
        pixels,_=project(np.asarray(center)+offsets,K,T)
        for p in pixels:cv2.circle(image,tuple(np.rint(p).astype(int)),2,30,-1,cv2.LINE_AA)
    return image,mask.astype(np.float32),K,T


def test_axial_texture_motion_is_separated_from_calibrated_camera_motion():
    tracker=SupportTexture();p=np.array([0.,0,.715]);axis=np.array([1.,0,0])
    old=scene(p,[0,-.65,1.])
    tracker.reset(*old,p,axis)
    actual=p+np.array([.003,0,0]);new=scene(actual,[.008,-.655,1.006])
    estimated,quality=tracker.update(*new,p,axis)
    assert quality['axial_position_measured'] and quality['texture_landmarks']>=8
    np.testing.assert_allclose(estimated,actual,atol=.0008)


def test_uniform_rod_or_missing_texture_never_fabricates_axial_measurement():
    tracker=SupportTexture();p=np.array([0.,0,.715]);axis=np.array([1.,0,0])
    with pytest.raises(ValueError,match='texture'):
        tracker.reset(*scene(p,[0,-.65,1.],False),p,axis)
    tracker.reset(*scene(p,[0,-.65,1.]),p,axis)
    new=list(scene(p,[.008,-.655,1.006]));new[1][:]=0
    with pytest.raises(ValueError,match='correspondence'):tracker.update(*new,p,axis)


def test_landmarks_replenish_only_after_successful_image_registration():
    tracker=SupportTexture();p=np.array([0.,0,.715]);axis=np.array([1.,0,0])
    tracker.reset(*scene(p,[0,-.65,1.]),p,axis)
    # Simulate earlier landmarks leaving view. Enough old image evidence must
    # survive before new features may inherit the measured cylinder pose.
    tracker.offsets=tracker.offsets[:20];tracker.pixels=tracker.pixels[:20]
    actual=p+np.array([.002,0,0])
    estimate,quality=tracker.update(*scene(actual,[.005,-.65,1.]),p,axis)
    assert quality['texture_replenishments']==1 and len(tracker.pixels)>20
    np.testing.assert_allclose(estimate,actual,atol=.0008)
    old_offsets=tracker.offsets.copy();old_pixels=tracker.pixels.copy()
    bad=list(scene(actual,[.006,-.65,1.]));bad[1][:]=0
    with pytest.raises(ValueError):tracker.update(*bad,estimate,axis)
    np.testing.assert_array_equal(tracker.offsets,old_offsets)
    np.testing.assert_array_equal(tracker.pixels,old_pixels)
    assert tracker.replenishments==1
