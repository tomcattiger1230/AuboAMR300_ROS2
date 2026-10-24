from types import SimpleNamespace
import numpy as np
import pytest
cv2=pytest.importorskip('cv2')
pytest.importorskip('scipy')
from rebar_training.vision.observation import look_at,ObservationMotion
from rebar_training.vision.capture import optical_transform
from rebar_training.vision.cylinder import cylinder_hull,resting_pose,gradient_edge
from rebar_training.vision.observer import CameraPacket,RobotPacket,ImageObserver
from rebar_training.vision.blind import blinded_control


def test_cropped_bar_boundary_rejects_connected_background_mask_bleed():
    from rebar_training.vision.cylinder import fit_edges
    image=np.full((400,320),180,np.uint8)
    image[:,120:171]=70
    # A nearby background transition is falsely connected at the top. Its
    # high-leverage samples must not tilt the real, independently visible edge.
    image[:110,200:]=230
    probability=np.zeros(image.shape,float);probability[:,120:171]=1.
    probability[:110,171:201]=1.
    edges=fit_edges(image,probability)
    for y in [0.,200.,399.]:
        boundaries=sorted(-(line[1]*y+line[2])/line[0] for line in (edges.left,edges.right))
        np.testing.assert_allclose(boundaries,[119.5,170.5],atol=1.)
    assert edges.rms_px<1.


def scene():
    K=np.array([[870.,0,512],[0,870.,307.5],[0,0,1]])
    T=np.asarray(optical_transform([-.5,0,1.25],look_at([-.5,0,1.25],[-1.15,0,.522])))
    center=np.array([-1.17,.017,.522]);axis=np.array([np.cos(.04),np.sin(.04),0.])
    hull=cylinder_hull(center,axis,K,T)
    image=np.full((615,1024),180,np.uint8);mask=np.zeros_like(image)
    cv2.fillConvexPoly(mask,np.round(hull).astype(np.int32),1);image[mask>0]=70
    return image,mask.astype(float),K,T,center,axis


def test_observation_pose_uses_fixed_hand_eye_not_a_cached_camera_frame():
    import torch
    from rebar_training.vision.blind import ForbiddenTruth
    robot=SimpleNamespace(data=SimpleNamespace(
        body_pos_w=SimpleNamespace(torch=torch.tensor([[[0.,0.,0.],[0.,0.,.1]]])),
        body_quat_w=SimpleNamespace(torch=torch.tensor([[[0.,0.,0.,1.],[0.,0.,0.,1.]]]))),
        find_bodies=lambda name:([1],['mono_camera_link']))
    env=SimpleNamespace(robot=robot,wrist_id=0,num_envs=1,device='cpu',
        scene=SimpleNamespace(env_origins=torch.zeros(1,3)),
        vision_cameras={'wrist':ForbiddenTruth('cached camera')},
        vision_mounts={'wrist':dict(body='mono_camera_link',position=[0,0,0],quaternion_xyzw=[0,0,0,1])})
    motion=ObservationMotion(env)
    T=np.asarray(optical_transform([-.5,0,1.25],look_at([-.5,0,1.25],[-1.15,0,.522])))
    np.testing.assert_allclose(motion.target.numpy()[0],T[:3,3]+.06*T[:3,2],atol=1e-6)


def robot(timestamp,**changes):
    fields=dict(tcp=np.array([-1.17,.017,.526]),quaternion_xyzw=np.array([0.,0.,0.,1.]),
        fingers=np.zeros(2),finger_forces=np.zeros(2),jaw_forces=np.zeros(4),timestamp_ns=timestamp)
    fields.update(changes)
    return RobotPacket(**fields)


def test_full_bar_mono_geometry_is_metric_and_preserves_axis_sign_ambiguity():
    image,mask,K,T,center,axis=scene()
    measured,direction,quality=resting_pose(image,mask,K,T)
    assert np.linalg.norm(measured-center)<.0015
    assert abs(direction @ axis)>.999
    assert quality['full_length'] and quality['residual_px']<1


@pytest.mark.parametrize('y',[-1e-5,0.,1e-5,.001])
def test_near_zero_tray_coordinates_keep_metric_fit_well_conditioned(y):
    K=np.array([[870.,0,512],[0,870.,307.5],[0,0,1.]])
    T=np.asarray(optical_transform([-.45,0,1.45],look_at([-.45,0,1.45],[.2927,0,.715],[0,1,0])))
    center=np.array([.2927,y,.715]);axis=np.array([0.,1.,0.])
    hull=cylinder_hull(center,axis,K,T)
    mask=np.zeros((615,1024),np.uint8);cv2.fillConvexPoly(mask,np.round(hull).astype(np.int32),1)
    image=np.where(mask,70,180).astype(np.uint8)
    p,a,quality=resting_pose(image,mask.astype(float),K,T,plane_height=.715)
    assert np.linalg.norm(p-center)<.0015 and abs(a@axis)>.999
    assert quality['residual_px']<1


def test_visible_cap_near_border_keeps_only_in_frame_gradient_samples():
    image=np.ones((80,80),np.float32)
    image[15:]=.2
    edge=gradient_edge(image,np.array([40.,0.]),np.array([0.,1.]),15.,-1,16.)
    assert edge is not None and abs(edge[0]-14.5)<1
    # A transition outside the image cannot be recovered by border replication.
    assert gradient_edge(np.ones_like(image),np.array([40.,0.]),np.array([0.,1.]),0.,-1,16.) is None


def test_cap_finder_does_not_replace_a_weak_silhouette_with_a_stronger_internal_rib():
    image=np.ones((100,80),np.float32)*.8
    image[15:]=.65;image[32:36]=.1
    edge=gradient_edge(image,np.array([40.,0.]),np.array([0.,1.]),30.,-1,40.,outer=-1)
    assert edge is not None and abs(edge[0]-14.5)<1


def test_supported_cap_tracking_handles_inverted_background_without_truth_or_threshold_relaxation():
    K=np.array([[870.,0,512],[0,870.,307.5],[0,0,1.]])
    T=np.asarray(optical_transform([-.45,0,1.45],look_at([-.45,0,1.45],[.2927,0,.715],[0,1,0])))
    center=np.array([.2927,-.003,.715]);axis=np.array([0.,1.,0.])
    hull=cylinder_hull(center,axis,K,T)
    mask=np.zeros((615,1024),np.uint8);cv2.fillConvexPoly(mask,np.round(hull).astype(np.int32),1)
    image=np.full(mask.shape,180,np.uint8)
    image[int(hull[:,1].max())-4:]=30
    image[mask>0]=100
    # A dark tray edge 25 pixels behind the actual cap is a detector false positive.
    probability=mask.astype(float)
    low=int(hull[:,1].max());probability[low:low+26,505:522]=.95
    p,a,q=resting_pose(image,probability,K,T,plane_height=.715,
        prior_center=center+[0,.001,0],prior_axis=axis)
    assert np.linalg.norm(p-center)<.0015 and abs(a@axis)>.999
    assert q['residual_px']<2.5 and q['axial_position_measured']
    # The visual predictor cannot manufacture an invisible end cap.
    image[low-12:,495:530]=100
    with pytest.raises(ValueError,match='Both|residual|boundary'):
        resting_pose(image,probability,K,T,plane_height=.715,prior_center=center,prior_axis=axis)


def initialized_observer(**kwargs):
    image,mask,K,T,_,_=scene()
    detector=SimpleNamespace(probabilities=lambda pixels:mask)
    observer=ImageObserver(detector,**kwargs)
    for frame in range(1,4):
        timestamp=frame*100_000_000
        observer.step(robot(timestamp),CameraPacket(image,K,T,timestamp,frame),survey_allowed=True)
    assert observer.acquired
    return observer


def test_recorded_visual_hint_still_requires_three_fresh_live_images(tmp_path):
    import json
    from PIL import Image
    from rebar_training.vision.support_packet import recorded_support_pose
    image,mask,K,T,_,_=scene()
    packet=tmp_path/'observed.json'
    packet.write_text(json.dumps(dict(K=K.tolist(),T_task_optical=T.tolist(),steel_truth='forbidden')))
    Image.fromarray(image).save(packet.with_suffix('.png'))
    detector=SimpleNamespace(probabilities=lambda pixels:mask)
    p,a,_=recorded_support_pose(packet,detector,.522)
    observer=ImageObserver(detector);observer.survey_prior=(p,a)
    assert not observer.step(robot(1),survey_allowed=True)
    for frame in range(1,4):
        timestamp=frame*100_000_000
        accepted=observer.step(robot(timestamp),CameraPacket(image,K,T,timestamp,frame),survey_allowed=True)
        assert accepted==(frame==3)
    assert not observer.step(robot(600_000_001),survey_allowed=True)


def test_full_visible_caps_update_supported_axial_slip_before_regrasp():
    image,mask,K,T,center,axis=scene();current_mask=[mask]
    observer=ImageObserver(SimpleNamespace(probabilities=lambda pixels:current_mask[0]),prefer_full_support=True)
    for frame in range(1,4):
        t=frame*100_000_000
        observer.step(robot(t),CameraPacket(image,K,T,t,frame),survey_allowed=True)
    shifted=center+.003*axis;hull=cylinder_hull(shifted,axis,K,T)
    new_mask=np.zeros(image.shape,np.uint8);cv2.fillConvexPoly(new_mask,np.round(hull).astype(np.int32),1)
    new_image=np.where(new_mask,70,180).astype(np.uint8);current_mask[0]=new_mask.astype(float)
    assert observer.step(robot(400_000_000),CameraPacket(new_image,K,T,400_000_000,4))
    assert abs((observer.position-shifted)@axis)<.0015
    assert observer.quality['axial_position_measured']


@pytest.mark.parametrize('fixed_support_prior',[False,True])
def test_grasp_prior_is_initialized_visually_and_freshness_is_enforced(fixed_support_prior):
    observer=initialized_observer(fixed_support_prior=fixed_support_prior)
    observed=observer.position.copy()
    packet=robot(300_000_000,finger_forces=np.array([20.,20.]))
    observer.latch_grasp(packet)
    moved=robot(333_333_333,tcp=packet.tcp+np.array([0,0,.1]),finger_forces=np.array([20.,20.]))
    assert observer.step(moved)
    np.testing.assert_allclose(observer.position,observed+[0,0,.1])
    assert not observer.step(robot(600_000_000,tcp=moved.tcp,finger_forces=np.array([20.,20.])))


@pytest.mark.parametrize('drift',['lateral','angle'])
def test_stationary_source_prior_cannot_walk_through_individually_small_image_errors(drift):
    image,mask,K,T,center,axis=scene();current_mask=[mask]
    observer=ImageObserver(SimpleNamespace(probabilities=lambda pixels:current_mask[0]),fixed_support_prior=True)
    for frame in range(1,4):
        t=frame*100_000_000
        observer.step(robot(t),CameraPacket(image,K,T,t,frame),survey_allowed=True)
    initial=observer.position.copy();initial_axis=observer.axis.copy()
    # Each change is inside the old per-frame bound; their sum is outside the
    # unchanged 50 mm / .15 rad support envelope. No stale success may refresh
    # the pose once the total image displacement exceeds the surveyed anchor.
    for frame,amount in [(4,1),(5,2),(6,3)]:
        if drift=='lateral':
            p=center+amount*.02*np.array([-axis[1],axis[0],0.]);a=axis
        else:
            p=center;a=np.array([np.cos(.04+amount*.06),np.sin(.04+amount*.06),0.])
        mask=np.zeros(image.shape,np.uint8)
        cv2.fillConvexPoly(mask,np.round(cylinder_hull(p,a,K,T)).astype(np.int32),1)
        current_mask[0]=mask.astype(float);new_image=np.where(mask,70,180).astype(np.uint8)
        t=frame*100_000_000
        observer.step(robot(t),CameraPacket(new_image,K,T,t,frame))
    assert observer.last_good_ns==500_000_000
    assert 'bounded pose prior' in observer.last_error
    assert np.linalg.norm(observer.position-initial)<.05
    assert observer.axis@initial_axis>=np.cos(.15)
    # A fresh consistent image still works after rejection; do not poison the
    # independent reference with an accepted earlier edge bias.
    image,mask,K,T,_,_=scene();current_mask[0]=mask
    assert observer.step(robot(800_000_000),CameraPacket(image,K,T,800_000_000,7))
    assert np.linalg.norm(observer.position-initial)<.0015
    np.testing.assert_array_equal(observer.initial_position,initial)
    np.testing.assert_array_equal(observer.initial_axis,initial_axis)


def test_stationary_source_reference_follows_odometry_and_still_accepts_pixels():
    observer=initialized_observer(fixed_support_prior=True)
    image,mask,K,T,_,_=scene()
    original=observer.position.copy();a=observer.initial_axis.copy()
    theta=.1;R=np.array([[np.cos(theta),-np.sin(theta),0.],[np.sin(theta),np.cos(theta),0.],[0.,0.,1.]])
    delta=np.eye(4);delta[:3,:3]=R;delta[:3,3]=[.01,-.02,0.]
    observer.reframe(delta)
    assert observer.step(robot(400_000_000),CameraPacket(image,K,delta@T,400_000_000,4))
    np.testing.assert_allclose(observer.position,R@original+delta[:3,3],atol=.0015)
    np.testing.assert_allclose(observer.initial_axis,R@a)


def test_source_reference_does_not_return_after_grasp_transport_and_tray_release():
    from rebar_training.vision.observer import rotate
    image,mask,K,T,_,_=scene();current_mask=[mask]
    observer=ImageObserver(SimpleNamespace(probabilities=lambda pixels:current_mask[0]),fixed_support_prior=True)
    for frame in range(1,4):
        t=frame*100_000_000
        observer.step(robot(t),CameraPacket(image,K,T,t,frame),survey_allowed=True)
    survey=observer.initial_position.copy()
    observer.latch_grasp(robot(300_000_000,finger_forces=np.full(2,20.)))
    center=np.array([.2927,0.,.715]);q=np.array([0.,0.,np.sin(np.pi/4),np.cos(np.pi/4)])
    tcp=center-rotate(q,observer.local_center)
    assert observer.step(robot(333_333_333,tcp=tcp,quaternion_xyzw=q,finger_forces=np.full(2,20.)))
    np.testing.assert_allclose(observer.position,center)
    axis=observer.axis.copy()
    # Same estimator transition as actual measured open/unloaded tray support.
    # The source survey remains historical; it is not the new rack's prior.
    observer.anchored=observer.released=False;observer.plane_height=.715
    T=np.asarray(optical_transform([-.45,0,1.45],look_at([-.45,0,1.45],center,[0,1,0])))
    mask=np.zeros(image.shape,np.uint8)
    cv2.fillConvexPoly(mask,np.round(cylinder_hull(center,axis,K,T)).astype(np.int32),1)
    image=np.where(mask,70,180).astype(np.uint8);current_mask[0]=mask.astype(float)
    assert observer.step(robot(400_000_000,tcp=tcp,quaternion_xyzw=q),CameraPacket(image,K,T,400_000_000,4))
    assert observer.last_error=='' and np.linalg.norm(observer.position-center)<.0015
    np.testing.assert_array_equal(observer.initial_position,survey)
    observer.reset()
    assert observer.local_center is None and not observer.acquired and observer.fixed_support_prior


def test_release_freezes_scene_prior_instead_of_following_retreating_robot():
    observer=initialized_observer();position=observer.position.copy()
    observer.latch_grasp(robot(300_000_000,finger_forces=np.array([20.,20.])))
    observer.step(robot(333_333_333,tcp=np.array([-1.17,.017,.65]),jaw_forces=np.full(4,15.)),release_requested=True)
    assert observer.released and not observer.anchored
    np.testing.assert_allclose(observer.position,position)


def test_machine_load_transfer_freezes_visual_prior_before_fingers_finish_opening():
    observer=initialized_observer();position=observer.position.copy()
    observer.latch_grasp(robot(300_000_000,finger_forces=np.array([20.,20.])))
    # Four tiny touches do not establish sufficient support for this payload.
    observer.step(robot(333_333_333,jaw_forces=np.ones(4),fingers=np.full(2,.02)),release_requested=True)
    assert observer.anchored
    observer.step(robot(366_666_666,tcp=np.array([-1.17,.017,.55]),jaw_forces=np.full(4,15.),
                        fingers=np.full(2,.02),finger_forces=np.full(2,20.)),release_requested=True)
    assert observer.released and not observer.anchored
    np.testing.assert_allclose(observer.position,position)


def test_world_supported_prior_reframes_with_odometry_without_changing_grasp_transform():
    observer=initialized_observer();position=observer.position.copy()
    observer.latch_grasp(robot(300_000_000,finger_forces=np.array([20.,20.])))
    grasp=observer.local_center.copy();observer.anchored=False
    T=np.eye(4);T[:3,3]=[.01,-.02,0.]
    observer.reframe(T)
    np.testing.assert_allclose(observer.position,position+T[:3,3])
    np.testing.assert_allclose(observer.initial_position,position+T[:3,3])
    np.testing.assert_array_equal(observer.local_center,grasp)


def test_control_guard_blocks_direct_indirect_and_reward_truth_and_restores_after_error():
    bar=SimpleNamespace(data=SimpleNamespace(root_pos_w=np.ones(3)))
    station=SimpleNamespace(data='privileged station pose')
    logic=SimpleNamespace(phase='privileged reward stage')
    env=SimpleNamespace(bar=bar,station=station,logic=logic,sampler=SimpleNamespace(seed=123),
                        scene=SimpleNamespace(rigid_objects={'rebar':bar,'station':station}))
    with blinded_control(env):
        for getter in (lambda:env.bar.data,lambda:env.station.data,lambda:env.logic.phase,
                       lambda:env.scene.rigid_objects['rebar'].data,lambda:env.sampler.seed):
            with pytest.raises(RuntimeError,match='Privileged truth access'):
                getter()
    assert env.bar is bar and env.station is station and env.logic is logic
    assert env.scene.rigid_objects['rebar'] is bar
