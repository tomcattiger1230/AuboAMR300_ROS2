from pathlib import Path

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from rebar_training.workcells import WorkcellLayout
from rebar_training.fiducial_localization import CameraCalibration,inverse,rotation_distance
from rebar_training.machine_fiducial import (make_board,CharucoFixtureEstimator,WIDTH,HEIGHT,
    fixture_from_board,station_from_board,attach_board,CAMERA_POSITION_STATION,CAMERA_AIM_BELOW_CENTRE_M)

CONFIG=Path(__file__).parents[1]/'config/multi_workcell_offline.json'


def setup_image(machine='machine_1'):
    layout=WorkcellLayout.load(CONFIG);k=np.array([[1000.,0,640],[0,1000,480],[0,0,1]])
    c=CameraCalibration('wrist','K','handeye',(1280,960),k,(0.,)*5,np.eye(4),
                        'base','flange','optical','clock')
    e=CharucoFixtureEstimator(c,layout,machine)
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',[np.pi-.25,.15,.02]).as_matrix();t[:3,3]=[.01,0.,.65]
    object_corners=np.array([[-WIDTH/2,HEIGHT/2,0],[WIDTH/2,HEIGHT/2,0],
                             [WIDTH/2,-HEIGHT/2,0],[-WIDTH/2,-HEIGHT/2,0]])
    target=cv2.projectPoints(object_corners,cv2.Rodrigues(t[:3,:3])[0],t[:3,3],k,None)[0].reshape(4,2)
    pattern=make_board(machine).generateImage((1400,1000))
    h=cv2.getPerspectiveTransform(np.float32([[0,0],[1399,0],[1399,999],[0,999]]),target.astype(np.float32))
    image=cv2.warpPerspective(pattern,h,(1280,960),flags=cv2.INTER_NEAREST,borderValue=255)
    return layout,e,image,t


def test_generated_charuco_pose_and_machine_identity():
    layout,e,image,truth=setup_image()
    r=e.estimate(image,machine_id='machine_1',frame_id='frame_1',image_stamp_s=1.,fk_stamp_s=1.,now_s=1.,
        base_from_flange=np.eye(4),camera_id='wrist',calibration_id='K',hand_eye_id='handeye',
        fk_base_frame_id='base',fk_flange_frame_id='flange',optical_frame_id='optical',clock_id='clock')
    assert len(r['charuco_corner_ids'])==24 and len(r['marker_ids'])==17
    expected=truth@np.linalg.inv(fixture_from_board(layout,'machine_1'))
    actual=np.array(r['base_from_fixture'])
    assert np.linalg.norm(actual[:3,3]-expected[:3,3])<.005
    assert rotation_distance(actual,expected)<np.deg2rad(1)
    _,_,wrong,_=setup_image('machine_2')
    with pytest.raises(ValueError):e.detect_corners(wrong)


def test_marker_ids_are_disjoint_between_same_type_machines():
    sets=[set(make_board(f'machine_{i}').getIds().reshape(-1)) for i in (1,2,3)]
    assert all(not a&b for i,a in enumerate(sets) for b in sets[i+1:])


def test_board_is_fixed_child_of_actual_machine_and_preserves_survey():
    from pxr import Usd,UsdGeom,Gf,UsdPhysics
    layout=WorkcellLayout.load(CONFIG);stage=Usd.Stage.CreateInMemory()
    parent=UsdGeom.Xform.Define(stage,'/World/ActualMachine')
    parent.AddTranslateOp().Set(Gf.Vec3d(3,2,0))
    record=attach_board(stage,'/World/ActualMachine',layout,'machine_1')
    prim=stage.GetPrimAtPath(record['board_prim'])
    assert str(prim.GetParent().GetPath())=='/World/ActualMachine'
    before=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T
    want=station_from_board();want[:3,3]+=layout.station('machine_1').pose_map.values()[:3]
    np.testing.assert_allclose(before,want,atol=1e-8)
    parent.GetOrderedXformOps()[0].Set(Gf.Vec3d(4,2,0))
    after=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T
    assert after[0,3]-before[0,3]==pytest.approx(1.)
    assert stage.GetPrimAtPath(record['board_prim']+'/RigidBacking').HasAPI(UsdPhysics.CollisionAPI)


def test_mount_does_not_cover_the_printed_front_face():
    from pxr import Usd,UsdGeom
    stage=Usd.Stage.CreateInMemory();UsdGeom.Xform.Define(stage,'/Machine')
    record=attach_board(stage,'/Machine',WorkcellLayout.load(CONFIG),'machine_1')
    for name in ('FrameBracket_0','FrameBracket_1','BoardBackClamp'):
        cube=UsdGeom.Xformable(stage.GetPrimAtPath(record['board_prim']+'/'+name))
        ops=cube.GetOrderedXformOps();center=np.array(ops[0].Get());size=np.array(ops[1].Get())
        if center[1]-size[1]/2<HEIGHT/2:
            assert center[2]+size[2]/2<-.0001


def test_surveyed_aim_keeps_whole_board_above_recorded_finger_occlusion():
    from rebar_training.vision.observation import look_at
    from rebar_training.vision.capture import optical_transform
    position=np.array(CAMERA_POSITION_STATION);aim=station_from_board()[:3,3].copy()
    aim[2]-=CAMERA_AIM_BELOW_CENTRE_M
    camera=np.array(optical_transform(position,look_at(position,aim,(0,0,1))))
    optical_board=inverse(camera)@station_from_board()
    corners=np.array([[-WIDTH/2,HEIGHT/2,0],[WIDTH/2,HEIGHT/2,0],
                      [WIDTH/2,-HEIGHT/2,0],[-WIDTH/2,-HEIGHT/2,0]])
    k=np.array([[1739.130493,0,1024],[0,1739.130493,615],[0,0,1]])
    pixels=cv2.projectPoints(corners,cv2.Rodrigues(optical_board[:3,:3])[0],optical_board[:3,3],k,None)[0].reshape(-1,2)
    assert pixels[:,1].min()>20 and pixels[:,1].max()<850
    assert pixels[:,0].min()>20 and pixels[:,0].max()<2028


def test_partial_board_is_rejected_even_with_many_visible_markers():
    _,estimator,image,_=setup_image()
    corners=estimator.detect_corners(image)
    middle=corners.mean(0).astype(int);occluded=image.copy()
    cv2.rectangle(occluded,tuple(middle-[65,65]),tuple(middle+[65,65]),255,-1)
    with pytest.raises(ValueError):estimator.detect_corners(occluded)


def test_another_machine_in_view_does_not_replace_or_hide_required_target_ids():
    _,estimator,image,_=setup_image()
    image[:220,:300]=255
    image[10:210,10:290]=make_board('machine_2').generateImage((280,200))
    assert estimator.detect_corners(image).shape==(24,2)
