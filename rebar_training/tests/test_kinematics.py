import numpy as np
import pytest
pytest.importorskip('scipy');pytest.importorskip('pxr')
from pxr import Usd,UsdGeom
from rebar_training.assets import build_assets
from rebar_training.kinematics import UsdArm
from rebar_training.spec import GRASP_JOINTS


def test_usd_joint_frame_fk_matches_authored_zero_pose_and_ik_round_trip(tmp_path):
    build_assets(tmp_path)
    stage=Usd.Stage.Open(str(tmp_path/'robot.usda'))
    expected=UsdGeom.Xformable(stage.GetPrimAtPath('/Robot/wrist3_Link')).ComputeLocalToWorldTransform(0)
    model=UsdArm(tmp_path/'robot.usda')
    np.testing.assert_allclose(model.wrist(np.zeros(6))[:3,3],list(expected.ExtractTranslation()),atol=1e-6)
    target=model.tcp(GRASP_JOINTS)
    candidates=model.solve(target,[np.asarray(GRASP_JOINTS)+.01])
    assert candidates
    np.testing.assert_allclose(model.tcp(candidates[0]),target,atol=1e-5)
    # Only kinematics is validated here, not collision freedom of a joint path.


def test_expanded_pickup_search_has_reachable_pose_and_entire_caps_inside_image(tmp_path):
    from scipy.spatial.transform import Rotation
    from rebar_training.vision.observation import PICKUP_SEARCH_VIEWS,look_at
    build_assets(tmp_path)
    stage=Usd.Stage.Open(str(tmp_path/'robot.usda'));cache=UsdGeom.XformCache()
    camera=stage.GetPrimAtPath('/Robot/mono_camera_link/camera_optical_frame/ros2_camera')
    wrist=stage.GetPrimAtPath('/Robot/wrist3_Link')
    def matrix(prim):return np.asarray(cache.GetLocalToWorldTransform(prim)).T
    gl_to_ros=np.diag([1.,-1.,-1.,1.])
    wrist_camera=np.linalg.inv(matrix(wrist))@matrix(camera)@gl_to_ros
    camera_cfg=UsdGeom.Camera(camera)
    fx=1024*camera_cfg.GetFocalLengthAttr().Get()/camera_cfg.GetHorizontalApertureAttr().Get()
    fy=615*camera_cfg.GetFocalLengthAttr().Get()/camera_cfg.GetVerticalApertureAttr().Get()
    K=np.array([[fx,0,512],[0,fy,307.5],[0,0,1.]])
    position,reference=PICKUP_SEARCH_VIEWS[0]
    T=np.eye(4);T[:3,3]=position
    T[:3,:3]=Rotation.from_quat(look_at(position,(-1.15,0,.522),reference)).as_matrix()
    tcp=T@np.linalg.inv(wrist_camera);tcp[:3,3]+=tcp[:3,:3]@[0,0,.16]
    model=UsdArm(tmp_path/'robot.usda')
    candidates=model.solve(tcp,[GRASP_JOINTS,(1.57,.74,1.34,-.97,.845,-1.57)])
    assert candidates, 'A visually useful survey pose must also have a physical USD IK solution'
    pixels=[]
    for bx in (-1.23,-1.07):
        for by in (-.08,.08):
            for yaw in (-.12,.12):
                axis=np.array([np.cos(yaw),np.sin(yaw),0.])
                side=np.array([-np.sin(yaw),np.cos(yaw),0.])
                for end in (-.3,.3):
                    for phi in np.linspace(0,2*np.pi,24,endpoint=False):
                        p=np.array([bx,by,.522])+end*axis+.012*(np.cos(phi)*side+np.sin(phi)*np.array([0,0,1.]))
                        optical=T[:3,:3].T@(p-T[:3,3]);assert optical[2]>0
                        uv=K@optical;pixels.append(uv[:2]/uv[2])
    pixels=np.asarray(pixels)
    assert np.all(pixels.min(0)>32) and np.all(pixels.max(0)<[1024-32,615-32])
