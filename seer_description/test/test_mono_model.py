"""Validate the standalone camera variants using a Python with USD (pxr)."""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from pxr import Gf, Usd, UsdGeom, UsdPhysics

ROOT=Path(__file__).resolve().parents[1]


class MonoModelTest(unittest.TestCase):
    def test_urdf_and_srdf_references(self):
        robot=ET.parse(ROOT/'urdf/composite_robot_stick_mono.urdf').getroot()
        links={e.get('name') for e in robot.findall('link')}
        joints={e.get('name') for e in robot.findall('joint')}
        self.assertIn('camera_optical_frame',links)
        self.assertNotIn('camera_depth_frame',links)
        srdf=ET.parse(ROOT.parent/'seer_aubo_stick_mono_moveit_config/config/seer_aubo_composite.srdf').getroot()
        for group in srdf.findall('group'):
            for joint in group.findall('joint'):self.assertIn(joint.get('name'),joints)
        for entry in srdf.findall('disable_collisions'):
            for key in ('link1','link2'):self.assertIn(entry.get(key),links)

    def test_sdf_has_only_a_monochrome_camera(self):
        sdf=ET.parse(ROOT/'urdf/composite_robot_stick_mono.sdf').getroot()
        cameras=sdf.findall(".//sensor[@type='camera']")
        self.assertEqual(len(cameras),1)
        self.assertEqual(cameras[0].findtext('camera/image/format'),'L8')
        self.assertEqual(cameras[0].findtext('camera/image/width'),'4096')
        self.assertEqual(cameras[0].findtext('camera/image/height'),'2460')
        self.assertFalse(sdf.findall(".//sensor[@type='rgbd_camera']"))
        self.assertAlmostEqual(float(cameras[0].findtext('camera/horizontal_fov')),2*math.atan(4096*.00345/24))

    def test_usd_camera_geometry_and_mount(self):
        stage=Usd.Stage.Open(str(ROOT/'urdf/warehouse_stick_mono_demo.usda'))
        root='/World/seer_aubo_composite'
        self.assertFalse(stage.GetPrimAtPath(root+'/camera_link').IsActive())
        self.assertFalse(stage.GetPrimAtPath(root+'/joints/camera_joint').IsActive())
        joint=UsdPhysics.FixedJoint(stage.GetPrimAtPath(root+'/joints/mono_camera_joint'))
        self.assertEqual(str(joint.GetBody0Rel().GetTargets()[0]),root+'/wrist3_Link')
        cache=UsdGeom.XformCache()
        body=stage.GetPrimAtPath(root+'/mono_camera_link')
        local=Gf.Matrix4d(1);local.SetRotate(Gf.Quatd(joint.GetLocalRot0Attr().Get()));local.SetTranslateOnly(Gf.Vec3d(joint.GetLocalPos0Attr().Get()))
        expected=local*cache.GetLocalToWorldTransform(stage.GetPrimAtPath(root+'/wrist3_Link'))
        actual=cache.GetLocalToWorldTransform(body)
        self.assertLess((expected.ExtractTranslation()-actual.ExtractTranslation()).GetLength(),1e-6)
        for axis in (Gf.Vec3d(1,0,0),Gf.Vec3d(0,1,0),Gf.Vec3d(0,0,1)):
            self.assertLess((expected.TransformDir(axis)-actual.TransformDir(axis)).GetLength(),1e-6)
        cameras=[p for p in stage.Traverse() if p.IsA(UsdGeom.Camera) and p.GetName()=='ros2_camera']
        self.assertEqual(len(cameras),1)
        camera=UsdGeom.Camera(cameras[0])
        self.assertEqual(camera.GetFocalLengthAttr().Get(),12.)
        self.assertAlmostEqual(camera.GetHorizontalApertureAttr().Get(),14.1312,places=5)
        self.assertAlmostEqual(camera.GetVerticalApertureAttr().Get(),8.487,places=5)
        for name in ('gripper1_joint','gripper2_joint'):
            finger=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(root+'/joints/'+name))
            self.assertTrue(finger.GetPrim().IsActive())
            self.assertAlmostEqual(finger.GetUpperLimitAttr().Get(),.04)


if __name__=='__main__':unittest.main()
