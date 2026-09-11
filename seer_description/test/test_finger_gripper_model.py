"""Validate the independent finger + motor_adapter robot model."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from pxr import Gf, Usd, UsdGeom, UsdPhysics


ROOT = Path(__file__).resolve().parents[1]
ROBOT = "/World/seer_aubo_composite"


class FingerGripperModelTest(unittest.TestCase):
    def test_urdf_interface_geometry_and_camera(self):
        robot = ET.parse(ROOT / "urdf/composite_robot_finger_mono.urdf").getroot()
        joints = {joint.get("name"): joint for joint in robot.findall("joint")}
        links = {link.get("name"): link for link in robot.findall("link")}
        for number, axis in ((1, "1 0 0"), (2, "-1 0 0")):
            joint = joints[f"gripper{number}_joint"]
            self.assertEqual(joint.get("type"), "prismatic")
            self.assertEqual(joint.find("axis").get("xyz"), axis)
            self.assertEqual(float(joint.find("limit").get("lower")), 0.0)
            self.assertEqual(float(joint.find("limit").get("upper")), 0.0285)
            mesh = links[f"gripper{number}_link"].find("visual/geometry/mesh")
            self.assertTrue(mesh.get("filename").endswith("/finger_centered.stl"))
            self.assertEqual(mesh.get("scale"), "1 1 1")
            collision = links[f"gripper{number}_link"].find("collision")
            self.assertEqual(collision.find("geometry/box").get("size"), "0.049 0.15 0.047786")
            inertia = links[f"gripper{number}_link"].find("inertial/inertia")
            self.assertAlmostEqual(float(inertia.get("iyy")), 0.000070267)
        motor = links["gripper_motor_link"].find("visual/geometry/mesh")
        self.assertTrue(motor.get("filename").endswith("/motor_new.stl"))
        self.assertEqual(motor.get("scale"), "1 1 1")
        self.assertEqual(joints["camera_joint"].find("parent").get("link"), "wrist3_Link")
        self.assertEqual(joints["camera_joint"].find("origin").get("xyz"), "0 0.1 0")

        left = links["gripper1_link"].find("collision")
        right = links["gripper2_link"].find("collision")
        left_center = float(joints["gripper1_joint"].find("origin").get("xyz").split()[0]) + 0.0285 + float(left.find("origin").get("xyz").split()[0])
        right_center = float(joints["gripper2_joint"].find("origin").get("xyz").split()[0]) - 0.0285 + float(right.find("origin").get("xyz").split()[0])
        finger_width = float(left.find("geometry/box").get("size").split()[0])
        self.assertGreater(right_center - left_center - finger_width, 0.0005)

    def test_moveit_references_only_existing_links_and_joints(self):
        robot = ET.parse(ROOT / "urdf/composite_robot_finger_mono.urdf").getroot()
        links = {item.get("name") for item in robot.findall("link")}
        joints = {item.get("name") for item in robot.findall("joint")}
        srdf = ET.parse(
            ROOT.parent / "seer_aubo_finger_mono_moveit_config/config/seer_aubo_composite.srdf"
        ).getroot()
        for group in srdf.findall("group"):
            for joint in group.findall("joint"):
                self.assertIn(joint.get("name"), joints)
        for pair in srdf.findall("disable_collisions"):
            self.assertIn(pair.get("link1"), links)
            self.assertIn(pair.get("link2"), links)

    def test_usd_joint_frames_meshes_and_camera(self):
        stage = Usd.Stage.Open(str(ROOT / "urdf/warehouse_finger_mono_demo.usda"))
        cache = UsdGeom.XformCache()
        for name in ("adapter_mount_joint", "motor_mount_joint", "gripper1_joint", "gripper2_joint"):
            joint = UsdPhysics.Joint(stage.GetPrimAtPath(f"{ROBOT}/joints/{name}"))
            frames = []
            for body, position, rotation in (
                (joint.GetBody0Rel(), joint.GetLocalPos0Attr(), joint.GetLocalRot0Attr()),
                (joint.GetBody1Rel(), joint.GetLocalPos1Attr(), joint.GetLocalRot1Attr()),
            ):
                local = Gf.Matrix4d(1)
                local.SetRotate(Gf.Quatd(rotation.Get()))
                local.SetTranslateOnly(Gf.Vec3d(position.Get()))
                frames.append(local * cache.GetLocalToWorldTransform(stage.GetPrimAtPath(body.GetTargets()[0])))
            self.assertLess((frames[0].ExtractTranslation() - frames[1].ExtractTranslation()).GetLength(), 1e-6)
            for axis in (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1)):
                self.assertLess((frames[0].TransformDir(axis) - frames[1].TransformDir(axis)).GetLength(), 1e-6)
        for name, asset in (("gripper_motor_link", "motor_adapter.usdc"), ("gripper1_link", "finger.usdc"), ("gripper2_link", "finger.usdc")):
            visual = stage.GetPrimAtPath(f"{ROBOT}/{name}/visual")
            self.assertIn(asset, str(visual.GetMetadata("references")))
        camera = stage.GetPrimAtPath(f"{ROBOT}/mono_camera_link/camera_optical_frame/ros2_camera")
        self.assertTrue(camera.IsA(UsdGeom.Camera))
        self.assertEqual(camera.GetAttribute("sensor:model").Get(), "MV-CH100-60UM")


if __name__ == "__main__":
    unittest.main()
