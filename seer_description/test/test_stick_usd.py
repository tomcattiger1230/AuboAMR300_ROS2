"""Run with a Python environment providing USD (pxr), without starting Isaac."""

from pathlib import Path
import unittest

from pxr import Gf, Usd, UsdGeom, UsdPhysics


class StickArticulationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assets = Path(__file__).resolve().parents[1] / "urdf"
        cls.stage = Usd.Stage.Open(str(assets / "seer_aubo_stick.usda"))
        cls.cache = UsdGeom.XformCache()
        cls.root = "/World/seer_aubo_composite"

    def test_i16h_j3_limits(self):
        for asset in ("seer_aubo_stick.usda", "seer_aubo_stick_mono.usda", "warehouse_stick_mono_demo.usda"):
            with self.subTest(asset=asset):
                stage = Usd.Stage.Open(str(Path(__file__).resolve().parents[1] / "urdf" / asset))
                joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(self.root + "/joints/foreArm_joint"))
                self.assertEqual(joint.GetLowerLimitAttr().Get(), -161.0)
                self.assertEqual(joint.GetUpperLimitAttr().Get(), 161.0)

    def test_stage_units_and_gravity_axis(self):
        self.assertEqual(UsdGeom.GetStageUpAxis(self.stage), UsdGeom.Tokens.z)
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(self.stage), 1.0)

    def test_caster_friction_is_not_averaged_with_ground(self):
        material = self.stage.GetPrimAtPath(self.root + '/PhysicsMaterials/CasterMaterial')
        self.assertEqual(material.GetAttribute('physxMaterial:frictionCombineMode').Get(), 'min')
        self.assertEqual(material.GetAttribute('physics:dynamicFriction').Get(), 0.)

    def test_joint_frames_match_at_zero_position(self):
        for name in ("adapter_mount_joint", "motor_mount_joint",
                     "gripper1_joint", "gripper2_joint"):
            with self.subTest(joint=name):
                joint = UsdPhysics.Joint(self.stage.GetPrimAtPath(self.root + "/joints/" + name))
                frames = []
                for body, position, rotation in (
                    (joint.GetBody0Rel(), joint.GetLocalPos0Attr(), joint.GetLocalRot0Attr()),
                    (joint.GetBody1Rel(), joint.GetLocalPos1Attr(), joint.GetLocalRot1Attr()),
                ):
                    prim = self.stage.GetPrimAtPath(body.GetTargets()[0])
                    self.assertTrue(prim.HasAPI(UsdPhysics.RigidBodyAPI))
                    parent = prim.GetParent()
                    while parent:
                        self.assertFalse(parent.HasAPI(UsdPhysics.RigidBodyAPI))
                        parent = parent.GetParent()
                    local = Gf.Matrix4d(1)
                    local.SetRotate(Gf.Quatd(rotation.Get()))
                    local.SetTranslateOnly(Gf.Vec3d(position.Get()))
                    frames.append(local * self.cache.GetLocalToWorldTransform(prim))
                self.assertLess((frames[0].ExtractTranslation() - frames[1].ExtractTranslation()).GetLength(), 1e-6)
                for axis in (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1)):
                    self.assertLess((frames[0].TransformDir(axis) - frames[1].TransformDir(axis)).GetLength(), 1e-6)

    def test_fingers_have_drives_limits_and_collisions(self):
        for number in (1, 2):
            with self.subTest(finger=number):
                prim = self.stage.GetPrimAtPath(f"{self.root}/joints/gripper{number}_joint")
                joint = UsdPhysics.PrismaticJoint(prim)
                self.assertTrue(joint)
                self.assertEqual(joint.GetAxisAttr().Get(), "Y")
                self.assertEqual(joint.GetLowerLimitAttr().Get(), 0.0)
                self.assertAlmostEqual(joint.GetUpperLimitAttr().Get(), 0.04)
                drive = UsdPhysics.DriveAPI(prim, "linear")
                self.assertGreater(drive.GetStiffnessAttr().Get(), 0)
                self.assertEqual(drive.GetMaxForceAttr().Get(), 50.0)
                body = self.stage.GetPrimAtPath(joint.GetBody1Rel().GetTargets()[0])
                colliders = [p for p in Usd.PrimRange(body, Usd.TraverseInstanceProxies())
                             if p.HasAPI(UsdPhysics.CollisionAPI)]
                self.assertTrue(colliders)


if __name__ == "__main__":
    unittest.main()
