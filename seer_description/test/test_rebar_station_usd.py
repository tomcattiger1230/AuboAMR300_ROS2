"""Validate the generated rebar station layer with USD (pxr), without Isaac.

Run with a Python environment that provides USD, e.g.:

    ~/isaacsim/python.sh test/test_rebar_station_usd.py
"""

from pathlib import Path
import unittest

simulation_app = None
try:
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics
except ModuleNotFoundError:
    # Isaac Sim 6 exposes USD modules after SimulationApp initialization.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": True})
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

STATION = "/World/RebarStation"


class RebarStationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assets = Path(__file__).resolve().parents[1] / "urdf"
        cls.stage = Usd.Stage.Open(str(assets / "rebar_station.usda"))
        cls.scene = Usd.Stage.Open(
            str(assets / "warehouse_stick_rebar_mono_demo.usda")
        )

    def test_stage_units_and_up_axis(self):
        for stage in (self.stage, self.scene):
            with self.subTest():
                self.assertEqual(UsdGeom.GetStageUpAxis(stage), UsdGeom.Tokens.z)
                self.assertEqual(UsdGeom.GetStageMetersPerUnit(stage), 1.0)

    def test_rebar_is_a_dynamic_rigid_body(self):
        rebar = self.stage.GetPrimAtPath(f"{STATION}/Rebar")
        self.assertTrue(rebar.IsValid())
        self.assertTrue(rebar.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertTrue(rebar.HasAPI(UsdPhysics.MassAPI))
        self.assertTrue(rebar.HasAPI(UsdPhysics.CollisionAPI))

        cylinder = UsdGeom.Cylinder(rebar)
        self.assertEqual(cylinder.GetAxisAttr().Get(), "X")
        self.assertAlmostEqual(cylinder.GetRadiusAttr().Get(), 0.012)
        self.assertAlmostEqual(cylinder.GetHeightAttr().Get(), 0.6)

        # Steel cylinder: pi * r^2 * h * 7850 ~= 2.13 kg.
        mass = UsdPhysics.MassAPI(rebar).GetMassAttr().Get()
        self.assertAlmostEqual(mass, 2.13, delta=0.05)

    def test_rebar_rests_above_the_support_blocks(self):
        cache = UsdGeom.XformCache()
        rebar_z = cache.GetLocalToWorldTransform(
            self.stage.GetPrimAtPath(f"{STATION}/Rebar")
        ).ExtractTranslation()[2]
        for name, expected_x in (
            ("SupportBlock_1", -1.15 - 0.18),
            ("SupportBlock_2", -1.15 + 0.18),
        ):
            block = self.stage.GetPrimAtPath(f"{STATION}/{name}")
            self.assertTrue(block.IsValid())
            # Blocks are static: collision without a rigid body.
            self.assertTrue(block.HasAPI(UsdPhysics.CollisionAPI))
            self.assertFalse(block.HasAPI(UsdPhysics.RigidBodyAPI))
            block_z = cache.GetLocalToWorldTransform(block).ExtractTranslation()[2]
            block_top = block_z + 0.03  # half of the 0.06 m block height
            self.assertGreater(rebar_z, block_top)
            self.assertLess(rebar_z - block_top, 0.05)

            block_x = cache.GetLocalToWorldTransform(block).ExtractTranslation()[0]
            self.assertAlmostEqual(block_x, expected_x)

    def test_rebar_material_is_high_friction(self):
        material = self.stage.GetPrimAtPath(f"{STATION}/RebarMaterial")
        self.assertTrue(material.IsValid())
        self.assertEqual(
            material.GetAttribute("physxMaterial:frictionCombineMode").Get(), "max"
        )
        self.assertGreater(
            material.GetAttribute("physics:dynamicFriction").Get(), 0.5
        )
        rebar = self.stage.GetPrimAtPath(f"{STATION}/Rebar")
        targets = rebar.GetRelationship("material:binding:physics").GetForwardedTargets()
        self.assertEqual(targets, [Sdf.Path(f"{STATION}/RebarMaterial")])

    def test_wrapper_scene_composes_all_three_layers(self):
        root_layer = self.scene.GetRootLayer()
        sub_layers = root_layer.subLayerPaths
        self.assertIn("seer_aubo_stick_mono.usda", " ".join(sub_layers))
        self.assertIn("warehouse_stick_demo.usda", " ".join(sub_layers))
        self.assertIn("rebar_station.usda", " ".join(sub_layers))
        # The composed scene exposes both the robot and the rebar station.
        self.assertTrue(self.scene.GetPrimAtPath("/World/seer_aubo_composite").IsValid())
        self.assertTrue(self.scene.GetPrimAtPath(f"{STATION}/Rebar").IsValid())


if __name__ == "__main__":
    result = unittest.main(exit=False).result
    if simulation_app is not None:
        simulation_app.close()
    raise SystemExit(0 if result.wasSuccessful() else 1)
