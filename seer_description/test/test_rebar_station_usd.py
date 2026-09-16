"""Validate the generated rebar station layer with USD (pxr), without Isaac.

Run with a Python environment that provides USD, e.g.:

    ~/isaacsim/python.sh test/test_rebar_station_usd.py
"""

from pathlib import Path
import unittest
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from rebar_experiment_geometry import (RACK_REBAR_Z, RACK_RELEASE_TCP_Z,
                                       RACK_PEDESTAL_BOTTOM_Z, RACK_DECK_Z, rack_boxes)

simulation_app = None
try:
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
except ModuleNotFoundError:
    # Isaac Sim 6 exposes USD modules after SimulationApp initialization.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": True})
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

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
            block_top = block_z + 0.027  # half of the pedestal height
            self.assertGreater(rebar_z, block_top)
            self.assertLess(rebar_z - block_top, 0.05)

            block_x = cache.GetLocalToWorldTransform(block).ExtractTranslation()[0]
            self.assertAlmostEqual(block_x, expected_x)

    def test_concave_supports_and_chassis_attachment(self):
        # Arc segments preserve the hollow seat, rather than filling it with
        # one convex collider; paired high sides constrain lateral rolling.
        for support in (1, 2):
            segments = [self.stage.GetPrimAtPath(f"{STATION}/SourceSaddle{support}_{i}")
                        for i in range(1, 15)]
            self.assertTrue(all(p.HasAPI(UsdPhysics.CollisionAPI) for p in segments))
            cache = UsdGeom.XformCache()
            centers = [cache.GetLocalToWorldTransform(p).ExtractTranslation() for p in segments]
            self.assertLess(centers[0][1], 0)
            self.assertGreater(centers[-1][1], 0)
            self.assertGreater(centers[0][2], centers[6][2] + .010)
        assets = Path(__file__).resolve().parents[1] / "urdf"
        loading = Usd.Stage.Open(str(assets / "warehouse_finger_rebar_loading_demo.usda"))
        rack = loading.GetPrimAtPath("/World/seer_aubo_composite/base_link/RebarRack")
        self.assertTrue(rack.GetParent().HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertEqual(len(rack.GetChildren()), 138)
        active = [p for p in rack.GetChildren()
                  if UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()]
        self.assertEqual(len(active), 74)
        self.assertEqual(len([p for p in active if 'VSeat' in p.GetName()]), 16)
        for p in rack.GetChildren():
            if 'Saddle' in p.GetName() and 4 <= int(p.GetName().rsplit('_', 1)[1]) <= 11:
                self.assertFalse(UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get())
        self.assertTrue(all(p.HasAPI(UsdPhysics.CollisionAPI) and
                            not p.HasAPI(UsdPhysics.RigidBodyAPI) for p in rack.GetChildren()))

    def test_prefilled_rack_contains_three_independent_dynamic_bars(self):
        assets = Path(__file__).resolve().parents[1] / "urdf"
        stage = Usd.Stage.Open(str(assets / "warehouse_finger_rebar_loading_prefilled_demo.usda"))
        for slot in (2, 3, 4):
            bar = stage.GetPrimAtPath(f"/World/LoadedRebar{slot}")
            self.assertTrue(bar.HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertEqual(UsdGeom.Cylinder(bar).GetAxisAttr().Get(), "Y")
            self.assertAlmostEqual(UsdGeom.Cylinder(bar).GetHeightAttr().Get(), .6)
            self.assertAlmostEqual(UsdPhysics.MassAPI(bar).GetMassAttr().Get(), 2.13, delta=.05)
        self.assertFalse(stage.GetPrimAtPath("/World/LoadedRebar1").IsValid())

    def test_raised_seats_match_planner_and_preserve_release_target(self):
        assets = Path(__file__).resolve().parents[1] / "urdf"
        stage = Usd.Stage.Open(str(assets / "warehouse_finger_rebar_loading_prefilled_demo.usda"))
        rack = stage.GetPrimAtPath("/World/seer_aubo_composite/base_link/RebarRack")
        for name, position, size, rpy in rack_boxes():
            prim = rack.GetChild(name)
            actual = prim.GetAttribute("xformOp:translate").Get()
            for coordinate, expected in zip(actual, position):
                self.assertAlmostEqual(coordinate, expected, places=6)
            if "Support" in name:
                actual_size = prim.GetAttribute("xformOp:scale").Get()
                self.assertAlmostEqual(actual[2] - actual_size[2]/2,
                                       RACK_PEDESTAL_BOTTOM_Z, places=6)
                self.assertAlmostEqual(actual_size[2], .095, places=6)
        self.assertAlmostEqual(RACK_RELEASE_TCP_Z, .720)
        self.assertAlmostEqual(RACK_RELEASE_TCP_Z - RACK_REBAR_Z, .005)
        for slot in (2, 3, 4):
            bar = stage.GetPrimAtPath(f"/World/LoadedRebar{slot}")
            self.assertAlmostEqual(bar.GetAttribute("xformOp:translate").Get()[2], RACK_REBAR_Z)

    def test_solid_mounting_beams_touch_visual_chassis_and_all_pedestals(self):
        assets = Path(__file__).resolve().parents[1] / "urdf"
        stage = Usd.Stage.Open(str(assets / "warehouse_finger_rebar_loading_demo.usda"))
        base = stage.GetPrimAtPath("/World/seer_aubo_composite/base_link")
        rack = base.GetChild("RebarRack")
        cache = UsdGeom.XformCache()
        inverse = cache.GetLocalToWorldTransform(base).GetInverse()
        deck_top = float("-inf")
        for prim in Usd.PrimRange(base.GetChild("visuals"), Usd.TraverseInstanceProxies()):
            if prim.IsA(UsdGeom.Mesh):
                matrix = cache.GetLocalToWorldTransform(prim) * inverse
                # This body has a flat upper deck under both mounting rails.
                points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
                deck_top = max(deck_top, max(matrix.Transform(p)[2] for p in points))
        self.assertAlmostEqual(deck_top, RACK_DECK_Z, places=5)
        for support in (1, 2):
            beam = rack.GetChild(f"MountingBeam{support}")
            center = beam.GetAttribute("xformOp:translate").Get()
            size = beam.GetAttribute("xformOp:scale").Get()
            bottom, top = center[2] - size[2]/2, center[2] + size[2]/2
            self.assertLess(bottom, deck_top)
            self.assertLess(deck_top - bottom, .002)
            self.assertGreater(top, RACK_PEDESTAL_BOTTOM_Z)
            self.assertTrue(UsdPhysics.CollisionAPI(beam).GetCollisionEnabledAttr().Get())
            self.assertFalse(beam.HasAPI(UsdPhysics.RigidBodyAPI))
            for slot in range(1, 5):
                pedestal = rack.GetChild(f"Slot{slot}Support{support}")
                p = pedestal.GetAttribute("xformOp:translate").Get()
                s = pedestal.GetAttribute("xformOp:scale").Get()
                for i in (0, 1):
                    self.assertGreaterEqual(center[i]+size[i]/2, p[i]+s[i]/2)
                    self.assertLessEqual(center[i]-size[i]/2, p[i]-s[i]/2)

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

    def test_ribbed_visuals_have_resolved_pbr_and_no_extra_physics(self):
        assets = Path(__file__).resolve().parents[1] / "urdf"
        stage = Usd.Stage.Open(str(assets / "warehouse_finger_rebar_loading_prefilled_demo.usda"))
        for path in (f"{STATION}/Rebar", *(f"/World/LoadedRebar{i}" for i in (2, 3, 4))):
            bar = stage.GetPrimAtPath(path)
            self.assertEqual(UsdGeom.Imageable(bar).ComputePurpose(), "default")
            invisible, _ = UsdShade.MaterialBindingAPI(bar).ComputeBoundMaterial()
            self.assertEqual(invisible.ComputeSurfaceSource()[0].GetInput("opacity").Get(), 0)
            visual = stage.GetPrimAtPath(path + "/SteelVisual")
            mesh = UsdGeom.Mesh(stage.GetPrimAtPath(path + "/SteelVisual/Surface"))
            self.assertEqual(mesh.ComputePurpose(), "default")
            self.assertTrue(all(not p.HasAPI(UsdPhysics.CollisionAPI) and
                                not p.HasAPI(UsdPhysics.RigidBodyAPI)
                                for p in Usd.PrimRange(visual)))
            points = mesh.GetPointsAttr().Get()
            self.assertEqual(len(mesh.GetExtentAttr().Get()), 2)
            self.assertEqual(len(points), len(mesh.GetNormalsAttr().Get()))
            self.assertEqual(len(points), len(UsdGeom.PrimvarsAPI(mesh).GetPrimvar("st").Get()))
            self.assertEqual(sum(mesh.GetFaceVertexCountsAttr().Get()),
                             len(mesh.GetFaceVertexIndicesAttr().Get()))
            self.assertLess(max(mesh.GetFaceVertexIndicesAttr().Get()), len(points))
            # Visual ribs fit inside the unchanged nominal collision envelope.
            radii = [(p[0]**2+p[1]**2)**.5 for p in points]
            self.assertLessEqual(max(radii), .012002)
            self.assertGreater(max(radii)-min(radii), .0006)
            self.assertAlmostEqual(max(p[2] for p in points)-min(p[2] for p in points), .6)
            bound, _ = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()
            self.assertTrue(bound)
            surface = bound.ComputeSurfaceSource()[0]
            self.assertEqual(surface.GetIdAttr().Get(), "UsdPreviewSurface")
            for name in ("BaseColor", "Roughness", "Metallic"):
                shader = UsdShade.Shader(stage.GetPrimAtPath(str(bound.GetPath()) + "/" + name))
                asset = shader.GetInput("file").Get()
                self.assertTrue(Path(asset.resolvedPath).is_file(), str(asset))
                self.assertEqual(shader.GetInput("sourceColorSpace").Get(),
                                 "sRGB" if name == "BaseColor" else "raw")

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
