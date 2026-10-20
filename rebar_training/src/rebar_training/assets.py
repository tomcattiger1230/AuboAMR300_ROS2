"""Create training-only USD wrappers around the repository assets.

Requires usd-core for offline preparation, or Isaac's pxr at runtime. Original
layers are never edited. All output references are relative and stay relocatable
when this repository (including generated artifacts) is copied to the 4090.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path

from .spec import ARM_JOINTS, FINGER_JOINTS, ISAAC_LAB_TAG, ISAAC_SIM_VERSION, JAW_NAMES, repository_root

SOURCE_ASSETS = {
    "robot": "seer_description/urdf/seer_aubo_finger_mono.usda",
    "station": "seer_description/urdf/rebar_station.usda",
    "machine": "seer_description/urdf/rebar_test_machine_lab.usda",
}


def build_assets(output_dir, repo_root=None, fixture_model="pads_v1"):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

    root = Path(repo_root or repository_root()).resolve()
    if fixture_model not in ("pads_v1", "full_v2"):
        raise ValueError("unknown fixture model")
    out = Path(output_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    for relative in SOURCE_ASSETS.values():
        if not (root / relative).is_file():
            raise FileNotFoundError(root / relative)

    def stage_for(filename, prim_name, type_name="Xform"):
        stage = Usd.Stage.CreateNew(str(out / filename))
        prim = stage.DefinePrim("/" + prim_name, type_name)
        stage.SetDefaultPrim(prim)
        UsdGeom.SetStageMetersPerUnit(stage, 1)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        return stage, prim

    def reference(prim, source, prim_path):
        prim.GetReferences().AddReference(os.path.relpath(root / source, out), prim_path)

    def origin(prim, position=(0, 0, 0), yaw=0):
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*position))
        attr = prim.GetAttribute("xformOp:orient")
        precision = UsdGeom.XformOp.PrecisionDouble if not attr or attr.GetTypeName() == Sdf.ValueTypeNames.Quatd else UsdGeom.XformOp.PrecisionFloat
        op = xform.AddOrientOp(precision=precision)
        q = Gf.Quatd(math.cos(yaw / 2), Gf.Vec3d(0, 0, math.sin(yaw / 2)))
        op.Set(q if precision == UsdGeom.XformOp.PrecisionDouble else Gf.Quatf(q))

    robot_stage, robot = stage_for("robot.usda", "Robot")
    # These imported sensor/visual references are unused by state training and
    # include unavailable cloud URLs / broken visual prims in the source model.
    for child in ("back_lidar_link/microScan3_b", "front_lidar_link/microScan3_f",
                  "base_footprint/visuals", "front_caster/visuals", "rear_caster/visuals"):
        unused = robot_stage.OverridePrim("/Robot/" + child)
        unused.GetReferences().SetReferences([])
        unused.SetActive(False)
    reference(robot, SOURCE_ASSETS["robot"], "/World/seer_aubo_composite")
    origin(robot)
    # One explicit articulation root and a parked base. Only the robot is fixed;
    # there is no joint, disable-gravity flag or kinematic API on the steel bar.
    for prim in list(Usd.PrimRange(robot)):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    UsdPhysics.ArticulationRootAPI.Apply(robot)
    base = robot_stage.GetPrimAtPath("/Robot/base_footprint")
    if not base or not base.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError("source robot must expose rigid base_footprint")
    fixed = UsdPhysics.FixedJoint.Define(robot_stage, "/Robot/training_base_lock")
    fixed.CreateBody1Rel().SetTargets([base.GetPath()])
    base_pose = UsdGeom.Xformable(base).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    fixed.CreateLocalPos0Attr().Set(Gf.Vec3f(*base_pose.ExtractTranslation()))
    fixed.CreateLocalRot0Attr().Set(Gf.Quatf(base_pose.ExtractRotationQuat()))
    # The wrist and motor housing overlap across the fixed adapter. PhysX only
    # excludes directly joint-connected links, so exclude this additional pair.
    wrist = robot_stage.GetPrimAtPath("/Robot/wrist3_Link")
    UsdPhysics.FilteredPairsAPI.Apply(wrist).CreateFilteredPairsRel().AddTarget("/Robot/gripper_motor_link")
    robot_source_layers = [Path(layer.realPath).resolve() for layer in robot_stage.GetUsedLayers() if layer.realPath]
    # A stronger inactive opinion does not prevent the source reference arcs
    # from being resolved by Kit's resolver. Bake the composed robot locally so
    # the disabled SICK cloud references cannot stall simulation startup.
    while True:
        instances = [prim for prim in robot_stage.Traverse() if prim.IsInstance()]
        if not instances:
            break
        for prim in instances:
            prim.SetInstanceable(False)
    flattened = robot_stage.Flatten(addSourceFileComment=False)
    UsdUtils.ModifyAssetPaths(flattened, lambda path: os.path.relpath(path, out) if os.path.isabs(path) else path)
    robot_stage.GetRootLayer().TransferContent(flattened)
    robot_stage.GetRootLayer().Save()

    station_stage, station = stage_for("station.usda", "Station")
    reference(station, SOURCE_ASSETS["station"], "/World/RebarStation")
    origin(station)
    station_stage.OverridePrim("/Station/Rebar").SetActive(False)
    UsdPhysics.RigidBodyAPI.Apply(station).CreateKinematicEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(station).CreateMassAttr(1.0)
    station_stage.GetRootLayer().Save()

    # UsdFileCfg creates an Xform at the spawn path. Keep the actual cylinder
    # below that container so the loader cannot override its geometry type.
    bar_stage, _ = stage_for("rebar.usda", "Rebar")
    bar = bar_stage.DefinePrim("/Rebar/body", "Cylinder")
    reference(bar, SOURCE_ASSETS["station"], "/World/RebarStation/Rebar")
    origin(bar)
    # Referencing the cylinder alone excludes its sibling physics material.
    material = UsdPhysics.MaterialAPI.Apply(bar_stage.DefinePrim("/Rebar/ContactMaterial", "Material"))
    material.CreateStaticFrictionAttr(0.8)
    material.CreateDynamicFrictionAttr(0.8)
    material.CreateRestitutionAttr(0.0)
    from pxr import UsdShade
    UsdShade.MaterialBindingAPI.Apply(bar).Bind(UsdShade.Material(material.GetPrim()), materialPurpose="physics")
    UsdPhysics.RigidBodyAPI.Apply(bar).CreateKinematicEnabledAttr(False)
    UsdPhysics.CollisionAPI.Apply(bar).CreateCollisionEnabledAttr(True)
    bar_stage.GetRootLayer().Save()

    machine_stage, machine = stage_for("machine.usda", "Machine")
    reference(machine, SOURCE_ASSETS["machine"], "/World/RebarTestMachine")
    origin(machine, (-0.95, -0.08, 0), math.pi)
    # Visual-only jaws in the demo are replaced by an explicit contact fixture.
    for name in ("UpperCarriage", "LowerCarriage"):
        machine_stage.OverridePrim("/Machine/" + name).SetActive(False)
    machine_stage.GetRootLayer().Save()

    fixture_stage, fixture = stage_for("fixture.usda", "Fixture")
    UsdPhysics.ArticulationRootAPI.Apply(fixture)
    mount = UsdGeom.Xform.Define(fixture_stage, "/Fixture/mount").GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(mount)
    UsdPhysics.MassAPI.Apply(mount).CreateMassAttr(10)
    lock = UsdPhysics.FixedJoint.Define(fixture_stage, "/Fixture/mount_lock")
    lock.CreateBody1Rel().SetTargets([mount.GetPath()])
    mat = UsdShade.Material.Define(fixture_stage, "/Fixture/PadMaterial")
    friction = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    friction.CreateStaticFrictionAttr(0.8)
    friction.CreateDynamicFrictionAttr(0.8)
    for name in JAW_NAMES:
        # Upper/lower end pads reproduce the source jaw-tip size. Sliding jaws
        # are a training proxy; their force, stroke and friction need calibration.
        left = name.endswith("left")
        sign = 1 if left else -1
        position = (-0.87, -0.08 + sign * 0.10, 1.795 if name.startswith("upper") else 1.195)
        pad = UsdGeom.Xform.Define(fixture_stage, "/Fixture/" + name).GetPrim()
        origin(pad, position)
        UsdPhysics.RigidBodyAPI.Apply(pad)
        UsdPhysics.MassAPI.Apply(pad).CreateMassAttr(2.0)
        cube = UsdGeom.Cube.Define(fixture_stage, str(pad.GetPath()) + "/collision")
        cube.CreateSizeAttr(1.0)
        UsdGeom.Xformable(cube).AddScaleOp().Set(Gf.Vec3f(0.10, 0.04, 0.035))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat, materialPurpose="physics")
        joint = UsdPhysics.PrismaticJoint.Define(fixture_stage, "/Fixture/" + name + "_joint")
        joint.CreateBody0Rel().SetTargets([mount.GetPath()])
        joint.CreateBody1Rel().SetTargets([pad.GetPath()])
        joint.CreateAxisAttr("Y")
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*position))
        joint.CreateLowerLimitAttr(-0.075 if left else 0.0)
        joint.CreateUpperLimitAttr(0.0 if left else 0.075)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(5000)
        drive.CreateDampingAttr(200)
        drive.CreateMaxForceAttr(100)
    fixture_stage.GetRootLayer().Save()

    if fixture_model == "full_v2":
        from .full_machine import upgrade_fixture
        upgrade_fixture(fixture_stage, root / SOURCE_ASSETS["machine"])

    sources = {root / p for p in SOURCE_ASSETS.values()}
    sources.update(path for path in robot_source_layers if path.is_relative_to(root) and not path.is_relative_to(out))
    for stage in (robot_stage, station_stage, bar_stage, machine_stage, fixture_stage):
        for layer in stage.GetUsedLayers():
            if layer.realPath:
                path = Path(layer.realPath).resolve()
                if path.is_relative_to(root) and not path.is_relative_to(out):
                    sources.add(path)
    manifest = {"schema_version": 1, "isaac_lab": ISAAC_LAB_TAG, "isaac_sim": ISAAC_SIM_VERSION,
                "task_frame": "parked robot base, Z-up, metres", "quaternion_order": "xyzw",
                "source_sha256": {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(sources)},
                "artificial_rebar_support": False, "fixture_is_training_proxy": True, "robot_flattened": True,
                "gpu_validated": False}
    manifest["fixture_model"] = fixture_model
    if fixture_model == "full_v2":
        from .full_machine import CARRIAGE_HEIGHTS
        manifest["machine_parameters"] = {
            "source": "photo-derived geometry; not manufacturer calibration",
            "carriage_operating_height_m": CARRIAGE_HEIGHTS,
            "carriage_mass_kg": 20, "carriage_effort_limit_N": 10000,
            "carriage_stiffness_N_m": 2000000,
            "crosshead_collider": "central 0.86 m span leaves estimated column guide clearance",
            "jaw_drive_and_contact": "unchanged from pads_v1; requires real calibration",
        }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return audit_assets(out)


def audit_assets(directory):
    """Open composed USD and inspect bodies/joints; no PhysX or GPU required."""
    from pxr import Usd, UsdGeom, UsdPhysics
    directory = Path(directory)
    stages = {name: Usd.Stage.Open(str(directory / (name + ".usda")))
              for name in ("robot", "station", "rebar", "machine", "fixture")}
    robot = stages["robot"]
    joints = {p.GetName() for p in robot.Traverse() if p.IsA(UsdPhysics.Joint)}
    missing = set(ARM_JOINTS + FINGER_JOINTS) - joints
    if missing:
        raise RuntimeError(f"robot joints missing: {sorted(missing)}")
    bar = stages["rebar"].GetPrimAtPath("/Rebar/body")
    if not bar.IsA(UsdGeom.Cylinder) or bar.GetAttribute("axis").Get() != "X":
        raise RuntimeError("rebar collider must remain a local-X cylinder")
    if (not math.isclose(bar.GetAttribute("radius").Get(), 0.012)
            or not math.isclose(bar.GetAttribute("height").Get(), 0.6)):
        raise RuntimeError("rebar dimensions must match the 24 mm x 0.6 m fixture")
    if not bar.HasAPI(UsdPhysics.RigidBodyAPI) or bar.GetAttribute("physics:kinematicEnabled").Get():
        raise RuntimeError("rebar must be a dynamic rigid body")
    if not bar.GetAttribute("physics:collisionEnabled").Get():
        raise RuntimeError("rebar contact must remain enabled")
    inactive_warnings = []
    for stage in stages.values():
        for error in stage.GetCompositionErrors():
            path = error.rootSite.path
            prim = stage.GetPrimAtPath(path)
            if prim and not prim.IsActive():
                inactive_warnings.append(str(path))
            else:
                raise RuntimeError(f"active USD composition error: {error}")
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Joint):
                bodies = prim.GetRelationship("physics:body0").GetTargets() + prim.GetRelationship("physics:body1").GetTargets()
                if any("Rebar" in str(path) for path in bodies):
                    raise RuntimeError("a joint must not attach the rebar")
    fixture_joints = [p for p in stages["fixture"].Traverse() if p.IsA(UsdPhysics.PrismaticJoint)]
    manifest = json.loads((directory / "manifest.json").read_text())
    full = manifest.get("fixture_model", "pads_v1") == "full_v2"
    if len(fixture_joints) != (6 if full else 4):
        raise RuntimeError("fixture joint count differs from its declared model")
    if full:
        from .full_machine import audit_full_fixture
        audit_full_fixture(stages["fixture"])
    return {"usd_composition": "passed", "robot_joint_count": len(joints), "fixture_joint_count": len(fixture_joints),
            "fixture_model": manifest.get("fixture_model", "pads_v1"),
            "rebar_dynamic": True, "rebar_collision_enabled": True,
            "inactive_source_reference_warnings": sorted(inactive_warnings), "gpu_validated": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--audit-only", action="store_true")
    parser.add_argument("--fixture-model", choices=("pads_v1", "full_v2"), default="pads_v1")
    args = parser.parse_args()
    result = audit_assets(args.output_dir) if args.audit_only else build_assets(args.output_dir, fixture_model=args.fixture_model)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
