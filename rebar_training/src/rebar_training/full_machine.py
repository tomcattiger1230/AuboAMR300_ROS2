"""Opt-in six-axis physical machine, retaining the source moving geometry.

Coordinates are those of the legacy fixture, before the loading-cell transform.
The original demo controller is never imported. Steel stays an independent body.
Masses, guide clearances and drives are estimates, not hardware calibration.
"""

CARRIAGE_HEIGHTS = {"upper": 1.88, "lower": 1.136}
CARRIAGE_TRAVEL = {"upper": (1.50, 1.88), "lower": (.92, 1.34)}


def upgrade_fixture(stage, source_path):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    source = Usd.Stage.Open(str(source_path))
    layer = stage.GetRootLayer()

    def source_cube(source_carriage, source_name, body, name, position, collision=True):
        path = str(body.GetPath()) + "/" + name
        Sdf.CopySpec(source.GetRootLayer(), f"/World/RebarTestMachine/{source_carriage}/{source_name}", layer, path)
        prim = stage.GetPrimAtPath(path)
        prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*position))
        if collision:
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        return prim

    for label, height in CARRIAGE_HEIGHTS.items():
        upper = label == "upper"
        source_carriage = "UpperCarriage" if upper else "LowerCarriage"
        body = UsdGeom.Xform.Define(stage, f"/Fixture/{label}_carriage").GetPrim()
        UsdGeom.Xformable(body).AddTranslateOp().Set(Gf.Vec3d(-.87, -.08, height))
        UsdPhysics.RigidBodyAPI.Apply(body)
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(20.)
        joint = UsdPhysics.PrismaticJoint.Define(stage, f"/Fixture/{label}_carriage_joint")
        joint.CreateBody0Rel().SetTargets([Sdf.Path("/Fixture/mount")])
        joint.CreateBody1Rel().SetTargets([body.GetPath()])
        joint.CreateAxisAttr("Z")
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(-.87, -.08, height))
        low, high = CARRIAGE_TRAVEL[label]
        joint.CreateLowerLimitAttr(low - height)
        joint.CreateUpperLimitAttr(high - height)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(2000000.)
        drive.CreateDampingAttr(20000.)
        drive.CreateMaxForceAttr(10000.)
        # The source crosshead spans the guide columns. A central collider
        # leaves the estimated guide bores clear, instead of intersecting the
        # columns or suppressing robot/steel collisions with the crosshead.
        z = .19 if upper else -.19
        source_cube(source_carriage, "Crosshead", body, "Crosshead", (-.07, 0, z), False)
        collider = UsdGeom.Cube.Define(stage, str(body.GetPath()) + "/CrossheadCollider")
        collider.CreateSizeAttr(1.)
        transform = UsdGeom.Xformable(collider)
        transform.AddTranslateOp().Set(Gf.Vec3d(-.07, 0, z))
        transform.AddScaleOp().Set(Gf.Vec3d(.52, .86, .15))
        UsdGeom.Imageable(collider).CreateVisibilityAttr("invisible")
        UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
        source_cube(source_carriage, "GripHead", body, "GripHead", (0, 0, .085 if upper else -.085))
        for side, sign in (("left", 1), ("right", -1)):
            name = f"{label}_{side}"
            pad = stage.GetPrimAtPath("/Fixture/" + name)
            pad.GetAttribute("xformOp:translate").Set(
                Gf.Vec3d(-.87, -.08 + sign * .10, height + (-.075 if upper else .075)))
            # Legacy left/right refer to world Y. Source local Y is reversed
            # by the machine's 180-degree yaw; explicitly swap source names.
            source_side = "Right" if side == "left" else "Left"
            source_cube(source_carriage, "Jaw" + source_side, pad, "Jaw", (0, 0, .075 if upper else -.075))
            tip = source.GetPrimAtPath(f"/World/RebarTestMachine/{source_carriage}/Jaw{source_side}Tip")
            UsdGeom.Gprim(pad.GetChild("collision")).CreateDisplayColorAttr(
                tip.GetAttribute("primvars:displayColor").Get())
            slider = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath("/Fixture/" + name + "_joint"))
            slider.GetBody0Rel().SetTargets([body.GetPath()])
            slider.GetLocalPos0Attr().Set(Gf.Vec3f(0, sign * .10, -.075 if upper else .075))
    layer.Save()


def audit_full_fixture(stage):
    from pxr import UsdPhysics
    for label in CARRIAGE_HEIGHTS:
        body = stage.GetPrimAtPath(f"/Fixture/{label}_carriage")
        if not body.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError("carriage must be a rigid body")
        for name in ("GripHead", "CrossheadCollider"):
            if not body.GetChild(name).HasAPI(UsdPhysics.CollisionAPI):
                raise RuntimeError("missing moving carriage collision")
        for side in ("left", "right"):
            name = f"{label}_{side}"
            jaw = stage.GetPrimAtPath(f"/Fixture/{name}/Jaw")
            if not jaw.HasAPI(UsdPhysics.CollisionAPI):
                raise RuntimeError("missing moving jaw collision")
            joint = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(f"/Fixture/{name}_joint"))
            if joint.GetBody0Rel().GetTargets() != [body.GetPath()]:
                raise RuntimeError("jaw must move with its carriage")
