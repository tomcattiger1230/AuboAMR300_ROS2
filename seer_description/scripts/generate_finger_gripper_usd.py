#!/usr/bin/env python3
"""Build metre-scale USD meshes and the finger-gripper Isaac Sim override."""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import struct

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


ROBOT = "/World/seer_aubo_composite"
GRIPPER_CLOSED = 0.0285


def read_binary_stl(path: Path, scale=1.0, rpy=(0.0, 0.0, 0.0)):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"Invalid binary STL: {path}")
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f"Unexpected binary STL length: {path}")
    points = []
    indices = []
    known = {}
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    for face in range(count):
        values = struct.unpack_from("<12fH", data, 84 + 50 * face)
        for vertex in range(3):
            source = tuple(values[3 + vertex * 3 + axis] * scale for axis in range(3))
            point = tuple(sum(rotation[row][axis] * source[axis] for axis in range(3)) for row in range(3))
            index = known.get(point)
            if index is None:
                index = len(points)
                known[point] = index
                points.append(Gf.Vec3f(*point))
            indices.append(index)
    return points, indices


def mesh_asset(stl: Path, output: Path, root_name: str, color, scale=1.0, rpy=(0.0, 0.0, 0.0)):
    points, indices = read_binary_stl(stl, scale, rpy)
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, f"/{root_name}")
    stage.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(stage, f"/{root_name}/mesh")
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([3] * (len(indices) // 3))
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    minimum = Gf.Vec3f(*(min(p[axis] for p in points) for axis in range(3)))
    maximum = Gf.Vec3f(*(max(p[axis] for p in points) for axis in range(3)))
    mesh.CreateExtentAttr([minimum, maximum])
    stage.GetRootLayer().Save()


def set_transform(prim, matrix):
    transform = UsdGeom.Xformable(prim)
    transform.ClearXformOpOrder()
    transform.AddTranslateOp().Set(matrix.ExtractTranslation())
    transform.AddOrientOp().Set(Gf.Quatf(matrix.ExtractRotationQuat()))


def rigid_body(stage, name, world, mass_value, inertia):
    prim = UsdGeom.Xform.Define(stage, f"{ROBOT}/{name}").GetPrim()
    prim.GetReferences().ClearReferences()
    set_transform(prim, world)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass = UsdPhysics.MassAPI.Apply(prim)
    mass.CreateMassAttr(mass_value)
    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
    mass.CreatePrincipalAxesAttr(Gf.Quatf(1))
    return prim


def referenced_visual(stage, body, name, asset, translate, rotate_z=0.0):
    visual = UsdGeom.Xform.Define(stage, f"{body.GetPath()}/{name}")
    visual.GetPrim().GetReferences().AddReference(asset)
    visual.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if rotate_z:
        visual.AddRotateZOp().Set(rotate_z)


def box_collision(stage, body, name, translate, size):
    box = UsdGeom.Cube.Define(stage, f"{body.GetPath()}/{name}")
    box.CreateSizeAttr(1.0)
    box.AddTranslateOp().Set(Gf.Vec3d(*translate))
    box.AddScaleOp().Set(Gf.Vec3f(*size))
    box.CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(box.GetPrim()).CreateCollisionEnabledAttr(True)


def fixed_joint(stage, name, body0, body1):
    joint = UsdPhysics.FixedJoint.Define(stage, f"{ROBOT}/joints/{name}")
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateLocalPos0Attr(Gf.Vec3f(0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0))
    joint.CreateLocalRot0Attr(Gf.Quatf(1))
    joint.CreateLocalRot1Attr(Gf.Quatf(1))
    joint.CreateCollisionEnabledAttr(False)


def prismatic_joint(stage, name, child, origin, reverse=False):
    joint = UsdPhysics.PrismaticJoint.Define(stage, f"{ROBOT}/joints/{name}")
    joint.CreateBody0Rel().SetTargets([f"{ROBOT}/gripper_motor_link"])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateAxisAttr("X")
    joint.CreateLowerLimitAttr(0.0)
    joint.CreateUpperLimitAttr(GRIPPER_CLOSED)
    joint.CreateLocalPos0Attr(Gf.Vec3f(*origin))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0))
    rotation = Gf.Quatf(0, 0, 0, 1) if reverse else Gf.Quatf(1)
    joint.CreateLocalRot0Attr(rotation)
    joint.CreateLocalRot1Attr(rotation)
    joint.CreateCollisionEnabledAttr(False)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(5000.0)
    drive.CreateDampingAttr(500.0)
    drive.CreateMaxForceAttr(50.0)
    drive.CreateTargetPositionAttr(0.0)


def generate(urdf_directory: Path, mesh_directory: Path):
    urdf_directory = urdf_directory.resolve()
    mesh_directory = mesh_directory.resolve()
    usd_meshes = mesh_directory / "usd"
    usd_meshes.mkdir(exist_ok=True)
    # Blender exports are already in metres. Bake the URDF visual rotations into
    # the mesh assets so each USD rigid-body frame matches its collision box.
    mesh_asset(mesh_directory / "finger_centered.stl", usd_meshes / "finger.usdc",
               "finger", (0.76, 0.78, 0.82), rpy=(-math.pi / 2, 0, 0))
    mesh_asset(mesh_directory / "motor_new.stl", usd_meshes / "motor_adapter.usdc",
               "motor_adapter", (0.16, 0.18, 0.21), rpy=(math.pi, 0, 0))

    source = Usd.Stage.Open(str(urdf_directory / "seer_aubo_stick_mono.usda"))
    if source is None:
        raise RuntimeError("Could not open seer_aubo_stick_mono.usda")
    wrist = source.GetPrimAtPath(f"{ROBOT}/wrist3_Link")
    wrist_world = UsdGeom.Xformable(wrist).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    output = urdf_directory / "seer_aubo_finger_mono.usda"
    stage = Usd.Stage.CreateNew(str(output))
    stage.GetRootLayer().subLayerPaths = ["./seer_aubo_stick_mono.usda"]
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    adapter = rigid_body(stage, "gripper_adapter_link", wrist_world, 0.01, (1e-6, 1e-6, 1e-6))
    motor = rigid_body(stage, "gripper_motor_link", wrist_world, 1.2,
                       (0.0022738, 0.005817125, 0.004337125))
    referenced_visual(stage, motor, "visual", "../meshes/usd/motor_adapter.usdc", (0, 0, 0))
    box_collision(stage, motor, "collision", (0.021777213, -0.000106815, 0.067125965),
                  (0.1985, 0.063, 0.137))

    left_local = Gf.Matrix4d(1)
    left_local.SetTranslateOnly(Gf.Vec3d(-0.0564, 0, 0.135))
    right_local = Gf.Matrix4d(1)
    right_local.SetTranslateOnly(Gf.Vec3d(0.0564, 0, 0.135))
    finger_inertia = (0.000371752, 0.000070267, 0.000373515)
    left = rigid_body(stage, "gripper1_link", left_local * wrist_world, 0.18, finger_inertia)
    right = rigid_body(stage, "gripper2_link", right_local * wrist_world, 0.18, finger_inertia)
    referenced_visual(stage, left, "visual", "../meshes/usd/finger.usdc", (0, 0, 0))
    referenced_visual(stage, right, "visual", "../meshes/usd/finger.usdc", (0, 0, 0), 180.0)
    box_collision(stage, left, "collision", (0.003100952, 0.000095278, 0.024519681),
                  (0.049, 0.15, 0.047786))
    box_collision(stage, right, "collision", (-0.003100952, -0.000095278, 0.024519681),
                  (0.049, 0.15, 0.047786))

    fixed_joint(stage, "adapter_mount_joint", f"{ROBOT}/wrist3_Link", str(adapter.GetPath()))
    fixed_joint(stage, "motor_mount_joint", str(adapter.GetPath()), str(motor.GetPath()))
    prismatic_joint(stage, "gripper1_joint", str(left.GetPath()), (-0.0564, 0, 0.135))
    prismatic_joint(stage, "gripper2_joint", str(right.GetPath()), (0.0564, 0, 0.135), reverse=True)
    stage.GetRootLayer().Save()

    warehouse = urdf_directory / "warehouse_finger_mono_demo.usda"
    layer = Sdf.Layer.CreateNew(str(warehouse))
    layer.subLayerPaths = ["./seer_aubo_finger_mono.usda", "./warehouse_demo.usda"]
    layer.Save()
    warehouse_stage = Usd.Stage.Open(str(warehouse))
    UsdGeom.SetStageUpAxis(warehouse_stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(warehouse_stage, 1.0)
    warehouse_stage.SetDefaultPrim(warehouse_stage.GetPrimAtPath("/World"))
    warehouse_stage.GetRootLayer().Save()
    for path in (output, warehouse):
        path.write_text(path.read_text().rstrip() + "\n", encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf_directory", type=Path)
    parser.add_argument("mesh_directory", type=Path)
    arguments = parser.parse_args()
    generate(arguments.urdf_directory, arguments.mesh_directory)
