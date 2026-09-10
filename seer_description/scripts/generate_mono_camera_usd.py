#!/usr/bin/env python3
"""Generate an independent camera override layer using a Python with pxr."""
from pathlib import Path
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


def generate(directory):
    directory=Path(directory)
    source=Usd.Stage.Open(str(directory/'seer_aubo_stick.usda'))
    output=directory/'seer_aubo_stick_mono.usda'
    stage=Usd.Stage.CreateNew(str(output))
    stage.GetRootLayer().subLayerPaths=['./seer_aubo_stick.usda']
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
    stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
    root='/World/seer_aubo_composite'
    # Disable the complete old camera articulation, including imported joints.
    for prim in source.Traverse():
        if prim.GetName().startswith('camera_'):
            stage.OverridePrim(prim.GetPath()).SetActive(False)
    path=root+'/mono_camera_link'
    prim=UsdGeom.Xform.Define(stage,path).GetPrim()
    mount=Gf.Matrix4d(1);mount.SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1),180));mount.SetTranslateOnly(Gf.Vec3d(0,.1,0))
    world=mount*UsdGeom.Xformable(source.GetPrimAtPath(root+'/gripper_motor_link')).ComputeLocalToWorldTransform(0)
    xf=UsdGeom.Xformable(prim);xf.AddTranslateOp().Set(world.ExtractTranslation());xf.AddOrientOp().Set(Gf.Quatf(world.ExtractRotationQuat()))
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass=UsdPhysics.MassAPI.Apply(prim);mass.CreateMassAttr(.193);mass.CreateCenterOfMassAttr(Gf.Vec3f(0,0,-.011));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(.00014,.00013,.00004))
    body=UsdGeom.Cube.Define(stage,path+'/body');body.CreateSizeAttr(1.);body.AddTranslateOp().Set(Gf.Vec3d(0,0,-.0295));body.AddScaleOp().Set(Gf.Vec3f(.029,.044,.059));body.CreateDisplayColorAttr([Gf.Vec3f(.16,.18,.21)]);UsdPhysics.CollisionAPI.Apply(body.GetPrim())
    lens=UsdGeom.Cylinder.Define(stage,path+'/lens_12mm');lens.CreateRadiusAttr(.0175);lens.CreateHeightAttr(.04);lens.CreateAxisAttr('Z');lens.AddTranslateOp().Set(Gf.Vec3d(0,0,.02));lens.CreateDisplayColorAttr([Gf.Vec3f(.035,.035,.035)]);UsdPhysics.CollisionAPI.Apply(lens.GetPrim())
    optical=UsdGeom.Xform.Define(stage,path+'/camera_optical_frame');optical.AddTranslateOp().Set(Gf.Vec3d(0,0,.04))
    camera=UsdGeom.Camera.Define(stage,path+'/camera_optical_frame/ros2_camera')
    UsdGeom.XformCommonAPI(camera).SetRotate((180.,0.,0.))
    camera.CreateFocalLengthAttr(12.);camera.CreateHorizontalApertureAttr(4096*.00345);camera.CreateVerticalApertureAttr(2460*.00345)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.01,100.))
    camera.GetPrim().CreateAttribute('sensor:model',Sdf.ValueTypeNames.String).Set('MV-CH100-60UM')
    camera.GetPrim().CreateAttribute('sensor:nativeResolution',Sdf.ValueTypeNames.Int2).Set(Gf.Vec2i(4096,2460))
    camera.GetPrim().CreateAttribute('sensor:pixelFormat',Sdf.ValueTypeNames.String).Set('mono8')
    joint=UsdPhysics.FixedJoint.Define(stage,root+'/joints/mono_camera_joint');joint.CreateBody0Rel().SetTargets([root+'/gripper_motor_link']);joint.CreateBody1Rel().SetTargets([path]);joint.CreateLocalPos0Attr(Gf.Vec3f(0,.1,0));joint.CreateLocalRot0Attr(Gf.Quatf(0,0,0,1));joint.CreateLocalPos1Attr(Gf.Vec3f(0));joint.CreateLocalRot1Attr(Gf.Quatf(1));joint.CreateCollisionEnabledAttr(False)
    stage.GetRootLayer().Save()
    warehouse=directory/'warehouse_stick_mono_demo.usda'
    # Reuse the scene, then apply the new robot as the stronger sublayer.
    layer=Sdf.Layer.CreateNew(str(warehouse));layer.subLayerPaths=['./seer_aubo_stick_mono.usda','./warehouse_stick_demo.usda'];layer.Save()
    stage=Usd.Stage.Open(str(warehouse));UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.);stage.SetDefaultPrim(stage.GetPrimAtPath('/World'));stage.GetRootLayer().Save()
    for file in (output, warehouse):
        file.write_text(file.read_text().rstrip()+"\n")


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('urdf_directory');generate(parser.parse_args().urdf_directory)
