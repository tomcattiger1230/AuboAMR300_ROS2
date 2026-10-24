"""Independent free-base variant with the repository's physical onboard rack.

This variant never changes the parked baseline or attaches the steel. The rack
is a fixed robot link so its own raw load can represent a tray load cell.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path

from .assets import audit_assets
from .spec import repository_root


def build_mobile_assets(parked, output, repo_root=None):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    parked, output = Path(parked).resolve(), Path(output).resolve()
    root = Path(repo_root or repository_root()).resolve()
    audit_assets(parked)
    if output.exists() and any(output.iterdir()):
        raise FileExistsError('Use a new mobile asset directory')
    output.mkdir(parents=True, exist_ok=True)
    rack_source = root/'seer_description/urdf/rebar_onboard_rack.usda'
    for name, prim in (('robot','Robot'),('station','Station'),('rebar','Rebar'),
                       ('machine','Machine'),('fixture','Fixture')):
        stage = Usd.Stage.CreateNew(str(output/(name+'.usda')))
        node = stage.DefinePrim('/'+prim, 'Xform');stage.SetDefaultPrim(node)
        UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,'Z')
        node.GetReferences().AddReference(os.path.relpath(parked/(name+'.usda'),output),'/'+prim)
        if name == 'robot':
            stage.OverridePrim('/Robot/training_base_lock').SetActive(False)
            # A root API on a free robot's container lets PhysX choose a
            # central link (foreArm here). Pin the articulation root identity
            # to the chassis rigid body without fixing that body to world.
            node.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath('/Robot/base_footprint'))
            rack = stage.DefinePrim('/Robot/tray_link','Xform')
            rack.GetReferences().AddReference(os.path.relpath(rack_source,output),
                                              '/World/seer_aubo_composite/base_link/RebarRack')
            UsdPhysics.RigidBodyAPI.Apply(rack)
            UsdPhysics.MassAPI.Apply(rack).CreateMassAttr(8.)
            joint = UsdPhysics.FixedJoint.Define(stage,'/Robot/tray_mount_joint')
            joint.CreateBody0Rel().SetTargets(['/Robot/base_link'])
            joint.CreateBody1Rel().SetTargets([rack.GetPath()])
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0));joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0))
            # The unpowered casters slide in the source mobile model. Finite
            # wheel torque supplies all translation, rotation and braking.
            for label, friction, combine, names in (
                ('Caster',0.,'min',('front_caster','rear_caster')),
                ('Drive',1.,'max',('left_wheel_link','right_wheel_link'))):
                mat = UsdShade.Material.Define(stage,'/Robot/Mobile'+label+'Material')
                api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
                api.CreateStaticFrictionAttr(friction);api.CreateDynamicFrictionAttr(friction)
                mat.GetPrim().AddAppliedSchema('PhysxMaterialAPI')
                mat.GetPrim().CreateAttribute('physxMaterial:frictionCombineMode',Sdf.ValueTypeNames.Token).Set(combine)
                for body_name in names:
                    body=stage.GetPrimAtPath('/Robot/'+body_name)
                    for collider in Usd.PrimRange(body):
                        if collider.HasAPI(UsdPhysics.CollisionAPI):
                            UsdShade.MaterialBindingAPI.Apply(collider).Bind(mat,materialPurpose='physics')
        stage.GetRootLayer().Save()
    manifest = json.loads((parked/'manifest.json').read_text())
    manifest.update(mobile_base=True, parked_assets=os.path.relpath(parked,output),
                    task_frame='world map plus measured base odometry; Z-up, metres',
                    rack_source=str(rack_source.relative_to(root)),
                    rack_sha256=hashlib.sha256(rack_source.read_bytes()).hexdigest(),
                    rack_mass_kg=8, rack_mass_calibrated=False,
                    tray_slot_center_base_m=[.2927,0.,.715],tray_axis_base=[0.,1.,0.],
                    artificial_rebar_support=False,gpu_validated=False)
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return audit_mobile_assets(output)


def audit_mobile_assets(directory):
    from pxr import Usd,UsdPhysics
    result=audit_assets(directory)
    stage=Usd.Stage.Open(str(Path(directory)/'robot.usda'))
    if stage.GetPrimAtPath('/Robot/training_base_lock').IsActive():
        raise RuntimeError('Mobile base is still locked to world')
    roots=[p.GetPath() for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if list(map(str,roots))!=['/Robot/base_footprint']:
        raise RuntimeError('Floating articulation root must be the chassis body')
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            a=prim.GetRelationship('physics:body0').GetTargets()
            b=prim.GetRelationship('physics:body1').GetTargets()
            if not a or not b:
                raise RuntimeError('Mobile robot contains a world attachment')
    tray=stage.GetPrimAtPath('/Robot/tray_link')
    colliders=[p for p in Usd.PrimRange(tray) if p.HasAPI(UsdPhysics.CollisionAPI)
               and p.GetAttribute('physics:collisionEnabled').Get() is not False]
    if not tray.HasAPI(UsdPhysics.RigidBodyAPI) or len(colliders)<4:
        raise RuntimeError('Physical tray link/colliders missing')
    return {**result,'base_world_attachment':False,'tray_contact_colliders':len(colliders),
            'tray_load_cell_link':True,'gpu_validated':False}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--parked',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();print(json.dumps(build_mobile_assets(args.parked,args.output),indent=2))


if __name__=='__main__':main()
