#!/usr/bin/env python3
"""Render ribbed steel with Isaac's RTX renderer, without robot motion.

Run through ~/isaacsim/python.sh. Outputs an RGB JPEG for material review;
this is a diagnostic render, not the monochrome wrist sensor's output.
"""
import argparse
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent /
                    'results/rebar_material_preview_20260916.jpg')
args = parser.parse_args()

from isaacsim import SimulationApp

app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting',
                     'width': 1280, 'height': 720})
try:
    import omni.usd
    import omni.replicator.core as rep
    from pxr import UsdGeom, UsdLux, UsdShade, Gf
    import numpy as np
    from PIL import Image

    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, 'Z')
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    asset = Path(__file__).resolve().parents[1] / 'urdf/rebar_visual.usda'
    for index in range(3):
        # Match the experiment's hidden cylinder and visible reference child.
        bar = UsdGeom.Cylinder.Define(stage, f'/World/Bar{index}')
        bar.CreateAxisAttr('X')
        bar.CreateRadiusAttr(.012)
        bar.CreateHeightAttr(.6)
        bar.AddTranslateOp().Set(Gf.Vec3d(0, (index-1)*.05, .015))
        visual = UsdGeom.Xform.Define(stage, f'/World/Bar{index}/SteelVisual')
        visual.GetPrim().GetReferences().AddReference(str(asset))
        visual.CreatePurposeAttr('default')
        visual.AddRotateYOp().Set(90)
        invisible = UsdShade.Material(stage.GetPrimAtPath(
            f'/World/Bar{index}/SteelVisual/ColliderInvisible'))
        UsdShade.MaterialBindingAPI.Apply(bar.GetPrim()).Bind(invisible)

    sky = UsdLux.DomeLight.Define(stage, '/World/Sky')
    sky.CreateIntensityAttr(300)
    key = UsdLux.SphereLight.Define(stage, '/World/Key')
    key.CreateIntensityAttr(15000)
    key.CreateRadiusAttr(.15)
    UsdGeom.Xformable(key).AddTranslateOp().Set(Gf.Vec3d(.1, -.3, .6))
    camera = UsdGeom.Camera.Define(stage, '/World/Camera')
    camera.CreateFocalLengthAttr(28)
    camera.CreateHorizontalApertureAttr(36)
    # USD's default near clipping plane is 1 scene unit: too far for this view.
    camera.CreateClippingRangeAttr(Gf.Vec2f(.001, 100))
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(.35, -.55, .35), Gf.Vec3d(0, 0, .015),
        Gf.Vec3d(0, 0, 1)).GetInverse())
    product = rep.create.render_product(str(camera.GetPath()), (1280, 720))
    annotator = rep.AnnotatorRegistry.get_annotator('rgb')
    annotator.attach([product])
    for _ in range(60):
        app.update()
    for _ in range(5):
        rep.orchestrator.step(rt_subframes=16)
    frame = np.array(annotator.get_data())[:, :, :3]
    assert frame.size and np.std(frame) > 2, 'Missing or blank material preview'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(frame).save(args.output, quality=94)
    print(f'Material preview: {args.output}, RGB {frame.shape}', flush=True)
except Exception:
    # Isaac's fast shutdown can mask an exception with exit status 0. Make
    # this standalone diagnostic fail explicitly; the OS releases its GPU.
    import os
    import sys
    import traceback
    traceback.print_exc()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(1)
app.close()
