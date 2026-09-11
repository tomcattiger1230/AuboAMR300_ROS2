#!/usr/bin/env python3
"""Isaac Sim Replicator 合成数据生成: 建筑钢筋 / 混凝土立方体试块 / 抗渗试块。

生成三类试样散放场景, 域随机化(位姿/材质/光照/相机), 每帧输出:
    rgb/    N.png        渲染图
    masks/  N.npy        实例 id 图(uint32, 与 meta 的 idToLabels 对应)
    meta/   N.json       实例id->语义类, 视场角, 画面尺寸

之后用 isaac/postprocess_sdg.py 把输出转成 YOLO-OBB 数据集。

运行(Isaac Sim 6.x 自带 Python 3.12):
    ~/isaacsim/python.sh isaac/synth_specimens_sdg.py --frames 2000 \
        --out datasets/sdg_out
    调试看画面: 加 --gui

类别 id 与 configs/specimens_obb.yaml 一致:
    0 rebar             钢筋(直径 12~32mm, 长 0.5~1.2m 圆柱)
    1 concrete_cube     立方体试块(边长 100mm)
    2 permeability_cone 抗渗试块(圆台 顶175/底185/高150mm)
"""

from __future__ import annotations

import argparse
import json
import math
import os

import numpy as np

# SimulationApp 必须在所有 omni/isaacsim/pxr 导入之前创建
ARGS = None


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--frames", type=int, default=2000)
    p.add_argument("--out", default="datasets/sdg_out")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=960)
    p.add_argument("--gui", action="store_true", help="带窗口运行(调试用)")
    p.add_argument("--seed", type=int, default=0)
    return p.parse_args()


ARGS = parse_args()
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": not ARGS.gui})

# --- 以下导入必须在 SimulationApp 之后 ---
import omni.replicator.core as rep  # noqa: E402
from pxr import Gf, Sdf, Semantics, Tf, Usd, UsdGeom, UsdLux, UsdShade  # noqa: E402
from pxr import UsdSemantics  # noqa: E402


def add_semantics(prim: Usd.Prim, label: str) -> None:
    """Isaac Sim 6.x 已移除 add_update_semantics, 按官方内部实现写语义。

    优先新 API UsdSemantics.LabelsAPI, 失败则退回 legacy
    pxr.Semantics.SemanticsAPI(与 omni.replicator.core 的 modify.py 相同)。
    """
    try:
        sem = UsdSemantics.LabelsAPI.Apply(prim, "class")
        sem.CreateLabelsAttr()
        sem.GetLabelsAttr().Set([label])
        return
    except AttributeError:
        pass
    sem = Semantics.SemanticsAPI.Apply(
        prim, Tf.MakeValidIdentifier(f"class_{label}"))
    sem.CreateSemanticTypeAttr()
    sem.CreateSemanticDataAttr()
    sem.GetSemanticTypeAttr().Set("class")
    sem.GetSemanticDataAttr().Set(label)

CLASS_NAMES = ["rebar", "concrete_cube", "permeability_cone"]
# 各类物体池大小(每帧随机取 1~N 个可见)
POOL = {"rebar": 6, "concrete_cube": 4, "permeability_cone": 3}
SPREAD_RADIUS = 0.45          # 物体散布半径(m)
MIN_GAP = 0.09                # 物体中心最小间距(m)
RNG = np.random.default_rng(ARGS.seed)


# ---------------------------------------------------------------- 几何
def make_truncated_cone(stage, path, r_top, r_bottom, height, segments=48):
    """圆台(抗渗试块)没有原生 USD 图元, 手工构网格。"""
    n = segments
    pts = []
    for r, z in ((r_top, height / 2), (r_bottom, -height / 2)):
        for i in range(n):
            a = 2 * math.pi * i / n
            pts.append([r * math.cos(a), r * math.sin(a), z])
    top_c, bot_c = len(pts), len(pts) + 1
    pts.append([0.0, 0.0, height / 2])
    pts.append([0.0, 0.0, -height / 2])

    counts, indices = [], []
    for i in range(n):  # 侧面
        j = (i + 1) % n
        counts += [3, 3]
        indices += [i, j, n + j, j, n + i, n + j]
    for i in range(n):  # 上/下盖
        j = (i + 1) % n
        counts += [3, 3]
        indices += [i, j, top_c, n + j, n + i, bot_c]

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateDoubleSidedAttr(True)
    return mesh.GetPrim()


# ---------------------------------------------------------------- 材质
def make_material(stage, path):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.5))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    mat.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface")
    return mat, shader


def randomize_material(shader, kind):
    if kind == "rebar":  # 钢筋: 金属感 + 生锈色调扰动
        rust = RNG.uniform(0.0, 0.6)
        color = Gf.Vec3f(0.30 + 0.25 * (1 - rust),
                         0.26 + 0.10 * (1 - rust), 0.22)
        metallic, rough = 0.85, float(RNG.uniform(0.35, 0.7))
    else:  # 混凝土: 灰白~深灰
        g = float(RNG.uniform(0.35, 0.7))
        color = Gf.Vec3f(g * float(RNG.uniform(0.95, 1.05)), g,
                         g * float(RNG.uniform(0.9, 1.0)))
        metallic, rough = 0.0, float(RNG.uniform(0.8, 1.0))
    shader.GetInput("diffuseColor").Set(color)
    shader.GetInput("metallic").Set(metallic)
    shader.GetInput("roughness").Set(rough)


# ---------------------------------------------------------------- 变换
def set_xform(obj: dict, pos, euler_deg) -> None:
    """写显式 xformOp:translate + xformOp:rotateXYZ。

    不用 XformCommonAPI: 本机 USD 构建实测其 SetTranslate/SetRotate
    不落进 xformOpOrder, 世界矩阵保持单位阵。显式 Add*Op 则必有保证。
    """
    obj["t_op"].Set(Gf.Vec3d(*map(float, pos)))
    obj["r_op"].Set(Gf.Vec3f(*map(float, euler_deg)))


def look_at(eye, target, up=(0, 0, 1)):
    """相机/平行光的世界变换: 本体 -Z 指向 target。

    SetLookAt 返回的是 view 矩阵(世界->相机), prim 变换需要其逆。
    """
    view = Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(*map(float, eye)),
        Gf.Vec3d(*map(float, target)),
        Gf.Vec3d(*map(float, up)),
    )
    return view.GetInverse()


# ---------------------------------------------------------------- 场景
def build_scene(stage):
    UsdGeom.SetStageUpAxis(stage, "Z")
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    ground = UsdGeom.Mesh.Define(stage, "/World/Ground")
    ground.CreatePointsAttr([[-5, -5, 0], [5, -5, 0], [5, 5, 0], [-5, 5, 0]])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    gmat, gshader = make_material(stage, "/World/Mats/Ground")
    randomize_material(gshader, "concrete")
    UsdShade.MaterialBindingAPI(ground.GetPrim()).Bind(gmat)

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    dome_xf = UsdGeom.Xformable(dome.GetPrim()).AddTransformOp()
    sun_xf = UsdGeom.Xformable(sun.GetPrim()).AddTransformOp()

    objects = []  # dict(prim, kind, half_z, shader)
    for kind in CLASS_NAMES:
        for i in range(POOL[kind]):
            path = f"/World/Objects/{kind}_{i}"
            if kind == "rebar":
                geom = UsdGeom.Cylinder.Define(stage, path)
                # USD 默认 radius=1/height=2, 必须显式设置真实尺寸
                geom.CreateRadiusAttr(0.012)
                geom.CreateHeightAttr(1.0)
                half_z = 0.5
            elif kind == "concrete_cube":
                geom = UsdGeom.Cube.Define(stage, path)
                geom.CreateSizeAttr(0.1)  # USD 默认 size=2, 即 2 米巨块!
                half_z = 0.05
            else:
                geom = UsdGeom.Mesh(make_truncated_cone(
                    stage, path, 0.0875, 0.0925, 0.15))
                half_z = 0.075
            prim = geom.GetPrim()
            add_semantics(prim, kind)
            xf = UsdGeom.Xformable(prim)
            t_op = xf.AddTranslateOp()
            r_op = xf.AddRotateXYZOp()
            mat, shader = make_material(stage, f"/World/Mats/{kind}_{i}")
            UsdShade.MaterialBindingAPI(prim).Bind(mat)
            objects.append({"prim": prim, "kind": kind, "half_z": half_z,
                            "shader": shader, "t_op": t_op, "r_op": r_op})

    cam = UsdGeom.Camera.Define(stage, "/World/Camera")
    cam.CreateHorizontalApertureAttr(36.0)  # mm, 35mm 全画幅约定
    # USD 默认近裁剪面是 1m, 本场景相机距物体 0.7~1.6m, 必须显式设置
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 50.0))
    cam_xf = UsdGeom.Xformable(cam.GetPrim()).AddTransformOp()
    return {"dome": dome, "dome_xf": dome_xf, "sun": sun, "sun_xf": sun_xf,
            "cam": cam, "cam_xf": cam_xf, "objects": objects}


# ---------------------------------------------------------------- 随机化
def sample_placement(taken):
    """在散布圆内取一个与已放物体保持间距的位置。"""
    for _ in range(50):
        xy = RNG.uniform(-SPREAD_RADIUS, SPREAD_RADIUS, 2)
        if all(np.hypot(*(xy - c)) > MIN_GAP for c in taken):
            taken.append(xy)
            return xy
    return xy  # 实在挤不下就允许重叠


def randomize_frame(scene):
    rng = RNG
    _randomize_objects(scene, rng)
    dome_sun_random(scene, rng)
    return _randomize_cam(scene, rng)


def _randomize_objects(scene, rng):
    for o in scene["objects"]:
        UsdGeom.Imageable(o["prim"]).MakeInvisible()
        # 每帧随机改变钢筋尺寸, 试块尺寸固定(标准件)
        if o["kind"] == "rebar":
            UsdGeom.Cylinder(o["prim"]).CreateRadiusAttr(
                float(rng.uniform(0.006, 0.016)))
            UsdGeom.Cylinder(o["prim"]).CreateHeightAttr(
                float(rng.uniform(0.5, 1.2)))

    taken = []
    for kind in CLASS_NAMES:
        pool = [o for o in scene["objects"] if o["kind"] == kind]
        n_vis = int(rng.integers(1, len(pool) + 1))
        for idx in rng.choice(len(pool), size=n_vis, replace=False):
            o = pool[int(idx)]
            prim = o["prim"]
            UsdGeom.Imageable(prim).MakeVisible()
            xy = sample_placement(taken)
            yaw = float(rng.uniform(0, 360))
            if o["kind"] == "rebar":  # 圆柱轴为 Z, 放倒=绕X转90°, 加小扰动
                rot = (90.0 + rng.uniform(-5, 5), rng.uniform(-5, 5), yaw)
                z = 0.02
            else:
                rot = (rng.uniform(-3, 3), rng.uniform(-3, 3), yaw)
                z = o["half_z"]
            set_xform(o, (xy[0], xy[1], z), rot)
            randomize_material(o["shader"], kind)


def dome_sun_random(scene, rng):
    # 光照: 半球光强度/色温 + 平行光方向/强度
    scene["dome"].CreateIntensityAttr(float(rng.uniform(200, 1200)))
    scene["dome"].CreateColorAttr(Gf.Vec3f(*rng.uniform(0.7, 1.3, 3)))
    scene["sun"].CreateIntensityAttr(float(rng.uniform(500, 4000)))
    az, el = rng.uniform(0, 360), rng.uniform(15, 80)
    sun_dir = np.array([math.cos(math.radians(az)) * math.cos(math.radians(el)),
                        math.sin(math.radians(az)) * math.cos(math.radians(el)),
                        -math.sin(math.radians(el))])
    scene["sun_xf"].Set(look_at(sun_dir * 5, (0, 0, 0)))


def _randomize_cam(scene, rng):
    # 相机: 环绕 + 俯角 + 距离 + 视场角随机
    # 注意: Isaac Sim 6.0.1 RTX 不遵守 camera.clippingRange 属性, 近裁剪面
    # 实测卡在 ~1.25m, 相机距离必须 >= 1.5m, 否则整帧黑
    az, el = rng.uniform(0, 360), rng.uniform(25, 75)
    dist = rng.uniform(1.5, 2.5)
    eye = np.array([math.cos(math.radians(az)) * math.cos(math.radians(el)),
                    math.sin(math.radians(az)) * math.cos(math.radians(el)),
                    math.sin(math.radians(el))]) * dist
    target = np.array([rng.uniform(-0.1, 0.1), rng.uniform(-0.1, 0.1), 0.1])
    scene["cam_xf"].Set(look_at(eye, target))
    hfov = float(rng.uniform(35, 70))
    # focalLength 与 aperture 同为 mm; 光圈 36mm 时 f = 18 / tan(hfov/2)
    scene["cam"].CreateFocalLengthAttr(
        18.0 / math.tan(math.radians(hfov / 2)))
    return hfov


# ---------------------------------------------------------------- 主流程
async def frame_loop(scene):
    """逐帧随机化 -> 渲染; 数据由 BasicWriter 落盘到 <out>/writer/。"""
    for i in range(ARGS.frames):
        try:
            randomize_frame(scene)
            await rep.orchestrator.step_async(rt_subframes=32)
        except Exception as e:  # noqa: BLE001 单帧失败不终止整个生成
            print(f"[sdg] 第 {i} 帧失败: {e}", flush=True)
            continue

        if (i + 1) % 50 == 0:
            print(f"[sdg] {i + 1}/{ARGS.frames}", flush=True)


def main():
    import asyncio

    import omni.usd
    stage = omni.usd.get_context().get_stage()

    out_root = os.path.abspath(ARGS.out)
    os.makedirs(out_root, exist_ok=True)
    import shutil
    shutil.rmtree(os.path.join(out_root, "writer"), ignore_errors=True)

    scene = build_scene(stage)
    rp = rep.create.render_product("/World/Camera", (ARGS.width, ARGS.height))
    rep.orchestrator.set_capture_on_play(False)
    # 官方 6.x 模式: BasicWriter+DiskBackend 驱动采集; 手动 annotator 在
    # 本版本返回陈旧/低质数据, 一律以 writer 输出为准
    backend = rep.backends.get("DiskBackend")
    backend.initialize(output_dir=os.path.join(out_root, "writer"))
    writer = rep.writers.get("BasicWriter")
    writer.initialize(backend=backend, rgb=True, instance_segmentation=True)
    writer.attach([rp])

    task = asyncio.ensure_future(frame_loop(scene))
    while not task.done() and not simulation_app.is_exiting():
        simulation_app.update()
    writer.detach()
    simulation_app.close()


if __name__ == "__main__":
    main()
