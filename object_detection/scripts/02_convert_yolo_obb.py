#!/usr/bin/env python3
"""ROI-1555 (LabelMe) -> YOLO-OBB 数据集转换。

输入(datasets/roi1555_raw): 各场景目录下 img_label/{stem}.jpg + {stem}.json,
    LabelMe 格式, 多边形标注, label 形如 "straight-3" / "hoop-12"
    (前缀为类别, 后缀为实例编号)。

输出(datasets/roi1555_yolo_obb):
    images/{train,val,test}/{scene}_{stem}.jpg
    labels/{train,val,test}/{scene}_{stem}.txt
    YOLO-OBB 行: class x1 y1 x2 y2 x3 y3 x4 y4 (0~1 归一化, 顺时针四角)

类别映射(默认): straight->0(直条钢筋), hoop->1(箍筋)。
    --single-rebar: 两者合并为 0=rebar(与三类试样模型一致)。

切分: 默认按场景目录整目录切分(避免近邻帧同时进训练/验证导致虚高),
    80/10/10; 场景数不足时用 --split-mode random 按图片随机切。

用法:
    python scripts/02_convert_yolo_obb.py [--out datasets/roi1555_yolo_obb]
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))
from obb_utils import polygon_to_obb  # noqa: E402

LABEL_PREFIXES = {"straight": 0, "hoop": 1}
MIN_AREA_PX = 30.0  # 过滤噪声多边形


def parse_shape(shape: dict, w: int, h: int) -> np.ndarray | None:
    """单个 LabelMe shape -> 像素 8 坐标旋转框, 无效返回 None。"""
    pts = np.asarray(shape["points"], dtype=np.float32)
    st = shape.get("shape_type", "polygon")
    if st == "rectangle" and len(pts) == 2:
        (x0, y0), (x1, y1) = pts
        pts = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]],
                       dtype=np.float32)
    if len(pts) < 3 or not (pts[:, 0] >= 0).all():
        return None
    box8 = polygon_to_obb(pts)
    xs, ys = box8[0::2], box8[1::2]
    if cv2.contourArea(box8.reshape(-1, 2).astype(np.float32)) < MIN_AREA_PX:
        return None
    if xs.max() < 0 or ys.max() < 0 or xs.min() > w or ys.min() > h:
        return None  # 完全出界
    return box8


def convert_one(json_path: Path, single_rebar: bool):
    data = json.loads(json_path.read_text())
    w, h = data["imageWidth"], data["imageHeight"]
    img_path = json_path.parent / data.get("imagePath", json_path.stem + ".jpg")
    if not img_path.exists():
        for ext in (".jpg", ".jpeg", ".png", ".JPG"):
            cand = json_path.parent / (json_path.stem + ext)
            if cand.exists():
                img_path = cand
                break
        else:
            return None, f"缺图: {json_path.name}"

    lines, unknown = [], []
    for shape in data["shapes"]:
        prefix = str(shape["label"]).split("-")[0].split("_")[0].lower()
        if prefix not in LABEL_PREFIXES:
            unknown.append(shape["label"])
            continue
        cls = LABEL_PREFIXES[prefix]
        if single_rebar:
            cls = 0
        box8 = parse_shape(shape, w, h)
        if box8 is None:
            continue
        xs = np.clip(box8[0::2] / w, 0.0, 1.0)
        ys = np.clip(box8[1::2] / h, 0.0, 1.0)
        coords = " ".join(f"{x:.6f} {y:.6f}" for x, y in zip(xs, ys))
        lines.append(f"{cls} {coords}")
    return (img_path, lines), unknown


def split_scenes(scene_counts: dict[str, int], ratios=(0.8, 0.1, 0.1)):
    """按场景整目录贪心凑比例; 场景数>=3 时保证 val/test 各至少一幕。"""
    order = ("train", "val", "test")
    total = sum(scene_counts.values())
    quotas = dict(zip(order, (r * total for r in ratios)))
    filled = dict.fromkeys(order, 0)
    assigned = {k: [] for k in order}
    for scene, n in sorted(scene_counts.items(), key=lambda kv: -kv[1]):
        k = min(order, key=lambda s: filled[s] - quotas[s])  # 欠额最大的桶
        assigned[k].append(scene)
        filled[k] += n
    for need in ("val", "test"):  # 修正空桶: 挪 train 里最小的场景
        if not assigned[need] and assigned["train"]:
            smallest = min(assigned["train"], key=lambda s: scene_counts[s])
            assigned["train"].remove(smallest)
            assigned[need].append(smallest)
    return assigned


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", default="datasets/roi1555_raw")
    parser.add_argument("--out", default="datasets/roi1555_yolo_obb")
    parser.add_argument("--single-rebar", action="store_true",
                        help="straight/hoop 合并为一个 rebar 类")
    parser.add_argument("--split-mode", choices=["scene", "random"],
                        default="scene")
    parser.add_argument("--val-ratio", type=float, default=0.1)
    parser.add_argument("--test-ratio", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    raw_dir = REPO_ROOT / args.raw if not Path(args.raw).is_absolute() else Path(args.raw)
    out_dir = REPO_ROOT / args.out if not Path(args.out).is_absolute() else Path(args.out)
    if not raw_dir.exists():
        sys.exit(f"[convert] 数据目录不存在: {raw_dir}")

    # 数据集两种布局: 1260/img_label/*.json 与 scen*/test2017/*.json
    containers = {"img_label", "test2017", "train2017", "val2017"}
    jsons = [p for p in sorted(raw_dir.rglob("*.json"))
             if p.parent.name in containers]
    if not jsons:
        sys.exit("[convert] 没找到标注 json, 检查 01 下载是否完成")
    print(f"[convert] 发现 {len(jsons)} 个 LabelMe 标注")

    # 场景 = 标注容器目录的上级目录名(1260, scen1..3)
    scenes = sorted({p.parent.parent.name for p in jsons})
    counts = Counter(p.parent.parent.name for p in jsons)
    print(f"[convert] 场景: {dict(counts)}")

    rng = np.random.default_rng(args.seed)
    if args.split_mode == "scene" and len(scenes) >= 3:
        assigned = split_scenes(counts)
        scene_split = {s: k for k, ss in assigned.items() for s in ss}
    else:
        scene_split = {}

    stats = Counter()
    unknown_labels = Counter()
    import shutil as _shutil
    for split in ("train", "val", "test"):  # 全量重建, 清掉旧产物
        _shutil.rmtree(out_dir / "images" / split, ignore_errors=True)
        _shutil.rmtree(out_dir / "labels" / split, ignore_errors=True)
        (out_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (out_dir / "labels" / split).mkdir(parents=True, exist_ok=True)

    for jp in jsons:
        scene = jp.parent.parent.name
        stem = f"{scene}_{jp.stem}"
        result, unknown = convert_one(jp, args.single_rebar)
        unknown_labels.update(unknown)
        if result is None:
            stats["missing_img"] += 1
            continue
        img_path, lines = result
        if args.split_mode == "scene" and scene_split:
            split = scene_split[scene]
        else:
            r = rng.random()
            split = ("train" if r < 1 - args.val_ratio - args.test_ratio
                     else "val" if r < 1 - args.test_ratio else "test")
        if not lines:
            stats[f"{split}_no_label"] += 1
            continue
        shutil.copy2(img_path,
                     out_dir / "images" / split / f"{stem}{img_path.suffix}")
        (out_dir / "labels" / split / f"{stem}.txt").write_text(
            "\n".join(lines) + "\n")
        stats[f"{split}_img"] += 1
        stats[f"{split}_box"] += len(lines)

    for k in ("train", "val", "test"):
        print(f"[convert] {k}: {stats[k + '_img']} 图 / {stats[k + '_box']} 框")
    if stats["missing_img"]:
        print(f"[convert] 跳过缺图 {stats['missing_img']} 个")
    if unknown_labels:
        print(f"[convert] 未知标签(已跳过): {dict(unknown_labels)}")
    print(f"[convert] 完成 -> {out_dir}")
    print("[convert] 下一步: python scripts/03_train_obb.py --data rebar")


if __name__ == "__main__":
    main()
