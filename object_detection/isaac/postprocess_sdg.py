#!/usr/bin/env python3
"""把 synth_specimens_sdg.py 的 BasicWriter 输出转成 YOLO-OBB 数据集。

输入(<out>/writer/, 由 Isaac Sim 生成):
    rgb_XXXX.png                          渲染图(RGBA)
    instance_segmentation_XXXX.png        实例分割(彩色化 RGBA)
    instance_segmentation_semantics_mapping_XXXX.json  颜色->类别
    instance_segmentation_mapping_XXXX.json            颜色->prim路径(备用)

输出(specimens_yolo_obb/):
    images/{train,val}/sdg_XXXX.png
    labels/{train,val}/sdg_XXXX.txt   YOLO-OBB: class x1 y1 ... x4 y4

用法:
    python isaac/postprocess_sdg.py [--sdg datasets/sdg_out] \
        [--out datasets/specimens_yolo_obb] [--val-ratio 0.1] [--min-px 60]
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))
from obb_utils import mask_to_obb  # noqa: E402

CLASS_ID = {"rebar": 0, "concrete_cube": 1, "permeability_cone": 2}
BACKGROUND = (0, 0, 0, 255)  # RGBA, 未标注像素


def parse_color(key: str) -> tuple[int, int, int, int]:
    return tuple(int(v) for v in re.findall(r"-?\d+", key)[:4])


def class_from_label(label) -> str | None:
    """优先语义类名; 退化用 prim 路径段名前缀匹配。"""
    if isinstance(label, dict):
        label = label.get("class", "")
    label = str(label)
    for name in CLASS_ID:
        if label == name or label.startswith(name + "_"):
            return name
    return None


def convert_frame(seg: np.ndarray, mapping: dict, w: int, h: int,
                  min_px: int) -> list[str]:
    lines = []
    rgba = cv2.cvtColor(seg, cv2.COLOR_BGRA2RGBA)
    for color_key, label in mapping.items():
        cls_name = class_from_label(label)
        if cls_name is None:
            continue
        rgba_color = np.array(parse_color(color_key), dtype=np.uint8)
        m = np.all(rgba == rgba_color, axis=-1)
        if m.sum() < min_px:  # 过滤边缘噪声/远小目标
            continue
        box8 = mask_to_obb(m)
        if box8 is None:
            continue
        xs = np.clip(box8[0::2] / w, 0.0, 1.0)
        ys = np.clip(box8[1::2] / h, 0.0, 1.0)
        coords = " ".join(f"{x:.6f} {y:.6f}" for x, y in zip(xs, ys))
        lines.append(f"{CLASS_ID[cls_name]} {coords}")
    return lines


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdg", default="datasets/sdg_out")
    parser.add_argument("--out", default="datasets/specimens_yolo_obb")
    parser.add_argument("--val-ratio", type=float, default=0.1)
    parser.add_argument("--min-px", type=int, default=60)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    writer_dir = (REPO_ROOT / args.sdg if not Path(args.sdg).is_absolute()
                  else Path(args.sdg)) / "writer"
    out_dir = (REPO_ROOT / args.out if not Path(args.out).is_absolute()
               else Path(args.out))
    rgbs = sorted(writer_dir.glob("rgb_*.png"))
    if not rgbs:
        sys.exit(f"[sdg2yolo] {writer_dir} 下没有 rgb_*.png, 先跑合成脚本")

    for split in ("train", "val"):
        (out_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (out_dir / "labels" / split).mkdir(parents=True, exist_ok=True)

    rng = np.random.default_rng(args.seed)
    n_img = n_box = 0
    class_counts = {k: 0 for k in CLASS_ID}
    for rgb_path in rgbs:
        stem = rgb_path.stem.replace("rgb_", "")
        seg_path = writer_dir / f"instance_segmentation_{stem}.png"
        sem_path = (writer_dir /
                    f"instance_segmentation_semantics_mapping_{stem}.json")
        prim_path = writer_dir / f"instance_segmentation_mapping_{stem}.json"
        if not (seg_path.exists() and sem_path.exists()):
            continue
        seg = cv2.imread(str(seg_path), cv2.IMREAD_UNCHANGED)
        rgb = cv2.imread(str(rgb_path), cv2.IMREAD_UNCHANGED)
        if seg is None or rgb is None:
            continue
        h, w = seg.shape[:2]
        mapping = json.loads(sem_path.read_text())
        # 语义映射缺失类别时, 合并 prim 路径映射
        if not any(class_from_label(v) for v in mapping.values()) and prim_path.exists():
            mapping = json.loads(prim_path.read_text())

        lines = convert_frame(seg, mapping, w, h, args.min_px)
        if not lines:
            continue
        split = "val" if rng.random() < args.val_ratio else "train"
        name = f"sdg_{stem}"
        shutil.copy2(rgb_path, out_dir / "images" / split / f"{name}.png")
        (out_dir / "labels" / split / f"{name}.txt").write_text(
            "\n".join(lines) + "\n")
        n_img += 1
        n_box += len(lines)
        for ln in lines:
            class_counts[list(CLASS_ID)[int(ln[0])]] += 1

    print(f"[sdg2yolo] 图片 {n_img} 张, 框 {n_box} 个, "
          f"类别分布 {class_counts} -> {out_dir}")
    print("[sdg2yolo] 下一步: python scripts/03_train_obb.py --data specimens")


if __name__ == "__main__":
    main()
