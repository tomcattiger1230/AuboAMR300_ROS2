#!/usr/bin/env python3
"""CementBlocks 标注清洗: 混合 detect(AABB 5值)/segment(多边形) -> 统一 YOLO-OBB (9值/行)。

Roboflow 导出的 CementBlocks 标签三种行混在同一文件:
  - 5 值:  cls x1 y1 x2 y2 (AABB)          -> 直接展开成轴对齐 OBB
  - 9 值:  cls + 4 点 (可能是 OBB 也可能是 4 点多边形, 等价) -> 保留
  - >9 值: cls + N 点 多边形               -> cv2.minAreaRect 拟合最小外接旋转框

输出: datasets/CementBlocks_obb/{train,valid,test}/{images,labels}
images 用符号链接, 不复制数据。
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

SRC = Path(__file__).resolve().parent.parent / "datasets" / "CementBlocks"
DST = SRC.parent / "CementBlocks_obb"


def row_to_obb(parts: list[str], img_w: int, img_h: int) -> list[float] | None:
    cls = parts[0]
    vals = np.array(parts[1:], dtype=np.float64).reshape(-1, 2)  # 归一化 xy 对
    pts = np.stack([vals[:, 0] * img_w, vals[:, 1] * img_h], axis=1).astype(np.float32)

    if len(vals) == 2:  # AABB: 两个对角点 -> 4 角点
        (x1, y1), (x2, y2) = pts
        quad = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float32)
    else:  # 多边形/已是 4 点 -> 最小外接旋转矩形(对 4 点 OBB 也是恒等)
        (cx, cy), (w, h), ang = cv2.minAreaRect(pts)
        box = cv2.boxPoints(((cx, cy), (w, h), ang))
        quad = box.astype(np.float32)

    if (quad[:, 0].max() - quad[:, 0].min()) < 2 or (quad[:, 1].max() - quad[:, 1].min()) < 2:  # 太小, 丢弃
        return None
    xy = np.stack([quad[:, 0] / img_w, quad[:, 1] / img_h], axis=1).reshape(-1)
    return [float(cls)] + xy.tolist()


def main() -> None:
    stats = {"kept": 0, "dropped": 0, "imgs": 0}
    for split in ("train", "valid", "test"):
        (DST / split / "labels").mkdir(parents=True, exist_ok=True)
        (DST / split / "images").mkdir(parents=True, exist_ok=True)
        img_dir = SRC / split / "images"

        for img_path in sorted(img_dir.iterdir()):
            if img_path.suffix.lower() not in (".jpg", ".jpeg", ".png", ".webp"):
                continue
            label_path = SRC / split / "labels" / (img_path.stem + ".txt")
            img = cv2.imread(str(img_path))
            if img is None:
                print(f"[skip] 读不了图: {img_path.name}")
                continue
            h, w = img.shape[:2]

            lines_out = []
            if label_path.exists():
                for line in label_path.read_text().splitlines():
                    parts = line.split()
                    if len(parts) < 5 or len(parts) % 2 == 0:
                        continue  # 非法行
                    obb = row_to_obb(parts, w, h)
                    if obb:
                        lines_out.append(
                            f"{obb[0]} " + " ".join(f"{v:.8f}" for v in obb[1:])
                        )
                        stats["kept"] += 1
                    else:
                        stats["dropped"] += 1

            out_label = DST / split / "labels" / (img_path.stem + ".txt")
            out_label.write_text("\n".join(lines_out) + ("\n" if lines_out else ""))
            dst_img = DST / split / "images" / img_path.name
            if not dst_img.exists():
                dst_img.symlink_to(img_path.resolve())
            stats["imgs"] += 1

    print(f"[done] imgs={stats['imgs']} kept={stats['kept']} dropped={stats['dropped']}")
    print(f"[out]  {DST}")


if __name__ == "__main__":
    sys.exit(main())
