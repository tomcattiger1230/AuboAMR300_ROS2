#!/usr/bin/env python3
"""YOLO-OBB 推理与可视化。

用法:
    # 单张/目录/视频
    python scripts/04_predict_obb.py --weights runs/obb/rebar_s/weights/best.pt \
        --source datasets/roi1555_yolo_obb/images/test --save-dir runs/predict

输出: 画好旋转框的结果图(含类别+角度), 以及每张图的 txt 结果
      (像素坐标 8 点 + 角度, 可直接用于机械臂抓取的视觉端)。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

REPO_ROOT = Path(__file__).resolve().parent.parent

# 三类试样统一配色 (BGR)
CLASS_COLORS = {
    0: (56, 168, 8),      # rebar 绿
    1: (32, 165, 218),    # concrete_cube 橙蓝
    2: (173, 76, 214),    # permeability_cone 紫
}


def draw_obb(img: np.ndarray, quad: np.ndarray, label: str, color) -> None:
    pts = quad.reshape(-1, 2).astype(np.int32)
    cv2.polylines(img, [pts], True, color, 2, cv2.LINE_AA)
    # 长边中点画朝向箭头(钢筋轴向), 抓取时直接用
    mid0 = (pts[0] + pts[1]) / 2
    mid2 = (pts[2] + pts[3]) / 2
    cv2.arrowedLine(img, tuple(mid0.astype(int)), tuple(mid2.astype(int)),
                    color, 2, cv2.LINE_AA)
    org = (int(pts[0][0]), max(12, int(pts[0][1]) - 6))
    cv2.putText(img, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
                cv2.LINE_AA)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", required=True, help="训练出的 .pt 权重")
    parser.add_argument("--source", required=True,
                        help="图片/目录/视频/gstreamer 源")
    parser.add_argument("--imgsz", type=int, default=1024)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--device", default="0")
    parser.add_argument("--save-dir", default=str(REPO_ROOT / "runs/predict"))
    args = parser.parse_args()

    save_dir = Path(args.save_dir)
    vis_dir = save_dir / "vis"
    txt_dir = save_dir / "obb_txt"
    vis_dir.mkdir(parents=True, exist_ok=True)
    txt_dir.mkdir(parents=True, exist_ok=True)

    model = YOLO(args.weights)
    results = model.predict(
        source=args.source, imgsz=args.imgsz, conf=args.conf,
        device=args.device, task="obb", verbose=False, stream=True,
    )

    n_frames = 0
    for res in results:
        n_frames += 1
        img = res.orig_img.copy()
        h, w = img.shape[:2]
        names = res.names
        detections = []
        if res.obb is not None:
            quads = res.obb.xyxyxyxy.cpu().numpy()          # (N,4,2) 像素
            confs = res.obb.conf.cpu().numpy()
            clss = res.obb.cls.cpu().numpy().astype(int)
            for quad, cf, cl in zip(quads, confs, clss):
                pts = quad.reshape(-1)
                dx01 = np.hypot(*(quad[1] - quad[0]))
                dx12 = np.hypot(*(quad[2] - quad[1]))
                ang = float(np.degrees(np.arctan2(
                    quad[1][1] - quad[0][1], quad[1][0] - quad[0][0])) % 180.0)
                if dx12 > dx01:  # 统一报告长边方向
                    ang = float(np.degrees(np.arctan2(
                        quad[2][1] - quad[1][1],
                        quad[2][0] - quad[1][0])) % 180.0)
                color = CLASS_COLORS.get(int(cl), (29, 161, 242))
                draw_obb(img, quad, f"{names[cl]} {cf:.2f} {ang:.0f}deg", color)
                detections.append({
                    "class_id": int(cl), "class": names[cl],
                    "conf": round(float(cf), 4), "angle_deg": round(ang, 2),
                    "quad_px": np.round(pts, 1).tolist(),
                })

        stem = Path(res.path).stem if res.path else f"frame_{n_frames:06d}"
        cv2.imwrite(str(vis_dir / f"{stem}.jpg"), img)
        (txt_dir / f"{stem}.txt").write_text(
            "\n".join(
                f"{d['class_id']} " + " ".join(f"{v:.1f}" for v in d["quad_px"])
                for d in detections
            ) + "\n"
        )
        (txt_dir / f"{stem}.json").write_text(json.dumps(detections, ensure_ascii=False, indent=1))

    print(f"[predict] 共处理 {n_frames} 帧; 可视化 -> {vis_dir}; 结果 -> {txt_dir}")


if __name__ == "__main__":
    main()
