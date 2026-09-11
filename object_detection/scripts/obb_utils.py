#!/usr/bin/env python3
"""掩码/多边形 -> YOLO-OBB 标注的共用工具。

YOLO-OBB(Ultralytics DOTA 约定) 每行:
    class_index x1 y1 x2 y2 x3 y3 x4 y1
其中 8 个坐标为 0~1 归一化的四边形四角(顺时针/逆时针均可, 但需按
绕行顺序连续给出, 不是 minX,minY,maxX,maxY 那种成对坐标)。
"""

from __future__ import annotations

import cv2
import numpy as np


def polygon_to_obb(poly: np.ndarray) -> np.ndarray:
    """任意四点/多点多边形 -> 最小外接旋转四边形的 4 个角点 (8,).

    poly: (N,2) float, 像素坐标, N>=4 或为二值掩码轮廓点集。
    返回: (8,) [x1,y1,x2,y2,x3,y3,x4,y4] 像素坐标, 顺时针绕行。
    """
    rect = cv2.minAreaRect(poly.astype(np.float32))
    box = cv2.boxPoints(rect)  # (4,2) 顺时针
    return box.reshape(-1)


def mask_to_obb(mask: np.ndarray) -> np.ndarray | None:
    """二值掩码 -> 最小外接旋转四边形 8 坐标 (像素)。空掩码返回 None。"""
    contours, _ = cv2.findContours(
        mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < 4:  # 噪声
        return None
    return polygon_to_obb(largest.reshape(-1, 2))


def format_yolo_obb_line(class_id: int, box8: np.ndarray, w: int, h: int) -> str:
    """像素 8 坐标 -> 归一化 YOLO-OBB 标注行。"""
    xs = box8[0::2] / w
    ys = box8[1::2] / h
    xs = np.clip(xs, 0.0, 1.0)
    ys = np.clip(ys, 0.0, 1.0)
    vals = []
    for x, y in zip(xs, ys):
        vals.extend((x, y))
    coords = " ".join(f"{v:.6f}" for v in vals)
    return f"{class_id} {coords}"


def obb_angle_deg(box8: np.ndarray) -> float:
    """旋转框长边方向角(度, 0~180), 用于抓取朝向与可视化。"""
    pts = box8.reshape(-1, 2)
    edge_lengths = [
        np.linalg.norm(pts[(i + 1) % 4] - pts[i]) for i in range(4)
    ]
    # 长边: 边0-1 与 边2-3 是一组, 边1-2 与 边3-0 是另一组
    if edge_lengths[0] + edge_lengths[2] >= edge_lengths[1] + edge_lengths[3]:
        v = pts[1] - pts[0]
    else:
        v = pts[2] - pts[1]
    ang = np.degrees(np.arctan2(v[1], v[0])) % 180.0
    return float(ang)
