#!/usr/bin/env python3
"""导出训练好的 OBB 权重为部署格式 (ONNX / TensorRT)。

用法:
    python scripts/05_export_onnx.py --weights runs/obb/rebar_s/weights/best.pt
    python scripts/05_export_onnx.py --weights ... --format engine  # TensorRT
"""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", required=True)
    parser.add_argument("--format", default="onnx",
                        choices=["onnx", "engine"], help="onnx 或 engine(TensorRT)")
    parser.add_argument("--imgsz", type=int, default=1024)
    parser.add_argument("--half", action="store_true", help="FP16(建议 Jetson)")
    parser.add_argument("--static-batch", action="store_true",
                        help="默认导出动态 batch; 加此开关则固定 batch=1")
    args = parser.parse_args()

    model = YOLO(args.weights)
    out = model.export(
        format=args.format,
        imgsz=args.imgsz,
        half=args.half,
        dynamic=not args.static_batch,
        simplify=True,
        opset=13,
    )
    print(f"[export] 导出完成: {out}")
    print("[export] ROS2 侧建议: onnxruntime + 相机话题回调里推理, "
          "输出 Detection2DArray 或自定义 OBB 消息(含 4 角点+角度)。")


if __name__ == "__main__":
    main()
