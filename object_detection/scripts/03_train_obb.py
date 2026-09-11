#!/usr/bin/env python3
"""YOLO11-OBB 训练入口。

用法示例(在本目录下, 先激活 .venv):
    # 用 ROI-1555 训钢筋侧面 OBB 检测
    python scripts/03_train_obb.py --data rebar --model yolo11s-obb.pt

    # 用合成+实拍三类试样数据训
    python scripts/03_train_obb.py --data specimens --model yolo11s-obb.pt

    # 断点续训
    python scripts/03_train_obb.py --resume runs/obb/rebar_s/weights/last.pt

模型规模: n/s/m/l/x 递增; s 在 1024 分辨率下是精度/速度的常用折中。
预训练权重(yolo11*-obb.pt, DOTA 预训练)首次运行自动下载,
国内网络不畅时设 HF_ENDPOINT 无效, 需设 ULTRALYTICS离线权重或手动放置。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from ultralytics import YOLO

REPO_ROOT = Path(__file__).resolve().parent.parent
RUNS_DIR = REPO_ROOT / "runs"

# 内置数据集选择: (yaml 模板, 数据目录, 任务名)
DATASETS = {
    "rebar": (
        REPO_ROOT / "configs/rebar_obb.yaml",
        REPO_ROOT / "datasets/roi1555_yolo_obb",
    ),
    "specimens": (
        REPO_ROOT / "configs/specimens_obb.yaml",
        REPO_ROOT / "datasets/specimens_yolo_obb",
    ),
    "cement": (
        REPO_ROOT / "configs/cementblocks_obb.yaml",
        REPO_ROOT / "datasets/CementBlocks_obb",  # 清洗后的统一 OBB 格式(06 脚本产出)
    ),
}


def load_data_spec(name_or_path: str) -> Path:
    """生成可直接传给 YOLO.train(data=...) 的 yaml 路径(path 为绝对路径)。"""
    import yaml

    if name_or_path in DATASETS:
        yaml_path, data_dir = DATASETS[name_or_path]
    else:
        yaml_path = Path(name_or_path)
        data_dir = None

    spec = yaml.safe_load(yaml_path.read_text())
    if data_dir is None:
        # 自定义 yaml: path 按相对 yaml 所在目录解析
        p = Path(spec["path"])
        data_dir = p if p.is_absolute() else (yaml_path.parent / p).resolve()
    if not data_dir.exists():
        sys.exit(
            f"[train] 数据目录不存在: {data_dir}\n"
            f"        先运行转换脚本生成数据, 或检查 --data 参数。"
        )
    spec["path"] = str(data_dir.resolve())
    out_yaml = data_dir / "dataset.yaml"  # ultralytics 只吃 yaml 路径
    out_yaml.write_text(yaml.safe_dump(spec, allow_unicode=True))
    return out_yaml


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", default="rebar",
                        help="rebar | specimens | 自定义 yaml 路径")
    parser.add_argument("--model", default="yolo11s-obb.pt",
                        help="初始权重 (yolo11n/s/m-obb.pt 或自训 .pt)")
    parser.add_argument("--epochs", type=int, default=150)
    parser.add_argument("--imgsz", type=int, default=1024,
                        help="训练分辨率; OBB/长条目标建议 >= 800")
    parser.add_argument("--batch", type=int, default=-1,
                        help="-1 为按显存自动(需要 batch>=2)")
    parser.add_argument("--device", default="0", help="如 0 / 0,1 / cpu")
    parser.add_argument("--name", default=None, help="run 名称, 默认按数据集+模型")
    parser.add_argument("--patience", type=int, default=30)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--resume", default=None,
                        help="从 last.pt 断点续训(指定路径)")
    args = parser.parse_args()

    if args.resume:
        model = YOLO(args.resume)
        data = None
        run_name = Path(args.resume).parts[-3]  # runs/obb/<name>/weights/last.pt
    else:
        data = load_data_spec(args.data)
        model = YOLO(args.model)
        run_name = args.name or (
            f"{args.data}_{Path(args.model).stem.replace('-obb', '')}_obb"
        )

    results = model.train(
        data=str(data) if data else data,
        task="obb",
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        patience=args.patience,
        workers=args.workers,
        project=str(RUNS_DIR),
        name=run_name,
        exist_ok=True,
        # 长条目标在原图中常较小, mosaic+多尺度对 OBB 帮助明显, 保持默认开启
        mosaic=1.0,
        degrees=180.0,      # 允许大角度旋转增广: 钢筋朝向任意
        flipud=0.5,
        fliplr=0.5,
        cache="disk",
    )
    best = Path(results.save_dir) / "weights/best.pt"
    print(f"[train] 最佳权重: {best}")
    print(f"[train] 下一步: python scripts/04_predict_obb.py --weights {best}")


if __name__ == "__main__":
    main()
