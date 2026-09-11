# object_detection — 建筑试样目标检测（钢筋 / 混凝土试块 / 抗渗试块）

为 Aubo AMR300 机械臂提供三类建筑试样的视觉识别，检测头采用 **YOLO11-OBB**
（旋转框，直接输出长条钢筋的朝向角，供抓取使用）：

| id | 类别 | 形态 | 数据来源 |
|---|---|---|---|
| 0 | `rebar` | 钢筋（整根/侧面，含直条与箍筋） | ROI-1555 公开数据集 + 合成 |
| 1 | `concrete_cube` | 混凝土抗压试块（100mm 立方体） | 合成 + 实拍 |
| 2 | `permeability_cone` | 抗渗试块（圆台 顶175/底185/高150mm） | 合成 + 实拍 |

## 目录

```
object_detection/
├── .venv/                  Python 3.12 专用环境(已建好, torch 2.14+cu130)
├── configs/                数据集 yaml(rebar_obb / specimens_obb)
├── scripts/
│   ├── 01_download_roi1555.py    下载 ROI-1555(镜像安全版)
│   ├── 02_convert_yolo_obb.py    LabelMe → YOLO-OBB 转换
│   ├── 03_train_obb.py           YOLO11-OBB 训练
│   ├── 04_predict_obb.py         推理+可视化(输出 4 角点+角度)
│   ├── 05_export_onnx.py         导出 ONNX / TensorRT
│   └── obb_utils.py              掩码/多边形 → 旋转框共用工具
├── isaac/
│   ├── synth_specimens_sdg.py    Isaac Sim 合成数据生成(Replicator)
│   └── postprocess_sdg.py        合成输出 → YOLO-OBB 数据集
├── datasets/               数据(不入库)
└── runs/                   训练产物(不入库)
```

## 快速开始

```bash
cd src/AuboAMR300_ROS2/object_detection
source .venv/bin/activate

# 1) 下载 ROI-1555(586MB, 走 hf-mirror; 断点可重跑)
python scripts/01_download_roi1555.py

# 2) 转 YOLO-OBB(直条=0 箍筋=1, 场景级切分 train/val/test)
python scripts/02_convert_yolo_obb.py

# 3) 训练钢筋模型(预训练权重首次自动下载)
python scripts/03_train_obb.py --data rebar --model yolo11s-obb.pt

# 4) 推理验证(输出 vis/ 可视化 + obb_txt/ 角点与角度)
python scripts/04_predict_obb.py \
    --weights runs/obb/rebar_yolo11s_obb/weights/best.pt \
    --source datasets/roi1555_yolo_obb/images/test
```

## Isaac Sim 合成数据（试块两类没有公开数据，用仿真铺量）

已在 Isaac Sim 6.0.1 实测调通（1280×960 约 0.5 秒/帧）：

```bash
# 生成(headless, 2000 帧 ~17 分钟); 调试可加 --gui
~/isaacsim/python.sh isaac/synth_specimens_sdg.py --frames 2000 --out datasets/sdg_out

# writer 输出转 YOLO-OBB 并切分 train/val
python isaac/postprocess_sdg.py

# 三类合并训练(合成 + 实拍补充后)
python scripts/03_train_obb.py --data specimens --model yolo26s-obb.pt
```

Isaac Sim 6.x 注意事项（详见脚本注释）：
- 数据以 BasicWriter 输出为准（`datasets/sdg_out/writer/`），手动 annotator 返回陈旧数据；
- 相机距物体必须 ≥1.5m：RTX 不遵守 `clippingRange`，近裁剪面卡在 ~1.25m，再近整帧黑；
- `focalLength` 单位是 mm（aperture 36mm 时 f=18/tan(hfov/2)），不能按像素公式算；
- USD 图元默认尺寸巨大（Cube=2m、Cylinder r=1/h=2），必须显式设置；
- 物体变换用显式 `AddTranslateOp/AddRotateXYZOp`，`XformCommonAPI.Set*` 在本机构建不生效。

实拍补充：把现场照片标成同样的 YOLO-OBB 格式放进
`datasets/specimens_yolo_obb/{images,labels}/train` 即可与合成数据合并训练
（推荐 X-AnyLabeling / CVAT 标注，先用合成训出的模型预标注可大幅省时）。

## 部署（ROS 2 侧）

```bash
python scripts/05_export_onnx.py --weights runs/obb/.../best.pt   # ONNX
# Jetson 上建议: --format engine --half  (TensorRT FP16)
```

ROS 2 节点用 onnxruntime 推理，输入相机话题，输出 4 角点+角度
（`04_predict_obb.py` 里的解析逻辑可直接复用）。

## 数据集与论文

- ROI-1555: Sun, Fan & Shao 2025, *Deep learning-based rebar detection and
  instance segmentation in images*, Advanced Engineering Informatics 65:103224.
  HuggingFace `tsrobcvai/ROI-1555_...`（CC 许可，LabelMe 格式）。
- 合成方案参考: Sun et al. 2025, *Rebar grasp detection using a synthetic
  model generator and domain randomization*, Automation in Construction。
