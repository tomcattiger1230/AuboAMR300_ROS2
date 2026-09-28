from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Tuple


@dataclass
class SMap:
    raw: Dict[str, Any]
    path: str = ""
    name: str = ""
    min_x: float = -5.0
    min_y: float = -5.0
    max_x: float = 5.0
    max_y: float = 5.0
    normal_points: List[Tuple[float, float]] = field(default_factory=list)
    advanced_points: List[Dict[str, Any]] = field(default_factory=list)
    advanced_lines: List[Dict[str, Any]] = field(default_factory=list)
    advanced_curves: List[Dict[str, Any]] = field(default_factory=list)

    @property
    def width(self) -> float:
        return max(0.1, self.max_x - self.min_x)

    @property
    def height(self) -> float:
        return max(0.1, self.max_y - self.min_y)


def _get_xy(obj: Dict[str, Any]) -> Tuple[float, float] | None:
    try:
        return float(obj.get("x")), float(obj.get("y"))
    except Exception:
        return None


def load_smap_file(path: str | Path) -> SMap:
    p = Path(path)
    data = p.read_bytes()
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as e:
        raise ValueError(
            "当前 .smap 不是 UTF-8 JSON 格式，可能是 protobuf 二进制地图。"
            "本版本先支持通过 4011 下载得到的 JSON 地图；如需解析二进制 .smap，需补充 message_map.proto 生成的 Python 解析器。"
        ) from e
    try:
        raw = json.loads(text)
    except Exception as e:
        raise ValueError(f".smap/JSON 解析失败: {type(e).__name__}: {e}") from e
    if not isinstance(raw, dict):
        raise ValueError("地图文件顶层不是 JSON object")
    return parse_smap(raw, str(p))


def save_smap_json(path: str | Path, raw: Dict[str, Any]) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("w", encoding="utf-8") as f:
        json.dump(raw, f, ensure_ascii=False, indent=2)


def parse_smap(raw: Dict[str, Any], path: str = "") -> SMap:
    header = raw.get("header") or raw.get("map_info") or raw.get("info") or {}
    name = header.get("mapName") or header.get("map_name") or raw.get("map_name") or raw.get("name") or Path(path).stem

    min_x = min_y = max_x = max_y = None
    min_pos = header.get("minPos") or header.get("min_pos") or {}
    max_pos = header.get("maxPos") or header.get("max_pos") or {}
    if isinstance(min_pos, dict) and isinstance(max_pos, dict):
        try:
            min_x, min_y = float(min_pos.get("x")), float(min_pos.get("y"))
            max_x, max_y = float(max_pos.get("x")), float(max_pos.get("y"))
        except Exception:
            min_x = min_y = max_x = max_y = None

    normal_points: List[Tuple[float, float]] = []
    for key in ["normalPosList", "normal_pos_list", "pointList", "points"]:
        arr = raw.get(key)
        if isinstance(arr, list):
            for item in arr:
                if isinstance(item, dict):
                    xy = _get_xy(item)
                    if xy is not None:
                        normal_points.append(xy)
            break

    advanced_points = raw.get("advancedPointList") or raw.get("advanced_point_list") or raw.get("stations") or []
    if not isinstance(advanced_points, list):
        advanced_points = []

    advanced_lines = raw.get("advancedLineList") or raw.get("advanced_line_list") or []
    if not isinstance(advanced_lines, list):
        advanced_lines = []

    advanced_curves = raw.get("advancedCurveList") or raw.get("advanced_curve_list") or []
    if not isinstance(advanced_curves, list):
        advanced_curves = []

    # 如果 header 没有边界，从点和线里自动推断。
    xs: List[float] = []
    ys: List[float] = []
    for x, y in normal_points:
        xs.append(x); ys.append(y)
    for p in advanced_points:
        if isinstance(p, dict):
            pos = p.get("pos") or p.get("point") or p
            if isinstance(pos, dict):
                xy = _get_xy(pos)
                if xy:
                    xs.append(xy[0]); ys.append(xy[1])
    for line_obj in advanced_lines:
        line = line_obj.get("line") if isinstance(line_obj, dict) else None
        if isinstance(line, dict):
            for k in ["startPos", "endPos", "start_pos", "end_pos"]:
                pos = line.get(k)
                if isinstance(pos, dict):
                    xy = _get_xy(pos)
                    if xy:
                        xs.append(xy[0]); ys.append(xy[1])
    if min_x is None or min_y is None or max_x is None or max_y is None:
        if xs and ys:
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
        else:
            min_x, min_y, max_x, max_y = -5.0, -5.0, 5.0, 5.0

    pad_x = max(0.5, (max_x - min_x) * 0.05)
    pad_y = max(0.5, (max_y - min_y) * 0.05)
    return SMap(
        raw=raw,
        path=path,
        name=str(name),
        min_x=float(min_x) - pad_x,
        min_y=float(min_y) - pad_y,
        max_x=float(max_x) + pad_x,
        max_y=float(max_y) + pad_y,
        normal_points=normal_points,
        advanced_points=advanced_points,
        advanced_lines=advanced_lines,
        advanced_curves=advanced_curves,
    )


def extract_station_like_points(smap: SMap) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for item in smap.advanced_points:
        if not isinstance(item, dict):
            continue
        name = item.get("instanceName") or item.get("id") or item.get("name") or ""
        class_name = item.get("className") or ""
        pos = item.get("pos") or item.get("point") or item
        if isinstance(pos, dict):
            xy = _get_xy(pos)
            if xy:
                out.append({"id": str(name), "className": str(class_name), "x": xy[0], "y": xy[1], "raw": item})
    return out
