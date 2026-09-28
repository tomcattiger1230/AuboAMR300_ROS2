from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

from PySide6.QtCore import QPoint, QPointF, Qt, QRectF, Signal
from PySide6.QtGui import QColor, QFont, QFontMetrics, QPainter, QPen, QBrush
from PySide6.QtWidgets import QWidget

from .smap import SMap, extract_station_like_points
from .styles import COLORS, color_with_alpha as _cw


class MapView(QWidget):
    """轻量 .smap 显示控件。

    V1.4：
    - 去掉大面积蓝色覆盖。
    - 支持地图点选手动重定位初值。
    - 支持双击站点/路径点选择导航目标。
    """

    relocalizePointPicked = Signal(float, float)
    stationDoubleClicked = Signal(str, float, float)
    cursorWorldChanged = Signal(float, float)  # 鼠标在世界坐标下的位置（实时显示）

    def __init__(self, parent=None):
        super().__init__(parent)
        # 小屏幕适配：地图区域允许缩小，左右控制区通过滚动条操作。
        self.setMinimumSize(500, 360)
        self.smap: Optional[SMap] = None
        self.agv: Optional[Dict[str, Any]] = None
        self.flip_y = False
        self.scale_factor = 1.0
        self.pan = QPointF(0.0, 0.0)
        self.last_mouse: Optional[QPoint] = None
        self.mouse_dragged = False

        self.show_grid = True
        self.show_normal_points = True
        self.show_lines = True
        self.show_curves = True
        self.show_points = True
        self.show_bounds = False
        self.show_agv = True

        self.point_step = 1
        self.pick_relocalize_mode = False
        self.selected_relocalize_point: Optional[Tuple[float, float]] = None
        self.selected_station_id: str = ""
        self._cursor_world: Optional[Tuple[float, float]] = None  # 鼠标当前世界坐标

    def set_map(self, smap: Optional[SMap]) -> None:
        self.smap = smap
        self.agv = None
        self.scale_factor = 1.0
        self.pan = QPointF(0.0, 0.0)
        self.selected_relocalize_point = None
        self.selected_station_id = ""
        self.update()

    def set_agv_pose(self, x: Optional[float], y: Optional[float], angle: Optional[float]) -> None:
        if x is None or y is None:
            self.agv = None
        else:
            self.agv = {"x": float(x), "y": float(y), "angle": float(angle or 0.0)}
        self.update()

    def set_flip_y(self, value: bool) -> None:
        self.flip_y = bool(value)
        self.update()

    def set_show_grid(self, value: bool) -> None:
        self.show_grid = bool(value)
        self.update()

    def set_show_normal_points(self, value: bool) -> None:
        self.show_normal_points = bool(value)
        self.update()

    def set_show_lines(self, value: bool) -> None:
        self.show_lines = bool(value)
        self.update()

    def set_show_curves(self, value: bool) -> None:
        self.show_curves = bool(value)
        self.update()

    def set_show_points(self, value: bool) -> None:
        self.show_points = bool(value)
        self.update()

    def set_show_bounds(self, value: bool) -> None:
        self.show_bounds = bool(value)
        self.update()

    def set_show_agv(self, value: bool) -> None:
        self.show_agv = bool(value)
        self.update()

    def set_pick_relocalize_mode(self, value: bool) -> None:
        self.pick_relocalize_mode = bool(value)
        self.setCursor(Qt.CrossCursor if self.pick_relocalize_mode else Qt.ArrowCursor)
        self.update()

    def fit_to_view(self) -> None:
        self.scale_factor = 1.0
        self.pan = QPointF(0.0, 0.0)
        self.update()

    def center_agv(self) -> None:
        if not self.agv or not self.smap:
            return
        center = QPointF(self.width() / 2, self.height() / 2)
        pos = self.world_to_screen(self.agv["x"], self.agv["y"])
        self.pan += center - pos
        self.update()

    def base_transform(self) -> Tuple[float, float, float, float, float]:
        if not self.smap:
            return 1.0, self.width() / 2, self.height() / 2, -5.0, -5.0
        margin = 28.0
        w = max(1.0, self.width() - 2 * margin)
        h = max(1.0, self.height() - 2 * margin)
        scale = min(w / self.smap.width, h / self.smap.height) * self.scale_factor
        ox = margin + (w - self.smap.width * scale) / 2 - self.smap.min_x * scale + self.pan.x()
        if self.flip_y:
            oy = margin + (h - self.smap.height * scale) / 2 - self.smap.min_y * scale + self.pan.y()
        else:
            oy = margin + (h - self.smap.height * scale) / 2 + self.smap.max_y * scale + self.pan.y()
        return scale, ox, oy, self.smap.min_x, self.smap.min_y

    def world_to_screen(self, x: float, y: float) -> QPointF:
        scale, ox, oy, _, _ = self.base_transform()
        sx = ox + x * scale
        sy = oy + (y * scale if self.flip_y else -y * scale)
        return QPointF(sx, sy)

    def screen_to_world(self, pt: QPoint | QPointF) -> Tuple[float, float]:
        scale, ox, oy, _, _ = self.base_transform()
        x = (float(pt.x()) - ox) / scale
        if self.flip_y:
            y = (float(pt.y()) - oy) / scale
        else:
            y = -(float(pt.y()) - oy) / scale
        return x, y

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor(COLORS["bg"]))
        if self.show_grid:
            self._draw_grid(painter)
        if self.smap:
            self._draw_smap(painter)
        else:
            painter.setPen(QPen(QColor(COLORS["text_muted"]), 1))
            painter.drawText(self.rect(), Qt.AlignCenter, "未加载 .smap/JSON 地图\n使用右侧“载入本地 .smap 地图”选择文件")
        if self.selected_relocalize_point is not None:
            self._draw_selected_relocalize_point(painter)
        if self.show_agv:
            self._draw_agv(painter)
        # 右上角：鼠标世界坐标
        if self._cursor_world is not None:
            self._draw_cursor_label(painter)
        # 右下角：比例尺
        if self.smap:
            self._draw_scale_bar(painter)
        if self.pick_relocalize_mode:
            # 顶部提示条（背景 + 文本，提高可读性）
            painter.setBrush(QBrush(_cw(COLORS["warn"], 60)))
            painter.setPen(QPen(QColor(COLORS["warn"]), 1))
            painter.drawRoundedRect(QRectF(8, 8, 360, 22), 4, 4)
            painter.setPen(QPen(QColor(COLORS["warn"]), 1))
            painter.drawText(QRectF(12, 10, 316, 18), Qt.AlignVCenter | Qt.AlignLeft,
                             "🎯 地图点选定位模式：单击地图填入 x/y")
        painter.end()

    def _draw_grid(self, painter: QPainter) -> None:
        painter.setPen(QPen(QColor(COLORS["map_grid"]), 1))
        step = 60
        for x in range(0, self.width(), step):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), step):
            painter.drawLine(0, y, self.width(), y)
        # 中线略深
        painter.setPen(QPen(QColor(COLORS["text_muted"]), 1, Qt.DashLine))
        painter.drawLine(self.width() // 2, 0, self.width() // 2, self.height())
        painter.drawLine(0, self.height() // 2, self.width(), self.height() // 2)

    def _draw_smap(self, painter: QPainter) -> None:
        assert self.smap is not None
        if self.show_normal_points and self.smap.normal_points:
            painter.setPen(QPen(QColor(COLORS["map_point"]), 1))
            step = max(1, int(self.point_step))
            for x, y in self.smap.normal_points[::step]:
                painter.drawPoint(self.world_to_screen(x, y))

        if self.show_lines:
            painter.setPen(QPen(QColor(COLORS["map_line"]), 1.4))
            for obj in self.smap.advanced_lines:
                line = obj.get("line") if isinstance(obj, dict) else None
                if not isinstance(line, dict):
                    continue
                sp = line.get("startPos") or line.get("start_pos")
                ep = line.get("endPos") or line.get("end_pos")
                if isinstance(sp, dict) and isinstance(ep, dict):
                    try:
                        p1 = self.world_to_screen(float(sp["x"]), float(sp["y"]))
                        p2 = self.world_to_screen(float(ep["x"]), float(ep["y"]))
                        painter.drawLine(p1, p2)
                    except Exception:
                        pass

        if self.show_curves:
            painter.setPen(QPen(QColor(COLORS["map_curve"]), 1.5))
            for obj in self.smap.advanced_curves:
                if not isinstance(obj, dict):
                    continue
                pts = self._curve_points(obj)
                if len(pts) >= 2:
                    for a, b in zip(pts[:-1], pts[1:]):
                        painter.drawLine(self.world_to_screen(a[0], a[1]), self.world_to_screen(b[0], b[1]))

        if self.show_points:
            painter.setFont(QFont("Sans", 9))
            stations = extract_station_like_points(self.smap)
            for st in stations:
                p = self.world_to_screen(st["x"], st["y"])
                is_sel = st.get("id") == self.selected_station_id
                painter.setPen(QPen(QColor(COLORS["primary_dark"]), 2 if is_sel else 1))
                painter.setBrush(QBrush(_cw(COLORS["map_station"], 230 if is_sel else 180)))
                painter.drawEllipse(p, 7 if is_sel else 5, 7 if is_sel else 5)
                painter.setPen(QPen(QColor(COLORS["text"]), 1))
                painter.drawText(p + QPointF(8, -8), st.get("id", ""))

        if self.show_bounds:
            painter.setPen(QPen(QColor(COLORS["map_bounds"]), 1, Qt.DashLine))
            p1 = self.world_to_screen(self.smap.min_x, self.smap.min_y)
            p2 = self.world_to_screen(self.smap.max_x, self.smap.max_y)
            painter.drawRect(QRectF(p1, p2).normalized())

    def _curve_points(self, obj: Dict[str, Any], n: int = 32) -> List[Tuple[float, float]]:
        def pos_from(item):
            if isinstance(item, dict):
                if "pos" in item and isinstance(item["pos"], dict):
                    item = item["pos"]
                if "x" in item and "y" in item:
                    return float(item["x"]), float(item["y"])
            return None

        p0 = pos_from(obj.get("startPos"))
        p3 = pos_from(obj.get("endPos"))
        p1 = pos_from(obj.get("controlPos1")) or p0
        p2 = pos_from(obj.get("controlPos2")) or p3
        if not (p0 and p1 and p2 and p3):
            return []
        pts = []
        for i in range(n + 1):
            t = i / n
            x = (1 - t) ** 3 * p0[0] + 3 * (1 - t) ** 2 * t * p1[0] + 3 * (1 - t) * t ** 2 * p2[0] + t ** 3 * p3[0]
            y = (1 - t) ** 3 * p0[1] + 3 * (1 - t) ** 2 * t * p1[1] + 3 * (1 - t) * t ** 2 * p2[1] + t ** 3 * p3[1]
            pts.append((x, y))
        return pts

    def _draw_selected_relocalize_point(self, painter: QPainter) -> None:
        assert self.selected_relocalize_point is not None
        x, y = self.selected_relocalize_point
        p = self.world_to_screen(x, y)
        painter.setPen(QPen(QColor(COLORS["map_reloc_pick"]), 2))
        painter.setBrush(QBrush(_cw(COLORS["map_reloc_pick"], 120)))
        painter.drawEllipse(p, 10, 10)
        painter.setPen(QPen(QColor(COLORS["map_reloc_pick"]), 1.5))
        painter.drawLine(p + QPointF(-14, 0), p + QPointF(14, 0))
        painter.drawLine(p + QPointF(0, -14), p + QPointF(0, 14))
        painter.setPen(QPen(QColor(COLORS["text"]), 1))
        painter.drawText(p + QPointF(10, -12), f"重定位点 ({x:.3f},{y:.3f})")

    def _draw_agv(self, painter: QPainter) -> None:
        if not self.agv:
            return
        x = self.agv["x"]
        y = self.agv["y"]
        a = self.agv.get("angle", 0.0)
        p = self.world_to_screen(x, y)
        # 车体圆圈
        painter.setPen(QPen(QColor(COLORS["map_agv"]), 2))
        painter.setBrush(QBrush(_cw(COLORS["map_agv"], 180)))
        painter.drawEllipse(p, 8, 8)
        # 朝向箭头（三角形）—— 比之前一条线更明显
        length = 22.0
        head_len = 10.0
        head_w = 6.0
        cos_a = math.cos(a)
        sin_a = math.sin(a)
        # 屏幕坐标系下 sin 需要根据 flip_y 反转
        sin_screen = sin_a if self.flip_y else -sin_a

        def tx(dx_world: float, dy_world: float) -> QPointF:
            return QPointF(cos_a * dx_world + (-sin_a) * dy_world,
                           sin_screen * dx_world + cos_a * dy_world)

        tip = p + tx(length, 0.0)
        left = p + tx(-head_w * 0.4, -head_w)
        right = p + tx(-head_w * 0.4, head_w)
        head_base = p + tx(-head_len * 0.3, 0.0)
        painter.setPen(QPen(QColor(COLORS["map_agv"]), 1.5))
        painter.setBrush(QBrush(_cw(COLORS["map_agv"], 220)))
        painter.drawPolygon([tip, left, head_base, right])
        # 坐标标签
        painter.setPen(QPen(QColor(COLORS["text"]), 1))
        painter.setFont(QFont("Sans", 9))
        painter.drawText(p + QPointF(10, 18), f"AGV ({x:.2f},{y:.2f},{a:.2f}rad)")

    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale_factor = max(0.05, min(80.0, self.scale_factor * factor))
        self.update()

    def _nearest_station(self, pos: QPoint, max_dist_px: float = 18.0) -> Optional[Dict[str, Any]]:
        if not self.smap:
            return None
        best = None
        best_d = max_dist_px
        for st in extract_station_like_points(self.smap):
            sp = self.world_to_screen(st["x"], st["y"])
            d = math.hypot(sp.x() - pos.x(), sp.y() - pos.y())
            if d <= best_d:
                best = st
                best_d = d
        return best

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            st = self._nearest_station(event.pos())
            if st:
                self.selected_station_id = st.get("id", "")
                self.stationDoubleClicked.emit(self.selected_station_id, float(st["x"]), float(st["y"]))
                self.update()
                return
        super().mouseDoubleClickEvent(event)

    def mousePressEvent(self, event):
        if self.pick_relocalize_mode and event.button() == Qt.LeftButton:
            x, y = self.screen_to_world(event.pos())
            self.selected_relocalize_point = (x, y)
            self.relocalizePointPicked.emit(x, y)
            self.update()
            return
        if event.button() == Qt.LeftButton:
            self.last_mouse = event.pos()
            self.mouse_dragged = False

    def mouseMoveEvent(self, event):
        # 记录鼠标世界坐标，右上角实时显示
        wx, wy = self.screen_to_world(event.pos())
        self._cursor_world = (wx, wy)
        self.cursorWorldChanged.emit(wx, wy)

        if self.last_mouse is not None:
            delta = event.pos() - self.last_mouse
            if abs(delta.x()) + abs(delta.y()) > 2:
                self.mouse_dragged = True
            self.pan += QPointF(delta.x(), delta.y())
            self.last_mouse = event.pos()
        self.update()

    def leaveEvent(self, event):
        self._cursor_world = None
        self.update()

    def _draw_cursor_label(self, painter: QPainter) -> None:
        """右上角：鼠标世界坐标。"""
        assert self._cursor_world is not None
        wx, wy = self._cursor_world
        text = f"📍 ({wx:.2f}, {wy:.2f}) m"
        font = QFont("Sans", 10)
        font.setBold(True)
        painter.setFont(font)
        fm = QFontMetrics(font)
        w = fm.horizontalAdvance(text) + 16
        h = fm.height() + 8
        rect = QRectF(self.width() - w - 10, 10, w, h)
        painter.setBrush(QBrush(_cw(COLORS["bg_panel"], 230)))
        painter.setPen(QPen(QColor(COLORS["border"]), 1))
        painter.drawRoundedRect(rect, 4, 4)
        painter.setPen(QPen(QColor(COLORS["text"]), 1))
        painter.drawText(rect.adjusted(8, 4, -8, -4), Qt.AlignVCenter | Qt.AlignLeft, text)

    def _draw_scale_bar(self, painter: QPainter) -> None:
        """右下角：比例尺。选 1/2/5/10/20/50/100m 中接近 80 像素的那个。"""
        if not self.smap:
            return
        scale, _, _, _, _ = self.base_transform()
        if scale <= 0:
            return
        target_px = 80
        target_m = target_px / scale
        # 选 1-2-5 序列里最近的
        candidates = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000]
        best = min(candidates, key=lambda m: abs(m - target_m))
        bar_px = best * scale
        # 画在右下角
        margin = 14
        x0 = self.width() - margin - bar_px
        y0 = self.height() - margin
        # 背景 pill
        text = f"{best:g} m"
        font = QFont("Sans", 9)
        font.setBold(True)
        painter.setFont(font)
        fm = QFontMetrics(font)
        text_w = fm.horizontalAdvance(text) + 12
        pill_w = max(bar_px + 12, text_w)
        pill_h = fm.height() + 14
        pill = QRectF(x0 - 6, y0 - pill_h + 4, pill_w, pill_h)
        painter.setBrush(QBrush(_cw(COLORS["bg_panel"], 220)))
        painter.setPen(QPen(QColor(COLORS["border"]), 1))
        painter.drawRoundedRect(pill, 6, 6)
        # 比例尺线（黑-白-黑三段）
        seg = bar_px / 4
        painter.setPen(QPen(QColor(COLORS["text"]), 2))
        painter.drawLine(int(x0), int(y0 - 8), int(x0 + bar_px), int(y0 - 8))
        painter.setPen(QPen(QColor(COLORS["text_invert"]), 2))
        painter.drawLine(int(x0 + seg), int(y0 - 8), int(x0 + 3 * seg), int(y0 - 8))
        painter.setPen(QColor(COLORS["text"]))
        painter.drawText(QRectF(x0, y0 - pill_h + 6, bar_px, fm.height() + 4), Qt.AlignCenter, text)

    def mouseReleaseEvent(self, event):
        self.last_mouse = None
        self.mouse_dragged = False
