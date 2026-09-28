"""UI 全局样式与状态码映射。

集中管理：
- 颜色调色板（语义色：成功/告警/失败/中性）
- 字号、间距
- 全局 QSS（应用启动时 QApplication.setStyleSheet）
- AGV 状态码 → 人话 + 颜色（导航状态、报警分级、急停源）
"""
from __future__ import annotations

from typing import Dict, Tuple

from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QStyleFactory


def color_with_alpha(name: str, alpha: int) -> QColor:
    """根据色名 + alpha 返回 QColor（PySide6 的 QColor 不支持 (str,int) 两参数签名）。"""
    c = QColor(name)
    c.setAlpha(max(0, min(255, alpha)))
    return c


# ─────────────────────────────────────────────────────────────
# 调色板
# ─────────────────────────────────────────────────────────────
COLORS: Dict[str, str] = {
    # 主色
    "primary":      "#2563eb",   # 蓝：操作主色（Ping、查询、连接）
    "primary_dark": "#1d4ed8",
    # 状态色
    "ok":           "#16a34a",   # 绿：成功 / 运行中 / 已连接
    "warn":         "#d97706",   # 橙：告警 / 暂停 / 注意
    "error":        "#dc2626",   # 红：失败 / 急停 / 报警
    "info":         "#0284c7",   # 浅蓝：信息
    "muted":        "#6b7280",   # 灰：未连接 / 待机 / N/A
    "busy":         "#7c3aed",   # 紫：进行中（导航中、上传中）
    # UI 表面
    "bg":           "#f8fafc",   # 主背景（淡冷灰）
    "bg_panel":     "#ffffff",   # 卡片背景
    "bg_subtle":    "#f1f5f9",   # 次级背景（输入框、hover）
    "border":       "#e2e8f0",   # 边框
    "border_focus": "#3b82f6",   # 聚焦边框
    "text":         "#0f172a",   # 主文本
    "text_muted":   "#64748b",   # 次级文本
    "text_invert":  "#ffffff",   # 反色文本（按钮上）
    # 地图语义色（与 map_view.py 保持一致）
    "map_grid":     "#cbd5e1",
    "map_point":    "#94a3b8",
    "map_line":     "#475569",
    "map_curve":    "#0ea5e9",
    "map_station":  "#f59e0b",
    "map_agv":      "#dc2626",
    "map_bounds":   "#a78bfa",
    "map_reloc_pick": "#16a34a",
}

# ─────────────────────────────────────────────────────────────
# 字号 / 间距
# ─────────────────────────────────────────────────────────────
FONT_FAMILY = '"PingFang SC", "Microsoft YaHei", "Helvetica Neue", "Arial", sans-serif'
FONT_FAMILY_MONO = '"JetBrains Mono", "SF Mono", "Menlo", "Consolas", monospace'

FONT_SIZE = {
    "xs": 10,
    "sm": 11,
    "md": 12,
    "lg": 14,
    "xl": 16,
    "xxl": 20,
}

SPACING = {
    "xs": 4,
    "sm": 6,
    "md": 10,
    "lg": 16,
    "xl": 24,
}


# ─────────────────────────────────────────────────────────────
# AGV 状态码映射（导航、定位、地图载入、报警）
# ─────────────────────────────────────────────────────────────

# 导航状态 (1020/3051)
NAV_STATUS: Dict[int, Tuple[str, str]] = {
    0: ("NONE",      COLORS["muted"]),
    1: ("WAITING",   COLORS["warn"]),
    2: ("RUNNING",   COLORS["busy"]),
    3: ("SUSPENDED", COLORS["warn"]),
    4: ("COMPLETED", COLORS["ok"]),
    5: ("FAILED",    COLORS["error"]),
    6: ("CANCELED",  COLORS["muted"]),
}

# 定位状态 (1021)
RELOc_STATUS: Dict[int, Tuple[str, str]] = {
    0: ("未重定位",      COLORS["warn"]),
    1: ("定位正常",      COLORS["ok"]),
    2: ("重定位中",      COLORS["busy"]),
    3: ("定位正常",      COLORS["ok"]),
}

# 地图载入状态 (1022)
LOADMAP_STATUS: Dict[int, Tuple[str, str]] = {
    0: ("未载入",        COLORS["muted"]),
    1: ("载入中",        COLORS["busy"]),
    2: ("已载入",        COLORS["ok"]),
    3: ("载入失败",      COLORS["error"]),
}

# 急停源（典型字段）
ESTOP_LABELS: Dict[str, str] = {
    "emergency":     "急停按钮",
    "driver_emc":    "驱动器急停",
    "electric":      "电气回路",
    "electric_emc":  "电气故障急停",
    "soft_emc":      "软件急停",
}


def nav_status_text(code) -> str:
    """把导航状态码映射成 (文本, 颜色)。未知码返回原值+灰。"""
    try:
        text, color = NAV_STATUS.get(int(code), (f"未知({code})", COLORS["muted"]))
        return text, color
    except (ValueError, TypeError):
        return "—", COLORS["muted"]


def reloc_status_text(code) -> Tuple[str, str]:
    try:
        return RELOc_STATUS.get(int(code), (f"未知({code})", COLORS["muted"]))
    except (ValueError, TypeError):
        return "—", COLORS["muted"]


def loadmap_status_text(code) -> Tuple[str, str]:
    try:
        return LOADMAP_STATUS.get(int(code), (f"未知({code})", COLORS["muted"]))
    except (ValueError, TypeError):
        return "—", COLORS["muted"]


# ─────────────────────────────────────────────────────────────
# 日志分级着色（INFO/OK/WARN/ERROR/DEBUG）
# ─────────────────────────────────────────────────────────────
LOG_LEVEL_COLORS: Dict[str, str] = {
    "INFO":  COLORS["info"],
    "OK":    COLORS["ok"],
    "WARN":  COLORS["warn"],
    "ERROR": COLORS["error"],
    "DEBUG": COLORS["muted"],
}


def classify_log_level(text: str) -> str:
    """根据日志文本前缀或关键词推断级别。"""
    t = text.strip()
    upper = t.upper()
    if upper.startswith("[ERROR]") or "失败" in t or "ERROR" in upper or "异常" in t:
        return "ERROR"
    if upper.startswith("[WARN]") or "告警" in t or "警告" in t or "WARN" in upper:
        return "WARN"
    if upper.startswith("[OK]") or "成功" in t or "完成" in t or "正常" in t:
        return "OK"
    if upper.startswith("[DEBUG]"):
        return "DEBUG"
    return "INFO"


# ─────────────────────────────────────────────────────────────
# 全局 QSS
# ─────────────────────────────────────────────────────────────
def get_qss() -> str:
    """返回 QApplication.setStyleSheet 用的样式表。"""
    return f"""
/* ──────── 全局 ──────── */
QWidget {{
    font-family: {FONT_FAMILY};
    font-size: {FONT_SIZE['md']}px;
    color: {COLORS['text']};
}}

QMainWindow {{
    background: {COLORS['bg']};
}}
QWidget[role="appRoot"], QWidget[role="panel"], QWidget[role="scrollViewport"] {{
    background: {COLORS['bg']};
}}
QWidget[role="panelWhite"] {{
    background: {COLORS['bg_panel']};
}}

/* ──────── GroupBox ──────── */
QGroupBox {{
    background: {COLORS['bg_panel']};
    border: 1px solid {COLORS['border']};
    border-radius: 8px;
    margin-top: 14px;
    padding: {SPACING['md']}px {SPACING['md']}px {SPACING['md']}px {SPACING['md']}px;
    font-weight: 600;
    font-size: {FONT_SIZE['lg']}px;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 2px {SPACING['sm']}px;
    color: {COLORS['primary']};
    background: {COLORS['bg_panel']};
}}

/* ──────── PushButton ──────── */
QPushButton {{
    background: {COLORS['bg_panel']};
    border: 1px solid {COLORS['border']};
    border-radius: 6px;
    padding: 6px 14px;
    min-height: 22px;
    color: {COLORS['text']};
}}
QPushButton:hover {{
    background: {COLORS['bg_subtle']};
    border-color: {COLORS['primary']};
}}
QPushButton:pressed {{
    background: {COLORS['primary']};
    color: {COLORS['text_invert']};
}}
QPushButton:disabled {{
    color: {COLORS['text_muted']};
    background: {COLORS['bg_subtle']};
}}
QPushButton[primary="true"] {{
    background: {COLORS['primary']};
    color: {COLORS['text_invert']};
    border-color: {COLORS['primary']};
}}
QPushButton[primary="true"]:hover {{
    background: {COLORS['primary_dark']};
}}
QPushButton[danger="true"] {{
    background: {COLORS['error']};
    color: {COLORS['text_invert']};
    border-color: {COLORS['error']};
}}
QPushButton[danger="true"]:hover {{
    background: #b91c1c;
}}
QPushButton[success="true"] {{
    background: {COLORS['ok']};
    color: {COLORS['text_invert']};
    border-color: {COLORS['ok']};
}}
QPushButton[success="true"]:hover {{
    background: #15803d;
}}
QPushButton[warning="true"] {{
    background: {COLORS['warn']};
    color: {COLORS['text_invert']};
    border-color: {COLORS['warn']};
}}
QPushButton[warning="true"]:hover {{
    background: #b45309;
}}

QToolButton {{
    background: transparent;
    border: 1px solid transparent;
    border-radius: 4px;
    padding: 3px 6px;
    color: {COLORS['text_muted']};
}}
QToolButton:hover {{
    background: {COLORS['bg_subtle']};
    border-color: {COLORS['border']};
    color: {COLORS['text']};
}}

/* ──────── 输入控件 ──────── */
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
    background: {COLORS['bg_panel']};
    border: 1px solid {COLORS['border']};
    border-radius: 4px;
    padding: 4px 8px;
    selection-background-color: {COLORS['primary']};
}}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{
    border-color: {COLORS['border_focus']};
}}

QComboBox::drop-down {{
    border: none;
    width: 22px;
}}

/* ──────── 标签 ──────── */
QLabel {{
    color: {COLORS['text']};
    background: transparent;
}}
QLabel[role="muted"] {{
    color: {COLORS['text_muted']};
}}
QLabel[role="mono"] {{
    font-family: {FONT_FAMILY_MONO};
    font-size: {FONT_SIZE['sm']}px;
}}
QLabel[role="title"] {{
    font-size: {FONT_SIZE['xxl']}px;
    font-weight: 700;
    color: {COLORS['text']};
}}
QLabel[role="subtitle"] {{
    font-size: {FONT_SIZE['sm']}px;
    color: {COLORS['text_muted']};
}}
QLabel[role="sourcePill"] {{
    color: {COLORS['text_muted']};
    background: {COLORS['bg_subtle']};
    border: 1px solid {COLORS['border']};
    border-radius: 4px;
    padding: 3px 6px;
    font-size: {FONT_SIZE['sm']}px;
}}

/* ──────── CheckBox ──────── */
QCheckBox {{
    spacing: 6px;
}}
QCheckBox::indicator {{
    width: 16px;
    height: 16px;
    border: 1px solid {COLORS['border']};
    border-radius: 3px;
    background: {COLORS['bg_panel']};
}}
QCheckBox::indicator:checked {{
    background: {COLORS['primary']};
    border-color: {COLORS['primary']};
}}

/* ──────── 文本编辑 ──────── */
QPlainTextEdit {{
    background: {COLORS['bg_panel']};
    border: 1px solid {COLORS['border']};
    border-radius: 6px;
    padding: 6px;
    font-family: {FONT_FAMILY_MONO};
    font-size: {FONT_SIZE['sm']}px;
    selection-background-color: {COLORS['primary']};
}}

/* ──────── ScrollArea ──────── */
QScrollArea {{
    border: none;
    background: {COLORS['bg']};
}}
QScrollBar:vertical {{
    background: transparent;
    width: 10px;
    margin: 0;
}}
QScrollBar::handle:vertical {{
    background: {COLORS['border']};
    border-radius: 5px;
    min-height: 24px;
}}
QScrollBar::handle:vertical:hover {{
    background: {COLORS['text_muted']};
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}

/* ──────── Splitter ──────── */
QSplitter::handle {{
    background: {COLORS['border']};
}}
QSplitter::handle:horizontal {{
    width: 1px;
}}

/* ──────── StatusBar ──────── */
QStatusBar {{
    background: {COLORS['bg_panel']};
    border-top: 1px solid {COLORS['border']};
    color: {COLORS['text_muted']};
    font-size: {FONT_SIZE['sm']}px;
}}
QStatusBar::item {{
    border: none;
}}

/* ──────── Header bar 自定义 ──────── */
QWidget[role="header"] {{
    background: {COLORS['bg_panel']};
    border-bottom: 1px solid {COLORS['border']};
}}

/* ──────── 状态卡片 ──────── */
QFrame[role="card"] {{
    background: {COLORS['bg_panel']};
    border: 1px solid {COLORS['border']};
    border-radius: 8px;
}}
QFrame[role="card"]:hover {{
    border-color: #bfdbfe;
}}

/* ──────── Tab ──────── */
QTabWidget::pane {{
    border: 1px solid {COLORS['border']};
    border-radius: 6px;
    background: {COLORS['bg_panel']};
    top: -1px;
}}
QTabBar::tab {{
    background: transparent;
    padding: 6px 14px;
    margin-right: 2px;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    color: {COLORS['text_muted']};
}}
QTabBar::tab:selected {{
    background: {COLORS['bg_panel']};
    color: {COLORS['primary']};
    border: 1px solid {COLORS['border']};
    border-bottom: none;
}}
QTabBar::tab:hover:!selected {{
    color: {COLORS['text']};
}}

/* ──────── 工具提示 ──────── */
QToolTip {{
    background: {COLORS['text']};
    color: {COLORS['text_invert']};
    border: none;
    padding: 4px 8px;
    border-radius: 4px;
    font-size: {FONT_SIZE['sm']}px;
}}
"""


def apply_app_style(app) -> None:
    """给 QApplication 应用浅色 palette + 全局 QSS。"""
    app.setStyle(QStyleFactory.create("Fusion"))
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(COLORS["bg"]))
    palette.setColor(QPalette.WindowText, QColor(COLORS["text"]))
    palette.setColor(QPalette.Base, QColor(COLORS["bg_panel"]))
    palette.setColor(QPalette.AlternateBase, QColor(COLORS["bg_subtle"]))
    palette.setColor(QPalette.ToolTipBase, QColor(COLORS["text"]))
    palette.setColor(QPalette.ToolTipText, QColor(COLORS["text_invert"]))
    palette.setColor(QPalette.Text, QColor(COLORS["text"]))
    palette.setColor(QPalette.Button, QColor(COLORS["bg_panel"]))
    palette.setColor(QPalette.ButtonText, QColor(COLORS["text"]))
    palette.setColor(QPalette.BrightText, QColor(COLORS["error"]))
    palette.setColor(QPalette.Highlight, QColor(COLORS["primary"]))
    palette.setColor(QPalette.HighlightedText, QColor(COLORS["text_invert"]))
    app.setPalette(palette)
    app.setStyleSheet(get_qss())
