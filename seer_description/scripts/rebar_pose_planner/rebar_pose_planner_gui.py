#!/usr/bin/env python3
"""Qt window for collision-checked onboard-rebar end-effector pose previews."""

from __future__ import annotations

import json
import sys
from pathlib import Path

from PySide6.QtCore import QProcess, Qt
from PySide6.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout,
    QGroupBox, QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton,
    QTableWidget, QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget,
)

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rebar_pose_planner_core import DEFAULT_TARGET, SLOT_X  # noqa: E402


FIELDS = (
    ("X (m)", "x", -20.0, 20.0, 4),
    ("Y (m)", "y", -20.0, 20.0, 4),
    ("Z (m)", "z", 0.1, 3.0, 4),
    ("Roll (°)", "roll", -360.0, 360.0, 2),
    ("Pitch (°)", "pitch", -360.0, 360.0, 2),
    ("Yaw (°)", "yaw", -360.0, 360.0, 2),
)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("钢筋末端姿态规划 · MoveIt 预览")
        self.resize(760, 720)
        self.result = None
        self.process = None
        self._stdout = bytearray()
        self._stderr = bytearray()

        root = QWidget()
        layout = QVBoxLayout(root)
        intro = QLabel(
            "选择车载取筋槽位，再指定钢筋中心 TCP 的目标位姿。"
            "规划从已夹稳、已抬升至 0.90 m 的状态开始；不会发送运动指令。"
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        source = QGroupBox("取钢筋点位")
        source_form = QFormLayout(source)
        self.slot = QComboBox()
        for number, x in enumerate(SLOT_X, 1):
            self.slot.addItem(f"{number} 号槽 · 车体 X={x:.4f} m", number)
        source_form.addRow("车载料架", self.slot)
        source_form.addRow("规划起点", QLabel("夹稳后抬升位 · base_footprint Z=0.90 m"))
        layout.addWidget(source)

        target = QGroupBox("目标位姿 · 钢筋中心 TCP")
        target_form = QFormLayout(target)
        self.frame = QComboBox()
        self.frame.addItem("world · 实验室世界坐标", "world")
        self.frame.addItem("base_footprint · 底盘坐标", "base_footprint")
        target_form.addRow("坐标系", self.frame)
        self.fields = {}
        for (label, key, low, high, decimals), value in zip(FIELDS, DEFAULT_TARGET):
            box = QDoubleSpinBox()
            box.setRange(low, high)
            box.setDecimals(decimals)
            box.setSingleStep(0.01 if decimals == 4 else 1.0)
            box.setValue(value)
            self.fields[key] = box
            target_form.addRow(label, box)
        preset = QPushButton("填入正面测试位")
        preset.clicked.connect(self.set_front_preset)
        target_form.addRow("快捷位", preset)
        layout.addWidget(target)

        note = QLabel(
            "快捷位说明：世界坐标 (6.00, 3.90, 1.50) m，"
            "RPY (90°, 0°, 180°)。工具轴朝世界 +Y，钢筋竖直。"
            "目标姿态是腕部方向，位置是经 0.16 m 标定后的钢筋中心 TCP。"
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #526173;")
        layout.addWidget(note)

        actions = QHBoxLayout()
        self.plan_button = QPushButton("规划并检查碰撞")
        self.plan_button.setObjectName("plan")
        self.plan_button.clicked.connect(self.start_plan)
        self.save_button = QPushButton("保存 JSON")
        self.save_button.setEnabled(False)
        self.save_button.clicked.connect(self.save_result)
        actions.addWidget(self.plan_button)
        actions.addWidget(self.save_button)
        layout.addLayout(actions)

        self.status = QLabel("等待规划。需先启动 Ubuntu 的钢筋实验室场景和 MoveIt。")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["检查项", "结果"])
        self.table.setColumnWidth(0, 215)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setMinimumHeight(155)
        self.table.setMaximumHeight(180)
        layout.addWidget(self.table)
        self.details = QTextEdit()
        self.details.setReadOnly(True)
        layout.addWidget(self.details, 1)
        self.setCentralWidget(root)
        self.setStyleSheet(
            "QWidget{font-size:14px} QGroupBox{font-weight:600;"
            "border:1px solid #b9c6d5;border-radius:6px;margin-top:10px;"
            "padding-top:12px} QGroupBox::title{subcontrol-origin:margin;left:10px} "
            "QPushButton{min-height:31px;padding:3px 10px} "
            "QPushButton#plan{background:#1769aa;color:white}"
        )

    def set_front_preset(self):
        self.frame.setCurrentIndex(0)
        for (_, key, *_), value in zip(FIELDS, DEFAULT_TARGET):
            self.fields[key].setValue(value)

    def start_plan(self):
        if self.process is not None:
            return
        backend = Path(__file__).with_name("rebar_pose_planner_backend.py")
        if not backend.exists():
            QMessageBox.critical(self, "程序文件缺失", str(backend))
            return
        args = [str(backend), "--slot", str(self.slot.currentData()),
                "--frame", str(self.frame.currentData())]
        for _, key, *_ in FIELDS:
            args += ["--" + key, str(self.fields[key].value())]
        self.result = None
        self.save_button.setEnabled(False)
        self.table.setRowCount(0)
        self.details.clear()
        self.status.setText("正在请求 MoveIt 逆解和碰撞检查规划…")
        self.status.setStyleSheet("color:#8b6100;")
        self.plan_button.setEnabled(False)
        self._stdout.clear()
        self._stderr.clear()
        self.process = QProcess(self)
        self.process.readyReadStandardOutput.connect(self.read_stdout)
        self.process.readyReadStandardError.connect(self.read_stderr)
        self.process.finished.connect(self.finished)
        self.process.start(sys.executable, args)

    def read_stdout(self):
        self._stdout.extend(bytes(self.process.readAllStandardOutput()))

    def read_stderr(self):
        self._stderr.extend(bytes(self.process.readAllStandardError()))

    def finished(self, exit_code, _status):
        self.read_stdout()
        self.read_stderr()
        self.plan_button.setEnabled(True)
        self.process.deleteLater()
        self.process = None
        lines = self._stdout.decode("utf-8", errors="replace").splitlines()
        try:
            result = json.loads(lines[-1])
        except (IndexError, json.JSONDecodeError):
            message = self._stderr.decode("utf-8", errors="replace").strip()
            self.status.setText("规划进程未返回有效 JSON。" + (" " + message[-500:] if message else ""))
            self.status.setStyleSheet("color:#b3261e;")
            return
        self.result = result
        self.save_button.setEnabled(True)
        self.render_result(result)

    def render_result(self, result):
        ok = bool(result.get("ok"))
        status = (
            "已找到携筋碰撞检查路径；尚未执行。" if ok else
            f"规划未通过 · {result.get('stage', 'unknown')}：{result.get('error', '请查看详情')}"
        )
        if result.get("scene_warnings"):
            status += "  场景提示：" + "；".join(result["scene_warnings"])
        self.status.setText(status)
        self.status.setStyleSheet("color:#18733b;" if ok else "color:#b3261e;")
        rows = []
        for stage in result.get("stages", []):
            name = stage.get("name", "")
            if name == "source_lift_ik":
                rows.append(("取料后抬升位逆解", f"通过 · FK 误差 {stage['fk_error_m'] * 1000:.1f} mm"))
            elif name == "target_ik":
                rows.append(("目标姿态逆解", f"通过 · FK 误差 {stage['fk_error_m'] * 1000:.1f} mm"))
            elif name == "collision_checked_path":
                rows.append(("携筋碰撞检查路径",
                             f"通过 · {stage['points']} 点 / {stage['duration_s']:.1f} s"))
        if not ok:
            rows.append(("停止阶段", str(result.get("stage", "未知"))))
        self.table.setRowCount(len(rows))
        for row, values in enumerate(rows):
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.table.setItem(row, col, item)
        self.details.setPlainText(json.dumps(result, ensure_ascii=False, indent=2))

    def save_result(self):
        if self.result is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "保存规划结果", f"rebar_pose_slot{self.result.get('slot', 1)}.json",
            "JSON 文件 (*.json)",
        )
        if path:
            Path(path).write_text(json.dumps(self.result, ensure_ascii=False, indent=2),
                                  encoding="utf-8")

    def closeEvent(self, event):
        process = self.process
        if process is not None:
            process.kill()
            process.waitForFinished(1000)
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
