#!/usr/bin/env python3
"""Qt (PySide6) manual control GUI for the Isaac rebar tensile tester.

Feature-equivalent to rebar_tester_gui.py (tkinter): four channels
(upper/lower carriage height, upper/lower jaw opening) with slider +
entry + Send, live *_state echo, preset buttons, bounds checking and a
disconnect warning when state messages go stale.
"""

from __future__ import annotations

import argparse
import math
import sys
from time import monotonic as time_monotonic

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

# label, key, unit, min, max, step, decimals
CHANNELS = [
    ("上横梁高度 (m)", "upper_z", 1.50, 1.88, 0.005, 3),
    ("下横梁高度 (m)", "lower_z", 0.92, 1.34, 0.005, 3),
    ("上爪开口 (m)", "upper_opening", 0.024, 0.16, 0.001, 3),
    ("下爪开口 (m)", "lower_opening", 0.024, 0.16, 0.001, 3),
]
STATE_TIMEOUT_S = 1.5


class TesterGuiNode(Node):
    def __init__(self):
        super().__init__("rebar_tester_gui")
        self._cmd_publishers = {}
        self.latest_state = {}
        self.state_stamps = {}
        for _, key, *_ in CHANNELS:
            group, axis = key.split("_", 1)
            topic = f"/rebar_tester/{group}/{axis}"
            self._cmd_publishers[key] = self.create_publisher(
                Float64, topic + "_cmd", 10
            )
            self.create_subscription(
                Float64,
                topic + "_state",
                self.state_callback(key),
                qos_profile_sensor_data,
            )

    def state_callback(self, key):
        def on_message(message):
            self.latest_state[key] = message.data
            self.state_stamps[key] = time_monotonic()
        return on_message

    def send(self, key, value):
        self._cmd_publishers[key].publish(Float64(data=value))

    def state(self, key):
        stamp = self.state_stamps.get(key)
        if stamp is None or time_monotonic() - stamp > STATE_TIMEOUT_S:
            return None
        return self.latest_state.get(key)



class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.rows = {}
        self.setWindowTitle("钢筋拉伸测试机 — 手动控制 (Qt)")
        self.status = QLabel("等待状态话题…")
        self.status.setStyleSheet("color: #888; padding: 6px;")

        central = QWidget()
        layout = QVBoxLayout(central)

        channels_box = QGroupBox("指令通道")
        form = QFormLayout(channels_box)
        for label, key, low, high, step, decimals in CHANNELS:
            row = QHBoxLayout()
            slider = QSlider(Qt.Horizontal)
            slider.setRange(int(low / step), int(high / step))
            slider.setValue(int(low / step))
            slider.setFixedWidth(200)
            spin = QDoubleSpinBox()
            spin.setRange(low, high)
            spin.setDecimals(decimals)
            spin.setSingleStep(step)
            spin.setValue(low)
            state = QLabel("—")
            state.setFixedWidth(70)
            state.setAlignment(Qt.AlignCenter)
            send = QPushButton("发送")
            send.setFixedWidth(60)
            row.addWidget(slider)
            row.addWidget(spin)
            row.addWidget(state)
            row.addWidget(send)
            form.addRow(QLabel(label), row)
            self.rows[key] = {
                "slider": slider, "spin": spin, "state": state,
                "low": low, "high": high, "step": step, "send": send,
            }
            slider.valueChanged.connect(self.slider_moved(key))
            spin.valueChanged.connect(self.spin_changed(key))
            send.clicked.connect(self.send_clicked(key))
        layout.addWidget(channels_box)

        presets = QGroupBox("快捷操作")
        preset_row = QHBoxLayout(presets)
        for text, values in (
            ("预置装填位\n(上1.88 下1.05 全开)",
             {"upper_z": 1.88, "lower_z": 1.05,
              "upper_opening": 0.16, "lower_opening": 0.16}),
            ("双爪全开", {"upper_opening": 0.16, "lower_opening": 0.16}),
            ("双爪夹紧", {"upper_opening": 0.024, "lower_opening": 0.024}),
        ):
            button = QPushButton(text)
            button.clicked.connect(
                lambda checked, v=values: self.preset(v)
            )
            preset_row.addWidget(button)
        layout.addWidget(presets)
        layout.addWidget(self.status)
        self.setCentralWidget(central)

        # Pump rclpy callbacks and refresh the state echo on the Qt clock.
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.tick)
        self.ros_timer.start(50)
        self.state_timer = QTimer(self)
        self.state_timer.timeout.connect(self.refresh_states)
        self.state_timer.start(100)

    # ---------- handlers ----------

    def tick(self):
        if not rclpy.ok():
            return  # shutting down; stop pumping callbacks
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def slider_moved(self, key):
        def on_move(value):
            row = self.rows[key]
            row["spin"].setValue(value * row["step"])
        return on_move

    def spin_changed(self, key):
        def on_change(value):
            row = self.rows[key]
            blocked = row["slider"].blockSignals(True)
            row["slider"].setValue(int(round(value / row["step"])))
            row["slider"].blockSignals(blocked)
        return on_change

    def send_clicked(self, key):
        def on_click():
            row = self.rows[key]
            value = row["spin"].value()
            if not math.isfinite(value):
                return
            self.node.send(key, value)
            self.flash(f"已发送 {key} = {value:g}")
        return on_click

    def preset(self, values):
        for key, value in values.items():
            row = self.rows[key]
            row["spin"].setValue(value)
            self.node.send(key, value)
        self.flash("预置指令已发送")

    def flash(self, text, error=False):
        self.status.setStyleSheet(
            f"color: {'#b00' if error else '#070'}; padding: 6px;"
        )
        self.status.setText(text)

    def refresh_states(self):
        alive = 0
        for _, key, *_ in CHANNELS:
            value = self.node.state(key)
            label = self.rows[key]["state"]
            if value is None:
                label.setText("—")
                label.setStyleSheet("color: #999;")
            else:
                alive += 1
                label.setText(f"{value:.3f}")
                label.setStyleSheet("color: #000;")
        if alive == 0:
            self.status.setStyleSheet("color: #b00; padding: 6px;")
            self.status.setText(
                "未收到 _state 话题：请确认仿真已启动、ROS_DOMAIN_ID 一致"
            )
        elif alive < len(CHANNELS):
            self.status.setStyleSheet("color: #a60; padding: 6px;")
            self.status.setText("部分状态未更新")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    rclpy.init()
    node = TesterGuiNode()
    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()
    try:
        app.exec()
    finally:
        window.ros_timer.stop()
        window.state_timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
