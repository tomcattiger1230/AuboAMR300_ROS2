#!/usr/bin/env python3
"""Manual control GUI for the Isaac rebar tensile tester.

Four rows (upper/lower carriage height, upper/lower jaw opening) with a
slider + entry pair and a Send button; publishes std_msgs/Float64 to the
corresponding /rebar_tester/<group>/<axis>_cmd topic. The four *_state
topics are echoed live so you can watch the mechanisms move. Works
alongside or instead of the automated insertion workflow.

Requires tkinter (Python standard library on desktop Ubuntu).
"""

from __future__ import annotations

import argparse
import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

# label, key, unit, min, max, resolution, help
CHANNELS = [
    ("上横梁高度", "upper_z", "m", 1.50, 1.88, 0.005, "1.50 – 1.88"),
    ("下横梁高度", "lower_z", "m", 0.92, 1.34, 0.005, "0.92 – 1.34"),
    ("上爪开口", "upper_opening", "m", 0.024, 0.16, 0.001, "0.024 – 0.16"),
    ("下爪开口", "lower_opening", "m", 0.024, 0.16, 0.001, "0.024 – 0.16"),
]
STATE_TIMEOUT_S = 1.5


def channel_topics(key):
    group, axis = key.split("_", 1)
    return f"/rebar_tester/{group}/{axis}_cmd", f"/rebar_tester/{group}/{axis}_state"


class TesterGuiNode(Node):
    def __init__(self):
        super().__init__("rebar_tester_gui")
        # Named with a leading underscore: rclpy Node reserves the
        # read-only `publishers` property.
        self._cmd_publishers = {}
        self.latest_state = {}
        self.state_stamps = {}
        for _, key, *_ in CHANNELS:
            cmd_topic, state_topic = channel_topics(key)
            self._cmd_publishers[key] = self.create_publisher(Float64, cmd_topic, 10)
            self.create_subscription(
                Float64,
                state_topic,
                self.state_callback(key),
                qos_profile_sensor_data,
            )
        self.create_timer(0.1, self.spin_tick)

    def state_callback(self, key):
        def on_message(message):
            self.latest_state[key] = message.data
            self.state_stamps[key] = self.get_clock().now()
        return on_message

    def spin_tick(self):
        # Keep rclpy processing while Tk owns the main loop.
        rclpy.spin_once(self, timeout_sec=0.0)

    def send(self, key, value):
        self._cmd_publishers[key].publish(Float64(data=value))

    def state(self, key):
        stamp = self.state_stamps.get(key)
        if stamp is None:
            return None
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age > STATE_TIMEOUT_S:
            return None
        return self.latest_state.get(key)


class TesterGui:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.rows = {}

        root.title("钢筋拉伸测试机 — 手动控制")
        root.resizable(False, False)

        header = ttk.Frame(root, padding=(10, 8, 10, 0))
        header.pack(fill="x")
        ttk.Label(
            header, text="指令 (m)", font=("sans", 9, "bold")
        ).grid(row=0, column=1)
        ttk.Label(header, text="实际状态", font=("sans", 9, "bold")).grid(
            row=0, column=3
        )
        ttk.Label(header, text="范围").grid(row=0, column=4)

        body = ttk.Frame(root, padding=10)
        body.pack(fill="x")

        for row_index, (label, key, unit, low, high, resolution, hint) in enumerate(
            CHANNELS, start=1
        ):
            ttk.Label(body, text=f"{label} ({unit})").grid(
                row=row_index, column=0, sticky="w", padx=(0, 6), pady=3
            )
            slider = ttk.Scale(
                body, from_=low, to=high, length=180,
                command=self.slider_moved(key),
            )
            slider.set(low)
            slider.grid(row=row_index, column=1, padx=4)
            entry = ttk.Entry(body, width=7, justify="center")
            entry.insert(0, f"{low:g}")
            entry.grid(row=row_index, column=2, padx=4)
            state_label = ttk.Label(body, text="—", width=8, anchor="center")
            state_label.grid(row=row_index, column=3, padx=4)
            ttk.Label(body, text=hint, foreground="#666").grid(
                row=row_index, column=4, padx=(4, 0)
            )
            send_button = ttk.Button(
                body, text="发送", width=6, command=self.send_clicked(key)
            )
            send_button.grid(row=row_index, column=5, padx=(6, 0))
            self.rows[key] = {
                "slider": slider,
                "entry": entry,
                "state": state_label,
                "low": low,
                "high": high,
                "resolution": resolution,
                "row": row_index,
            }

        presets = ttk.Frame(root, padding=(10, 0, 10, 8))
        presets.pack(fill="x")
        ttk.Button(
            presets, text="预置装填位\n(上1.88 下1.05 全开)",
            command=lambda: self.preset(
                {"upper_z": 1.88, "lower_z": 1.05,
                 "upper_opening": 0.16, "lower_opening": 0.16}
            ),
        ).pack(side="left", padx=4)
        ttk.Button(
            presets, text="双爪全开",
            command=lambda: self.preset(
                {"upper_opening": 0.16, "lower_opening": 0.16}
            ),
        ).pack(side="left", padx=4)
        ttk.Button(
            presets, text="双爪夹紧",
            command=lambda: self.preset(
                {"upper_opening": 0.024, "lower_opening": 0.024}
            ),
        ).pack(side="left", padx=4)

        self.status = ttk.Label(
            root, text="等待状态话题…", foreground="#888", padding=(10, 0, 10, 8)
        )
        self.status.pack(fill="x")
        self.root.after(100, self.refresh_states)

    # ---------- handlers ----------

    def slider_moved(self, key):
        def on_move(value):
            # Slider.set() fires this during construction, before the row
            # dictionary entry exists; just ignore those early callbacks.
            row = self.rows.get(key)
            if row is None:
                return
            quantized = round(float(value) / row["resolution"]) * row["resolution"]
            if row["entry"].focus_get() != row["entry"]:
                row["entry"].delete(0, tk.END)
                row["entry"].insert(0, f"{quantized:g}")
        return on_move

    def entry_value(self, key):
        row = self.rows[key]
        try:
            value = float(row["entry"].get())
        except ValueError:
            self.flash(f"无效输入：{row['entry'].get()!r}")
            return None
        if not math.isfinite(value):
            self.flash("输入必须是有限数值")
            return None
        if value < row["low"] - 1e-9 or value > row["high"] + 1e-9:
            self.flash(
                f"越界：{value:g} 超出 [{row['low']:g}, {row['high']:g}]，已拒绝"
            )
            return None
        return value

    def send_clicked(self, key):
        def on_click():
            value = self.entry_value(key)
            if value is None:
                return
            self.node.send(key, value)
            self.flash(f"已发送 {key} = {value:g}")
        return on_click

    def preset(self, values):
        for key, value in values.items():
            row = self.rows[key]
            row["slider"].set(value)
            row["entry"].delete(0, tk.END)
            row["entry"].insert(0, f"{value:g}")
            self.node.send(key, value)
        self.flash("预置指令已发送")

    def flash(self, text, error=False):
        self.status.configure(
            text=text, foreground="#b00" if error else "#070"
        )

    def refresh_states(self):
        alive = 0
        for _, key, *_ in CHANNELS:
            value = self.node.state(key)
            label = self.rows[key]["state"]
            if value is None:
                label.configure(text="—", foreground="#999")
            else:
                alive += 1
                label.configure(text=f"{value:.3f}", foreground="#000")
        if alive == 0:
            self.status.configure(
                text="未收到 _state 话题：请确认仿真已启动、ROS_DOMAIN_ID 一致",
                foreground="#b00",
            )
        elif alive < len(CHANNELS):
            self.status.configure(text="部分状态未更新", foreground="#a60")
        self.root.after(100, self.refresh_states)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    rclpy.init()
    node = TesterGuiNode()
    root = tk.Tk()
    try:
        TesterGui(root, node)
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
