#!/usr/bin/env python3
"""Reverse slowly from standoff to the tester while holding a vertical bar."""

import json
import math
import sys
import time
import traceback
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from rebar_tester_geometry import (  # noqa: E402
    BAR_CENTER_Z, BASE_DRIVE_WAYPOINTS_XY, BASE_TARGET_YAW, GRIP_LINE_XY,
)
from test_rebar_tester_load import TesterLoadTest, build_parser  # noqa: E402


class DriveWithVerticalRebar(TesterLoadTest):
    def run_drive(self):
        state_path = Path(self.args.state_file)
        state = json.loads(state_path.read_text(encoding="utf-8"))
        if state.get("completed") != "preposition_at_standoff":
            raise RuntimeError("first preposition the vertical bar at standoff")
        self.spin(2.0)
        self.wait_for_robot_attachment()
        expected = tuple(state["approach_tcp_base"])
        start = self.base_pose()
        safe_start = (math.dist(start[:2], BASE_DRIVE_WAYPOINTS_XY[-2]) < 0.08
                      and abs(math.atan2(math.sin(start[2] - BASE_TARGET_YAW),
                                         math.cos(start[2] - BASE_TARGET_YAW))) < 0.12)
        self.record("vertical_drive_safe_start", safe_start,
                    base_pose=[round(v, 4) for v in start])
        if not safe_start:
            raise RuntimeError("base is not at standoff")
        self.check_payload_while_parked(expected, tolerance=0.04)
        if abs(self.rebar_axis_in_base()[2]) < 0.97:
            raise RuntimeError("bar is not vertical before driving")
        if self._base_locked:
            self.release_base_lock()

        target = BASE_DRIVE_WAYPOINTS_XY[-1]
        deadline = time.monotonic() + 180.0
        last_report = 0.0
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                self.spin(0.03)
                x, y, yaw = self.base_pose()
                distance = math.dist((x, y), target)
                yaw_error = math.atan2(math.sin(BASE_TARGET_YAW - yaw),
                                       math.cos(BASE_TARGET_YAW - yaw))
                bar = self.rebar_in_base()
                axis = self.rebar_axis_in_base()
                slip = math.dist(bar, expected)
                if (slip > 0.045 or abs(axis[2]) < 0.97
                        or abs(x - target[0]) > 0.07
                        or abs(yaw_error) > 0.16
                        or y > target[1] + 0.025):
                    raise RuntimeError(
                        f"vertical drive safety stop: bar error {slip:.3f} m, "
                        f"base ({x:.3f}, {y:.3f}, {yaw:.3f})"
                    )
                if distance < 0.02 and abs(yaw_error) < 0.03:
                    self.record("vertical_drive_at_tester", True,
                                base_pose=[round(x, 4), round(y, 4), round(yaw, 4)],
                                bar_base=[round(v, 4) for v in bar],
                                slip_m=round(slip, 4))
                    break
                command = Twist()
                command.linear.x = -min(0.08, 0.8 * max(0.0, target[1] - y))
                command.angular.z = max(-0.12, min(0.12, 1.2 * yaw_error))
                self._cmd_vel.publish(command)
                now = time.monotonic()
                if now - last_report > 2.0:
                    last_report = now
                    self.get_logger().info(
                        f"vertical drive base=({x:.3f},{y:.3f},{yaw:.3f}) "
                        f"bar slip={slip:.3f} m remaining={distance:.3f} m"
                    )
                time.sleep(0.1)
            else:
                raise RuntimeError("vertical drive timed out")
        finally:
            self.stop_base()
        self.spin(0.4)
        self.wait_for_base_lock()
        actual = self.rebar_position()
        world_ok = (abs(actual[0] - GRIP_LINE_XY[0]) < 0.05
                    and abs(actual[1] - (GRIP_LINE_XY[1] - 0.30)) < 0.06
                    and abs(actual[2] - BAR_CENTER_Z) < 0.06)
        self.record("vertical_bar_before_micro_insert", world_ok,
                    bar_world=[round(v, 4) for v in actual],
                    axis_base=[round(v, 5) for v in self.rebar_axis_in_base()],
                    base_pose=[round(v, 4) for v in self.base_pose()])
        if not world_ok:
            raise RuntimeError("bar missed the pre-insertion area")
        state["completed"] = "drive_vertical_to_tester"
        temporary = state_path.with_suffix(state_path.suffix + ".tmp")
        temporary.write_text(json.dumps(state, indent=2), encoding="utf-8")
        temporary.replace(state_path)
        self.write_report()
        return self._passed


def main():
    parser = build_parser()
    parser.description = __doc__
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DriveWithVerticalRebar(args)
    try:
        passed = node.run_drive()
    except Exception as exc:
        node.get_logger().error(traceback.format_exc())
        node.record("vertical_drive_error", False, error=str(exc))
        node.write_report()
        passed = False
    finally:
        node.stop_base()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
