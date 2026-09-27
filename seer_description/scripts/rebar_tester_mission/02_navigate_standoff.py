#!/usr/bin/env python3
"""Carry the rack-seated bar to the open waypoint before the tester."""

import math
import sys
import traceback
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(Path(__file__).resolve().parent))
from navigate_onboard import OnboardTransport  # noqa: E402
from test_rebar_tester_load import build_parser  # noqa: E402
from rebar_tester_geometry import (  # noqa: E402
    BASE_DRIVE_WAYPOINTS_XY, BASE_TARGET_YAW,
)


class StandoffTransport(OnboardTransport):
    def run_standoff(self, slot):
        self.spin(2.0)
        x, y, _ = self.base_pose()
        at_source = math.dist((x, y), BASE_DRIVE_WAYPOINTS_XY[0]) < 0.10
        self.record("base_at_source", at_source, position=[round(x, 3), round(y, 3)])
        if not at_source:
            raise RuntimeError("base must start at the source waypoint")
        fingers = self.finger_positions()
        released = all(abs(value) < 0.0015 for value in fingers)
        self.record("gripper_released_after_loading", released, fingers=fingers)
        if not released:
            raise RuntimeError("robot is still holding the rebar")
        self.check_seated("rebar_seated_before_drive", slot)
        waypoints = BASE_DRIVE_WAYPOINTS_XY
        for index in range(1, len(waypoints) - 1):
            target = waypoints[index]
            if index == len(waypoints) - 2:
                yaw_target = BASE_TARGET_YAW
            else:
                following = waypoints[index + 1]
                yaw_target = math.atan2(following[1] - target[1],
                                        following[0] - target[0])
            label = "turn_south" if index == len(waypoints) - 2 else f"leg_{index}"
            self.drive_to(target, yaw_target, label)
            self.check_seated(f"rebar_seated_after_{label}", slot)
        self.wait_for_base_lock()
        self.check_seated("rebar_seated_at_standoff", slot)
        self.write_report()
        return self._passed


def main():
    parser = build_parser()
    parser.description = __doc__
    args = parser.parse_args()
    slot = args.onboard_slot or 1
    if slot not in range(1, 5):
        parser.error("--onboard-slot must be 1, 2, 3, or 4")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = StandoffTransport(args)
    try:
        passed = node.run_standoff(slot)
    except Exception as exc:
        node.get_logger().error(traceback.format_exc())
        node.record("transport_error", False, error=str(exc))
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
