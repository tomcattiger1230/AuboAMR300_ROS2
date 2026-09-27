#!/usr/bin/env python3
"""After vertical-bar driving, make two short arm moves into the tester line."""

import json
import math
import sys
import traceback
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from rebar_tester_geometry import (  # noqa: E402
    BAR_CENTER_Z, BAR_HOLD_Z, BASE_DRIVE_WAYPOINTS_XY, BASE_TARGET_YAW,
    GRIP_LINE_XY, INSERT_Y_INSET,
)
from test_rebar_grasp import quat_conjugate, quat_rotate  # noqa: E402
from test_rebar_tester_load import TesterLoadTest, build_parser  # noqa: E402


class MicroInsert(TesterLoadTest):
    def run_micro_insert(self):
        state_path = Path(self.args.state_file)
        state = json.loads(state_path.read_text(encoding="utf-8"))
        if state.get("completed") != "drive_vertical_to_tester":
            raise RuntimeError("first drive the vertical bar to the tester")
        for client in (self._cartesian_client, self._fk_client):
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError(f"service unavailable: {client.srv_name}")
        if not self._execute_client.wait_for_server(timeout_sec=10):
            raise RuntimeError("execute_trajectory action unavailable")
        self.spin(2.0)
        x, y, yaw = self.base_pose()
        parked = (math.dist((x, y), BASE_DRIVE_WAYPOINTS_XY[-1]) < 0.05
                  and abs(math.atan2(math.sin(yaw - BASE_TARGET_YAW),
                                     math.cos(yaw - BASE_TARGET_YAW))) < 0.08)
        bar = self.rebar_position()
        ready = (parked and abs(bar[0] - GRIP_LINE_XY[0]) < 0.05
                 and abs(bar[1] - (GRIP_LINE_XY[1] - 0.30)) < 0.07
                 and abs(bar[2] - BAR_CENTER_Z) < 0.06
                 and abs(self.rebar_axis_in_base()[2]) > 0.97)
        self.record("short_insert_start", ready,
                    base_pose=[round(x, 4), round(y, 4), round(yaw, 4)],
                    bar_world=[round(v, 4) for v in bar])
        if not ready:
            raise RuntimeError("bar is not staged for a short insertion")
        self.wait_for_robot_attachment()
        self.payload_scene(True)
        self.add_tester_to_scene()
        if self._tester_gripped:
            raise RuntimeError("tester jaws are already gripping the bar")
        if self._base_locked:
            self.release_base_lock()
        self._base_hold_target = (BASE_DRIVE_WAYPOINTS_XY[-1], BASE_TARGET_YAW)
        wrist_pose_for = self.wrist_pose_factory(state)
        insert_quat = tuple(state["insert_quat"])

        for label, target_y in (("micro_midway", GRIP_LINE_XY[1] - 0.15),
                                ("micro_to_gripline", GRIP_LINE_XY[1] - INSERT_Y_INSET)):
            self.spin(0.2)
            base_position, base_quat = self._base_position, self._base_quat
            target_world = (GRIP_LINE_XY[0], target_y, BAR_HOLD_Z)
            relative = tuple(a - b for a, b in zip(target_world, base_position))
            tcp = quat_rotate(quat_conjugate(base_quat), relative)
            self.cartesian_motion_min(
                label, [wrist_pose_for(tcp, insert_quat).pose],
                min_fraction=0.97 if label == "micro_midway" else 0.50,
                speed_scale=0.04,
            )
            self.spin(0.3)
            bar = self.rebar_position()
            x, y, yaw = self.base_pose()
            safe = (abs(bar[0] - GRIP_LINE_XY[0]) < 0.05
                    and abs(bar[2] - BAR_CENTER_Z) < 0.08
                    and abs(self.rebar_axis_in_base()[2]) > 0.95
                    and math.dist((x, y), BASE_DRIVE_WAYPOINTS_XY[-1]) < 0.08
                    and abs(math.atan2(math.sin(yaw - BASE_TARGET_YAW),
                                       math.cos(yaw - BASE_TARGET_YAW))) < 0.12)
            self.record(label + "_safety", safe,
                        bar_world=[round(v, 4) for v in bar],
                        base_pose=[round(x, 4), round(y, 4), round(yaw, 4)])
            if not safe:
                raise RuntimeError("bar or base left the safe insertion corridor")

        for attempt in range(1, 4):
            self.spin(0.2)
            bar = self.rebar_position()
            if (abs(bar[0] - GRIP_LINE_XY[0]) < 0.035
                    and abs(bar[1] - GRIP_LINE_XY[1]) < 0.055
                    and abs(bar[2] - BAR_CENTER_Z) < 0.08):
                break
            base_position, base_quat = self._base_position, self._base_quat
            target_world = (GRIP_LINE_XY[0],
                            GRIP_LINE_XY[1] - INSERT_Y_INSET, BAR_HOLD_Z)
            relative = tuple(a - b for a, b in zip(target_world, base_position))
            tcp = quat_rotate(quat_conjugate(base_quat), relative)
            self.cartesian_motion_min(
                f"micro_align_{attempt}",
                [wrist_pose_for(tcp, insert_quat).pose],
                min_fraction=0.50, speed_scale=0.04,
            )
        self.spin(0.3)
        bar = self.rebar_position()
        axis = self.rebar_axis_in_base()
        aligned = (abs(bar[0] - GRIP_LINE_XY[0]) < 0.035
                   and abs(bar[1] - GRIP_LINE_XY[1]) < 0.055
                   and abs(bar[2] - BAR_CENTER_Z) < 0.08
                   and abs(axis[2]) > 0.95)
        self.record("rebar_aligned_in_jaws", aligned,
                    bar_world=[round(v, 4) for v in bar],
                    axis_base=[round(v, 5) for v in axis],
                    robot_attached=self._robot_attached,
                    tester_gripped=self._tester_gripped)
        if not aligned:
            raise RuntimeError("bar did not reach the tester grip line")
        self._base_hold_target = None
        self.wait_for_base_lock()
        state["completed"] = "micro_insert"
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
    node = MicroInsert(args)
    try:
        passed = node.run_micro_insert()
    except Exception as exc:
        node.get_logger().error(traceback.format_exc())
        node.record("micro_insert_error", False, error=str(exc))
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
