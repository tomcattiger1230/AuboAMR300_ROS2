#!/usr/bin/env python3
"""Raise a rack-picked bar to vertical at the open 6.0, 2.6 m waypoint."""

import json
import math
import sys
import traceback
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from rebar_tester_geometry import (  # noqa: E402
    BASE_DRIVE_WAYPOINTS_XY, BASE_TARGET_YAW, ONBOARD_VERTICALIZE_TCP_BASE,
)
from test_rebar_grasp import quat_from_basis  # noqa: E402
from test_rebar_tester_load import (  # noqa: E402
    TesterLoadTest, build_parser, quat_slerp,
)


class VerticalizeAtStandoff(TesterLoadTest):
    def run_verticalize(self):
        state_path = Path(self.args.state_file)
        state = json.loads(state_path.read_text(encoding="utf-8"))
        if state.get("source") != "onboard_slot" or state.get("slot") != 1:
            raise RuntimeError("checkpoint is not the first onboard rack slot")
        if state.get("pickup_waypoint") != list(BASE_DRIVE_WAYPOINTS_XY[-2]):
            raise RuntimeError("bar was not picked at the standoff waypoint")
        for client in (self._fk_client, self._cartesian_client, self._plan_client):
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError(f"service unavailable: {client.srv_name}")
        if not self._execute_client.wait_for_server(timeout_sec=10):
            raise RuntimeError("execute_trajectory action unavailable")
        self.spin(2.0)
        x, y, yaw = self.base_pose()
        parked = (math.dist((x, y), BASE_DRIVE_WAYPOINTS_XY[-2]) < 0.05
                  and abs(math.atan2(math.sin(yaw - BASE_TARGET_YAW),
                                     math.cos(yaw - BASE_TARGET_YAW))) < 0.08)
        self.record("base_at_standoff", parked,
                    base_pose=[round(x, 3), round(y, 3), round(yaw, 3)])
        if not parked:
            raise RuntimeError("base left the standoff waypoint")
        self.wait_for_robot_attachment()
        self.payload_scene(True)
        self.add_tester_to_scene()
        if self._base_locked:
            self.release_base_lock()
        self._base_hold_target = (BASE_DRIVE_WAYPOINTS_XY[-2], BASE_TARGET_YAW)

        grasp_quat = tuple(state["grasp_quat"])
        wrist_pose_for = self.wrist_pose_factory(state)
        staging = ONBOARD_VERTICALIZE_TCP_BASE
        self.cartesian_motion("move_to_open_verticalize_staging",
                              [wrist_pose_for(staging, grasp_quat).pose])
        insert_quat = quat_from_basis((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
        poses = [wrist_pose_for(
            staging, quat_slerp(grasp_quat, insert_quat, index / 24.0)
        ).pose for index in range(1, 25)]
        self.cartesian_slow("verticalize_before_base_approach", poses)
        axis = self.rebar_axis_in_base()
        vertical = abs(axis[2]) > 0.95
        self.record("rebar_vertical_at_standoff", vertical,
                    axis_in_base=[round(v, 5) for v in axis],
                    rebar_world=[round(v, 4) for v in self.rebar_position()])
        if not vertical:
            raise RuntimeError("bar is not vertical at standoff")
        self.check_payload_while_parked(staging, tolerance=0.06)
        self.spin(0.3)
        self.wait_for_base_lock()
        state["completed"] = "verticalize_at_standoff"
        # Preserve the vertical bar through the following 90-degree yaw.
        # Tool +Z then points toward the tester, normal to its XZ front.
        front_quat = quat_from_basis((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0))
        state["insert_quat"] = list(front_quat)
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
    node = VerticalizeAtStandoff(args)
    try:
        passed = node.run_verticalize()
    except Exception as exc:
        node.get_logger().error(traceback.format_exc())
        node.record("verticalize_error", False, error=str(exc))
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
