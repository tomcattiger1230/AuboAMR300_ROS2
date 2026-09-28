#!/usr/bin/env python3
"""Set a vertical bar to the tester approach pose while the base is clear."""

import json
import math
import sys
import traceback
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from rebar_tester_geometry import (  # noqa: E402
    BASE_DRIVE_WAYPOINTS_XY, BASE_TARGET_YAW,
)
from test_rebar_grasp import ARM_JOINTS, quat_rotate  # noqa: E402
from test_rebar_tester_load import TesterLoadTest, build_parser  # noqa: E402


# Measured collision-valid front-facing branch from the Isaac run.
# FK is checked against the requested TCP before any motion in this run.
APPROACH_JOINT_CANDIDATES = (
    (2.597, -0.357, -1.451, -1.094, -1.026, 0.0),
    (2.600, -0.355, -1.449, -1.095, -1.029, 0.0),
)
APPROACH_TCP_BASE = (-0.80, -0.08, 1.50)


class PrepositionAtStandoff(TesterLoadTest):
    def run_preposition(self):
        state_path = Path(self.args.state_file)
        state = json.loads(state_path.read_text(encoding="utf-8"))
        if state.get("completed") != "verticalize_at_standoff":
            raise RuntimeError("first verticalize the bar at standoff")
        for client in (self._fk_client, self._plan_client):
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError(f"service unavailable: {client.srv_name}")
        if not self._execute_client.wait_for_server(timeout_sec=10):
            raise RuntimeError("execute_trajectory action unavailable")
        self.spin(2.0)
        x, y, yaw = self.base_pose()
        parked = (math.dist((x, y), BASE_DRIVE_WAYPOINTS_XY[-2]) < 0.06
                  and abs(math.atan2(math.sin(yaw - BASE_TARGET_YAW),
                                     math.cos(yaw - BASE_TARGET_YAW))) < 0.08)
        self.record("base_at_standoff_before_preposition", parked,
                    base_pose=[round(x, 3), round(y, 3), round(yaw, 3)])
        if not parked:
            raise RuntimeError("base left the standoff waypoint")
        self.wait_for_robot_attachment()
        self.payload_scene(True)
        self.add_tester_to_scene()
        if abs(self.rebar_axis_in_base()[2]) < 0.95:
            raise RuntimeError("bar is no longer vertical")
        if state.get("compact_preposition_complete"):
            self.check_payload_while_parked(APPROACH_TCP_BASE, tolerance=.035)
            wrist = self.fk(self.current_arm(), "wrist3_Link")
            q = (wrist.orientation.x, wrist.orientation.y, wrist.orientation.z, wrist.orientation.w)
            tool = quat_rotate(q, (0, 0, 1))
            closing = quat_rotate(q, (1, 0, 0))
            valid = -tool[0] > .98 and -closing[1] > .98
            self.record("compact_preposition_verified_without_replanning", valid,
                        rebar_base=self.rebar_in_base(), tool_axis_base=tool, closing_axis_base=closing)
            if not valid:
                raise RuntimeError("compact front pose did not remain aligned")
            self.wait_for_base_lock()
            state["completed"] = "preposition_at_standoff"
            state["approach_tcp_base"] = list(APPROACH_TCP_BASE)
            temporary = state_path.with_suffix(".tmp")
            temporary.write_text(json.dumps(state, indent=2))
            temporary.replace(state_path)
            self.write_report()
            return self._passed
        if self._base_locked:
            self.release_base_lock()
        self._base_hold_target = (BASE_DRIVE_WAYPOINTS_XY[-2], BASE_TARGET_YAW)

        desired = self.wrist_pose_factory(state)(
            APPROACH_TCP_BASE, tuple(state["insert_quat"])
        ).pose
        current = self.current_arm()
        selected = None
        for values in APPROACH_JOINT_CANDIDATES:
            candidate = dict(zip(ARM_JOINTS, values))
            fk = self.fk(candidate, "wrist3_Link")
            position_error = math.dist(
                (fk.position.x, fk.position.y, fk.position.z),
                (desired.position.x, desired.position.y, desired.position.z),
            )
            alignment = abs(sum(a * b for a, b in zip(
                (fk.orientation.x, fk.orientation.y,
                 fk.orientation.z, fk.orientation.w),
                (desired.orientation.x, desired.orientation.y,
                 desired.orientation.z, desired.orientation.w),
            )))
            valid_fk = position_error < 0.02 and alignment > 0.999
            self.record("known_approach_branch_fk", valid_fk,
                        joints=[round(v, 4) for v in values],
                        position_error=round(position_error, 4),
                        alignment=round(alignment, 6))
            if not valid_fk:
                continue
            for trial in range(1, 4):
                plan = self.joint_plan(
                    self.bounded_arm_for_planning(current),
                    self.bounded_arm_for_planning(candidate),
                )
                if plan.error_code.val == 1:
                    selected = (candidate, plan, trial)
                    break
            if selected:
                break
        if not selected:
            raise RuntimeError("no collision-valid standoff preposition path")
        candidate, plan, trial = selected
        points = len(plan.trajectory.joint_trajectory.points)
        self.record("standoff_preposition_plan", True, trial=trial,
                    points=points, target=candidate)
        for point in plan.trajectory.joint_trajectory.points:
            stamp = point.time_from_start
            nanoseconds = (stamp.sec * 1_000_000_000 + stamp.nanosec) * 4
            stamp.sec, stamp.nanosec = divmod(nanoseconds, 1_000_000_000)
            point.velocities = [value / 4 for value in point.velocities]
            point.accelerations = [value / 16 for value in point.accelerations]
        self.run_motion("standoff_preposition_exec", plan)
        measured = self.rebar_in_base()
        axis = self.rebar_axis_in_base()
        wrist = self.fk(self.current_arm(), "wrist3_Link")
        wrist_quat = (wrist.orientation.x, wrist.orientation.y,
                      wrist.orientation.z, wrist.orientation.w)
        tool_axis = quat_rotate(wrist_quat, (0.0, 0.0, 1.0))
        closing_axis = quat_rotate(wrist_quat, (1.0, 0.0, 0.0))
        error = math.dist(measured, APPROACH_TCP_BASE)
        reached = (error < 0.035 and abs(axis[2]) > 0.95
                   and -tool_axis[0] > 0.98 and -closing_axis[1] > 0.98)
        self.record("vertical_approach_pose_at_standoff", reached,
                    rebar_base=[round(v, 4) for v in measured],
                    axis_base=[round(v, 5) for v in axis],
                    tool_axis_base=[round(v, 5) for v in tool_axis],
                    closing_axis_base=[round(v, 5) for v in closing_axis],
                    position_error=round(error, 4))
        if not reached:
            raise RuntimeError("steel missed its approach pose")
        self.spin(0.2)
        x, y, yaw = self.base_pose()
        stable_base = (math.dist((x, y), BASE_DRIVE_WAYPOINTS_XY[-2]) < 0.08
                       and abs(math.atan2(math.sin(yaw - BASE_TARGET_YAW),
                                          math.cos(yaw - BASE_TARGET_YAW))) < 0.12)
        self.record("base_safe_after_standoff_preposition", stable_base,
                    base_pose=[round(x, 3), round(y, 3), round(yaw, 3)])
        if not stable_base:
            raise RuntimeError("arm preposition shifted the base too far")
        self.wait_for_base_lock()
        state["completed"] = "preposition_at_standoff"
        state["approach_tcp_base"] = list(APPROACH_TCP_BASE)
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
    node = PrepositionAtStandoff(args)
    try:
        passed = node.run_preposition()
    except Exception as exc:
        node.get_logger().error(traceback.format_exc())
        node.record("preposition_error", False, error=str(exc))
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
