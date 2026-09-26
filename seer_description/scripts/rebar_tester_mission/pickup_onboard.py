#!/usr/bin/env python3
"""Pick a rebar from the parked chassis rack for tester insertion.

Run after navigate_onboard.py in the same Isaac scene. This node leaves the
bar held by the robot and writes a checkpoint accepted by 04_insert.py.
"""

import json
import math
import sys
import traceback
from pathlib import Path

import rclpy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from rebar_experiment_geometry import RACK_REBAR_Z, RACK_RELEASE_TCP_Z, SLOT_X  # noqa: E402
from rebar_tester_geometry import (  # noqa: E402
    BASE_DRIVE_WAYPOINTS_XY,
    BASE_TARGET_YAW,
)
from test_rebar_grasp import GoalStatus, quat_from_basis, quat_rotate  # noqa: E402
from test_rebar_tester_load import TesterLoadTest, build_parser  # noqa: E402


class OnboardPickup(TesterLoadTest):
    def remove_seated_collision_object(self, slot):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        attached = AttachedCollisionObject(link_name="base_link")
        attached.object.id = f"onboard_rebar_slot_{slot}"
        attached.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects = [attached]
        result = self.call(self._scene_apply,
                           ApplyPlanningScene.Request(scene=scene))
        self.record("seated_rebar_removed_from_planning_scene", result.success,
                    slot=slot)
        if not result.success:
            raise RuntimeError("MoveIt rejected removal of the seated bar")

    def run_pickup(self, slot):
        for client in (self._scene_apply, self._cartesian_client, self._plan_client):
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError(f"service unavailable: {client.srv_name}")
        if not self._execute_client.wait_for_server(timeout_sec=10):
            raise RuntimeError("execute_trajectory action unavailable")
        if not self._gripper_client.wait_for_server(timeout_sec=10):
            raise RuntimeError("gripper action unavailable")
        self.spin(2.0)

        x, y, yaw = self.base_pose()
        parked = (math.dist((x, y), BASE_DRIVE_WAYPOINTS_XY[-1]) < 0.04
                  and abs(math.atan2(math.sin(yaw - BASE_TARGET_YAW),
                                     math.cos(yaw - BASE_TARGET_YAW))) < 0.04
                  and self._base_locked)
        self.record("base_parked_before_onboard_pickup", parked,
                    position=[round(x, 3), round(y, 3)],
                    yaw=round(yaw, 3), brake_locked=self._base_locked)
        if not parked:
            raise RuntimeError("base is not parked and locked at the tester")

        expected = (SLOT_X[slot - 1], 0.0, RACK_REBAR_Z)
        rebar = self.rebar_in_base()
        axis = self.rebar_axis_in_base()
        seated = math.dist(rebar, expected) < 0.025 and abs(axis[1]) > 0.95
        self.record("rebar_seated_for_pickup", seated, slot=slot,
                    position=[round(v, 4) for v in rebar],
                    axis=[round(v, 4) for v in axis])
        if not seated:
            raise RuntimeError("rebar is not in the selected chassis slot")
        fingers = self.finger_positions()
        if not self.args.resume_clamped:
            open_gripper = all(value is not None and abs(value) < 0.0015
                               for value in fingers)
            self.record("gripper_open_before_pickup", open_gripper, fingers=fingers)
            if not open_gripper:
                raise RuntimeError("gripper is not open")

        self.add_tester_to_scene()
        # Loading placed the bar lengthwise along base Y, with gripper motor Z
        # down and finger closing axis along base X. The calibrated TCP is
        # 0.16 m along wrist Z (same wrist and tool as the loading node).
        grasp_quat = quat_from_basis((1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
        calibration = {"grasp_quat": grasp_quat,
                       "tcp_in_wrist": (0.0, 0.0, self.args.tcp_z)}
        wrist_pose_for = self.wrist_pose_factory(calibration)
        pregrasp = (rebar[0], rebar[1], RACK_RELEASE_TCP_Z + 0.15)
        grasp = (rebar[0], rebar[1], RACK_RELEASE_TCP_Z)
        before = self.rebar_in_base()
        if self.args.resume_clamped:
            motor = self.fk(self.current_arm(), "gripper_motor_link")
            orientation = (motor.orientation.x, motor.orientation.y,
                           motor.orientation.z, motor.orientation.w)
            offset = quat_rotate(orientation, (0.0, self.args.tcp_y,
                                               self.args.tcp_z))
            tcp = (motor.position.x + offset[0],
                   motor.position.y + offset[1],
                   motor.position.z + offset[2])
            contact = (math.dist(tcp, before) < 0.03
                       and all(value is not None
                               and 0.003 <= value <= self.args.close_position - 0.003
                               for value in fingers))
            self.record("resume_clamped_onboard_rebar", contact,
                        tcp=[round(v, 4) for v in tcp],
                        rebar=[round(v, 4) for v in before], fingers=fingers)
            if not contact:
                raise RuntimeError("cannot confirm existing onboard grasp")
        else:
            self.cartesian_motion("move_over_onboard_rebar",
                                  [wrist_pose_for(pregrasp).pose])
            self.cartesian_motion("descend_to_onboard_rebar",
                                  [wrist_pose_for(grasp).pose])
            start_fingers = self.finger_positions()
            handle = self.send_gripper(self.args.close_position, 2.0)
            stalled = self.wait_finger_settle(start_fingers, timeout=10.0)
            result = self.wait_gripper_result(handle, timeout=2.0)
            contact = (result is not None
                       and result.status == GoalStatus.STATUS_SUCCEEDED
                       and all(now is not None and old is not None
                               and now - old >= 0.003
                               and now <= self.args.close_position - 0.003
                               for now, old in zip(stalled, start_fingers)))
            self.record("grasp_onboard_rebar", contact, fingers=stalled,
                        action_status=result.status if result else None)
            if not contact:
                raise RuntimeError("fingers did not close against the onboard rebar")

        self.remove_seated_collision_object(slot)

        # The start state overlaps the rack saddle by design: the steel is
        # resting in its curved seat and the closed fingers surround it.
        # Move straight up only 4 cm without MoveIt's collision gate, then
        # restore collision-checked motion after the bar clears the saddle.
        clear = (rebar[0], rebar[1], 0.76)
        self.cartesian_nc("clear_onboard_saddle",
                          [wrist_pose_for(clear).pose])
        cleared = self.rebar_in_base()
        free_of_saddle = cleared[2] - before[2] > 0.025
        self.record("rebar_clear_of_onboard_saddle", free_of_saddle,
                    position=[round(v, 4) for v in cleared])
        if not free_of_saddle:
            raise RuntimeError("steel did not lift clear of the saddle")

        lift = (rebar[0], rebar[1], 0.90)
        self.cartesian_motion("lift_from_onboard_slot",
                              [wrist_pose_for(lift).pose])
        after = self.rebar_in_base()
        lifted = after[2] - before[2] > 0.12
        self.record("rebar_lifted_from_onboard_slot", lifted,
                    before=[round(v, 4) for v in before],
                    after=[round(v, 4) for v in after])
        if not lifted:
            raise RuntimeError("steel did not follow the gripper out of the rack")

        self._robot_attach_publisher.publish(Bool(data=True))
        self.wait_for_robot_attachment()
        self.payload_scene(True)
        # 04_insert.py starts by moving to its verticalisation staging point.
        # Raise above the deck first so that the horizontal bar clears its rack.
        transit = (rebar[0], rebar[1], 1.05)
        self.cartesian_motion("raise_above_onboard_rack",
                              [wrist_pose_for(transit).pose])
        self.check_payload_while_parked(transit, tolerance=0.06)

        state = {
            "completed": "navigate",
            "route_xy": [list(point) for point in BASE_DRIVE_WAYPOINTS_XY],
            "target_yaw": BASE_TARGET_YAW,
            "grasp_quat": list(grasp_quat),
            "tcp_in_wrist": list(calibration["tcp_in_wrist"]),
            "lift_tcp": list(lift),
            "source": "onboard_slot",
            "slot": slot,
        }
        path = Path(self.args.state_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(state, indent=2), encoding="utf-8")
        temporary.replace(path)
        self.record("onboard_pickup_checkpoint", True, path=str(path))
        self.write_report()
        return self._passed


def main():
    parser = build_parser()
    parser.description = __doc__
    parser.add_argument("--resume-clamped", action="store_true",
                        help="resume after confirmed finger contact at the rack")
    parser.set_defaults(state_file="/tmp/rebar_onboard_tester_state.json")
    args = parser.parse_args()
    if args.onboard_slot is None:
        args.onboard_slot = 1
    if args.onboard_slot not in range(1, 5):
        parser.error("--onboard-slot must be 1, 2, 3, or 4")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = OnboardPickup(args)
    try:
        passed = node.run_pickup(args.onboard_slot)
    except Exception as exc:
        node.get_logger().error(traceback.format_exc())
        node.record("onboard_pickup_error", False, error=str(exc))
        node.write_report()
        passed = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
