#!/usr/bin/env python3
"""Validate and execute the compact rack-to-front curve from live feedback."""
import json
import math
import sys
import traceback
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents, RobotTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from compact_curve import SCHEDULE, nearest_target, retime
from rebar_tester_geometry import BASE_DRIVE_WAYPOINTS_XY, BASE_TARGET_YAW
from test_rebar_grasp import ARM_JOINTS, GRIPPER_JOINTS, quat_from_basis, quat_rotate
from test_rebar_tester_load import TesterLoadTest, build_parser

APPROACH_TCP = (-.8, -.08, 1.5)


class CompactVertical(TesterLoadTest):
    def add_nearby_devices(self):
        scene = PlanningScene(is_diff=True)
        for index, x in enumerate((2.2, 4.0), 1):
            obj = CollisionObject(id=f"compact_etm6m_{index}")
            obj.header.frame_id = "world"
            obj.operation = CollisionObject.ADD
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[1.1, .9, .9])]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, 4., .45
            pose.orientation.w = 1.
            obj.primitive_poses = [pose]
            scene.world.collision_objects.append(obj)
        response = self.call(self._scene_apply, ApplyPlanningScene.Request(scene=scene))
        if not response.success:
            raise RuntimeError("nearby equipment collision update rejected")

    def run_compact(self):
        state_path = Path(self.args.state_file)
        state = json.loads(state_path.read_text())
        if not state.get("compact_start") or state.get("slot") != 1:
            raise RuntimeError("requires fresh pickup_onboard --compact-start in slot 1")
        for client in (self._fk_client, self._scene_apply):
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError(f"service unavailable: {client.srv_name}")
        validity = self.create_client(GetStateValidity, "/check_state_validity")
        scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        for client in (validity, scene_client):
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError(f"service unavailable: {client.srv_name}")
        self.spin(2.)
        base = self.base_pose()
        parked = (math.dist(base[:2], BASE_DRIVE_WAYPOINTS_XY[-2]) < .05
                  and abs(math.atan2(math.sin(base[2] - BASE_TARGET_YAW),
                                     math.cos(base[2] - BASE_TARGET_YAW))) < .08)
        if not self.record("compact_base_at_standoff", parked, base_pose=base):
            raise RuntimeError("base is not at the clear turning waypoint")
        self.wait_for_robot_attachment()
        self.payload_scene(True)
        self.add_tester_to_scene()
        self.add_nearby_devices()
        scene_request = GetPlanningScene.Request()
        scene_request.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES |
                                                PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        scene = self.call(scene_client, scene_request).scene
        attached = [entry.object.id for entry in scene.robot_state.attached_collision_objects]
        world = [entry.id for entry in scene.world.collision_objects]
        loaded = ("carried_rebar" in attached and "onboard_rebar_rack" in attached
                  and "rebar_test_machine" in world)
        if not self.record("compact_live_collision_scene", loaded, attached=attached, world=world):
            raise RuntimeError("live collision scene is incomplete")

        start = tuple(self.current_arm()[name] for name in ARM_JOINTS)
        measured = self.rebar_in_base()
        if not self.record("compact_source_feedback", math.dist(measured, (.2927, 0, .90)) < .035,
                           joints_rad=start, rebar_base=measured, fingers=self.finger_positions()):
            raise RuntimeError("source TCP is not the lifted rack bar")
        target = nearest_target(start)
        schedule = SCHEDULE
        if self.args.schedule_file:
            params = json.loads(Path(self.args.schedule_file).read_text())
            schedule = tuple(params["schedule_parameters"])
            target = nearest_target(start, params.get("target_joints_rad", target))
        desired = self.wrist_pose_factory(state)(APPROACH_TCP, quat_from_basis((0, -1, 0), (-1, 0, 0))).pose
        fk = self.fk(dict(zip(ARM_JOINTS, target)), "wrist3_Link")
        error = math.dist((fk.position.x, fk.position.y, fk.position.z),
                          (desired.position.x, desired.position.y, desired.position.z))
        alignment = abs(sum(a*b for a, b in zip(
            (fk.orientation.x, fk.orientation.y, fk.orientation.z, fk.orientation.w),
            (desired.orientation.x, desired.orientation.y, desired.orientation.z, desired.orientation.w))))
        if not self.record("compact_target_fk", error < .003 and alignment > .99999,
                           target_joints_rad=target, position_error_m=error, alignment=alignment):
            raise RuntimeError("compact target does not match front TCP")

        rows, duration = retime(start, target, schedule)
        fingers = dict(zip(GRIPPER_JOINTS, self.finger_positions()))
        max_step = max(abs(b-a) for previous, current in zip(rows, rows[1:])
                       for a, b in zip(previous[0], current[0]))
        if max_step > math.radians(.5):
            raise RuntimeError("compact samples exceed the 0.5 degree validation spacing")
        for index, (positions, _, _) in enumerate(rows):
            values = dict(zip(ARM_JOINTS, positions))
            values.update(fingers)
            request = GetStateValidity.Request()
            request.group_name = "arm"
            request.robot_state = self.robot_state(values, gripper_open=False)
            result = self.call(validity, request, timeout=10.)
            if not result.valid:
                contacts = [(contact.contact_body_1, contact.contact_body_2) for contact in result.contacts]
                self.record("compact_fcl_path", False, sample=index, progress=index/(len(rows)-1),
                            contacts=contacts, joints_rad=positions)
                raise RuntimeError("live MoveIt collision screening rejected the compact curve")
        self.record("compact_fcl_path", True, samples=len(rows),
                    max_joint_step_deg=math.degrees(max_step), closed_fingers=fingers,
                    duration_s=duration, schedule_parameters=schedule)
        export = {"start_joints_rad": start, "target_joints_rad": target,
                  "schedule_parameters": schedule, "duration_s": duration,
                  "base_pose": base, "rows": rows}
        Path(self.args.output).with_suffix(".trajectory.json").write_text(json.dumps(export))
        # Only the freshly screened trajectory can proceed to execution.
        self.spin(.1)
        if max(abs(self.current_arm()[name] - value) for name, value in zip(ARM_JOINTS, start)) > .02:
            raise RuntimeError("arm moved after validation; fresh planning required")
        if math.dist(self.base_pose()[:2], base[:2]) > .01:
            raise RuntimeError("base moved after validation")
        if self.args.validate_only or self.args.plan_only:
            self.write_report()
            return self._passed
        if self._base_locked:
            self.release_base_lock()
        self._base_hold_target = (BASE_DRIVE_WAYPOINTS_XY[-2], BASE_TARGET_YAW)
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = list(ARM_JOINTS)
        for index, (positions, velocities, accelerations) in enumerate(rows):
            nanos = int((.2 + duration * index / (len(rows)-1)) * 1e9)
            trajectory.joint_trajectory.points.append(JointTrajectoryPoint(
                positions=positions, velocities=[v/duration for v in velocities],
                accelerations=[a/duration**2 for a in accelerations],
                time_from_start=Duration(sec=nanos//1_000_000_000, nanosec=nanos%1_000_000_000)))
        motion_started = time.monotonic()
        telemetry = []
        def capture_motion():
            try:
                telemetry.append({"elapsed_s": time.monotonic() - motion_started,
                                  "joints_rad": [self.current_arm()[name] for name in ARM_JOINTS],
                                  "rebar_base": list(self.rebar_in_base()), "base_pose": list(self.base_pose())})
            except (KeyError, RuntimeError):
                pass
        timer = self.create_timer(.1, capture_motion)
        try:
            code = self.execute(trajectory, timeout=duration + 30.)
        finally:
            self.destroy_timer(timer)
            Path(self.args.output).with_suffix(".motion.json").write_text(json.dumps(telemetry))
        if not self.record("compact_vertical_exec", code == 1, execution_code=code, duration_s=duration):
            raise RuntimeError("compact execution failed")
        self.check_payload_while_parked(APPROACH_TCP, tolerance=.035)
        axis = self.rebar_axis_in_base()
        wrist = self.fk(self.current_arm(), "wrist3_Link")
        q = (wrist.orientation.x, wrist.orientation.y, wrist.orientation.z, wrist.orientation.w)
        tool, closing = quat_rotate(q, (0, 0, 1)), quat_rotate(q, (1, 0, 0))
        valid = abs(axis[2]) > .98 and -tool[0] > .98 and -closing[1] > .98
        if not self.record("compact_front_vertical_pose", valid, rebar_base=self.rebar_in_base(),
                           axis_base=axis, tool_axis_base=tool, closing_axis_base=closing):
            raise RuntimeError("compact front vertical pose not reached")
        self.wait_for_base_lock()
        state.update(completed="verticalize_at_standoff", compact_preposition_complete=True,
                     approach_tcp_base=list(APPROACH_TCP), insert_quat=list(quat_from_basis((0,-1,0),(-1,0,0))))
        temp = state_path.with_suffix(".tmp")
        temp.write_text(json.dumps(state, indent=2))
        temp.replace(state_path)
        self.write_report()
        return self._passed


def main():
    parser = build_parser()
    parser.add_argument("--schedule-file", help="parameters replanned from actual source joints")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CompactVertical(args)
    try:
        passed = node.run_compact()
    except Exception as exc:
        node.get_logger().error(traceback.format_exc())
        node.record("compact_error", False, error=str(exc))
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
