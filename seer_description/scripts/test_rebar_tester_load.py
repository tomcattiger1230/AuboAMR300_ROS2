#!/usr/bin/env python3
"""Load a rebar into the tensile tester: grasp, drive, insert, clamp, release.

Full workflow against the lab scene (warehouse_finger_rebar_lab_demo.usda):

  1. Pre-position the tester (upper 1.88 m, lower 1.05 m, jaws open 0.16 m).
  2. Add the tester frame to the MoveIt planning scene (world boxes).
  3. Grasp the 0.6 m rebar at the source station (same logic as
     test_rebar_grasp.py, re-used through class inheritance).
  4. Carry the bar high and retracted, drive the base to (6.0, 2.9) yaw -90
     with odometry-closed-loop cmd_vel.
  5. Reorient the bar to vertical, insert it horizontally onto the grip line
     (world 6.00, 3.92, bar centre z 1.50).
  6. Close the tester jaws: lower carriage to 1.12, upper to 1.87, both
     openings to 0.024 m. With --manual-jaws, pause and let a human use
     rebar_tester_gui.py instead.
  7. Open the robot fingers, verify the bar stays on the grip line, retract.

Planning goes through the move_group services exactly like
test_rebar_grasp.py; this file only imports that module and never modifies
it. Every step appends a report entry; exit code 0 means all checks passed.
"""

import argparse
import json
import math
import sys
import time
import traceback
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Float64

sys.path.insert(0, str(Path(__file__).resolve().parent))

from test_rebar_grasp import (  # noqa: E402
    ARM_JOINTS,
    GoalStatus,
    RebarGraspTest,
    SEED_CONFIGS,
    quat_conjugate,
    quat_from_basis,
    quat_normalize,
    quat_rotate,
)
from rebar_tester_geometry import (  # noqa: E402
    APPROACH_FROM_SOUTH_Y,
    BAR_CENTER_Z,
    BAR_HOLD_Z,
    BASE_DRIVE_WAYPOINTS_XY,
    BASE_MAX_ANGULAR,
    BASE_MAX_LINEAR,
    BASE_PUBLISH_HZ,
    BASE_TARGET_YAW,
    BASE_TOLERANCE_XY,
    BASE_TOLERANCE_YAW,
    CARRY_TCP_BASE,
    GRIP_LINE_XY,
    INSERT_Y_INSET,
    JAW_CLOSE_OPENING,
    JAW_INSERT_OPENING,
    LOWER_JAW_GRIP_Z,
    UPPER_JAW_GRIP_Z,
    VERTICALIZE_TCP_BASE,
    machine_boxes,
)

TESTER_CHANNELS = ("upper_z", "lower_z", "upper_opening", "lower_opening")
STATE_TOLERANCE = {"z": 0.005, "opening": 0.004}


def quat_slerp(a, b, fraction):
    """Spherical interpolation between two quaternions (x, y, z, w)."""
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:
        b = tuple(-value for value in b)
        dot = -dot
    if dot > 0.9995:
        result = tuple(x + (y - x) * fraction for x, y in zip(a, b))
        return quat_normalize(result)
    angle = math.acos(min(1.0, dot))
    sin_angle = math.sin(angle)
    wa = math.sin((1.0 - fraction) * angle) / sin_angle
    wb = math.sin(fraction * angle) / sin_angle
    return tuple(wa * x + wb * y for x, y in zip(a, b))


def yaw_from_quat(quat):
    x, y, z, w = quat
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class TesterLoadTest(RebarGraspTest):
    def __init__(self, args):
        super().__init__(args)
        self._tester_state = {}
        self._tester_state_time = {}
        self._tester_gripped = False
        self._tester_gripped_time = 0.0
        self._robot_attached = False
        self._robot_attached_time = 0.0
        self._base_locked = False
        self._base_locked_time = 0.0
        self._robot_attach_publisher = self.create_publisher(
            Bool, "/rebar_tester/robot_attach_cmd", 10
        )
        self._base_lock_publisher = self.create_publisher(
            Bool, "/rebar_tester/base_lock_cmd", 10
        )
        self._tester_publishers = {}
        self._cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self._base_hold_target = None
        self.create_timer(0.05, self.hold_parked_base)
        for key in TESTER_CHANNELS:
            group, axis = key.split("_", 1)
            topic = f"/rebar_tester/{group}/{axis}"
            self._tester_publishers[key] = self.create_publisher(
                Float64, topic + "_cmd", 10
            )
            self.create_subscription(
                Float64,
                topic + "_state",
                self._tester_state_callback(key),
                qos_profile_sensor_data,
            )
        self._scene_apply = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.create_subscription(
            Bool, "/rebar_tester/rebar_gripped", self._on_rebar_gripped, 10
        )
        self.create_subscription(
            Bool, "/rebar_tester/robot_attached", self._on_robot_attached, 10
        )
        self.create_subscription(
            Bool, "/rebar_tester/base_locked", self._on_base_locked, 10
        )

    # ---------- tester bridge ----------

    def _tester_state_callback(self, key):
        def on_message(message):
            self._tester_state[key] = message.data
            self._tester_state_time[key] = time.monotonic()
        return on_message

    def _tester_publisher(self, key):
        return self._tester_publishers[key]

    def _on_rebar_gripped(self, message):
        self._tester_gripped = bool(message.data)
        self._tester_gripped_time = time.monotonic()

    def _on_robot_attached(self, message):
        self._robot_attached = bool(message.data)
        self._robot_attached_time = time.monotonic()

    def _on_base_locked(self, message):
        self._base_locked = bool(message.data)
        self._base_locked_time = time.monotonic()

    def wait_for_base_lock(self, timeout=10.0):
        self.stop_base()
        self._base_lock_publisher.publish(Bool(data=True))
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin(0.1)
            if (self._base_locked
                    and time.monotonic() - self._base_locked_time < 1.5):
                self.record("base_parking_brake_locked", True,
                            base_pose=[round(v, 3) for v in self.base_pose()])
                return
        self.record("base_parking_brake_locked", False)
        raise RuntimeError("Isaac did not confirm the base parking brake")

    def wait_for_robot_attachment(self, timeout=10.0):
        self._robot_attach_publisher.publish(Bool(data=True))
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin(0.1)
            if (self._robot_attached
                    and time.monotonic() - self._robot_attached_time < 1.5):
                self.record("robot_rebar_attached", True)
                return
        self.record("robot_rebar_attached", False)
        raise RuntimeError("robot grasp joint was not confirmed by Isaac")

    def wait_for_tester_grip(self, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin(0.1)
            if (self._tester_gripped
                    and time.monotonic() - self._tester_gripped_time < 1.5
                    and not self._robot_attached
                    and time.monotonic() - self._robot_attached_time < 1.5):
                self.record("tester_rebar_gripped", True,
                            robot_joint_released=True)
                return
        self.record("tester_rebar_gripped", False)
        raise RuntimeError("tester has not confirmed that it holds the rebar")

    def send_tester(self, key, value):
        self._tester_publisher(key).publish(Float64(data=value))

    def hold_parked_base(self):
        """Actively counter arm reaction torques using the wheel controller."""
        if self._base_hold_target is None:
            return
        if time.monotonic() - self._base_pose_time > 0.5:
            return
        (target_x, target_y), target_yaw = self._base_hold_target
        x, y, yaw = self.base_pose()
        dx, dy = target_x - x, target_y - y
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        yaw_error = math.atan2(
            math.sin(target_yaw - yaw), math.cos(target_yaw - yaw)
        )
        command = Twist()
        if abs(forward) > 0.005:
            command.linear.x = max(-0.3, min(0.3, 1.5 * forward))
        if abs(yaw_error) > 0.005:
            command.angular.z = max(-0.5, min(0.5, 2.0 * yaw_error))
        self._cmd_vel.publish(command)

    def wait_for_base_to_settle(self, timeout=30.0):
        """Allow the wheels to recover the parked pose after arm motion."""
        end = time.monotonic() + timeout
        last_report = 0.0
        while time.monotonic() < end:
            self.spin(0.05)
            x, y, yaw = self.base_pose()
            distance = math.hypot(
                x - BASE_DRIVE_WAYPOINTS_XY[-1][0],
                y - BASE_DRIVE_WAYPOINTS_XY[-1][1],
            )
            yaw_error = math.atan2(
                math.sin(BASE_TARGET_YAW - yaw),
                math.cos(BASE_TARGET_YAW - yaw),
            )
            if distance < 0.025 and abs(yaw_error) < 0.03:
                self.record("base_settled_for_clamping", True,
                            base_pose=[round(x, 3), round(y, 3), round(yaw, 3)])
                return
            now = time.monotonic()
            if now - last_report > 2.0:
                last_report = now
                self.get_logger().info(
                    f"settling base: {distance:.3f} m, {math.degrees(yaw_error):.1f} deg"
                )
        self.record("base_settled_for_clamping", False,
                    base_pose=[round(v, 3) for v in self.base_pose()])
        raise RuntimeError("base did not settle at the tester parking pose")

    def tester_state(self, key, max_age=1.0):
        stamp = self._tester_state_time.get(key)
        if stamp is None or time.monotonic() - stamp > max_age:
            return None
        return self._tester_state.get(key)

    def command_tester(self, key, value, timeout=180.0, tolerance=None):
        """Command one channel and wait until the state reports it.

        The tester mechanisms move at 0.08-0.12 m/s in sim time; a low
        real-time factor stretches the wall-clock wait, hence the generous
        timeout.
        """
        kind = "opening" if key.endswith("opening") else "z"
        tolerance = tolerance or STATE_TOLERANCE[kind]
        self.send_tester(key, value)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin(0.1)
            state = self.tester_state(key)
            if state is not None and abs(state - value) <= tolerance:
                return state
        raise RuntimeError(
            f"tester {key} did not reach {value} (last "
            f"{self.tester_state(key)})"
        )

    def wait_for_enter(self, prompt):
        """Block for a manual confirmation while a thread keeps ROS alive."""
        import threading

        stop = threading.Event()

        def spin_until_confirmed():
            while not stop.is_set() and rclpy.ok():
                self.spin(0.1)

        spin_thread = threading.Thread(target=spin_until_confirmed, daemon=True)
        spin_thread.start()
        try:
            input(prompt)
        finally:
            stop.set()
            spin_thread.join(timeout=2.0)

    # ---------- MoveIt scene ----------

    def add_tester_to_scene(self):
        from moveit_msgs.msg import PlanningScene

        boxes = list(machine_boxes())
        for object_id, indices in (
            ("rebar_test_machine", (0, len(boxes) - 1)),
            ("rebar_tester_console", (len(boxes) - 1, len(boxes))),
        ):
            scene = PlanningScene(is_diff=True)
            collision = CollisionObject()
            collision.id = object_id
            collision.header.frame_id = "world"
            collision.operation = CollisionObject.ADD
            for index in range(*indices):
                name, center, size = boxes[index]
                primitive = SolidPrimitive(type=SolidPrimitive.BOX)
                primitive.dimensions = list(size)
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = center
                collision.primitives.append(primitive)
                collision.primitive_poses.append(pose)
            scene.world.collision_objects = [collision]
            # MoveIt may advertise the service before the world TF frame
            # arrives; stable object ids make retries safe.
            for attempt in range(1, 11):
                future = self._scene_apply.call_async(
                    ApplyPlanningScene.Request(scene=scene)
                )
                end = time.monotonic() + 5.0
                while not future.done() and time.monotonic() < end:
                    rclpy.spin_once(self, timeout_sec=0.05)
                if future.done() and future.result() and future.result().success:
                    break
                if attempt == 10:
                    raise RuntimeError(f"MoveIt rejected {object_id}")
                self.get_logger().warning(
                    f"planning scene not ready; retrying ({attempt}/10)"
                )
        self.record(
            "tester_planning_scene",
            True,
            boxes=[name for name, _, _ in boxes],
        )

    def detach_payload(self):
        from moveit_msgs.msg import AttachedCollisionObject, PlanningScene

        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        attached = AttachedCollisionObject()
        attached.object.id = "carried_rebar"
        attached.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects = [attached]
        if not self._scene_apply.wait_for_service(timeout_sec=10):
            raise RuntimeError("ApplyPlanningScene unavailable")
        result = self.call(
            self._scene_apply, ApplyPlanningScene.Request(scene=scene)
        )
        if not result.success:
            raise RuntimeError("payload detach rejected")
        self._payload_monitor = False

    def cartesian_motion_min(self, label, poses, min_fraction=0.93,
                             allow_joint_fallback=False):
        """Cartesian move tolerating an incomplete path (the next stage's
        endpoint corrects the residual); used for long approach translates."""
        response = self.cartesian(self.current_arm(), poses)
        if response.fraction < min_fraction:
            self.record(label + "_cartesian", allow_joint_fallback,
                        fraction=response.fraction,
                        planning_code=response.error_code.val,
                        fallback="joint" if allow_joint_fallback else None)
            if allow_joint_fallback:
                return False
            raise RuntimeError(f"{label}: incomplete Cartesian path")
        self.record(label, True, fraction=round(response.fraction, 4),
                    planning_code=response.error_code.val)
        if self.args.plan_only:
            return True
        self.run_motion(label + "_exec", response, "solution")
        return True

    def joint_fallback(self, label, target_pose_stamped):
        """Reach an exact pose via joint-space planning when the Cartesian
        solver stalls on an IK branch switch (single-point IK still solves,
        the corridor is verified clear, and the short translate of a
        vertical bar is safe without a straight-line guarantee)."""
        from moveit_msgs.srv import GetPositionIK

        current = self.current_arm()

        def unwind(angle, reference):
            return reference + math.atan2(
                math.sin(angle - reference), math.cos(angle - reference)
            )

        candidates = []
        for seed in SEED_CONFIGS + [tuple(current[j] for j in ARM_JOINTS)]:
            request = GetPositionIK.Request()
            request.ik_request.group_name = "arm"
            request.ik_request.ik_link_name = "wrist3_Link"
            request.ik_request.robot_state = self.robot_state(
                dict(zip(ARM_JOINTS, seed))
            )
            request.ik_request.pose_stamped = target_pose_stamped
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout.sec = 2
            result = self.call(self._ik_client, request, timeout=30)
            if result.error_code.val != 1:
                continue
            candidate = dict(
                zip(
                    result.solution.joint_state.name,
                    result.solution.joint_state.position,
                )
            )
            if not all(j in candidate for j in ARM_JOINTS):
                continue
            candidate = {j: unwind(candidate[j], current[j]) for j in ARM_JOINTS}
            cost = max(abs(candidate[j] - current[j]) for j in ARM_JOINTS)
            if all(max(abs(candidate[j] - other[j]) for j in ARM_JOINTS) > 0.02
                   for _, other in candidates):
                candidates.append((cost, candidate))
        if not candidates:
            self.record(label + "_ik", False)
            raise RuntimeError(f"{label}: no IK solution for the joint fallback")
        # The closest IK branch can fold a finger into the upper arm halfway
        # through OMPL's interpolated path. Try the other collision-checked
        # branches before declaring this insertion waypoint unreachable.
        for attempt, (_, solution) in enumerate(
            sorted(candidates, key=lambda item: item[0]), 1
        ):
            plan = self.joint_plan(current, solution)
            if plan.error_code.val == 1:
                self.record(label + "_joint_branch", True, attempt=attempt,
                            candidates=len(candidates))
                self.run_motion(label + "_joint", plan)
                return
        self.record(label + "_joint_branch", False, candidates=len(candidates))
        raise RuntimeError(f"{label}: all IK branches failed collision-checked planning")

    def cartesian_nc(self, label, poses):
        """Cartesian move without collision checking - fallback for the
        insertion corridor, whose geometry (jaws open 0.16 m around a
        0.024 m bar) is verified safe but confuses the planner."""
        from moveit_msgs.srv import GetCartesianPath

        client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        request = GetCartesianPath.Request()
        request.header.frame_id = "base_footprint"
        request.group_name = "arm"
        request.link_name = "wrist3_Link"
        request.start_state = self.robot_state(self.current_arm())
        request.waypoints = poses
        request.max_step = 0.005
        request.avoid_collisions = False
        if hasattr(request, "max_velocity_scaling_factor"):
            request.max_velocity_scaling_factor = 0.15
            request.max_acceleration_scaling_factor = 0.15
        response = self.call(client, request)
        if response.fraction < 0.97:
            self.record(label, False, fraction=response.fraction,
                        planning_code=response.error_code.val, collision_free=False)
            raise RuntimeError(f"{label}: path incomplete even without collision checks")
        self.record(label, True, fraction=round(response.fraction, 4),
                    planning_code=response.error_code.val, collision_checked=False)
        if self.args.plan_only:
            return
        self.run_motion(label + "_exec", response, "solution")

    def cartesian_slow(self, label, poses):
        """Slow Cartesian move (half the usual scaling) for stages where the
        clamped payload must not be swung around, e.g. rotating the bar
        upright: the V-edge grip sheds the bar if swung too fast."""
        from moveit_msgs.srv import GetCartesianPath

        client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        request = GetCartesianPath.Request()
        request.header.frame_id = "base_footprint"
        request.group_name = "arm"
        request.link_name = "wrist3_Link"
        request.start_state = self.robot_state(self.current_arm())
        request.waypoints = poses
        request.max_step = 0.004
        request.avoid_collisions = True
        if hasattr(request, "max_velocity_scaling_factor"):
            request.max_velocity_scaling_factor = 0.06
            request.max_acceleration_scaling_factor = 0.06
        response = self.call(client, request)
        if response.fraction < 0.97:
            self.record(label, False, fraction=response.fraction,
                        planning_code=response.error_code.val)
            raise RuntimeError(f"{label}: incomplete path")
        self.record(label, True, fraction=round(response.fraction, 4))
        if self.args.plan_only:
            return
        self.run_motion(label + "_exec", response, "solution")

    # ---------- base driving ----------

    def base_pose(self, max_age=1.0):
        if time.monotonic() - self._base_pose_time > max_age:
            raise RuntimeError("odom -> base_footprint feedback stale")
        return self._base_position[0], self._base_position[1], yaw_from_quat(
            self._base_quat
        )

    def stop_base(self):
        for _ in range(5):
            self._cmd_vel.publish(Twist())
            self.spin(0.02)
            time.sleep(0.02)

    def drive_to(self, target_xy, target_yaw, label):
        """Odometry-closed-loop drive: rotate, translate, final rotate.

        Headless lab scenes can run well below real-time factor, so the
        timeout is generous and backed by a progress watchdog instead of a
        fixed deadline alone.
        """
        deadline = time.monotonic() + 1500.0
        last_progress_position = None
        stalled_since = None
        period = 1.0 / BASE_PUBLISH_HZ
        command = Twist()
        last_report = 0.0
        try:
            while rclpy.ok():
                self.spin(0.02)
                x, y, yaw = self.base_pose()
                dx, dy = target_xy[0] - x, target_xy[1] - y
                distance = math.hypot(dx, dy)
                heading = math.atan2(dy, dx)
                yaw_error = math.atan2(
                    math.sin(target_yaw - yaw), math.cos(target_yaw - yaw)
                )
                if (
                    distance < BASE_TOLERANCE_XY
                    and abs(yaw_error) < BASE_TOLERANCE_YAW
                ):
                    self.record(
                        f"base_{label}",
                        True,
                        position=(round(x, 3), round(y, 3)),
                        yaw=round(yaw, 3),
                        distance_error=round(distance, 4),
                    )
                    return
                if time.monotonic() > deadline:
                    raise RuntimeError(
                        f"base_{label}: drive timeout at "
                        f"({x:.2f}, {y:.2f}, {yaw:.2f})"
                    )
                # Fail fast if nothing moves for a long stretch (stuck);
                # slow-but-moving is fine at low real-time factor.
                if last_progress_position is None:
                    last_progress_position = (x, y, yaw)
                    stalled_since = time.monotonic()
                elif (
                    math.hypot(
                        x - last_progress_position[0], y - last_progress_position[1]
                    )
                    > 0.05
                    or abs(yaw - last_progress_position[2]) > 0.05
                ):
                    last_progress_position = (x, y, yaw)
                    stalled_since = time.monotonic()
                elif time.monotonic() - stalled_since > 90.0:
                    raise RuntimeError(
                        f"base_{label}: no progress for 90 s at "
                        f"({x:.2f}, {y:.2f}, {yaw:.2f})"
                    )
                if distance >= BASE_TOLERANCE_XY:
                    heading_error = math.atan2(
                        math.sin(heading - yaw), math.cos(heading - yaw)
                    )
                    if abs(heading_error) > 2.6 and distance < 1.2:
                        # Goal is right behind and close: reverse toward it
                        # (steering mirrored so the rear axle leads).
                        command.linear.x = -min(BASE_MAX_LINEAR, 1.2 * distance)
                        command.angular.z = max(
                            -BASE_MAX_ANGULAR,
                            min(BASE_MAX_ANGULAR, -1.2 * heading_error),
                        )
                    elif abs(heading_error) > 0.35:
                        # Rotate in place toward the goal heading first.
                        command.linear.x = 0.0
                        command.angular.z = max(
                            -BASE_MAX_ANGULAR,
                            min(BASE_MAX_ANGULAR, 1.6 * heading_error),
                        )
                    else:
                        command.linear.x = max(
                            -BASE_MAX_LINEAR,
                            min(BASE_MAX_LINEAR, 1.2 * distance),
                        )
                        command.angular.z = max(
                            -BASE_MAX_ANGULAR,
                            min(BASE_MAX_ANGULAR, 1.2 * heading_error),
                        )
                else:
                    command.linear.x = 0.0
                    command.angular.z = max(
                        -BASE_MAX_ANGULAR, min(BASE_MAX_ANGULAR, 1.6 * yaw_error)
                    )
                self._cmd_vel.publish(command)
                now = time.monotonic()
                if now - last_report > 2.0:
                    last_report = now
                    self.get_logger().info(
                        f"drive {label}: at ({x:.2f},{y:.2f},{math.degrees(yaw):.0f}deg) "
                        f"remaining {distance:.2f} m"
                    )
                time.sleep(period)
        finally:
            self.stop_base()

    def check_payload_while_parked(self, expected_base, tolerance=0.05):
        """Confirm the bar is still where the gripper left it (base frame)."""
        actual = self.rebar_in_base()
        offset = math.dist(actual, expected_base)
        self.record(
            "payload_retained_while_driving",
            offset < tolerance,
            actual=[round(v, 3) for v in actual],
            expected=list(expected_base),
            offset_m=round(offset, 3),
        )
        if offset >= tolerance:
            raise RuntimeError("rebar slipped out of the gripper while driving")

    # ---------- main sequence ----------

    def run(self):
        args = self.args
        for client in (
            self._plan_client,
            self._fk_client,
            self._ik_client,
            self._cartesian_client,
        ):
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError(f"service unavailable: {client.srv_name}")
        if not self._execute_client.wait_for_server(timeout_sec=10):
            raise RuntimeError("execute_trajectory action unavailable")
        if not self._gripper_client.wait_for_server(timeout_sec=10):
            raise RuntimeError(f"action unavailable: {args.action_name}")
        self.spin(2.0)

        # --- 1. tester bridge alive + pre-position for insertion ---------
        for key in TESTER_CHANNELS:
            if self.tester_state(key) is None:
                self.spin(1.0)
        missing = [key for key in TESTER_CHANNELS if self.tester_state(key) is None]
        self.record("tester_bridge_alive", not missing, missing=missing)
        if missing:
            raise RuntimeError(
                "tester state topics unavailable; start the lab demo with "
                "--rebar-tester and matching ROS_DOMAIN_ID"
            )
        if not args.plan_only and not args.skip_preposition:
            self.command_tester("upper_z", 1.88)
            self.command_tester("lower_z", 1.05)
            self.command_tester("upper_opening", JAW_INSERT_OPENING)
            self.command_tester("lower_opening", JAW_INSERT_OPENING)
            self.record(
                "tester_prepositioned",
                True,
                upper_z=self.tester_state("upper_z"),
                lower_z=self.tester_state("lower_z"),
            )

        # --- 2. machine frame into the MoveIt planning scene -------------
        self.add_tester_to_scene()

        # --- 3. grasp the rebar at the source station --------------------
        grasp = self.grasp_at_station()
        if args.plan_only:
            self.check_insertion_reachability(grasp)
            self.write_report()
            return self._passed

        # Physical finger contact and the initial lift have already been
        # verified. Add a temporary simulated joint for transport and rotation.
        self.wait_for_robot_attachment()

        # --- 4. carry pose: high and retracted ---------------------------
        # Raising straight above the station exceeds the arm envelope
        # (wrist radius ~1.04 m there); leave sideways at lift height
        # first, follow the verified low route around the chassis, then
        # climb to transport height once retracted close to the body.
        wrist_pose_for, grasp_quat = grasp["wrist_pose_for"], grasp["grasp_quat"]
        self.payload_scene(True)
        lift_tcp = grasp["lift_tcp"]
        # Raise straight to the proven 0.90 m clearance above the saddle,
        # then one direct diagonal to the body side (the horizontal bar
        # sweeps above the deck/rack at 0.90 m with clear headroom; the
        # two-leg route around the chassis is IK-borderline in the
        # closing-Y orientation).
        carry_stages = [
            (lift_tcp[0], lift_tcp[1], 0.90),
            (CARRY_TCP_BASE[0], CARRY_TCP_BASE[1], 0.90),
            CARRY_TCP_BASE,
        ]
        for index, tcp in enumerate(carry_stages, 1):
            self.cartesian_motion(
                f"carry_stage_{index}", [wrist_pose_for(tcp, grasp_quat).pose]
            )
            self.check_payload_while_parked(tcp, tolerance=0.05)

        # --- 5. drive to the machine --------------------------------------
        # Intermediate legs steer toward the next waypoint; the turning
        # waypoint (6.0, 2.6) and the parking spot both command the final
        # south-facing yaw, so the sweep-heavy 180 deg turn happens well
        # clear of the cabinet and the last 0.4 m is a short reverse.
        waypoints = BASE_DRIVE_WAYPOINTS_XY
        for index in range(1, len(waypoints)):
            target = waypoints[index]
            if index >= len(waypoints) - 2:
                yaw_target = BASE_TARGET_YAW
            else:
                following = waypoints[index + 1]
                yaw_target = math.atan2(
                    following[1] - target[1], following[0] - target[0]
                )
            label = (
                "turn_south" if index == len(waypoints) - 2
                else f"leg_{index}" if index < len(waypoints) - 1
                else "park_at_tester"
            )
            self.drive_to(target, yaw_target, label)
            self.check_payload_while_parked(CARRY_TCP_BASE)

        # --- 6. reorient the bar to vertical, then insert -----------------
        x, y, yaw = self.base_pose()
        self.record(
            "base_parked_at_tester",
            True,
            position=(round(x, 3), round(y, 3)),
            yaw_deg=round(math.degrees(yaw), 1),
        )
        self._base_hold_target = (BASE_DRIVE_WAYPOINTS_XY[-1], BASE_TARGET_YAW)
        # Insertion orientation: wrist Z points north (+Y world) toward the
        # machine, wrist X stays world X, so motor Y (the bar) is vertical.
        insert_quat = quat_from_basis((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0))

        # Reorient: first translate to the staging point keeping the bar
        # horizontal, then rotate upright in place (the sweep plane stays
        # clear of the shoulder at this distance).
        self.cartesian_motion(
            "move_to_verticalize_staging",
            [wrist_pose_for(VERTICALIZE_TCP_BASE, grasp_quat).pose],
        )
        reorient_poses = []
        for index in range(1, 25):
            fraction = index / 24.0
            orientation = quat_slerp(grasp_quat, insert_quat, fraction)
            reorient_poses.append(
                wrist_pose_for(VERTICALIZE_TCP_BASE, orientation).pose
            )
        self.cartesian_slow("reorient_to_vertical", reorient_poses)
        axis = self.rebar_axis_in_base()
        vertical = abs(axis[2]) > 0.95
        self.record("rebar_vertical", vertical, axis_in_base=list(axis))
        if not vertical:
            raise RuntimeError("bar did not end up vertical after reorientation")

        # Sample the current pose after reorientation: wheel feedback keeps
        # the base parked without overconstraining the physical arm.
        self.spin(0.2)
        self.record("base_after_verticalization", True,
                    base_pose=[round(v, 3) for v in self.base_pose()])
        base_position, base_quat = self._base_position, self._base_quat

        def world_to_base(world_xyz):
            relative = tuple(
                a - b for a, b in zip(world_xyz, base_position)
            )
            return quat_rotate(quat_conjugate(base_quat), relative)

        grip_base = world_to_base(
            (GRIP_LINE_XY[0], GRIP_LINE_XY[1] - INSERT_Y_INSET, BAR_HOLD_Z)
        )
        preinsert_base = world_to_base(
            (GRIP_LINE_XY[0], GRIP_LINE_XY[1] - 0.30, BAR_HOLD_Z)
        )

        # This diagonal transition's Cartesian solver stops near its end at
        # the 1.50 m bar height. Follow its verified collision-free prefix;
        # the next short insertion target closes the remaining distance.
        self.cartesian_motion_min(
            "insert_preposition", [wrist_pose_for(preinsert_base, insert_quat).pose],
            min_fraction=0.80,
        )
        # Two short pushes instead of one long one: the KDL chain solves
        # more reliably over short segments near the workspace edge.
        mid_base = world_to_base(
            (GRIP_LINE_XY[0], GRIP_LINE_XY[1] - 0.15, BAR_HOLD_Z)
        )
        for label, tcp in (("insert_midway", mid_base), ("insert_to_gripline", grip_base)):
            pose = wrist_pose_for(tcp, insert_quat).pose
            if label == "insert_to_gripline":
                # The last ~2 cm can exceed the IK envelope. The jaw window
                # accepts this collision-free prefix, and the measured bar
                # alignment gate below decides whether clamping is safe.
                self.cartesian_motion_min(label, [pose], min_fraction=0.80)
                continue
            if not self.cartesian_motion_min(
                label, [pose], min_fraction=0.97, allow_joint_fallback=True
            ):
                # The Cartesian solver stalls on an IK branch switch here;
                # reach the same pose through joint-space planning instead.
                self.joint_fallback(label, wrist_pose_for(tcp, insert_quat))

        self.wait_for_base_to_settle()
        inserted = self.rebar_position()
        axis = self.rebar_axis_in_base()
        aligned = (
            abs(inserted[0] - GRIP_LINE_XY[0]) < 0.035
            and abs(inserted[1] - GRIP_LINE_XY[1]) < 0.055
            and abs(inserted[2] - BAR_CENTER_Z) < 0.08
            and abs(axis[2]) > 0.95
        )
        self.record(
            "rebar_aligned_in_jaws", aligned,
            position=[round(value, 3) for value in inserted],
            axis_in_base=[round(value, 3) for value in axis],
            base_pose=[round(value, 3) for value in self.base_pose()],
        )
        if not aligned:
            raise RuntimeError("rebar is outside the tester jaw grip window")

        # --- 7. clamp with the tester jaws --------------------------------
        if args.manual_jaws:
            self.get_logger().warning(
                "manual jaw mode: use rebar_tester_gui.py to close the jaws "
                "(lower_z 1.12, upper_z 1.87, openings 0.024), then press Enter"
            )
            self.wait_for_enter("闭合抱爪后按回车继续…")
            lower = self.tester_state("lower_z", max_age=5.0)
            upper = self.tester_state("upper_z", max_age=5.0)
            openings = (
                self.tester_state("upper_opening", max_age=5.0),
                self.tester_state("lower_opening", max_age=5.0),
            )
            clamped = (
                lower is not None and abs(lower - LOWER_JAW_GRIP_Z) < 0.02
                and upper is not None and abs(upper - UPPER_JAW_GRIP_Z) < 0.02
                and all(
                    value is not None and abs(value - JAW_CLOSE_OPENING) < 0.01
                    for value in openings
                )
            )
            self.record(
                "tester_jaws_clamped_manual", clamped,
                lower_z=lower, upper_z=upper, openings=openings,
            )
            if not clamped:
                raise RuntimeError("tester jaws are not at the requested clamp position")
        else:
            self.command_tester("lower_z", LOWER_JAW_GRIP_Z)
            self.command_tester("upper_z", UPPER_JAW_GRIP_Z)
            lower_open = self.command_tester(
                "lower_opening", JAW_CLOSE_OPENING, tolerance=0.004
            )
            upper_open = self.command_tester(
                "upper_opening", JAW_CLOSE_OPENING, tolerance=0.004
            )
            self.record(
                "tester_jaws_clamped",
                True,
                lower_z=self.tester_state("lower_z"),
                upper_z=self.tester_state("upper_z"),
                openings=(upper_open, lower_open),
            )
        # --- 8. release and verify the bar stays on the grip line ---------
        self.wait_for_tester_grip()
        self.detach_payload()
        handle = self.send_gripper(0.0, 1.5)
        wrapped = self.wait_gripper_result(handle)
        self.spin(0.5)
        opened = self.finger_positions()
        released = (
            wrapped is not None
            and wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and all(value is not None and abs(value) < 0.003 for value in opened)
        )
        self.record(
            "robot_fingers_released",
            released,
            fingers=opened,
        )
        if not released:
            raise RuntimeError("robot fingers did not open after tester clamping")
        self.spin(2.0)
        held = self.rebar_position()
        stayed = (
            abs(held[0] - GRIP_LINE_XY[0]) < 0.03
            and abs(held[1] - GRIP_LINE_XY[1]) < 0.05
            and abs(held[2] - BAR_CENTER_Z) < 0.08
        )
        self.record(
            "rebar_held_by_tester",
            stayed,
            position=[round(v, 3) for v in held],
            expected=[GRIP_LINE_XY[0], GRIP_LINE_XY[1], BAR_CENTER_Z],
        )
        if not stayed:
            self.write_report()
            raise RuntimeError("rebar was not retained by the tester jaws")

        # --- 9. retract the arm --------------------------------------------
        self.cartesian_motion(
            "retract_preposition", [wrist_pose_for(preinsert_base, insert_quat).pose]
        )
        self.cartesian_motion(
            "retract_to_staging",
            [wrist_pose_for(VERTICALIZE_TCP_BASE, insert_quat).pose],
        )
        park = dict(zip(ARM_JOINTS, (0.0, -0.35, 0.6, 0.0, 0.35, 0.0)))
        plan = self.joint_plan(self.current_arm(), park)
        self.run_motion("arm_parked", plan)

        self.spin(1.0)
        final_bar = self.rebar_position()
        final_held = (
            abs(final_bar[0] - GRIP_LINE_XY[0]) < 0.03
            and abs(final_bar[1] - GRIP_LINE_XY[1]) < 0.05
            and abs(final_bar[2] - BAR_CENTER_Z) < 0.08
            and self._tester_gripped
            and time.monotonic() - self._tester_gripped_time < 1.5
        )
        self.record(
            "rebar_stable_after_arm_retract", final_held,
            position=[round(v, 3) for v in final_bar],
            tester_gripped=self._tester_gripped,
        )
        if not final_held:
            raise RuntimeError("rebar was lost after the arm retracted")

        self._base_hold_target = None
        self.stop_base()

        self.write_report()
        return self._passed

    # ---------- grasp phase (same logic as test_rebar_grasp.run steps 1-8) --

    def grasp_at_station(self):
        args = self.args
        rebar_center = (args.rebar_x, args.rebar_y, args.rebar_z)

        observed = self.rebar_position()
        on_station = math.dist(observed, rebar_center) < 0.03
        self.record(
            "rebar_pose_feedback",
            on_station,
            position=list(observed),
        )
        if not on_station:
            raise RuntimeError("rebar is not at the source station pickup pose")

        handle = self.send_gripper(0.0, 1.2)
        wrapped = self.wait_gripper_result(handle)
        opened = self.finger_positions()
        self.record(
            "open_gripper",
            wrapped is not None
            and wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and all(value is not None and abs(value) < 0.0015 for value in opened),
            fingers_opened=opened,
        )

        seed_values = seed_wrist = None
        best_distance = None
        for candidate in SEED_CONFIGS:
            values = dict(zip(ARM_JOINTS, candidate))
            try:
                pose = self.fk(values, "wrist3_Link")
            except RuntimeError:
                continue
            distance = math.dist(
                (pose.position.x, pose.position.y, pose.position.z), rebar_center
            )
            if best_distance is None or distance < best_distance:
                best_distance, seed_values, seed_wrist = distance, values, pose
        if seed_values is None:
            raise RuntimeError("no seed pose produced FK")

        motor_pose = self.fk(seed_values, "gripper_motor_link")
        wrist_position = (
            seed_wrist.position.x, seed_wrist.position.y, seed_wrist.position.z
        )
        wrist_quat = (
            seed_wrist.orientation.x, seed_wrist.orientation.y,
            seed_wrist.orientation.z, seed_wrist.orientation.w,
        )
        motor_quat = (
            motor_pose.orientation.x, motor_pose.orientation.y,
            motor_pose.orientation.z, motor_pose.orientation.w,
        )
        offset = quat_rotate(
            motor_quat, (0.0, args.tcp_y, args.tcp_z)
        )
        motor_position = (
            motor_pose.position.x + offset[0],
            motor_pose.position.y + offset[1],
            motor_pose.position.z + offset[2],
        )
        relative = tuple(t - w for t, w in zip(motor_position, wrist_position))
        tcp_in_wrist = quat_rotate(quat_conjugate(wrist_quat), relative)
        self.record(
            "tcp_calibration", True,
            seed={k: round(v, 4) for k, v in seed_values.items()},
            tcp_in_wrist=[round(v, 4) for v in tcp_in_wrist],
        )

        closing_axis = {
            "y": (0.0, 1.0, 0.0), "-y": (0.0, -1.0, 0.0),
            "x": (1.0, 0.0, 0.0), "-x": (-1.0, 0.0, 0.0),
        }[args.wrist_x_axis]
        grasp_quat = quat_from_basis(closing_axis, (0.0, 0.0, -1.0))

        def wrist_pose_for(tcp_world_target, orientation=grasp_quat):
            rotated = quat_rotate(orientation, tcp_in_wrist)
            position = tuple(t - r for t, r in zip(tcp_world_target, rotated))
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = position
            pose.orientation.x = orientation[0]
            pose.orientation.y = orientation[1]
            pose.orientation.z = orientation[2]
            pose.orientation.w = orientation[3]
            stamped = PoseStamped()
            stamped.header.frame_id = "base_footprint"
            stamped.pose = pose
            return stamped

        pregrasp_tcp = (
            rebar_center[0], rebar_center[1], rebar_center[2] + args.approach_height
        )
        current = self.current_arm()

        def unwind(angle, reference):
            return reference + math.atan2(
                math.sin(angle - reference), math.cos(angle - reference)
            )

        pregrasp_joints = None
        best_cost = None
        for seed in SEED_CONFIGS + [tuple(current[j] for j in ARM_JOINTS)]:
            request = GetPositionIK.Request()
            request.ik_request.group_name = "arm"
            request.ik_request.ik_link_name = "wrist3_Link"
            request.ik_request.robot_state = self.robot_state(
                dict(zip(ARM_JOINTS, seed))
            )
            request.ik_request.pose_stamped = wrist_pose_for(pregrasp_tcp)
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout.sec = 2
            result = self.call(self._ik_client, request, timeout=30)
            if result.error_code.val != 1:
                continue
            solution = dict(
                zip(
                    result.solution.joint_state.name,
                    result.solution.joint_state.position,
                )
            )
            if not all(joint in solution for joint in ARM_JOINTS):
                continue
            solution = {j: unwind(solution[j], current[j]) for j in ARM_JOINTS}
            cost = max(abs(solution[j] - current[j]) for j in ARM_JOINTS)
            cost += 2.0 * abs(solution["shoulder_joint"])
            if best_cost is None or cost < best_cost:
                best_cost, pregrasp_joints = cost, solution
        if pregrasp_joints is None:
            self.record("pregrasp_ik", False)
            raise RuntimeError("pre-grasp IK failed for every seed")
        self.record(
            "pregrasp_ik", True,
            joints={k: round(v, 4) for k, v in pregrasp_joints.items()},
        )

        plan = self.joint_plan(self.current_arm(), pregrasp_joints)
        self.run_motion("approach_pregrasp", plan)

        descend_start = pregrasp_joints if self.args.plan_only else self.current_arm()
        descend = self.cartesian(
            descend_start, [wrist_pose_for(rebar_center).pose]
        )
        entry = {"fraction": descend.fraction, "code": descend.error_code.val}
        if descend.error_code.val != 1 or descend.fraction < 0.99:
            self.record("descend_to_grasp", False, **entry)
            raise RuntimeError(f"descend failed: {entry}")
        if self.args.plan_only:
            self.record("descend_to_grasp", True, **entry)
            return {
                "wrist_pose_for": wrist_pose_for,
                "grasp_quat": grasp_quat,
                "lift_tcp": None,
            }
        entry["execution_code"] = self.execute(descend.solution)
        self.record("descend_to_grasp", entry["execution_code"] == 1, **entry)
        if entry["execution_code"] != 1:
            raise RuntimeError("descend execution failed")

        close_start = self.finger_positions()
        close_handle = self.send_gripper(args.close_position, 2.0)
        stalled = self.wait_finger_settle(close_start, timeout=10.0)
        preload = self.wait_gripper_result(close_handle, timeout=2.0)
        valid = all(
            value is not None and start is not None
            and value - start >= 0.003
            and value <= args.close_position - 0.003
            for value, start in zip(stalled, close_start)
        ) and preload is not None and (
            preload.status == GoalStatus.STATUS_SUCCEEDED
        )
        self.record(
            "grasp_close_on_rebar", valid, fingers_stalled=stalled
        )
        if not valid:
            self.write_report()
            raise RuntimeError("fingers did not stall on the rebar")
        grasp_stall = stalled

        lift_tcp = (
            rebar_center[0], rebar_center[1], rebar_center[2] + args.lift_height
        )
        lift = self.cartesian(self.current_arm(), [wrist_pose_for(lift_tcp).pose])
        entry = {"fraction": lift.fraction, "code": lift.error_code.val}
        if lift.error_code.val != 1 or lift.fraction < 0.99:
            self.record("lift", False, **entry)
            raise RuntimeError(f"lift failed: {entry}")
        entry["execution_code"] = self.execute(lift.solution)
        self.record("lift", entry["execution_code"] == 1, **entry)
        if entry["execution_code"] != 1:
            raise RuntimeError("lift execution failed")

        time.sleep(0.5)
        self.spin(1.0)
        retained = self.finger_positions()
        holding = all(
            value is not None and abs(value - stall) < 0.004
            for value, stall in zip(retained, grasp_stall)
        )
        self.record(
            "payload_retained_after_lift", holding, fingers=retained
        )
        if not holding:
            self.write_report()
            raise RuntimeError("fingers did not retain the payload after lift")

        return {
            "wrist_pose_for": wrist_pose_for,
            "grasp_quat": grasp_quat,
            "lift_tcp": lift_tcp,
        }

    # ---------- plan-only reachability of the insertion chain --------------

    def check_insertion_reachability(self, grasp):
        """IK-check carry/reorient/insert poses at the planned final pose."""
        base_xy = BASE_DRIVE_WAYPOINTS_XY[-1]
        base_quat = quat_from_yaw(BASE_TARGET_YAW)

        def world_to_base(world_xyz):
            relative = tuple(a - b for a, b in zip(world_xyz, (base_xy[0], base_xy[1], 0.0)))
            return quat_rotate(quat_conjugate(base_quat), relative)

        insert_quat = quat_from_basis((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
        tcp_in_wrist = (0.0, 0.0, self.args.tcp_z)
        targets = {
            "carry": (CARRY_TCP_BASE, quat_slerp(grasp["grasp_quat"], insert_quat, 0.0)),
            # Arriving at the staging point still wrist-down is its own
            # reachability case (the wrist sits above the TCP there).
            "staging_arrival": (VERTICALIZE_TCP_BASE, grasp["grasp_quat"]),
            "verticalize": (VERTICALIZE_TCP_BASE, insert_quat),
            "preinsert": (
                world_to_base((GRIP_LINE_XY[0], GRIP_LINE_XY[1] - 0.30, BAR_HOLD_Z)),
                insert_quat,
            ),
            "insert": (
                world_to_base(
                    (GRIP_LINE_XY[0], GRIP_LINE_XY[1] - INSERT_Y_INSET, BAR_HOLD_Z)
                ),
                insert_quat,
            ),
        }
        for label, (tcp_base, orientation) in targets.items():
            rotated = quat_rotate(orientation, tcp_in_wrist)
            pose = Pose()
            pose.position.x = tcp_base[0] - rotated[0]
            pose.position.y = tcp_base[1] - rotated[1]
            pose.position.z = tcp_base[2] - rotated[2]
            pose.orientation.x, pose.orientation.y = orientation[0], orientation[1]
            pose.orientation.z, pose.orientation.w = orientation[2], orientation[3]
            stamped = PoseStamped()
            stamped.header.frame_id = "base_footprint"
            stamped.pose = pose
            solved = False
            # A single KDL seed often misses solutions at the edge of the
            # workspace; try every seed like the grasp-phase IK does.
            for seed in SEED_CONFIGS:
                request = GetPositionIK.Request()
                request.ik_request.group_name = "arm"
                request.ik_request.ik_link_name = "wrist3_Link"
                request.ik_request.robot_state = self.robot_state(
                    dict(zip(ARM_JOINTS, seed))
                )
                request.ik_request.pose_stamped = stamped
                request.ik_request.avoid_collisions = False
                request.ik_request.timeout.sec = 2
                result = self.call(self._ik_client, request, timeout=30)
                if result.error_code.val == 1:
                    solved = True
                    break
            self.record(
                f"ik_reachable_{label}", solved,
                tcp_base=[round(v, 3) for v in tcp_base],
            )
            if not solved:
                raise RuntimeError(f"insertion chain unreachable at {label}")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--action-name", default="/aubo_arm_controller/follow_joint_trajectory")
    # Station defaults match the lab scene source rebar (0.6 m bar).
    parser.add_argument("--rebar-x", type=float, default=-1.15)
    parser.add_argument("--rebar-y", type=float, default=0.0)
    parser.add_argument("--rebar-z", type=float, default=0.522)
    parser.add_argument("--diameter", type=float, default=0.024)
    parser.add_argument("--length", type=float, default=0.6)
    parser.add_argument("--open-gap", type=float, default=0.0464)
    parser.add_argument("--close-position", type=float, default=0.0285)
    parser.add_argument("--wrist-x-axis", choices=["y", "-y", "x", "-x"], default="y")
    parser.add_argument("--approach-height", type=float, default=0.15)
    parser.add_argument("--lift-height", type=float, default=0.18)
    parser.add_argument("--tcp-y", type=float, default=0.0)
    parser.add_argument("--tcp-z", type=float, default=0.16)
    parser.add_argument("--release-dx", type=float, default=0.0)
    parser.add_argument("--release-dy", type=float, default=0.18)
    parser.add_argument("--clearance-height", type=float, default=0.90)
    parser.add_argument("--occupied-slots", default="")
    parser.add_argument("--onboard-slot", type=int, default=None)
    parser.add_argument("--manual-jaws", action="store_true",
                        help="pause for human jaw control via rebar_tester_gui.py")
    parser.add_argument("--skip-preposition", action="store_true",
                        help="assume the tester is already pre-positioned")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", default="rebar_tester_load_result.json")
    return parser


def main():
    args = build_parser().parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TesterLoadTest(args)
    try:
        passed = node.run()
    except Exception as exc:  # noqa: BLE001 - report any failure, then exit
        node.get_logger().error(traceback.format_exc())
        node.record("workflow_error", False, error=str(exc))
        node.write_report()
        passed = False
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
