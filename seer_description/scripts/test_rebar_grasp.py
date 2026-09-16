#!/usr/bin/env python3
"""Rebar grasp and release test against the Isaac Sim finger gripper.

Planning uses the same move_group services as test_isaac_arm.py (OMPL joint
plans + Cartesian paths) and execution goes through the ExecuteTrajectory
action - no MoveItPy dependency. The gripper uses the action bridge directly.

Sequence: open gripper -> joint plan above the rebar -> Cartesian descend ->
close on the rebar (fingers must stall at contact, not fully close) -> lift ->
transfer sideways -> open (fingers must return to zero). Every step appends a
report entry; the process exit code reflects the overall result.

Finger stall detection: the close goal commands the full travel, but a
grasped 24 mm bar stops both fingers near (0.0464 - diameter) / 2 ~= 0.011 m.
We watch /joint_states for the stall instead of waiting for the action
result, because the action bridge fails a goal whose fingers never reach the
commanded target.
"""

import argparse
import json
import math
import time
import traceback
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = (
    "shoulder_joint",
    "upperArm_joint",
    "foreArm_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
)
GRIPPER_JOINTS = ("gripper1_joint", "gripper2_joint")

# Candidate arm poses that reach toward -X (the arm mount faces -X). Used as
# IK seeds; wrist2 near pi/2 converges into the "wrist down, elbow down"
# branch that reaches over the table.
SEED_CONFIGS = [
    (0.0, -0.4, 0.9, 0.0, 0.5, 0.0),
    (0.0, -0.6, 1.1, 0.0, 0.5, 0.0),
    (0.0, -0.3, 0.7, 0.0, 0.6, 0.0),
    (0.0, -0.5, 1.0, -1.57, 0.5, 0.0),
    (0.0, -0.2, 0.5, 0.0, 0.4, 0.0),
    (0.0, -0.5, 1.0, 0.0, 1.5, 0.0),
    (0.0, -0.7, 1.2, 0.0, 1.5, 0.0),
    (0.3, -0.6, 1.1, 0.0, 1.5, 0.0),
]


def quat_rotate(quat, vector):
    """Rotate a 3-vector by a quaternion (x, y, z, w)."""
    x, y, z, w = quat
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_normalize(q):
    norm = math.sqrt(sum(component * component for component in q))
    return tuple(component / norm for component in q)


def quat_from_basis(x_axis, z_axis):
    """Quaternion from an orthonormal right-handed basis (columns)."""
    x_axis = tuple(v / math.sqrt(sum(v * v for v in x_axis)) for v in x_axis)
    z_axis = tuple(v / math.sqrt(sum(v * v for v in z_axis)) for v in z_axis)
    y_axis = (
        z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
        z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
        z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0],
    )
    trace = x_axis[0] + y_axis[1] + z_axis[2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quat = (
            (y_axis[2] - z_axis[1]) / scale,
            (z_axis[0] - x_axis[2]) / scale,
            (x_axis[1] - y_axis[0]) / scale,
            0.25 * scale,
        )
    elif x_axis[0] > y_axis[1] and x_axis[0] > z_axis[2]:
        scale = math.sqrt(1.0 + x_axis[0] - y_axis[1] - z_axis[2]) * 2.0
        quat = (
            0.25 * scale,
            (y_axis[0] + x_axis[1]) / scale,
            (z_axis[0] + x_axis[2]) / scale,
            (y_axis[2] - z_axis[1]) / scale,
        )
    elif y_axis[1] > z_axis[2]:
        scale = math.sqrt(1.0 + y_axis[1] - x_axis[0] - z_axis[2]) * 2.0
        quat = (
            (y_axis[0] + x_axis[1]) / scale,
            0.25 * scale,
            (z_axis[1] + y_axis[2]) / scale,
            (z_axis[0] - x_axis[2]) / scale,
        )
    else:
        scale = math.sqrt(1.0 + z_axis[2] - x_axis[0] - y_axis[1]) * 2.0
        quat = (
            (z_axis[0] + x_axis[2]) / scale,
            (z_axis[1] + y_axis[2]) / scale,
            0.25 * scale,
            (x_axis[1] - y_axis[0]) / scale,
        )
    return quat_normalize(quat)


class RebarGraspTest(Node):
    def __init__(self, args):
        super().__init__("rebar_grasp_test")
        self.args = args
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self._report = []
        self._passed = True
        self._positions = {}
        self._rebar_position = None
        self._rebar_pose_time = -math.inf

        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        self._plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self._cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, args.action_name
        )

    # ---------- plumbing ----------

    def _on_joint_state(self, message):
        self._positions.update(zip(message.name, message.position))

    def _on_tf(self, message):
        for transform in message.transforms:
            if transform.child_frame_id.lstrip("/") != "rebar":
                continue
            translation = transform.transform.translation
            self._rebar_position = (translation.x, translation.y, translation.z)
            self._rebar_pose_time = time.monotonic()

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, client, request, timeout=20):
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError(f"service call timed out: {client.srv_name}")
        return future.result()

    def record(self, name, passed, **details):
        entry = {"test": name, "pass": bool(passed)}
        entry.update(details)
        self._report.append(entry)
        self.get_logger().info(json.dumps(entry))
        if not passed:
            self._passed = False
        return passed

    @staticmethod
    def robot_state(values, gripper_open=True):
        state = dict(values)
        if gripper_open:
            state.update({joint: 0.0 for joint in GRIPPER_JOINTS})
        result = RobotState()
        result.joint_state.name = list(state)
        result.joint_state.position = list(state.values())
        result.is_diff = True
        return result

    def current_arm(self):
        return {joint: self._positions[joint] for joint in ARM_JOINTS}

    def rebar_position(self):
        if (self._rebar_position is None
                or time.monotonic() - self._rebar_pose_time > 1.0):
            raise RuntimeError("fresh world -> rebar transform unavailable")
        return self._rebar_position

    # ---------- MoveIt helpers (test_isaac_arm.py pattern) ----------

    def fk(self, values, link):
        request = GetPositionFK.Request()
        request.header.frame_id = "base_footprint"
        request.fk_link_names = [link]
        request.robot_state = self.robot_state(values)
        result = self.call(self._fk_client, request)
        if result.error_code.val != 1:
            raise RuntimeError(f"FK failed for {link}: {result.error_code.val}")
        return result.pose_stamped[0].pose

    def joint_plan(self, start, target, scaling=0.2):
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name = "arm"
        plan.pipeline_id = "ompl"
        plan.num_planning_attempts = 5
        plan.allowed_planning_time = 5.0
        plan.max_velocity_scaling_factor = scaling
        plan.max_acceleration_scaling_factor = scaling
        plan.start_state = self.robot_state(start)
        constraints = Constraints()
        constraints.joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=value,
                tolerance_above=0.001,
                tolerance_below=0.001,
                weight=1.0,
            )
            for name, value in target.items()
        ]
        plan.goal_constraints = [constraints]
        return self.call(self._plan_client, request).motion_plan_response

    def cartesian(self, start, waypoints):
        request = GetCartesianPath.Request()
        request.header.frame_id = "base_footprint"
        request.group_name = "arm"
        request.link_name = "wrist3_Link"
        request.start_state = self.robot_state(start)
        request.waypoints = waypoints
        request.max_step = 0.005
        request.avoid_collisions = True
        if hasattr(request, "max_velocity_scaling_factor"):
            request.max_velocity_scaling_factor = 0.15
            request.max_acceleration_scaling_factor = 0.15
        return self.call(self._cartesian_client, request)

    def execute(self, trajectory, timeout=90):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        goal.controller_names = ["aubo_arm_controller"]
        handle_future = self._execute_client.send_goal_async(goal)
        end = time.monotonic() + 20
        while not handle_future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = handle_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("execution rejected")
        result_future = handle.get_result_async()
        end = time.monotonic() + timeout
        while not result_future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError("execution timed out")
        self.spin(1.0)
        return wrapped.result.error_code.val

    def run_motion(self, label, plan_response, trajectory_key="trajectory"):
        """Shared plan/execute bookkeeping for joint and Cartesian motions."""
        entry = {
            "planning_code": plan_response.error_code.val,
            "points": len(
                getattr(plan_response, trajectory_key).joint_trajectory.points
            ),
        }
        if plan_response.error_code.val != 1:
            self.record(label, False, **entry)
            raise RuntimeError(f"{label}: planning failed")
        if self.args.plan_only:
            self.record(label, True, **entry)
            return
        entry["execution_code"] = self.execute(
            getattr(plan_response, trajectory_key)
        )
        self.record(label, entry["execution_code"] == 1, **entry)
        if entry["execution_code"] != 1:
            raise RuntimeError(f"{label}: execution failed")

    # ---------- gripper helpers ----------

    def gripper_goal(self, position, duration=1.5, steps=30):
        start = [self._positions.get(joint, 0.0) for joint in GRIPPER_JOINTS]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(GRIPPER_JOINTS)
        for index in range(1, steps + 1):
            t = index / steps
            smooth = 3.0 * t * t - 2.0 * t * t * t
            point = JointTrajectoryPoint()
            point.positions = [
                initial + (position - initial) * smooth for initial in start
            ]
            whole = int(duration * t)
            point.time_from_start = Duration(
                sec=whole, nanosec=int((duration * t - whole) * 1e9)
            )
            goal.trajectory.points.append(point)
        # Contact is the expected endpoint when closing on an object. An
        # unbounded action tolerance lets the bridge finish while preserving
        # the final drive target, so the fingers keep applying preload.
        goal.goal_tolerance = [
            JointTolerance(name=joint, position=-1.0) for joint in GRIPPER_JOINTS
        ]
        return goal

    def send_gripper(self, position, duration=1.5):
        goal_future = self._gripper_client.send_goal_async(
            self.gripper_goal(position, duration)
        )
        end = time.monotonic() + 15
        while not goal_future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return goal_future.result()

    def finger_positions(self):
        return [self._positions.get(joint) for joint in GRIPPER_JOINTS]

    def wait_finger_settle(
        self, start, window=0.6, motion=0.0005, min_travel=0.003, timeout=8.0
    ):
        """Wait until both fingers stop moving; returns the stalled positions."""
        deadline = time.monotonic() + timeout
        reference = self.finger_positions()
        reference_time = time.monotonic()
        motion_started = False
        while time.monotonic() < deadline:
            self.spin(0.1)
            current = self.finger_positions()
            if not motion_started:
                motion_started = all(
                    now is not None and initial is not None
                    and abs(now - initial) >= min_travel
                    for now, initial in zip(current, start)
                )
                reference = current
                reference_time = time.monotonic()
                continue
            moved = max(abs(now - then) for now, then in zip(current, reference))
            if moved < motion:
                if time.monotonic() - reference_time >= window:
                    return current
            else:
                reference = current
                reference_time = time.monotonic()
        return self.finger_positions()

    def wait_gripper_result(self, handle, timeout=10):
        result_future = handle.get_result_async()
        end = time.monotonic() + timeout
        while not result_future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return result_future.result()

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
        self.spin(3.0)

        observed_rebar = self.rebar_position()
        self.record(
            "rebar_pose_feedback",
            math.dist(observed_rebar, (args.rebar_x, args.rebar_y, args.rebar_z))
            < 0.03,
            position=list(observed_rebar),
        )

        rebar_center = (args.rebar_x, args.rebar_y, args.rebar_z)
        rebar_half_gap = (args.open_gap - args.diameter) / 2.0

        # 1. Open the gripper.
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

        # 2. Pick the seed pose whose FK lands closest to the rebar; its FK
        # also yields the constant TCP offset in the wrist frame.
        seed_values = None
        seed_wrist = None
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
                best_distance = distance
                seed_values = values
                seed_wrist = pose
        if seed_values is None:
            self.record("seed_selection", False)
            raise RuntimeError("no seed pose produced FK")

        motor_pose = self.fk(seed_values, "gripper_motor_link")
        wrist_position = (
            seed_wrist.position.x,
            seed_wrist.position.y,
            seed_wrist.position.z,
        )
        wrist_quat = (
            seed_wrist.orientation.x,
            seed_wrist.orientation.y,
            seed_wrist.orientation.z,
            seed_wrist.orientation.w,
        )
        motor_position = (
            motor_pose.position.x,
            motor_pose.position.y,
            motor_pose.position.z,
        )
        motor_quat = (
            motor_pose.orientation.x,
            motor_pose.orientation.y,
            motor_pose.orientation.z,
            motor_pose.orientation.w,
        )
        tcp_world_offset = quat_rotate(motor_quat, (0.0, args.tcp_y, args.tcp_z))
        tcp_world = tuple(m + o for m, o in zip(motor_position, tcp_world_offset))
        relative = tuple(t - w for t, w in zip(tcp_world, wrist_position))
        tcp_in_wrist = quat_rotate(quat_conjugate(wrist_quat), relative)
        self.record(
            "tcp_calibration",
            True,
            seed={k: round(v, 4) for k, v in seed_values.items()},
            seed_wrist_to_rebar=round(best_distance, 4),
            tcp_in_wrist=[round(v, 4) for v in tcp_in_wrist],
        )

        # Grasp orientation (explicit): wrist Z points straight down and the
        # wrist X axis (the finger closing axis) lies horizontally across the
        # rebar, so the pads straddle it from both sides.
        closing_axis = {
            "y": (0.0, 1.0, 0.0),
            "-y": (0.0, -1.0, 0.0),
            "x": (1.0, 0.0, 0.0),
            "-x": (-1.0, 0.0, 0.0),
        }[args.wrist_x_axis]
        grasp_quat = quat_from_basis(closing_axis, (0.0, 0.0, -1.0))

        def wrist_pose_for(tcp_world_target):
            rotated = quat_rotate(grasp_quat, tcp_in_wrist)
            position = tuple(t - r for t, r in zip(tcp_world_target, rotated))
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = position
            pose.orientation.x = grasp_quat[0]
            pose.orientation.y = grasp_quat[1]
            pose.orientation.z = grasp_quat[2]
            pose.orientation.w = grasp_quat[3]
            stamped = PoseStamped()
            stamped.header.frame_id = "base_footprint"
            stamped.pose = pose
            return stamped

        # 3. IK to the pre-grasp point (approach height above the rebar).
        # KDL may return flipped or angle-wrapped configurations; try every
        # seed, unwrap near the current state, prefer the forward branch.
        pregrasp_tcp = (
            rebar_center[0],
            rebar_center[1],
            rebar_center[2] + args.approach_height,
        )
        current = self.current_arm()

        def unwind(angle, reference):
            return reference + math.atan2(
                math.sin(angle - reference), math.cos(angle - reference)
            )

        pregrasp_joints = None
        best_cost = None
        for seed in SEED_CONFIGS + [tuple(current[joint] for joint in ARM_JOINTS)]:
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
            # Prefer the arm reaching forward over mirrored branches.
            cost += 2.0 * abs(solution["shoulder_joint"])
            if best_cost is None or cost < best_cost:
                best_cost = cost
                pregrasp_joints = solution
        if pregrasp_joints is None:
            self.record("pregrasp_ik", False)
            raise RuntimeError("pre-grasp IK failed for every seed")
        self.record(
            "pregrasp_ik",
            True,
            max_joint_delta=round(
                max(abs(pregrasp_joints[j] - current[j]) for j in ARM_JOINTS), 4
            ),
            joints={k: round(v, 4) for k, v in pregrasp_joints.items()},
        )

        # 4. Joint plan from the current pose to the pre-grasp, execute it.
        plan_response = self.joint_plan(self.current_arm(), pregrasp_joints)
        self.run_motion("approach_pregrasp", plan_response)

        # 5. Cartesian descend onto the rebar. In plan-only mode the approach
        # was never executed, so descend from the planned pre-grasp state.
        descend_start = pregrasp_joints if self.args.plan_only else self.current_arm()
        descend = self.cartesian(
            descend_start, [wrist_pose_for(rebar_center).pose]
        )
        entry = {"fraction": descend.fraction, "code": descend.error_code.val}
        if descend.error_code.val != 1 or descend.fraction < 0.99:
            entry["points"] = len(descend.solution.joint_trajectory.points)
            self.record("descend_to_grasp", False, **entry)
            raise RuntimeError(f"descend failed: {entry}")
        if self.args.plan_only:
            entry["points"] = len(descend.solution.joint_trajectory.points)
            self.record("descend_to_grasp", True, **entry)
            self.write_report()
            return self._passed
        entry["points"] = len(descend.solution.joint_trajectory.points)
        entry["execution_code"] = self.execute(descend.solution)
        self.record("descend_to_grasp", entry["execution_code"] == 1, **entry)
        if entry["execution_code"] != 1:
            raise RuntimeError("descend execution failed")

        # 6. Close on the rebar; fingers must stall at contact before the
        # full travel. The exact stall value depends on pad contact geometry,
        # so contact + later payload retention is the two-stage criterion.
        close_start = self.finger_positions()
        close_handle = self.send_gripper(args.close_position, 2.0)
        stalled = self.wait_finger_settle(close_start, timeout=10.0)
        preload_result = self.wait_gripper_result(close_handle, timeout=2.0)
        valid_stall = all(
            value is not None
            and initial is not None
            and value - initial >= 0.003
            and value <= args.close_position - 0.003
            for value, initial in zip(stalled, close_start)
        ) and preload_result is not None and (
            preload_result.status == GoalStatus.STATUS_SUCCEEDED
        )
        inferred_gap = None
        if all(value is not None for value in stalled):
            inferred_gap = round(args.open_gap - sum(stalled), 4)
        self.record(
            "grasp_close_on_rebar",
            valid_stall,
            fingers_stalled=stalled,
            inferred_gap_m=inferred_gap,
            preload_target=args.close_position,
            preload_status=(preload_result.status if preload_result else None),
        )
        if not valid_stall:
            self.write_report()
            raise RuntimeError("fingers did not stall on the rebar")
        grasp_stall = stalled
        grasp_rebar = self.rebar_position()

        # The completed gripper goal leaves the full-close drive target active,
        # so the fingers continue applying preload throughout transport.

        # 7. Lift straight up.
        lift_tcp = (
            rebar_center[0],
            rebar_center[1],
            rebar_center[2] + args.lift_height,
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

        lifted_rebar = self.rebar_position()
        lifted_distance = lifted_rebar[2] - grasp_rebar[2]
        lifted = lifted_distance >= args.lift_height * 0.6
        self.record(
            "rebar_lifted",
            lifted,
            before=list(grasp_rebar),
            after=list(lifted_rebar),
            vertical_distance=lifted_distance,
        )
        if not lifted:
            raise RuntimeError("arm lifted but the rebar did not follow")

        # 8. Payload retention: fingers still stalled after the lift settles.
        # With the contact-based close criterion this is the authoritative
        # grasp check: if nothing was held, the fingers would not stay put.
        time.sleep(0.5)
        self.spin(1.0)
        retained = self.finger_positions()
        holding = all(
            value is not None and abs(value - stall) < 0.004
            for value, stall in zip(retained, grasp_stall)
        )
        self.record(
            "payload_retained_after_lift",
            holding,
            fingers=retained,
            grasp_stall=grasp_stall,
        )
        if not holding:
            self.write_report()
            raise RuntimeError("fingers did not retain the payload after lift")

        # 9. Transfer sideways to the release point.
        release_tcp = (
            rebar_center[0] + args.release_dx,
            rebar_center[1] + args.release_dy,
            rebar_center[2] + args.lift_height,
        )
        transfer = self.cartesian(
            self.current_arm(), [wrist_pose_for(release_tcp).pose]
        )
        entry = {"fraction": transfer.fraction, "code": transfer.error_code.val}
        if transfer.error_code.val != 1 or transfer.fraction < 0.99:
            self.record("transfer_to_release", False, **entry)
            raise RuntimeError(f"transfer failed: {entry}")
        entry["execution_code"] = self.execute(transfer.solution)
        self.record("transfer_to_release", entry["execution_code"] == 1, **entry)
        if entry["execution_code"] != 1:
            raise RuntimeError("transfer execution failed")

        transferred_rebar = self.rebar_position()
        transfer_distance = math.dist(lifted_rebar[:2], transferred_rebar[:2])
        expected_transfer = math.hypot(args.release_dx, args.release_dy)
        transferred = transfer_distance >= expected_transfer * 0.6
        self.record(
            "rebar_transferred",
            transferred,
            before=list(lifted_rebar),
            after=list(transferred_rebar),
            horizontal_distance=transfer_distance,
        )
        if not transferred:
            raise RuntimeError("gripper moved but the rebar did not follow")

        # 10. Replace the preload target with open and release.
        release_position = self.rebar_position()
        handle = self.send_gripper(0.0, 1.5)
        wrapped = self.wait_gripper_result(handle)
        open_deadline = time.monotonic() + 3.0
        released = self.finger_positions()
        while (
            time.monotonic() < open_deadline
            and not all(
                value is not None and abs(value) < 0.0015 for value in released
            )
        ):
            self.spin(0.1)
            released = self.finger_positions()
        self.spin(0.75)
        dropped_position = self.rebar_position()
        drop_distance = release_position[2] - dropped_position[2]
        self.record(
            "release_open",
            wrapped is not None
            and wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and all(value is not None and abs(value) < 0.0015 for value in released)
            and drop_distance > 0.03,
            fingers_released=released,
            rebar_before=list(release_position),
            rebar_after=list(dropped_position),
            rebar_drop_distance=drop_distance,
        )

        self.write_report()
        return self._passed

    def write_report(self):
        Path(self.args.output).write_text(
            json.dumps(self._report, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        self.get_logger().info(f"report written to {self.args.output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--action-name", default="/aubo_arm_controller/follow_joint_trajectory")
    parser.add_argument("--rebar-x", type=float, default=-1.15)
    parser.add_argument("--rebar-y", type=float, default=0.0)
    parser.add_argument("--rebar-z", type=float, default=0.522)
    parser.add_argument("--diameter", type=float, default=0.024)
    parser.add_argument(
        "--open-gap",
        type=float,
        default=0.0464,
        help="Distance between the finger pads at travel 0 (measured in sim)",
    )
    parser.add_argument(
        "--close-position",
        type=float,
        default=0.0285,
        help="Finger travel commanded for grasping (finger gripper upper limit)",
    )
    parser.add_argument(
        "--wrist-x-axis",
        choices=["y", "-y", "x", "-x"],
        default="y",
        help="World direction of the wrist X (finger closing) axis at grasp",
    )
    parser.add_argument("--approach-height", type=float, default=0.15)
    parser.add_argument("--lift-height", type=float, default=0.18)
    parser.add_argument("--release-dx", type=float, default=0.0)
    parser.add_argument("--release-dy", type=float, default=0.18)
    parser.add_argument("--tcp-y", type=float, default=0.0, help="TCP Y in the motor frame")
    parser.add_argument("--tcp-z", type=float, default=0.16, help="TCP Z in the motor frame")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--output", default="rebar_grasp_result.json")
    args = parser.parse_args()

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = RebarGraspTest(args)
    try:
        passed = node.run()
    except Exception:  # noqa: BLE001 - report any failure, then exit
        node.get_logger().error(traceback.format_exc())
        node.write_report()
        passed = False
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
