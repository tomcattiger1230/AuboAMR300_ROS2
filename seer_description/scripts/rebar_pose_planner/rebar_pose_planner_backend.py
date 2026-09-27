#!/usr/bin/env python3
"""Read-only motion preview from an onboard rebar slot to a target TCP pose.

The preview starts after the selected bar has been gripped and lifted to
0.90 m in base_footprint. It never commands the arm, gripper or mobile base.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    AttachedCollisionObject, CollisionObject, Constraints, JointConstraint,
    PlanningScene, PlanningSceneComponents, RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene, GetMotionPlan, GetPlanningScene, GetPositionFK,
    GetPositionIK,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from rebar_pose_planner_core import (  # noqa: E402
    PlanInput, source_grasp_quat, source_lift_tcp, target_axes_world,
    target_in_base, wrist_from_tcp,
)
from rebar_tester_geometry import machine_boxes  # noqa: E402


ARM = ("shoulder_joint", "upperArm_joint", "foreArm_joint",
       "wrist1_joint", "wrist2_joint", "wrist3_joint")
FINGERS = ("gripper1_joint", "gripper2_joint")
GRASP_FINGER_POSITION = 0.017
SEEDS = (
    (0.0, -0.7, 1.2, 0.0, 1.5, 0.0),
    (-2.8, 1.0, 1.1, 0.1, 2.8, 0.0),
    (2.597, -0.357, -1.451, -1.094, -1.026, 0.0),
    (2.717, -0.230, -1.377, -1.147, -1.146, 0.0),
)


class PreviewError(RuntimeError):
    def __init__(self, stage, message):
        super().__init__(message)
        self.stage = stage


class Planner(Node):
    def __init__(self):
        super().__init__("rebar_pose_planner")
        self.positions = {}
        self.existing_attached_bar = False
        self.existing_world_bar = False
        self.last_joint_state = 0.0
        self.create_subscription(JointState, "/joint_states", self._joints,
                                 qos_profile_sensor_data)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.apply_client = self.create_client(ApplyPlanningScene,
                                               "/apply_planning_scene")

    def _joints(self, message):
        self.positions.update(zip(message.name, message.position))
        self.last_joint_state = time.monotonic()

    def call(self, client, request, timeout=15.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise PreviewError("connection", f"{client.srv_name} 超时")
        result = future.result()
        if result is None:
            raise PreviewError("connection", f"{client.srv_name} 无响应")
        return result

    def ready(self):
        for client in (self.ik_client, self.fk_client, self.plan_client,
                       self.scene_client, self.apply_client):
            if not client.wait_for_service(timeout_sec=8.0):
                raise PreviewError("connection", f"MoveIt 服务未就绪：{client.srv_name}")
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(j in self.positions for j in ARM + FINGERS):
                break
        if (not all(j in self.positions for j in ARM + FINGERS)
                or time.monotonic() - self.last_joint_state > 1.5):
            raise PreviewError("connection", "缺少新鲜 /joint_states")
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.tf.can_transform("world", "base_footprint", Time()):
                break
        else:
            raise PreviewError("connection", "缺少 world→base_footprint TF")
        transform = self.tf.lookup_transform("world", "base_footprint", Time())
        p = transform.transform.translation
        q = transform.transform.rotation
        return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)

    @staticmethod
    def virtual_bar():
        attached = AttachedCollisionObject()
        attached.link_name = "gripper_motor_link"
        attached.touch_links = ["gripper_motor_link", "gripper1_link", "gripper2_link"]
        attached.object.id = "planner_preview_rebar"
        attached.object.header.frame_id = "gripper_motor_link"
        attached.object.operation = CollisionObject.ADD
        attached.object.primitives = [SolidPrimitive(
            type=SolidPrimitive.CYLINDER, dimensions=[0.62, 0.020]
        )]
        pose = Pose()
        pose.position.z = 0.16
        pose.orientation.x = math.sqrt(0.5)
        pose.orientation.w = math.sqrt(0.5)
        attached.object.primitive_poses = [pose]
        return attached

    def robot_state(self, arm, with_bar=True):
        state = RobotState()
        names = list(ARM + FINGERS)
        state.joint_state.name = names
        state.joint_state.position = [float(arm[j]) for j in ARM] + [
            GRASP_FINGER_POSITION for _ in FINGERS
        ]
        state.is_diff = True
        if with_bar and not self.existing_attached_bar:
            state.attached_collision_objects = [self.virtual_bar()]
        return state

    @staticmethod
    def pose_stamped(tcp, orientation):
        wrist = wrist_from_tcp(tcp, orientation)
        stamped = PoseStamped()
        stamped.header.frame_id = "base_footprint"
        stamped.pose.position.x, stamped.pose.position.y, stamped.pose.position.z = wrist
        (stamped.pose.orientation.x, stamped.pose.orientation.y,
         stamped.pose.orientation.z, stamped.pose.orientation.w) = orientation
        return stamped

    def ensure_scene(self):
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        response = self.call(self.scene_client, request)
        world_ids = {obj.id for obj in response.scene.world.collision_objects}
        attached_ids = {
            obj.object.id for obj in response.scene.robot_state.attached_collision_objects
        }
        self.existing_attached_bar = "carried_rebar" in attached_ids
        self.existing_world_bar = "carried_rebar" in world_ids
        if "onboard_rebar_rack" not in attached_ids:
            raise PreviewError("scene", "规划场景缺少车载料架；请启动钢筋实验室场景")
        if {"rebar_test_machine", "rebar_tester_console"} <= world_ids:
            return
        boxes = list(machine_boxes())
        for object_id, entries in (
            ("rebar_test_machine", boxes[:-1]),
            ("rebar_tester_console", boxes[-1:]),
        ):
            scene = PlanningScene(is_diff=True)
            obj = CollisionObject()
            obj.id = object_id
            obj.header.frame_id = "world"
            obj.operation = CollisionObject.ADD
            for _, center, size in entries:
                obj.primitives.append(SolidPrimitive(
                    type=SolidPrimitive.BOX, dimensions=list(size)
                ))
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = center
                pose.orientation.w = 1.0
                obj.primitive_poses.append(pose)
            scene.world.collision_objects = [obj]
            result = self.call(self.apply_client,
                               ApplyPlanningScene.Request(scene=scene))
            if not result.success:
                raise PreviewError("scene", f"无法加载碰撞体：{object_id}")

    def ik(self, pose, seeds):
        candidates = []
        seen = set()
        for seed in seeds:
            arm = dict(zip(ARM, seed))
            request = GetPositionIK.Request()
            request.ik_request.group_name = "arm"
            request.ik_request.ik_link_name = "wrist3_Link"
            request.ik_request.robot_state = self.robot_state(arm)
            request.ik_request.pose_stamped = pose
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout.sec = 1
            response = self.call(self.ik_client, request, timeout=4.0)
            if response.error_code.val != 1:
                continue
            values = dict(zip(response.solution.joint_state.name,
                              response.solution.joint_state.position))
            if not all(j in values for j in ARM):
                continue
            result = {j: float(values[j]) for j in ARM}
            signature = tuple(round(result[j], 2) for j in ARM)
            if signature not in seen:
                seen.add(signature)
                candidates.append(result)
        return candidates

    def plan(self, source_arm, target_arm):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = "arm"
        motion.pipeline_id = "ompl"
        motion.num_planning_attempts = 5
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = 0.2
        motion.max_acceleration_scaling_factor = 0.2
        motion.start_state = self.robot_state(source_arm)
        motion.goal_constraints = [Constraints()]
        motion.goal_constraints[0].joint_constraints = [
            JointConstraint(joint_name=j, position=target_arm[j],
                            tolerance_above=0.001, tolerance_below=0.001,
                            weight=1.0)
            for j in ARM
        ]
        return self.call(self.plan_client, request, timeout=12.0).motion_plan_response

    def fk_check(self, arm, expected_pose):
        request = GetPositionFK.Request()
        request.header.frame_id = "base_footprint"
        request.fk_link_names = ["wrist3_Link"]
        request.robot_state = self.robot_state(arm)
        response = self.call(self.fk_client, request)
        if response.error_code.val != 1:
            raise PreviewError("fk", "目标关节解的正解校验失败")
        actual = response.pose_stamped[0].pose
        desired = expected_pose.pose
        position_error = math.dist(
            (actual.position.x, actual.position.y, actual.position.z),
            (desired.position.x, desired.position.y, desired.position.z),
        )
        alignment = abs(sum(a * b for a, b in zip(
            (actual.orientation.x, actual.orientation.y,
             actual.orientation.z, actual.orientation.w),
            (desired.orientation.x, desired.orientation.y,
             desired.orientation.z, desired.orientation.w),
        )))
        if position_error > 0.015 or alignment < 0.998:
            raise PreviewError("fk", f"目标正解误差过大：{position_error:.3f} m")
        return position_error

    def preview(self, request):
        base_position, base_quat = self.ready()
        self.ensure_scene()
        source_tcp = source_lift_tcp(request.slot)
        source_pose = self.pose_stamped(source_tcp, source_grasp_quat())
        target_tcp, target_quat = target_in_base(request, base_position, base_quat)
        target_pose = self.pose_stamped(target_tcp, target_quat)
        tool_axis, bar_axis = target_axes_world(request, base_quat)
        result = {
            "ok": False,
            "slot": request.slot,
            "target_input": {
                "frame": request.frame,
                "tcp_xyz": [request.x, request.y, request.z],
                "rpy_deg": [request.roll_deg, request.pitch_deg, request.yaw_deg],
            },
            "base_world_xyz": list(base_position),
            "source_lift_tcp_base": list(source_tcp),
            "assumed_finger_position_m": GRASP_FINGER_POSITION,
            "target_tcp_base": list(target_tcp),
            "target_tool_axis_world": list(tool_axis),
            "target_bar_axis_world": list(bar_axis),
            "payload_collision_source": (
                "existing_carried_rebar" if self.existing_attached_bar
                else "virtual_preview_rebar"
            ),
            "scene_warnings": (["场景中已有一根钢筋，测试位可能被占用"]
                               if self.existing_world_bar else []),
            "stages": [],
            "scope": (
                "仅规划车载钢筋夹稳并抬升到 0.90 m 之后的机械臂路径；"
                "不规划抓取接触、底盘导航，不执行运动。"
            ),
        }
        current = [self.positions[j] for j in ARM]
        source_candidates = self.ik(source_pose, (current, *SEEDS))
        if not source_candidates:
            raise PreviewError("source_ik", "所选槽位的取料后抬升姿态无碰撞逆解")
        result["source_ik_candidates"] = len(source_candidates)
        any_target_ik = False
        path_attempts = 0
        for source_arm in source_candidates[:4]:
            source_error = self.fk_check(source_arm, source_pose)
            source_stage = {
                "name": "source_lift_ik", "ok": True,
                "fk_error_m": source_error,
                "joints_rad": [source_arm[j] for j in ARM],
            }
            result["stages"] = [source_stage]
            target_candidates = self.ik(
                target_pose,
                ([source_arm[j] for j in ARM], current, *SEEDS),
            )
            any_target_ik |= bool(target_candidates)
            for target_arm in target_candidates[:4]:
                target_error = self.fk_check(target_arm, target_pose)
                path_attempts += 1
                trajectory = self.plan(source_arm, target_arm)
                if trajectory.error_code.val != 1:
                    continue
                points = trajectory.trajectory.joint_trajectory.points
                if not points:
                    continue
                duration = points[-1].time_from_start
                result["ok"] = True
                result["stages"].append({
                    "name": "target_ik", "ok": True,
                    "fk_error_m": target_error,
                    "joints_rad": [target_arm[j] for j in ARM],
                })
                result["stages"].append({
                    "name": "collision_checked_path", "ok": True,
                    "points": len(points),
                    "duration_s": duration.sec + duration.nanosec * 1e-9,
                    "attached_bar": True,
                })
                break
            if result["ok"]:
                break
        result["path_attempts"] = path_attempts
        if not result["ok"]:
            result["stage"] = "path" if any_target_ik else "target_ik"
            result["error"] = (
                "目标姿态有逆解，但未找到携筋碰撞检查路径" if any_target_ik
                else "目标 TCP 无碰撞逆解；可修改位置、姿态或底盘停靠点"
            )
        return result


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--slot", type=int, required=True, choices=(1, 2, 3, 4))
    parser.add_argument("--frame", choices=("world", "base_footprint"), default="world")
    for name in ("x", "y", "z", "roll", "pitch", "yaw"):
        parser.add_argument("--" + name, type=float, required=True)
    parser.add_argument("--output", type=Path)
    return parser.parse_args(argv)


def main(argv=None):
    args = arguments(argv)
    request = PlanInput(args.slot, args.frame, args.x, args.y, args.z,
                        args.roll, args.pitch, args.yaw)
    try:
        request.validate()
    except ValueError as exc:
        result = {"ok": False, "stage": "input", "error": str(exc)}
    else:
        rclpy.init()
        node = Planner()
        try:
            result = node.preview(request)
        except PreviewError as exc:
            result = {"ok": False, "stage": exc.stage, "error": str(exc)}
        except Exception as exc:
            result = {"ok": False, "stage": "unexpected", "error": str(exc)}
        finally:
            node.destroy_node()
            rclpy.shutdown()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2),
                               encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
