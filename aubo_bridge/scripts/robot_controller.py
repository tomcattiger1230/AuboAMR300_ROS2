#!/usr/bin/env python3
"""

Examples:
    robot = RobotController()
    robot.wait_until_ready()
    robot.print_robot_info()

    # Move by publishing message to topic, not service:
    robot.publish_joint_target([0, 0, 0, 0, 0, 0])
    robot.publish_line_target(0.3, 0.1, 0.4)          # absolute target position
    robot.publish_line_target_rpy(0.3, 0.1, 0.4, 0, 0, 90)
    robot.publish_line_offset(0.05, 0.0, 0.0)         # relative offset
    robot.publish_line_offset_rpy(0.05, 0, 0, 0, 0, 10)
"""

import argparse
import math
from typing import Iterable, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from aubo_bridge_msgs.msg import (
    TrajectoryCommand,
    TrajectoryPoint,
    RobotEvent,
    JointStateEx,
    RobotStatus,
)
from aubo_bridge_msgs.srv import (
    MoveToPose,
    MoveToJointAngles,
    GetRobotInfo,
    MoveJoint,
    MoveLine,
)


class RobotController(Node):
    """High-level Python controller for Aubo robot via ROS2."""

    # 函数说明：初始化上层控制器：创建 topic publisher、状态 subscriber 和 service client。
    def __init__(self, node_name: str = "robot_controller"):
        super().__init__(node_name)

        # -----------------------------
        # Topic 发布器：适合“发一条消息就让机械臂执行”的控制方式。
        # 与 service 相比，topic 不会直接返回执行结果，结果要看 bridge 日志/状态。
        # -----------------------------
        self.traj_pub_ = self.create_publisher(
            TrajectoryCommand, "/aubo/trajectory_command", 10
        )
        self.joint_move_pub_ = self.create_publisher(
            TrajectoryPoint, "/aubo/joint_move_command", 10
        )
        self.line_move_pub_ = self.create_publisher(
            TrajectoryPoint, "/aubo/line_move_command", 10
        )
        self.info_pub_ = self.create_publisher(
            String, "/aubo/info_command", 10
        )

        # -----------------------------
        # 状态订阅器：把 bridge 发布的状态缓存到本类，方便 get_current_* 查询。
        # -----------------------------
        self.event_sub_ = self.create_subscription(
            RobotEvent, "/aubo/events", self._on_event, 10
        )
        self.joint_state_sub_ = self.create_subscription(
            JointStateEx, "/aubo/joint_states_ex", self._on_joint_state, 10
        )
        self.status_sub_ = self.create_subscription(
            RobotStatus, "/aubo/status", self._on_status, 10
        )

        # -----------------------------
        # Service 客户端：适合需要等待响应、确认是否成功的控制方式。
        # -----------------------------
        self.move_pose_client_ = self.create_client(MoveToPose, "/aubo/move_to_pose")
        self.move_joint_angles_client_ = self.create_client(
            MoveToJointAngles, "/aubo/move_to_joint_angles"
        )
        self.get_info_client_ = self.create_client(GetRobotInfo, "/aubo/get_robot_info")
        self.move_joint_client_ = self.create_client(MoveJoint, "/aubo/move_joint")
        self.move_line_client_ = self.create_client(MoveLine, "/aubo/move_line")

        # State cache
        self.last_joint_state_: Optional[JointStateEx] = None
        self.last_status_: Optional[RobotStatus] = None
        self.event_history_: List[RobotEvent] = []

    # ------------------------------------------------------------------
    # Callbacks and state helpers
    # ------------------------------------------------------------------
    # 函数说明：接收机械臂事件并缓存，方便调试或后续报警处理。
    def _on_event(self, msg: RobotEvent):
        self.event_history_.append(msg)
        self.get_logger().info(
            f"Robot Event: type={msg.event_type}, sev={msg.severity}, {msg.description}"
        )

    # 函数说明：缓存最近一次关节状态消息。
    def _on_joint_state(self, msg: JointStateEx):
        self.last_joint_state_ = msg

    # 函数说明：缓存最近一次机械臂状态消息。
    def _on_status(self, msg: RobotStatus):
        self.last_status_ = msg

    # 函数说明：检查输入长度并转换为 float list，避免发出非法 ROS 消息。
    @staticmethod
    def _check_len(values: Sequence[float], length: int, name: str) -> List[float]:
        values = list(values)
        if len(values) != length:
            raise ValueError(f"{name} must contain {length} values, got {len(values)}")
        out = [float(v) for v in values]
        if not all(math.isfinite(v) for v in out):
            raise ValueError(f"{name} contains NaN or Inf: {values}")
        return out

    # 函数说明：把 roll/pitch/yaw 转换成 ROS 顺序 xyzw 四元数。
    @staticmethod
    def rpy_to_quat_xyzw(roll: float, pitch: float, yaw: float, degrees: bool = True) -> Tuple[float, float, float, float]:
        """Convert roll/pitch/yaw to ROS-order quaternion [x, y, z, w]."""
        # RPY 控制只保留 degree；degrees 参数保留是为了兼容旧调用，内部始终按 degree 处理。
        roll, pitch, yaw = (math.radians(float(v)) for v in (roll, pitch, yaw))
        if not all(math.isfinite(v) for v in (roll, pitch, yaw)):
            raise ValueError("roll/pitch/yaw contains NaN or Inf")
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return RobotController.normalize_quat_xyzw((qx, qy, qz, qw))

    # 函数说明：检查并归一化 ROS 顺序 xyzw 四元数。
    @staticmethod
    def normalize_quat_xyzw(quat_xyzw: Sequence[float]) -> Tuple[float, float, float, float]:
        """Validate and normalize ROS-order quaternion [x, y, z, w]."""
        q = [float(v) for v in quat_xyzw]
        if len(q) != 4:
            raise ValueError(f"quat_xyzw must contain 4 values, got {len(q)}")
        if not all(math.isfinite(v) for v in q):
            raise ValueError(f"quat_xyzw contains NaN or Inf: {quat_xyzw}")
        norm = math.sqrt(sum(v * v for v in q))
        if norm < 1e-8:
            raise ValueError(f"quat_xyzw norm is too small: {norm:.3e}")
        return tuple(v / norm for v in q)

    # 函数说明：把 ROS 顺序 xyzw 转成 Aubo SDK 常用的 wxyz 顺序。
    @staticmethod
    def quat_xyzw_to_wxyz(quat_xyzw: Sequence[float]) -> Tuple[float, float, float, float]:
        qx, qy, qz, qw = RobotController.normalize_quat_xyzw(quat_xyzw)
        return (qw, qx, qy, qz)

    # 函数说明：把 SDK/服务返回的 wxyz 四元数转换成 degree 制 RPY。
    @staticmethod
    def quat_wxyz_to_rpy_deg(quat_wxyz: Sequence[float]) -> Tuple[float, float, float]:
        w, x, y, z = (float(v) for v in quat_wxyz)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    # 函数说明：等待核心 service 可用，确认 bridge 节点已经启动。
    def wait_until_ready(self, timeout: float = 5.0) -> bool:
        """Wait for core bridge services."""
        clients = [
            self.get_info_client_,
            self.move_joint_client_,
            self.move_line_client_,
        ]
        ok = True
        for client in clients:
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f"Service not available: {client.srv_name}")
                ok = False
        return ok

    # 函数说明：返回 subscriber 缓存的最近一次关节角。
    def get_current_joints(self) -> Optional[list]:
        """Get last published joint positions from /aubo/joint_states_ex."""
        if self.last_joint_state_:
            return list(self.last_joint_state_.position)
        return None

    # 函数说明：返回 subscriber 缓存的最近一次末端位置。
    def get_current_pose(self) -> Optional[dict]:
        """Get last published TCP position from /aubo/status."""
        if self.last_status_:
            return {
                "x": self.last_status_.tool_position_x,
                "y": self.last_status_.tool_position_y,
                "z": self.last_status_.tool_position_z,
            }
        return None

    # ------------------------------------------------------------------
    # Service API: functions from test_files moved into reusable methods
    # ------------------------------------------------------------------
    # 函数说明：调用 /aubo/get_robot_info service 并返回完整响应。
    def get_robot_info(self, timeout: float = 10.0):
        """Call /aubo/get_robot_info and return the response."""
        request = GetRobotInfo.Request()
        future = self.get_info_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError("/aubo/get_robot_info call failed or timed out")
        return future.result()

    # 函数说明：打印当前关节角、末端绝对位置、RPY degree、四元数和关节速度/加速度限制。
    def print_robot_info(self) -> bool:
        """Print current joints, TCP pose, RPY(deg), quaternion, and joint limits."""
        response = self.get_robot_info()
        rpy = self.quat_wxyz_to_rpy_deg(response.current_ori)
        print("\n" + "=" * 50)
        print(" Robot Info")
        print("=" * 50)
        print(f"Success: {response.success}")
        print(f"Message: {response.message}")
        print(f"Joint status: {response.joint_status}")
        print("Current joint angles (rad):")
        for i, val in enumerate(response.current_joint):
            print(f"  Joint {i}: {val:.6f}")
        print("Current absolute flange position on base (m):")
        print(f"  X: {response.current_pos[0]:.6f}")
        print(f"  Y: {response.current_pos[1]:.6f}")
        print(f"  Z: {response.current_pos[2]:.6f}")
        print("Current flange orientation RPY (deg):")
        print(f"  Roll : {rpy[0]:.3f}")
        print(f"  Pitch: {rpy[1]:.3f}")
        print(f"  Yaw  : {rpy[2]:.3f}")
        print("Current flange orientation quaternion (wxyz):")
        print(f"  W: {response.current_ori[0]:.6f}")
        print(f"  X: {response.current_ori[1]:.6f}")
        print(f"  Y: {response.current_ori[2]:.6f}")
        print(f"  Z: {response.current_ori[3]:.6f}")
        print("Max joint acceleration (rad/s^2):")
        for i, val in enumerate(response.joint_maxacc):
            print(f"  Joint {i}: {val:.4f}")
        print("Max joint velocity (rad/s):")
        for i, val in enumerate(response.joint_maxvelc):
            print(f"  Joint {i}: {val:.4f}")
        print("=" * 50)
        return bool(response.success)

    # 函数说明：通过 topic 请求 bridge 打印当前绝对位姿信息。
    def publish_info_request(self, text: str = "show") -> None:
        msg = String()
        msg.data = str(text)
        self.info_pub_.publish(msg)
        self.get_logger().info("Published info request to /aubo/info_command")

    # 函数说明：通过 /aubo/move_joint service 执行关节空间运动。
    def move_joint_service(
        self,
        joint_angles: Sequence[float],
        max_acc: Sequence[float] = (0.5, 0.5, 0.5, 0.5, 0.5, 0.5),
        max_vel: Sequence[float] = (0.5, 0.5, 0.5, 0.5, 0.5, 0.5),
        enable_move: bool = True,
        timeout: float = 60.0,
    ) -> bool:
        """Move by /aubo/move_joint service."""
        request = MoveJoint.Request()
        request.target_joint = self._check_len(joint_angles, 6, "joint_angles")
        request.max_acc = self._check_len(max_acc, 6, "max_acc")
        request.max_vel = self._check_len(max_vel, 6, "max_vel")
        request.enable_move = bool(enable_move)

        self.get_logger().info(f"Service move_joint: {request.target_joint}")
        future = self.move_joint_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            self.get_logger().error("/aubo/move_joint service call failed")
            return False
        response = future.result()
        if response.success:
            self.get_logger().info(f"Move completed: {response.result_joint}")
        else:
            self.get_logger().error(f"Move failed: {response.message}")
        return bool(response.success)

    # 函数说明：通过 /aubo/move_line service 执行基座坐标系相对位移。
    def move_line_relative_service(
        self,
        relative_pos: Sequence[float],
        max_acc: Sequence[float] = (0.3, 0.3, 0.3, 0.3, 0.3, 0.3),
        max_vel: Sequence[float] = (0.3, 0.3, 0.3, 0.3, 0.3, 0.3),
        enable_move: bool = True,
        timeout: float = 60.0,
    ) -> bool:
        """Move by /aubo/move_line service with base-frame relative offset.

        Relative quaternion control is removed. This service helper keeps the
        current orientation; use topic/CLI --relative --rpy for degree control.
        """
        request = MoveLine.Request()
        request.relative_pos = self._check_len(relative_pos, 3, "relative_pos")
        # Relative quaternion control has been removed from the public API.
        # Keep current orientation by sending identity quaternion.
        request.relative_ori = [1.0, 0.0, 0.0, 0.0]
        request.max_acc = self._check_len(max_acc, 6, "max_acc")
        request.max_vel = self._check_len(max_vel, 6, "max_vel")
        request.enable_move = bool(enable_move)

        self.get_logger().info(f"Service move_line relative: pos={request.relative_pos}, keep current orientation")
        future = self.move_line_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            self.get_logger().error("/aubo/move_line service call failed")
            return False
        response = future.result()
        if response.success:
            rpy = self.quat_wxyz_to_rpy_deg(response.target_ori)
            self.get_logger().info(
                f"Line move completed | abs_pos=({response.target_pos[0]:.6f}, {response.target_pos[1]:.6f}, {response.target_pos[2]:.6f}) | "
                f"rpy_deg=({rpy[0]:.3f}, {rpy[1]:.3f}, {rpy[2]:.3f}) | "
                f"quat_wxyz=({response.target_ori[0]:.6f}, {response.target_ori[1]:.6f}, {response.target_ori[2]:.6f}, {response.target_ori[3]:.6f})"
            )
        else:
            self.get_logger().error(f"Line move failed: {response.message}")
        return bool(response.success)

    # 函数说明：通过 service 执行相对位移，并追加相对 RPY 姿态增量。
    def move_line_relative_rpy_service(
        self,
        relative_pos: Sequence[float],
        roll: float,
        pitch: float,
        yaw: float,
        degrees: bool = True,
        max_acc: Sequence[float] = (0.3, 0.3, 0.3, 0.3, 0.3, 0.3),
        max_vel: Sequence[float] = (0.3, 0.3, 0.3, 0.3, 0.3, 0.3),
        enable_move: bool = True,
        timeout: float = 60.0,
    ) -> bool:
        """Relative position + relative roll/pitch/yaw through /aubo/move_line service."""
        # The MoveLine.srv has no degree-RPY fields, and relative quaternion control
        # is removed. Use the topic command path, encoding degree RPY in label.
        if not enable_move:
            self.get_logger().warn("IK-only relative RPY is not supported without service quaternion fields; publishing topic command instead")
        self.publish_line_offset_rpy(
            relative_pos[0],
            relative_pos[1],
            relative_pos[2],
            roll,
            pitch,
            yaw,
            degrees=True,
        )
        rclpy.spin_once(self, timeout_sec=0.2)
        return True

    # Compatibility wrappers for old names
    # 函数说明：兼容旧接口：通过 /aubo/move_to_joint_angles 移动到关节目标。
    def move_to_joints(
        self,
        joint_angles: Sequence[float],
        max_velocity: float = 0.5,
        max_acceleration: float = 0.5,
        timeout: float = 30.0,
    ) -> bool:
        """Move robot arm to specified joint angles through old service."""
        request = MoveToJointAngles.Request()
        request.joint_angles = self._check_len(joint_angles, 6, "joint_angles")
        request.max_velocity = float(max_velocity)
        request.max_acceleration = float(max_acceleration)
        request.blocking = True

        future = self.move_joint_angles_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            self.get_logger().error("MoveToJointAngles service call failed")
            return False
        response = future.result()
        if not response.success:
            self.get_logger().error(f"Move failed: {response.message}")
        return bool(response.success)

    # 函数说明：通过 /aubo/move_to_pose 移动到绝对末端位姿。
    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        qw: float = 1.0,
        max_velocity: float = 0.3,
        max_acceleration: float = 0.3,
        timeout: float = 60.0,
    ) -> bool:
        """Move robot TCP to an absolute Cartesian pose through service."""
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)

        request = MoveToPose.Request()
        request.target_pose = pose
        request.max_velocity = float(max_velocity)
        request.max_acceleration = float(max_acceleration)
        request.blocking = True

        self.get_logger().info(f"Service move_to_pose: ({x}, {y}, {z})")
        future = self.move_pose_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            self.get_logger().error("MoveToPose service call failed")
            return False
        response = future.result()
        if not response.success:
            self.get_logger().error(f"Move failed: {response.message}")
        return bool(response.success)

    # ------------------------------------------------------------------
    # Message/topic API: move by publishing msg
    # ------------------------------------------------------------------
    # 函数说明：通过 topic 发布关节目标，不等待 service 返回。
    def publish_joint_target(self, joint_angles: Sequence[float], label: str = "joint_target") -> None:
        """Publish a joint target message to /aubo/joint_move_command."""
        msg = TrajectoryPoint()
        msg.joint_positions = self._check_len(joint_angles, 6, "joint_angles")
        msg.joint_velocities = [0.0] * 6
        msg.label = label
        self.joint_move_pub_.publish(msg)
        self.get_logger().info(f"Published joint target msg: {msg.joint_positions}")

    # 函数说明：通过 topic 发布绝对末端位置/四元数目标。
    def publish_line_target(
        self,
        x: float,
        y: float,
        z: float,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        qw: float = 0.0,
        label: str = "absolute_pose",
    ) -> None:
        """Publish an absolute Cartesian target to /aubo/line_move_command.

        Quaternion order here is ROS order xyzw. If all quaternion values are zero,
        the bridge keeps current robot orientation. Nonzero quaternion is checked
        and normalized before publishing.
        """
        msg = TrajectoryPoint()
        msg.pose_position_x = float(x)
        msg.pose_position_y = float(y)
        msg.pose_position_z = float(z)
        if not all(math.isfinite(v) for v in (msg.pose_position_x, msg.pose_position_y, msg.pose_position_z)):
            raise ValueError("target position contains NaN or Inf")

        quat = (float(qx), float(qy), float(qz), float(qw))
        if all(abs(v) <= 1e-12 for v in quat):
            qx, qy, qz, qw = quat
        else:
            qx, qy, qz, qw = self.normalize_quat_xyzw(quat)
        # TrajectoryPoint 使用 ROS 四元数顺序 xyzw；bridge 内部会转换成 SDK 的 wxyz。
        msg.pose_orientation_x = float(qx)
        msg.pose_orientation_y = float(qy)
        msg.pose_orientation_z = float(qz)
        msg.pose_orientation_w = float(qw)
        msg.label = label
        self.line_move_pub_.publish(msg)
        self.get_logger().info(
            f"Published line command | pos=({float(x):.6f}, {float(y):.6f}, {float(z):.6f}) | "
            f"quat_xyzw=({float(qx):.6f}, {float(qy):.6f}, {float(qz):.6f}, {float(qw):.6f}) | label='{label}'"
        )

    # 函数说明：通过 topic 发布绝对末端位置 + RPY 姿态目标。
    def publish_line_target_rpy(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        degrees: bool = True,
        label: str = "absolute_pose_rpy_deg",
    ) -> None:
        """Publish absolute Cartesian target with degree roll/pitch/yaw orientation."""
        qx, qy, qz, qw = self.rpy_to_quat_xyzw(roll, pitch, yaw, degrees=True)
        self.publish_line_target(x, y, z, qx=qx, qy=qy, qz=qz, qw=qw, label=label)

    # 函数说明：通过 topic 发布基座坐标系相对位移；只输入 dx/dy/dz，四元数字段废弃。
    def publish_line_offset(
        self,
        dx: float,
        dy: float,
        dz: float,
        label: str = "relative_offset",
    ) -> None:
        """Publish a base-frame relative Cartesian offset and keep current orientation.

        Pure relative offset only uses dx/dy/dz. Quaternion fields are intentionally
        published as zeros and ignored by the bridge. The relative offset + angle
        helper publish_line_offset_rpy(...) is unchanged.
        """
        if "relative" not in label.lower() and "offset" not in label.lower():
            label = f"relative_offset {label}"
        self.publish_line_target(
            dx,
            dy,
            dz,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=0.0,
            label=label,
        )

    # 函数说明：通过 topic 发布相对位移 + 相对 RPY 旋转。
    def publish_line_offset_rpy(
        self,
        dx: float,
        dy: float,
        dz: float,
        roll: float,
        pitch: float,
        yaw: float,
        degrees: bool = True,
        label: str = "relative_offset",
    ) -> None:
        """Publish relative position + relative roll/pitch/yaw rotation.

        RPY is encoded in the label. The bridge uses angle-addition semantics:
        target_rpy = current_rpy + relative_rpy_deg, then converts to quaternion.
        """
        # 只保留 degree 版 label；bridge 端先做 RPY 加法，再计算目标四元数。
        label = f"{label} relative_rpy_deg={float(roll)},{float(pitch)},{float(yaw)}"
        self.publish_line_target(
            dx,
            dy,
            dz,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            qw=0.0,
            label=label,
        )

    # 函数说明：通过 /aubo/trajectory_command 发布多点关节轨迹。
    def execute_joint_trajectory(
        self,
        waypoints: Sequence[Sequence[float]],
        max_joint_velocity: float = 0.5,
        max_joint_acceleration: float = 0.5,
        blocking: bool = True,
    ) -> None:
        """Publish a joint trajectory command."""
        cmd = TrajectoryCommand()
        cmd.command = TrajectoryCommand.EXECUTE_TRAJECTORY
        cmd.max_joint_velocity = float(max_joint_velocity)
        cmd.max_joint_acceleration = float(max_joint_acceleration)
        cmd.blocking = bool(blocking)
        for i, joints in enumerate(waypoints):
            point = TrajectoryPoint()
            point.joint_positions = self._check_len(joints, 6, f"waypoint_{i}")
            point.label = f"waypoint_{i}"
            cmd.trajectory.append(point)
        self.traj_pub_.publish(cmd)
        self.get_logger().info(f"Published joint trajectory with {len(waypoints)} waypoints")

    # 函数说明：通过 /aubo/trajectory_command 发布多点笛卡尔直线目标。
    def execute_line_trajectory(
        self,
        poses_xyz: Sequence[Sequence[float]],
        blocking: bool = True,
    ) -> None:
        """Publish Cartesian line trajectory command using absolute target positions."""
        cmd = TrajectoryCommand()
        cmd.command = TrajectoryCommand.EXECUTE_LINE_MOVE
        cmd.blocking = bool(blocking)
        for i, xyz in enumerate(poses_xyz):
            xyz = self._check_len(xyz, 3, f"pose_{i}")
            point = TrajectoryPoint()
            point.pose_position_x = xyz[0]
            point.pose_position_y = xyz[1]
            point.pose_position_z = xyz[2]
            point.label = f"line_pose_{i}"
            cmd.trajectory.append(point)
        self.traj_pub_.publish(cmd)
        self.get_logger().info(f"Published line trajectory with {len(poses_xyz)} poses")


# ----------------------------------------------------------------------
# Optional CLI for quick testing
# ----------------------------------------------------------------------
# 函数说明：命令行入口：解析 --info/--joint/--line/--rpy 等参数并执行对应控制。
def main():
    # 命令行参数说明：
    # --info                  读取当前机械臂状态；
    # --joint J0..J5          发送关节目标；
    # --line X Y Z            发送绝对位置或相对位移；
    # --relative              把 --line 解释为相对位移；
    # --rpy R P Y             追加 degree 制姿态控制；
    # --rpy-deg               兼容旧命令的空开关；RPY 始终按 degree 处理。
    # --info-topic            通过 topic 请求 bridge 打印当前绝对位姿。
    parser = argparse.ArgumentParser(description="Aubo ROS2 high-level controller")
    parser.add_argument("--info", action="store_true", help="Print robot info through service")
    parser.add_argument("--info-topic", action="store_true", help="Ask bridge to print current info through /aubo/info_command topic")
    parser.add_argument(
        "--joint",
        nargs=6,
        type=float,
        metavar=("J0", "J1", "J2", "J3", "J4", "J5"),
        help="Send joint target by topic msg",
    )
    parser.add_argument(
        "--line",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Send Cartesian line target by topic msg. Absolute by default.",
    )
    parser.add_argument(
        "--relative",
        action="store_true",
        help="Use --line values as base-frame relative offset",
    )
    parser.add_argument(
        "--rpy",
        nargs=3,
        type=float,
        metavar=("ROLL", "PITCH", "YAW"),
        help="Optional roll/pitch/yaw orientation for --line, unit is degrees. Rad is removed.",
    )
    parser.add_argument(
        "--rpy-deg",
        action="store_true",
        help="Compatibility flag only; --rpy is already interpreted as degrees",
    )
    parser.add_argument(
        "--service",
        action="store_true",
        help="Use service call for --joint or relative --line instead of topic msg",
    )
    args = parser.parse_args()

    rclpy.init()
    robot = RobotController()

    try:
        robot.wait_until_ready(timeout=5.0)

        if args.info:
            robot.print_robot_info()

        if args.info_topic:
            robot.publish_info_request()
            rclpy.spin_once(robot, timeout_sec=0.2)

        if args.joint is not None:
            if args.service:
                robot.move_joint_service(args.joint)
            else:
                robot.publish_joint_target(args.joint)
                # Let rclpy flush publication.
                rclpy.spin_once(robot, timeout_sec=0.2)

        if args.line is not None:
            if args.service:
                if not args.relative:
                    robot.get_logger().warn(
                        "--service --line only supports relative offset through /aubo/move_line. "
                        "For absolute line target, omit --service."
                    )
                if args.rpy is not None:
                    robot.move_line_relative_rpy_service(
                        args.line,
                        args.rpy[0],
                        args.rpy[1],
                        args.rpy[2],
                        degrees=True,
                    )
                else:
                    robot.move_line_relative_service(args.line)
            else:
                if args.relative:
                    if args.rpy is not None:
                        robot.publish_line_offset_rpy(
                            *args.line,
                            args.rpy[0],
                            args.rpy[1],
                            args.rpy[2],
                            degrees=True,
                        )
                    else:
                        robot.publish_line_offset(*args.line)
                else:
                    if args.rpy is not None:
                        robot.publish_line_target_rpy(
                            *args.line,
                            args.rpy[0],
                            args.rpy[1],
                            args.rpy[2],
                            degrees=True,
                        )
                    else:
                        robot.publish_line_target(*args.line)
                rclpy.spin_once(robot, timeout_sec=0.2)

        if not any([args.info, args.info_topic, args.joint is not None, args.line is not None]):
            rclpy.spin(robot)

    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()