#!/usr/bin/env python3

import os
import sys
import time
import threading
import math
import re
from typing import Iterable, Optional, Sequence, Tuple

# Add Python binding to path (relative to this script's location)
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
AUBO_BRIDGE_DIR = os.path.dirname(SCRIPT_DIR)
PYTHON_BINDING_PATH = os.path.join(
    AUBO_BRIDGE_DIR,
    "libpyauboi5-v1.5.1.x64-for-python3.x",
    "python3.x",
)
if PYTHON_BINDING_PATH not in sys.path:
    sys.path.insert(0, PYTHON_BINDING_PATH)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Import Aubo Python SDK
try:
    import libpyauboi5  # noqa: F401
    from robotcontrol import (
        Auboi5Robot,
        RobotErrorType,
        RobotEventType,  # noqa: F401
        RobotError,
    )

    AUBO_SDK_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Aubo Python SDK not available: {e}")
    AUBO_SDK_AVAILABLE = False

from aubo_bridge_msgs.msg import (
    TrajectoryCommand,
    TrajectoryPoint,
    RobotEvent,  # noqa: F401 - kept for future event bridge
    JointStateEx,
    RobotStatus,
)
from aubo_bridge_msgs.srv import (
    MoveToPose,
    MoveToJointAngles,
    ClearError,
    GetRobotInfo,
    MoveJoint,
    MoveLine,
)


class AuboBridgeNode(Node):
    """ROS2 Bridge Node for Aubo Robot using robotcontrol SDK."""

    DEFAULT_ACC_03 = (0.3, 0.3, 0.3, 0.3, 0.3, 0.3)
    DEFAULT_ACC_05 = (0.5, 0.5, 0.5, 0.5, 0.5, 0.5)
    DEFAULT_ACC_10 = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)

    # 函数说明：初始化 ROS2 节点：声明参数、创建 publisher/subscriber/service，并连接 Aubo SDK。
    def __init__(self):
        super().__init__("aubo_bridge")

        # -----------------------------
        # ROS2 参数区：这些参数可以通过 launch 文件覆盖。
        # 这里不要写死速度/加速度/四元数策略，方便现场调试。
        # -----------------------------
        self.declare_parameter("robot_host", "192.168.3.250")
        self.declare_parameter("robot_port", 8899)
        self.declare_parameter("collision_level", 6)
        self.declare_parameter("state_publish_rate", 10.0)
        self.declare_parameter("startup_robot", True)
        self.declare_parameter("default_joint_acc", 0.5)
        self.declare_parameter("default_joint_vel", 0.5)
        self.declare_parameter("default_line_acc", 0.3)
        self.declare_parameter("default_line_vel", 0.3)
        self.declare_parameter("auto_normalize_quaternion", True)
        self.declare_parameter("quat_norm_tolerance", 1e-3)

        self.robot_host = str(self.get_parameter("robot_host").value)
        self.robot_port = int(self.get_parameter("robot_port").value)
        self.collision_level = int(self.get_parameter("collision_level").value)
        self.state_publish_rate = float(self.get_parameter("state_publish_rate").value)
        self.startup_robot = bool(self.get_parameter("startup_robot").value)
        self.default_joint_acc = float(self.get_parameter("default_joint_acc").value)
        self.default_joint_vel = float(self.get_parameter("default_joint_vel").value)
        self.default_line_acc = float(self.get_parameter("default_line_acc").value)
        self.default_line_vel = float(self.get_parameter("default_line_vel").value)
        self.auto_normalize_quaternion = bool(self.get_parameter("auto_normalize_quaternion").value)
        self.quat_norm_tolerance = float(self.get_parameter("quat_norm_tolerance").value)

        # -----------------------------
        # SDK 状态区：
        # self.robot 是真实机械臂 SDK 句柄；
        # connected/initialized 用来区分“已连上控制柜”和“已经 startup 可运动”；
        # lock 用于保护 SDK 调用，避免状态发布线程和运动指令同时访问。
        # -----------------------------
        self.robot: Optional[Auboi5Robot] = None
        self.connected = False
        self.initialized = False
        self.lock = threading.RLock()
        self._monitor_stop = threading.Event()

        # -----------------------------
        # Publisher：向外发布机械臂事件、扩展关节状态和简化状态。
        # -----------------------------
        self.event_pub = self.create_publisher(RobotEvent, "/aubo/events", 10)
        self.joint_state_pub = self.create_publisher(
            JointStateEx, "/aubo/joint_states_ex", 10
        )
        self.status_pub = self.create_publisher(RobotStatus, "/aubo/status", 10)

        # -----------------------------
        # Subscriber：topic 控制入口。
        # 1) joint_move_command：直接关节运动；
        # 2) line_move_command：绝对/相对笛卡尔直线运动；
        # 3) trajectory_command：多点轨迹或模式分发。
        # -----------------------------
        self.traj_sub = self.create_subscription(
            TrajectoryCommand,
            "/aubo/trajectory_command",
            self.on_trajectory_command,
            10,
        )
        self.joint_move_sub = self.create_subscription(
            TrajectoryPoint,
            "/aubo/joint_move_command",
            self.on_joint_move_command,
            10,
        )
        self.line_move_sub = self.create_subscription(
            TrajectoryPoint,
            "/aubo/line_move_command",
            self.on_line_move_command,
            10,
        )
        # Topic 信息入口：不改 msg/srv 包，发布任意 std_msgs/String 即可让 bridge 打印当前绝对位姿。
        self.info_sub = self.create_subscription(
            String,
            "/aubo/info_command",
            self.on_info_command,
            10,
        )

        # -----------------------------
        # 兼容旧 service：保留原有接口，便于老脚本继续调用。
        # -----------------------------
        self.move_pose_srv = self.create_service(
            MoveToPose, "/aubo/move_to_pose", self.on_move_to_pose
        )
        self.move_joint_angles_srv = self.create_service(
            MoveToJointAngles,
            "/aubo/move_to_joint_angles",
            self.on_move_to_joint_angles,
        )
        self.clear_error_srv = self.create_service(
            ClearError, "/aubo/clear_error", self.on_clear_error
        )

        # -----------------------------
        # SDK 级 service：与 test_files 中读取信息、move_joint、move_line 的功能对应。
        # -----------------------------
        self.get_robot_info_srv = self.create_service(
            GetRobotInfo, "/aubo/get_robot_info", self.on_get_robot_info
        )
        self.move_joint_srv = self.create_service(
            MoveJoint, "/aubo/move_joint", self.on_move_joint
        )
        self.move_line_srv = self.create_service(
            MoveLine, "/aubo/move_line", self.on_move_line
        )

        # -----------------------------
        # 节点启动后立即尝试连接真实机械臂；
        # 如果 SDK 不存在，只启动 ROS2 节点但不会运动。
        # -----------------------------
        if AUBO_SDK_AVAILABLE:
            self.initialize_robot()
            self.monitor_thread = threading.Thread(
                target=self.state_monitor_loop,
                daemon=True,
            )
            self.monitor_thread.start()
        else:
            self.get_logger().error("Aubo SDK not available, node will not connect")

        self.get_logger().info(
            f"Aubo Bridge Node initialized (robot={self.robot_host}:{self.robot_port})"
        )
        self.get_logger().info(
            "Message commands: /aubo/joint_move_command, /aubo/line_move_command, "
            "/aubo/trajectory_command, /aubo/info_command"
        )

    # ---------------------------------------------------------------------
    # SDK helpers
    # ---------------------------------------------------------------------
    # 函数说明：统一判断 Aubo SDK 返回值是否表示成功，兼容枚举值和整数 0。
    @staticmethod
    def _sdk_success(result) -> bool:
        """Return True when SDK result means success."""
        try:
            return result == RobotErrorType.RobotError_SUCC or int(result) == int(
                RobotErrorType.RobotError_SUCC
            )
        except Exception:
            return result == RobotErrorType.RobotError_SUCC or result == 0

    # 函数说明：把 ROS 消息里的数组安全转换成固定长度 tuple；支持空值、单值广播和长度检查。
    @staticmethod
    def _to_tuple(values: Optional[Iterable[float]], length: int, default: float) -> Tuple[float, ...]:
        """Convert ROS array/list to fixed-length tuple with safe defaults."""
        if values is None:
            return tuple([default] * length)
        values = list(values)
        if len(values) == 0:
            return tuple([default] * length)
        if len(values) == 1:
            return tuple([float(values[0])] * length)
        if len(values) != length:
            raise ValueError(f"Expected {length} values, got {len(values)}: {values}")
        return tuple(float(v) for v in values)

    # 函数说明：判断一组数值是否可以视为全 0；用于“没有输入姿态/保持当前姿态”的语义。
    @staticmethod
    def _all_zero(values: Sequence[float], eps: float = 1e-12) -> bool:
        return all(abs(float(v)) <= eps for v in values)

    # 函数说明：判断 wxyz 四元数是否接近单位旋转 [1,0,0,0]。
    @staticmethod
    def _identity_quat_wxyz(values: Sequence[float], eps: float = 1e-9) -> bool:
        if values is None:
            return False
        values = list(values)
        if len(values) != 4:
            return False
        return (
            abs(float(values[0]) - 1.0) <= eps
            and abs(float(values[1])) <= eps
            and abs(float(values[2])) <= eps
            and abs(float(values[3])) <= eps
        )

    # 函数说明：检查输入长度以及是否包含 NaN/Inf，避免非法值进入 IK 或 SDK。
    @staticmethod
    def _finite_tuple(values: Sequence[float], length: int, name: str) -> Tuple[float, ...]:
        values = list(values)
        if len(values) != length:
            raise ValueError(f"{name} must contain {length} values, got {len(values)}: {values}")
        out = tuple(float(v) for v in values)
        if not all(math.isfinite(v) for v in out):
            raise ValueError(f"{name} contains NaN or Inf: {values}")
        return out

    # 函数说明：对 SDK 顺序 wxyz 四元数做合理性检测与归一化。
    def _normalize_quat_wxyz(self, quat_wxyz: Sequence[float], name: str = "quaternion") -> Tuple[float, float, float, float]:
        """Validate and normalize a SDK-order quaternion [w, x, y, z]."""
        q = self._finite_tuple(quat_wxyz, 4, name)
        norm = math.sqrt(sum(v * v for v in q))
        if norm < 1e-8:
            raise ValueError(f"{name} is invalid: norm is too small ({norm:.3e})")
        if abs(norm - 1.0) > self.quat_norm_tolerance:
            if not self.auto_normalize_quaternion:
                raise ValueError(f"{name} norm must be 1.0, got {norm:.6f}")
            self.get_logger().warn(f"{name} norm is {norm:.6f}; auto-normalizing")
        return tuple(v / norm for v in q)

    # 函数说明：计算两个 wxyz 四元数乘法；用于当前姿态叠加相对旋转。
    @staticmethod
    def _quat_multiply_wxyz(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float, float]:
        """Hamilton product for SDK-order quaternions [w, x, y, z]."""
        aw, ax, ay, az = (float(v) for v in a)
        bw, bx, by, bz = (float(v) for v in b)
        return (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )

    # 函数说明：把 roll/pitch/yaw 欧拉角转换成 SDK 顺序 wxyz 四元数。
    @staticmethod
    def _rpy_to_quat_wxyz(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """Convert roll/pitch/yaw radians to SDK-order quaternion [w, x, y, z]."""
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )

    # 函数说明：把 SDK 顺序 wxyz 四元数转换成 roll/pitch/yaw，输出单位为 degree。
    @staticmethod
    def _quat_wxyz_to_rpy_deg(quat_wxyz: Sequence[float]) -> Tuple[float, float, float]:
        """Convert SDK-order quaternion [w, x, y, z] to roll/pitch/yaw degrees."""
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

    # 函数说明：把角度规整到 [-180, 180) 区间，避免 RPY 加法后数值无限增大。
    @staticmethod
    def _wrap_angle_deg(angle: float) -> float:
        return ((float(angle) + 180.0) % 360.0) - 180.0

    # 函数说明：相对 RPY 采用“角度加法”语义：目标角度 = 当前角度 + 相对角度，然后再转四元数。
    def _current_plus_relative_rpy_deg_to_quat_wxyz(
        self,
        current_ori_wxyz: Sequence[float],
        relative_rpy_deg: Sequence[float],
        *,
        name: str = "current_rpy + relative_rpy_deg",
    ) -> Tuple[float, float, float, float]:
        current_ori = self._normalize_quat_wxyz(current_ori_wxyz, name="current_ori")
        current_rpy_deg = self._quat_wxyz_to_rpy_deg(current_ori)
        rel_rpy_deg = self._finite_tuple(relative_rpy_deg, 3, "relative_rpy_deg")
        target_rpy_deg = tuple(
            self._wrap_angle_deg(c + r) for c, r in zip(current_rpy_deg, rel_rpy_deg)
        )
        target_rpy_rad = tuple(math.radians(v) for v in target_rpy_deg)
        return self._normalize_quat_wxyz(
            self._rpy_to_quat_wxyz(*target_rpy_rad),
            name=name,
        )

    # 函数说明：显式禁止弧度制 rpy/relative_rpy 标签，只保留 degree 版本。
    def _reject_radian_rpy_labels(self, label: str) -> None:
        label = label or ""
        if self._extract_label_triplet(label, "relative_rpy") is not None:
            raise ValueError("relative_rpy(rad) has been removed; use relative_rpy_deg=roll,pitch,yaw")
        if self._extract_label_triplet(label, "rpy") is not None:
            raise ValueError("rpy(rad) has been removed; use rpy_deg=roll,pitch,yaw or absolute quaternion")

    # 函数说明：统一格式化绝对位姿，避免日志里输出过长数组。
    def _format_abs_pose(self, pos: Sequence[float], quat_wxyz: Sequence[float]) -> str:
        quat = self._normalize_quat_wxyz(quat_wxyz, name="format_quat_wxyz")
        r, p, y = self._quat_wxyz_to_rpy_deg(quat)
        return (
            f"abs_pos=({float(pos[0]):.6f}, {float(pos[1]):.6f}, {float(pos[2]):.6f}) | "
            f"rpy_deg=({r:.3f}, {p:.3f}, {y:.3f}) | "
            f"quat_wxyz=({quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f})"
        )

    # 函数说明：读取并格式化当前机械臂绝对位姿，供 /aubo/info_command 和运动后日志复用。
    def _current_abs_pose_text(self) -> str:
        waypoint = self._get_waypoint()
        pos = self._finite_tuple(waypoint["pos"], 3, "current_pos")
        quat = self._normalize_quat_wxyz(waypoint["ori"], name="current_ori")
        return self._format_abs_pose(pos, quat)

    # 函数说明：从 TrajectoryPoint.label 中解析形如 key=a,b,c 的三元组。
    @staticmethod
    def _extract_label_triplet(label: str, key: str) -> Optional[Tuple[float, float, float]]:
        """Parse label tokens like 'rpy=0,0,1.57' or 'relative_rpy_deg=0,0,90'."""
        if not label:
            return None
        pattern = rf"(?:^|[;\s]){re.escape(key)}\s*=\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)"
        match = re.search(pattern, label)
        if not match:
            return None
        vals = tuple(float(match.group(i)) for i in range(1, 4))
        if not all(math.isfinite(v) for v in vals):
            raise ValueError(f"{key} contains NaN or Inf: {vals}")
        return vals

    # 函数说明：从 label 里解析 degree-only RPY；rad 版本已删除，只保留 rpy_deg/relative_rpy_deg。
    def _rpy_label_to_quat_wxyz(self, label: str, *, relative_only: bool = False):
        """Return (mode, quat). Only degree labels are supported.

        Supported:
        - rpy_deg=roll,pitch,yaw              absolute orientation, degrees
        - relative_rpy_deg=roll,pitch,yaw     relative orientation delta, degrees

        Removed/forbidden:
        - rpy=...
        - relative_rpy=...
        """
        label = label or ""

        # 显式禁止 rad 标签，避免误把角度当弧度导致大幅转动。
        self._reject_radian_rpy_labels(label)

        for key, mode in (
            ("relative_rpy_deg", "relative"),
            ("rpy_deg", "absolute"),
        ):
            if relative_only and mode != "relative":
                continue
            vals = self._extract_label_triplet(label, key)
            if vals is not None:
                roll, pitch, yaw = (v * math.pi / 180.0 for v in vals)
                quat = self._rpy_to_quat_wxyz(roll, pitch, yaw)
                return mode, self._normalize_quat_wxyz(quat, name=key)
        return None, None

    # 函数说明：运动前检查 SDK 句柄和连接状态。
    def _require_robot(self) -> bool:
        if not self.robot or not self.connected:
            self.get_logger().error("Robot not connected")
            return False
        return True

    # 函数说明：在每次运动前设置关节最大加速度和最大速度。
    def _set_profile(self, max_acc: Sequence[float], max_vel: Sequence[float]) -> None:
        """Set joint max acceleration and velocity before motion."""
        self.robot.set_joint_maxacc(tuple(max_acc))
        self.robot.set_joint_maxvelc(tuple(max_vel))

    # 函数说明：切换到基座坐标系；失败时只报警，不直接让节点退出。
    def _safe_set_base_coord(self) -> bool:
        """Set base coordinate, matching test_files/test_line.py and test_joint.py."""
        try:
            ret = self.robot.set_base_coord()
            if not self._sdk_success(ret):
                self.get_logger().warn(f"set_base_coord returned: {ret}")
                return False
            return True
        except Exception as e:
            self.get_logger().warn(f"set_base_coord failed: {e}")
            return False

    # 函数说明：读取当前 waypoint，并确保 joint/pos/ori 三类字段存在。
    def _get_waypoint(self):
        waypoint = self.robot.get_current_waypoint()
        if not waypoint:
            raise RuntimeError("Failed to get current waypoint")
        if "joint" not in waypoint or "pos" not in waypoint or "ori" not in waypoint:
            raise RuntimeError(f"Waypoint missing fields: {waypoint}")
        return waypoint

    # 函数说明：调用 SDK 的 move_joint；兼容不同 SDK 版本的同步参数写法。
    def _call_move_joint(self, target_joint: Sequence[float], issync: bool = True):
        """SDK versions differ: support keyword and positional sync argument."""
        try:
            return self.robot.move_joint(tuple(target_joint), issync=issync)
        except TypeError:
            return self.robot.move_joint(tuple(target_joint), issync)

    # 函数说明：调用 SDK 的 move_line，输入是 IK 求出的目标关节角。
    def _call_move_line(self, target_joint: Sequence[float]):
        return self.robot.move_line(tuple(target_joint))

    # 函数说明：关节空间运动的统一实现：检查输入、设置速度/加速度、执行或仅检查。
    def _move_joint_impl(
        self,
        target_joint: Sequence[float],
        max_acc: Optional[Sequence[float]] = None,
        max_vel: Optional[Sequence[float]] = None,
        enable_move: bool = True,
        sync: bool = True,
    ) -> Tuple[bool, str, list]:
        if not self._require_robot():
            return False, "Robot not connected", [0.0] * 6

        target_joint = self._to_tuple(target_joint, 6, 0.0)
        max_acc = self._to_tuple(max_acc, 6, self.default_joint_acc)
        max_vel = self._to_tuple(max_vel, 6, self.default_joint_vel)

        with self.lock:
            self._set_profile(max_acc, max_vel)
            self._safe_set_base_coord()

            if not enable_move:
                return True, "Joint command checked (move disabled)", list(target_joint)

            result = self._call_move_joint(target_joint, issync=sync)
            if self._sdk_success(result):
                waypoint = self._get_waypoint()
                return True, "Joint move completed", list(waypoint.get("joint", target_joint))
            return False, f"Joint move failed: {result}", [0.0] * 6

    # 函数说明：解析 topic 中的姿态：绝对运动支持 label RPY 或 ROS xyzw 四元数；纯相对位移不走这里。
    def _orientation_from_point(
        self,
        point: TrajectoryPoint,
        current_ori_wxyz: Sequence[float],
        *,
        is_relative_motion: bool = False,
    ) -> Tuple[float, float, float, float]:
        """Decode target orientation from TrajectoryPoint.

        Supported command styles without changing the msg definition:
        - Absolute mode: pose_orientation_x/y/z/w is a ROS-order quaternion [x,y,z,w],
          or label contains rpy_deg=roll,pitch,yaw.
        - Relative offset + angle mode is handled in _move_line_from_point by
          label relative_rpy_deg=roll,pitch,yaw. Its semantics are
          target_rpy = current_rpy + relative_rpy.
        - Pure relative offset mode ignores pose_orientation_x/y/z/w and keeps
          current orientation.
        - Radian labels rpy=... and relative_rpy=... are intentionally removed.
        """
        current_ori = self._normalize_quat_wxyz(current_ori_wxyz, name="current_ori")
        label = (point.label or "").lower()

        # relative_rpy_deg 使用角度加法语义：先 current_rpy + relative_rpy，再转四元数。
        relative_rpy_deg = self._extract_label_triplet(label, "relative_rpy_deg")
        if relative_rpy_deg is not None:
            return self._current_plus_relative_rpy_deg_to_quat_wxyz(
                current_ori,
                relative_rpy_deg,
                name="current_rpy + relative_rpy_deg",
            )

        mode, label_quat = self._rpy_label_to_quat_wxyz(label)
        if label_quat is not None:
            return label_quat

        q_xyzw = (
            float(point.pose_orientation_x),
            float(point.pose_orientation_y),
            float(point.pose_orientation_z),
            float(point.pose_orientation_w),
        )
        if self._all_zero(q_xyzw):
            return current_ori

        if is_relative_motion:
            raise ValueError(
                "Relative quaternion control has been removed; "
                "use label='relative_offset relative_rpy_deg=roll,pitch,yaw' instead"
            )

        qx, qy, qz, qw = self._finite_tuple(q_xyzw, 4, "pose_orientation_xyzw")
        # Absolute quaternion is checked and normalized before IK.
        return self._normalize_quat_wxyz((qw, qx, qy, qz), name="absolute_pose_orientation_xyzw")

    # 函数说明：绝对笛卡尔直线运动：目标位置/姿态 -> IK -> move_line。
    def _move_line_to_pose_impl(
        self,
        target_pos: Sequence[float],
        target_ori_wxyz: Optional[Sequence[float]] = None,
        max_acc: Optional[Sequence[float]] = None,
        max_vel: Optional[Sequence[float]] = None,
        enable_move: bool = True,
    ) -> Tuple[bool, str, list, list, list]:
        """Move TCP along a Cartesian line to an absolute target pose.

        The absolute target orientation is checked before IK:
        - all-zero / None means keep current orientation;
        - nonzero quaternion must be finite and have nonzero norm;
        - non-unit quaternion is normalized when auto_normalize_quaternion=True.
        """
        if not self._require_robot():
            return False, "Robot not connected", [0.0] * 6, [0.0] * 3, [1.0, 0.0, 0.0, 0.0]

        target_pos = self._finite_tuple(target_pos, 3, "target_pos")
        max_acc = self._to_tuple(max_acc, 6, self.default_line_acc)
        max_vel = self._to_tuple(max_vel, 6, self.default_line_vel)

        with self.lock:
            self._set_profile(max_acc, max_vel)
            self._safe_set_base_coord()
            waypoint = self._get_waypoint()

            current_joint = tuple(waypoint["joint"])
            current_ori = self._normalize_quat_wxyz(waypoint["ori"], name="current_ori")
            if target_ori_wxyz is None or self._all_zero(target_ori_wxyz):
                target_ori = current_ori
            else:
                target_ori = self._normalize_quat_wxyz(target_ori_wxyz, name="target_ori_wxyz")

            # IK 是笛卡尔运动的关键步骤：SDK 的 move_line 接收目标关节角，
            # 所以必须先用当前关节角 + 目标位姿求逆解。
            ik_result = self.robot.inverse_kin(current_joint, target_pos, target_ori)
            if not ik_result or "joint" not in ik_result:
                return (
                    False,
                    "Inverse kinematics failed - target unreachable",
                    [0.0] * 6,
                    list(target_pos),
                    list(target_ori),
                )

            target_joint = tuple(ik_result["joint"])
            if not enable_move:
                return True, "IK computed (move disabled)", list(target_joint), list(target_pos), list(target_ori)

            # IK 成功后再执行 move_line，机械臂会按 SDK 定义的直线路径运动。
            result = self._call_move_line(target_joint)
            if self._sdk_success(result):
                # 运动成功后再次读取真实当前 waypoint，后续日志输出的是运动后的绝对位姿。
                final_waypoint = self._get_waypoint()
                final_pos = self._finite_tuple(final_waypoint["pos"], 3, "final_pos")
                final_ori = self._normalize_quat_wxyz(final_waypoint["ori"], name="final_ori")
                return True, "Line move completed", list(target_joint), list(final_pos), list(final_ori)
            return False, f"Line move failed: {result}", list(target_joint), list(target_pos), list(target_ori)

    # 函数说明：相对笛卡尔运动：当前位姿 + 相对位移/相对旋转 -> IK -> move_line。
    def _move_line_relative_impl(
        self,
        relative_pos: Sequence[float],
        relative_ori_wxyz: Optional[Sequence[float]] = None,
        max_acc: Optional[Sequence[float]] = None,
        max_vel: Optional[Sequence[float]] = None,
        enable_move: bool = True,
        relative_rpy: Optional[Sequence[float]] = None,
        relative_rpy_degrees: bool = False,
    ) -> Tuple[bool, str, list, list, list]:
        """Move TCP by a relative offset in base frame and optional relative rotation.

        Relative quaternion control has been removed from topic/user-facing control.
        Identity/all-zero orientation keeps current orientation. relative_rpy uses
        angle-addition semantics: target_rpy = current_rpy + relative_rpy.
        """
        if not self._require_robot():
            return False, "Robot not connected", [0.0] * 6, [0.0] * 3, [1.0, 0.0, 0.0, 0.0]

        relative_pos = self._finite_tuple(relative_pos, 3, "relative_pos")
        with self.lock:
            self._safe_set_base_coord()
            waypoint = self._get_waypoint()
            current_pos = self._finite_tuple(waypoint["pos"], 3, "current_pos")
            current_ori = self._normalize_quat_wxyz(waypoint["ori"], name="current_ori")
            # 相对位移在基座坐标系下叠加到当前末端位置。
            target_pos = (
                current_pos[0] + relative_pos[0],
                current_pos[1] + relative_pos[1],
                current_pos[2] + relative_pos[2],
            )

            if relative_rpy is not None:
                rpy = self._finite_tuple(relative_rpy, 3, "relative_rpy")
                if relative_rpy_degrees:
                    relative_rpy_deg = rpy
                else:
                    relative_rpy_deg = tuple(math.degrees(v) for v in rpy)
                target_ori = self._current_plus_relative_rpy_deg_to_quat_wxyz(
                    current_ori,
                    relative_rpy_deg,
                    name="current_rpy + relative_rpy",
                )
            elif (
                relative_ori_wxyz is None
                or self._all_zero(relative_ori_wxyz)
                or self._identity_quat_wxyz(relative_ori_wxyz)
            ):
                target_ori = current_ori
            else:
                raise ValueError(
                    "Relative quaternion control has been removed; "
                    "use relative_rpy_deg=roll,pitch,yaw through /aubo/line_move_command"
                )

        return self._move_line_to_pose_impl(
            target_pos,
            target_ori,
            max_acc=max_acc,
            max_vel=max_vel,
            enable_move=enable_move,
        )

    # 函数说明：把 TrajectoryPoint 消息转换成实际 line move 指令，自动判断 relative/absolute。
    def _move_line_from_point(self, point: TrajectoryPoint, enable_move: bool = True):
        """Execute a line command encoded as TrajectoryPoint."""
        if not self._require_robot():
            return False, "Robot not connected", [0.0] * 6, [0.0] * 3, [1.0, 0.0, 0.0, 0.0]

        label = (point.label or "").lower()
        position = self._finite_tuple(
            (
                float(point.pose_position_x),
                float(point.pose_position_y),
                float(point.pose_position_z),
            ),
            3,
            "point.pose_position",
        )
        is_relative = "relative" in label or "offset" in label

        # Velocity/acceleration defaults come from bridge parameters. TrajectoryPoint
        # does not have max_acc/max_vel fields, so topic commands use defaults.
        with self.lock:
            waypoint = self._get_waypoint()
            current_pos = self._finite_tuple(waypoint["pos"], 3, "current_pos")
            current_ori = self._normalize_quat_wxyz(waypoint["ori"], name="current_ori")

        if is_relative:
            # 纯相对位移 topic：只使用 pose_position_x/y/z，废弃/忽略四元数字段。
            # “相对位移 + 角度”路径：先 current_rpy + relative_rpy_deg，再转四元数控制。
            self._reject_radian_rpy_labels(label)
            relative_rpy_deg = self._extract_label_triplet(label, "relative_rpy_deg")
            if relative_rpy_deg is not None:
                target_ori = self._current_plus_relative_rpy_deg_to_quat_wxyz(
                    current_ori,
                    relative_rpy_deg,
                    name="current_rpy + relative_rpy_deg",
                )
            else:
                target_ori = current_ori

            q_xyzw = (
                float(point.pose_orientation_x),
                float(point.pose_orientation_y),
                float(point.pose_orientation_z),
                float(point.pose_orientation_w),
            )
            if not self._all_zero(q_xyzw):
                self.get_logger().warn(
                    "Relative /aubo/line_move_command ignores pose_orientation_x/y/z/w; "
                    "publish only pose_position_x/y/z, or use label relative_rpy_deg=roll,pitch,yaw"
                )

            target_pos = (
                current_pos[0] + position[0],
                current_pos[1] + position[1],
                current_pos[2] + position[2],
            )
            return self._move_line_to_pose_impl(
                target_pos,
                target_ori_wxyz=target_ori,
                max_acc=[self.default_line_acc] * 6,
                max_vel=[self.default_line_vel] * 6,
                enable_move=enable_move,
            )

        target_ori = self._orientation_from_point(
            point,
            current_ori,
            is_relative_motion=False,
        )
        return self._move_line_to_pose_impl(
            position,
            target_ori_wxyz=target_ori,
            max_acc=[self.default_line_acc] * 6,
            max_vel=[self.default_line_vel] * 6,
            enable_move=enable_move,
        )

    # ---------------------------------------------------------------------
    # Initialization and state publication
    # ---------------------------------------------------------------------
    # 函数说明：初始化 Aubo SDK、创建上下文、连接控制柜、启动机械臂并设置默认 profile。
    def initialize_robot(self):
        """Initialize connection to robot via robotcontrol SDK."""
        try:
            self.get_logger().info("Initializing Aubo SDK...")
            result = Auboi5Robot.initialize()
            if not self._sdk_success(result):
                self.get_logger().error(f"SDK init failed: {result}")
                return False

            self.robot = Auboi5Robot()
            handle = self.robot.create_context()
            self.get_logger().info(f"Created context: {handle}")

            self.get_logger().info(
                f"Connecting to {self.robot_host}:{self.robot_port}..."
            )
            result = self.robot.connect(self.robot_host, self.robot_port)
            if not self._sdk_success(result):
                self.get_logger().error(f"Connect failed: {result}")
                return False

            self.connected = True
            self.get_logger().info("Robot connected successfully")

            if self.startup_robot:
                self.get_logger().info("Starting up robot...")
                startup_result = self.robot.robot_startup(collision=self.collision_level)
                if not self._sdk_success(startup_result):
                    self.get_logger().warn(f"Robot startup returned: {startup_result}")
                else:
                    self.initialized = True
                    self.get_logger().info("Robot initialized and ready")
            else:
                self.initialized = True
                self.get_logger().warn("startup_robot=False, skipping robot_startup()")

            self.robot.init_profile()
            self._set_profile(
                [self.default_joint_acc] * 6,
                [self.default_joint_vel] * 6,
            )
            self._safe_set_base_coord()
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to initialize robot: {e}")
            self.connected = False
            return False

    # 函数说明：后台状态发布线程，以固定频率发布 joint state 和 robot status。
    def state_monitor_loop(self):
        """Publish robot state periodically."""
        period = 1.0 / max(self.state_publish_rate, 0.1)
        while rclpy.ok() and not self._monitor_stop.is_set():
            if self.connected:
                try:
                    self.publish_joint_state()
                    self.publish_robot_status()
                except Exception as e:
                    self.get_logger().warn(f"State monitor error: {e}")
            time.sleep(period)

    # 函数说明：读取当前关节角并发布到 /aubo/joint_states_ex。
    def publish_joint_state(self):
        """Publish current joint state from SDK."""
        if not self.robot:
            return

        msg = JointStateEx()
        try:
            waypoint = self.robot.get_current_waypoint()
            if waypoint and "joint" in waypoint:
                msg.position = list(waypoint["joint"])
            else:
                msg.position = [0.0] * 6
            msg.velocity = [0.0] * 6
            msg.effort = [0.0] * 6
            msg.joint_temperatures = [0.0] * 6
            msg.joint_currents = [0.0] * 6
            msg.joint_error_flags = [False] * 6
        except Exception as e:
            self.get_logger().warn(f"Failed to get joint state: {e}")
            msg.position = [0.0] * 6
            msg.velocity = [0.0] * 6
            msg.effort = [0.0] * 6
            msg.joint_temperatures = [0.0] * 6
            msg.joint_currents = [0.0] * 6
            msg.joint_error_flags = [False] * 6

        self.joint_state_pub.publish(msg)

    # 函数说明：发布机械臂连接/初始化状态以及当前末端位置。
    def publish_robot_status(self):
        """Publish robot status."""
        msg = RobotStatus()
        msg.power_on = bool(self.connected)
        msg.brakes_released = bool(self.initialized)

        if not self.connected:
            msg.robot_state = RobotStatus.DISCONNECTED
        elif not self.initialized:
            msg.robot_state = RobotStatus.BOOTING
        else:
            msg.robot_state = RobotStatus.IDLE
            try:
                waypoint = self.robot.get_current_waypoint()
                if waypoint and "pos" in waypoint:
                    msg.tool_position_x = float(waypoint["pos"][0])
                    msg.tool_position_y = float(waypoint["pos"][1])
                    msg.tool_position_z = float(waypoint["pos"][2])
            except Exception:
                pass

        self.status_pub.publish(msg)

    # ---------------------------------------------------------------------
    # Topic handlers: message commands
    # ---------------------------------------------------------------------
    # 函数说明：处理 /aubo/trajectory_command：根据 command 字段分发到关节轨迹或直线轨迹。
    def on_trajectory_command(self, msg: TrajectoryCommand):
        """Handle trajectory command topic."""
        self.get_logger().info(
            f"TrajectoryCommand: command={msg.command}, points={len(msg.trajectory)}"
        )

        if msg.command == TrajectoryCommand.EXECUTE_TRAJECTORY:
            self.execute_joint_trajectory(msg)
        elif msg.command == TrajectoryCommand.EXECUTE_JOINT_MOVE:
            points = list(msg.trajectory)
            if not points:
                self.get_logger().error("EXECUTE_JOINT_MOVE requires at least one point")
                return
            for point in points:
                self.on_joint_move_command(point)
                if not msg.blocking:
                    break
        elif msg.command == TrajectoryCommand.EXECUTE_LINE_MOVE:
            points = list(msg.trajectory)
            if not points:
                self.get_logger().error("EXECUTE_LINE_MOVE requires at least one point")
                return
            for point in points:
                self.on_line_move_command(point)
                if not msg.blocking:
                    break
        elif msg.command == TrajectoryCommand.STOP_TRAJECTORY:
            self.stop_trajectory()
        elif msg.command == TrajectoryCommand.PAUSE_TRAJECTORY:
            self.pause_trajectory()
        elif msg.command == TrajectoryCommand.RESUME_TRAJECTORY:
            self.resume_trajectory()
        else:
            self.get_logger().warn(f"Unknown trajectory command: {msg.command}")

    # 函数说明：处理 /aubo/joint_move_command topic，执行单次关节目标运动。
    def on_joint_move_command(self, msg: TrajectoryPoint):
        """Handle direct joint move command from topic."""
        self.get_logger().info(f"Topic joint move: {list(msg.joint_positions)}")
        try:
            ok, message, result_joint = self._move_joint_impl(
                msg.joint_positions,
                max_acc=[self.default_joint_acc] * 6,
                max_vel=[self.default_joint_vel] * 6,
                enable_move=True,
                sync=True,
            )
            if ok:
                self.get_logger().info(f"{message}: {result_joint}")
            else:
                self.get_logger().error(message)
        except Exception as e:
            self.get_logger().error(f"Topic joint move failed: {e}")

    # 函数说明：处理 /aubo/line_move_command topic，执行绝对或相对笛卡尔直线运动。
    def on_line_move_command(self, msg: TrajectoryPoint):
        """Handle Cartesian line move command from topic.

        Default: absolute target position.
        If msg.label contains "relative" or "offset", pose_position_x/y/z is treated
        as base-frame relative displacement. RPY labels are degree-only.
        """
        try:
            ok, message, result_joint, final_pos, final_ori = self._move_line_from_point(
                msg,
                enable_move=True,
            )
            if ok:
                self.get_logger().info(f"Moved | {self._format_abs_pose(final_pos, final_ori)}")
            else:
                self.get_logger().error(message)
        except Exception as e:
            self.get_logger().error(f"Topic line move failed: {e}")

    # 函数说明：处理 /aubo/info_command topic，打印当前绝对位置、RPY degree 和四元数。
    def on_info_command(self, msg: String):
        """Print current robot info through topic. Publish any std_msgs/String to trigger it."""
        try:
            self.get_logger().info(f"Current | {self._current_abs_pose_text()}")
        except Exception as e:
            self.get_logger().error(f"Info command failed: {e}")

    # 函数说明：顺序执行多个关节 waypoint；任一失败则停止后续轨迹。
    def execute_joint_trajectory(self, msg: TrajectoryCommand):
        """Execute a sequence of joint targets via SDK."""
        with self.lock:
            try:
                max_acc = [msg.max_joint_acceleration or self.default_joint_acc] * 6
                max_vel = [msg.max_joint_velocity or self.default_joint_vel] * 6
                self._set_profile(max_acc, max_vel)
                for i, point in enumerate(msg.trajectory):
                    self.get_logger().info(
                        f"Trajectory waypoint {i}: {list(point.joint_positions)}"
                    )
                    ok, message, _ = self._move_joint_impl(
                        point.joint_positions,
                        max_acc=max_acc,
                        max_vel=max_vel,
                        enable_move=True,
                        sync=True,
                    )
                    if not ok:
                        self.get_logger().error(f"Trajectory failed at {i}: {message}")
                        return
                self.get_logger().info("Joint trajectory completed")
            except Exception as e:
                self.get_logger().error(f"Trajectory error: {e}")

    # 函数说明：请求停止当前运动；如果 SDK 有 move_stop 则调用。
    def stop_trajectory(self):
        """Stop current trajectory if SDK exposes a stop method."""
        self.get_logger().info("Stop trajectory requested")
        try:
            if self.robot and hasattr(self.robot, "move_stop"):
                self.robot.move_stop()
            elif self.robot and hasattr(self.robot, "robot_shutdown"):
                self.get_logger().warn("No move_stop method found; not shutting down robot")
        except Exception as e:
            self.get_logger().warn(f"Stop trajectory failed: {e}")

    # 函数说明：占位接口：当前 SDK 绑定未实现 pause。
    def pause_trajectory(self):
        self.get_logger().info("Pause trajectory requested, SDK pause not implemented")

    # 函数说明：占位接口：当前 SDK 绑定未实现 resume。
    def resume_trajectory(self):
        self.get_logger().info("Resume trajectory requested, SDK resume not implemented")

    # ---------------------------------------------------------------------
    # Service handlers
    # ---------------------------------------------------------------------
    # 函数说明：处理 /aubo/move_to_pose service：绝对位姿 -> IK -> move_line。
    def on_move_to_pose(self, request, response):
        """Service: move TCP to an absolute Cartesian pose by IK + move_line."""
        self.get_logger().info("Service: /aubo/move_to_pose")
        try:
            target_pos = (
                request.target_pose.position.x,
                request.target_pose.position.y,
                request.target_pose.position.z,
            )
            target_ori = (
                request.target_pose.orientation.w,
                request.target_pose.orientation.x,
                request.target_pose.orientation.y,
                request.target_pose.orientation.z,
            )
            max_acc = [request.max_acceleration or self.default_line_acc] * 6
            max_vel = [request.max_velocity or self.default_line_vel] * 6
            ok, message, result_joint, _, _ = self._move_line_to_pose_impl(
                target_pos,
                target_ori,
                max_acc=max_acc,
                max_vel=max_vel,
                enable_move=True,
            )
            response.success = ok
            response.message = message
            response.joint_angles = list(result_joint)
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.joint_angles = [0.0] * 6
        return response

    # 函数说明：处理兼容旧接口 /aubo/move_to_joint_angles。
    def on_move_to_joint_angles(self, request, response):
        """Service: compatibility joint move handler."""
        self.get_logger().info(f"Service: /aubo/move_to_joint_angles: {request.joint_angles}")
        try:
            max_acc = [request.max_acceleration or self.default_joint_acc] * 6
            max_vel = [request.max_velocity or self.default_joint_vel] * 6
            ok, message, _ = self._move_joint_impl(
                request.joint_angles,
                max_acc=max_acc,
                max_vel=max_vel,
                enable_move=True,
                sync=bool(request.blocking),
            )
            response.success = ok
            response.message = message
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # 函数说明：处理 /aubo/clear_error；SDK 暴露 clear_error 时才真正执行。
    def on_clear_error(self, request, response):
        """Service: clear robot error if SDK exposes a clear method."""
        self.get_logger().info("Service: /aubo/clear_error")
        try:
            if self.robot and hasattr(self.robot, "clear_error"):
                result = self.robot.clear_error()
                response.success = self._sdk_success(result)
                response.message = f"clear_error returned: {result}"
            else:
                response.success = False
                response.message = "clear_error is not exposed by this SDK binding"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # 函数说明：处理 /aubo/get_robot_info，返回当前关节、末端位姿、速度/加速度限制。
    def on_get_robot_info(self, request, response):
        """Get robot info: joint status, max acc/vel, current waypoint."""
        self.get_logger().info("Service: /aubo/get_robot_info")
        try:
            if not self.connected:
                response.success = False
                response.message = "Robot not connected"
                return response

            waypoint = self.robot.get_current_waypoint()
            if waypoint:
                response.current_joint = list(waypoint.get("joint", [0.0] * 6))
                response.current_pos = list(waypoint.get("pos", [0.0] * 3))
                response.current_ori = list(waypoint.get("ori", [1.0, 0.0, 0.0, 0.0]))
            else:
                response.current_joint = [0.0] * 6
                response.current_pos = [0.0] * 3
                response.current_ori = [1.0, 0.0, 0.0, 0.0]

            response.joint_maxacc = list(self.robot.get_joint_maxacc())
            response.joint_maxvelc = list(self.robot.get_joint_maxvelc())

            joint_status = self.robot.get_joint_status()
            try:
                response.joint_status = int(joint_status) if joint_status else 0
            except Exception:
                response.joint_status = 0

            response.success = True
            response.message = "Robot info retrieved"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    # 函数说明：处理 /aubo/move_joint service，执行或仅检查关节目标。
    def on_move_joint(self, request, response):
        """Service: move to target joint angles."""
        self.get_logger().info(f"Service: /aubo/move_joint: {list(request.target_joint)}")
        try:
            ok, message, result_joint = self._move_joint_impl(
                request.target_joint,
                max_acc=request.max_acc,
                max_vel=request.max_vel,
                enable_move=bool(request.enable_move),
                sync=True,
            )
            response.success = ok
            response.message = message
            response.result_joint = list(result_joint)
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.result_joint = [0.0] * 6
        return response

    # 函数说明：处理 /aubo/move_line service，按基座坐标系相对位移执行直线运动。
    def on_move_line(self, request, response):
        """Service: move by a relative Cartesian offset in base frame.

        This preserves your test_files/test_line.py behavior:
        current_pos + relative_pos -> inverse_kin -> move_line.
        """
        self.get_logger().info(f"Service: /aubo/move_line relative_pos={list(request.relative_pos)}")
        try:
            ok, message, result_joint, target_pos, target_ori = self._move_line_relative_impl(
                request.relative_pos,
                relative_ori_wxyz=request.relative_ori,
                max_acc=request.max_acc,
                max_vel=request.max_vel,
                enable_move=bool(request.enable_move),
            )
            response.success = ok
            response.message = message
            response.result_joint = list(result_joint)
            response.target_pos = list(target_pos)
            response.target_ori = list(target_ori)
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.result_joint = [0.0] * 6
            response.target_pos = [0.0] * 3
            response.target_ori = [1.0, 0.0, 0.0, 0.0]
        return response

    # ---------------------------------------------------------------------
    # Shutdown
    # ---------------------------------------------------------------------
    # 函数说明：节点退出时断开机械臂连接并反初始化 SDK。
    def shutdown(self):
        """Clean shutdown of robot connection."""
        self._monitor_stop.set()
        if self.robot and self.connected:
            try:
                self.robot.disconnect()
                self.connected = False
            except Exception as e:
                self.get_logger().info(f"Robot disconnect: {e}")
        if AUBO_SDK_AVAILABLE:
            try:
                Auboi5Robot.uninitialize()
            except Exception as e:
                self.get_logger().info(f"Robot SDK uninitialize: {e}")


# 函数说明：ROS2 程序入口：初始化 rclpy、创建节点、spin、最后清理资源。
def main(args=None):
    rclpy.init(args=args)
    node = AuboBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()