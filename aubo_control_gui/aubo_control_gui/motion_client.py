"""Common MoveIt client for both Isaac and hardware; no vendor SDK imports."""
import math
import time

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetPositionFK
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

ARM = ('shoulder_joint', 'upperArm_joint', 'foreArm_joint',
       'wrist1_joint', 'wrist2_joint', 'wrist3_joint')
GRIPPER = ('gripper1_joint', 'gripper2_joint')


class MotionClient(Node):
    def __init__(self):
        super().__init__('aubo_motion_client')
        self.declare_parameter('enable_motion', False)
        self.declare_parameter('planning_frame', 'base_footprint')
        self.declare_parameter('tool_link', 'wrist3_Link')
        self.state = {}
        self.received = {}
        self.pose = None
        self.pose_received = 0.0
        self.busy = False
        self.cancel_requested = False
        self.goal_handle = None
        self.last_result = None
        self.active_names = ARM
        self.on_result = lambda message, error: None
        self.move = ActionClient(self, MoveGroup, 'move_action')
        self.fk = self.create_client(GetPositionFK, 'compute_fk')
        self.fk_pending = None
        self.create_subscription(JointState, 'joint_states', self._state, qos_profile_sensor_data)
        self.create_timer(0.2, self._refresh_pose)

    def _state(self, msg):
        now = time.monotonic()
        for name, value in zip(msg.name, msg.position):
            if math.isfinite(value):
                self.state[name] = value
                self.received[name] = now

    def fresh(self, names=ARM):
        now = time.monotonic()
        return all(name in self.state and now-self.received[name] < 1.5 for name in names)

    def _refresh_pose(self):
        if self.busy and not self.cancel_requested and not self.fresh(self.active_names):
            self.stop()
            self.on_result('反馈超过 1.5 秒，已请求取消运动', True)
        if self.fk_pending is not None or not self.fresh() or not self.fk.service_is_ready():
            return
        req = GetPositionFK.Request()
        req.header.frame_id = self.get_parameter('planning_frame').value
        req.fk_link_names = [self.get_parameter('tool_link').value]
        req.robot_state.joint_state.name = list(self.state)
        req.robot_state.joint_state.position = list(self.state.values())
        req.robot_state.is_diff = True
        sample_time = time.monotonic()
        self.fk_pending = self.fk.call_async(req)
        def done(future):
            self.fk_pending = None
            try:
                response = future.result()
                if response.error_code.val == 1 and response.pose_stamped:
                    self.pose = response.pose_stamped[0].pose
                    self.pose_received = sample_time
            except Exception as exc:
                self.get_logger().warning(str(exc))
        self.fk_pending.add_done_callback(done)

    def _send(self, group, constraints, velocity, acceleration):
        if self.busy:
            raise ValueError('已有运动或停止请求尚未结束')
        names = ARM if group == 'arm' else GRIPPER
        if not self.fresh(names):
            raise ValueError('关节反馈缺失或超过 1.5 秒，拒绝执行')
        if not self.move.server_is_ready():
            raise ValueError('MoveIt action 不可用')
        if not all(math.isfinite(v) and 0 < v <= 1 for v in (velocity, acceleration)):
            raise ValueError('速度和加速度比例必须在 (0, 1]')
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.pipeline_id = 'ompl'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = velocity
        req.max_acceleration_scaling_factor = acceleration
        req.start_state.is_diff = True
        req.goal_constraints = [constraints]
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = not self.get_parameter('enable_motion').value
        self.busy = True
        self.active_names = names
        self.cancel_requested = False
        self.last_result = None
        self.goal_handle = None
        self.on_result('正在规划' if goal.planning_options.plan_only else '正在规划并执行', False)
        try:
            future = self.move.send_goal_async(goal)
            future.add_done_callback(self._accepted)
        except Exception:
            self.busy = False
            raise

    def _accepted(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self._finish('运动请求被拒绝', True)
                return
            self.goal_handle = handle
            # Handles Stop pressed while send_goal was still awaiting acceptance.
            if self.cancel_requested:
                handle.cancel_goal_async()
            handle.get_result_async().add_done_callback(self._completed)
        except Exception as exc:
            self._finish(str(exc), True)

    def _completed(self, future):
        try:
            result = future.result()
            self.last_result = result
            ok = result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code.val == 1
            if self.cancel_requested:
                self._finish('停止请求已结束，未发送后续运动', not ok and result.status != GoalStatus.STATUS_CANCELED)
            else:
                self._finish('规划/执行完成' if ok else f'运动失败：{result.result.error_code.val}', not ok)
        except Exception as exc:
            self._finish(str(exc), True)

    def _finish(self, message, error):
        self.busy = False
        self.goal_handle = None
        self.on_result(message, error)

    def stop(self):
        if not self.busy:
            self.on_result('当前没有本界面发起的运动', False)
            return
        self.cancel_requested = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.on_result('已请求取消，等待控制器结束反馈', False)

    def joint_target(self, target, velocity=0.2, acceleration=0.2, group='arm'):
        names = ARM if group == 'arm' else GRIPPER
        if len(target) != len(names) or not all(math.isfinite(v) for v in target):
            raise ValueError('目标关节数据无效')
        if group == 'arm' and abs(target[2]) > math.radians(161):
            raise ValueError('i16H J3 目标超出 ±161° 限位')
        limits = (-2*math.pi, 2*math.pi) if group == 'arm' else (0.0, 0.04)
        if any(not limits[0] <= v <= limits[1] for v in target):
            raise ValueError('目标超出模型限位')
        c = Constraints()
        c.joint_constraints = [JointConstraint(joint_name=n, position=float(v),
            tolerance_above=0.001, tolerance_below=0.001, weight=1.0) for n,v in zip(names,target)]
        self._send(group, c, velocity, acceleration)

    def pose_target(self, xyz, velocity=0.2, acceleration=0.2):
        if self.pose is None or time.monotonic()-self.pose_received > 1.5:
            raise ValueError('末端位姿反馈已过期')
        if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
            raise ValueError('末端目标无效')
        c = Constraints()
        position = PositionConstraint()
        position.header.frame_id = self.get_parameter('planning_frame').value
        position.link_name = self.get_parameter('tool_link').value
        position.weight = 1.0
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
        pose.orientation.w = 1.0
        position.constraint_region.primitives = [sphere]
        position.constraint_region.primitive_poses = [pose]
        orientation = OrientationConstraint()
        orientation.header = position.header
        orientation.link_name = position.link_name
        orientation.orientation = self.pose.orientation
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        orientation.absolute_z_axis_tolerance = 0.01
        orientation.weight = 1.0
        c.position_constraints = [position]
        c.orientation_constraints = [orientation]
        self._send('arm', c, velocity, acceleration)
