"""Common MoveIt client for both Isaac and hardware; no vendor SDK imports."""
import math
import copy
import time

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, PlanningScene
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
        self.phase = 'idle'
        self.revision = 0
        self.request_revision = 0
        self.plan = None
        self.plan_start = {}
        self.plan_group = None
        self.request_start = {}
        self.on_plan = lambda trajectory: None
        self.scene_signature = None
        self.scene_frames = {}
        self.on_result = lambda message, error: None
        self.move = ActionClient(self, MoveGroup, 'move_action')
        self.execute = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.stop_event = self.create_publisher(String, 'trajectory_execution_event', 1)
        self.create_subscription(PlanningScene, 'monitored_planning_scene', self._scene, 1)
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

    def _scene(self, scene):
        # Keep a baseline so tiny odometry/physics jitter does not invalidate every preview.
        # Cumulative frame movement beyond 1 mm or 0.001 rad still invalidates the plan.
        frame_changed = False
        for frame in scene.fixed_frame_transforms:
            key = (frame.header.frame_id, frame.child_frame_id)
            t, q = frame.transform.translation, frame.transform.rotation
            value = ([t.x,t.y,t.z], [q.x,q.y,q.z,q.w])
            old = self.scene_frames.get(key)
            if old is None:
                frame_changed |= bool(self.scene_frames)
                self.scene_frames[key] = value
            else:
                dot = min(1., abs(sum(a*b for a,b in zip(old[1],value[1]))))
                if math.dist(old[0],value[0]) > .001 or 2*math.acos(dot) > .001:
                    frame_changed = True
                    self.scene_frames[key] = value
        if frame_changed:
            self.invalidate_plan('场景坐标变换已变化，请重新规划')
        if (scene.is_diff and not scene.world.collision_objects
                and not scene.world.octomap.octomap.data
                and not scene.robot_state.attached_collision_objects
                and not scene.allowed_collision_matrix.entry_names
                and not scene.allowed_collision_matrix.default_entry_names):
            return
        scene = copy.deepcopy(scene)
        objects = list(scene.world.collision_objects) + [a.object for a in scene.robot_state.attached_collision_objects]
        for obj in objects:
            obj.header.stamp.sec = 0
            obj.header.stamp.nanosec = 0
        scene.world.octomap.header.stamp.sec = 0
        scene.world.octomap.header.stamp.nanosec = 0
        signature = repr((scene.world, scene.robot_state.attached_collision_objects,
                          scene.allowed_collision_matrix))
        if self.scene_signature is not None and signature != self.scene_signature:
            self.invalidate_plan('规划场景已变化，请重新规划')
        self.scene_signature = signature

    def invalidate_plan(self, reason='目标或参数已改变，请重新规划'):
        self.revision += 1
        had_plan = self.plan is not None
        self.plan = None
        self.on_plan(None)
        if not self.busy:
            self.phase = 'idle'
        if had_plan:
            self.on_result(reason, False)

    def _start_matches(self, start):
        return self.fresh(ARM + GRIPPER) and all(
            name in self.state and abs(self.state[name]-value) <= (.003 if name in GRIPPER else .01)
            for name, value in start.items())

    def plan_ready(self):
        if self.plan is not None and not self._start_matches(self.plan_start):
            self.invalidate_plan('起始状态已改变或反馈过期，请重新规划')
        return self.plan is not None and not self.busy

    def execute_planned(self):
        if self.busy:
            raise ValueError('已有运动或停止请求尚未结束')
        if not self.get_parameter('enable_motion').value:
            raise ValueError('当前客户端禁止执行，请使用允许执行的启动方式')
        if not self.plan_ready():
            raise ValueError('没有有效轨迹，请先规划')
        if not self.execute.server_is_ready():
            raise ValueError('MoveIt ExecuteTrajectory action 不可用')
        if self.stop_event.get_subscription_count() == 0:
            raise ValueError('MoveIt 停止事件接口未就绪，请稍后重试')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = copy.deepcopy(self.plan)
        self.plan = None  # A trajectory can be executed only once.
        self.on_plan(None)
        self.phase = 'executing'
        self.busy = True
        self.active_names = ARM + GRIPPER
        self.cancel_requested = False
        self.goal_handle = None
        self.last_result = None
        self.on_result('正在执行已预览的轨迹', False)
        try:
            self.execute.send_goal_async(goal).add_done_callback(self._accepted)
        except Exception:
            self.busy = False
            self.phase = 'error'
            raise

    def fresh(self, names=ARM):
        now = time.monotonic()
        return all(name in self.state and now-self.received[name] < 1.5 for name in names)

    def _refresh_pose(self):
        self.plan_ready()
        if self.busy and self.cancel_requested and self.phase == 'executing':
            self._stop_execution()
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
        names = ARM + GRIPPER
        if not self.fresh(names):
            raise ValueError('关节反馈缺失或超过 1.5 秒，拒绝执行')
        if not self.move.server_is_ready():
            raise ValueError('MoveIt action 不可用')
        if not all(math.isfinite(v) and 0 < v <= 1 for v in (velocity, acceleration)):
            raise ValueError('速度和加速度比例必须在 (0, 1]')
        self.invalidate_plan()
        self.request_revision = self.revision
        self.request_start = {n: self.state[n] for n in ARM + GRIPPER}
        self.plan_group = group
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.pipeline_id = 'ompl'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = velocity
        req.max_acceleration_scaling_factor = acceleration
        req.start_state.is_diff = True
        req.start_state.joint_state.name = list(self.state)
        req.start_state.joint_state.position = list(self.state.values())
        req.goal_constraints = [constraints]
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = True
        self.phase = 'planning'
        self.busy = True
        self.active_names = names
        self.cancel_requested = False
        self.last_result = None
        self.goal_handle = None
        self.on_result('正在规划，机器人保持不动', False)
        try:
            future = self.move.send_goal_async(goal)
            future.add_done_callback(self._accepted)
        except Exception:
            self.busy = False
            self.phase = 'error'
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
                if self.phase == 'executing': self._stop_execution()
            handle.get_result_async().add_done_callback(self._completed)
        except Exception as exc:
            self._finish(str(exc), True)

    def _completed(self, future):
        try:
            result = future.result()
            self.last_result = result
            ok = result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code.val == 1
            if self.cancel_requested and ok and self.phase == 'executing':
                self.phase = 'done'
                self._finish('取消请求到达前轨迹已执行完成', False)
            elif self.cancel_requested:
                self.phase = 'stopped'
                self._finish('已停止，原轨迹已失效', result.status != GoalStatus.STATUS_CANCELED and result.result.error_code.val != -7 and not ok)
            elif not ok:
                self.phase = 'error'
                self._finish(f'请求失败：{result.result.error_code.val}', True)
            elif self.phase == 'planning':
                trajectory = result.result.planned_trajectory
                jt = trajectory.joint_trajectory
                if self.request_revision != self.revision or not self._start_matches(self.request_start):
                    self.phase = 'idle'
                    self._finish('目标、场景或起始状态已改变，请重新规划', True)
                elif not self._valid_trajectory(trajectory):
                    self.phase = 'error'
                    self._finish('规划轨迹无效或起始点与反馈不一致', True)
                else:
                    self.plan = copy.deepcopy(trajectory)
                    self.plan_start = dict(self.request_start)
                    self.phase = 'ready'
                    self._finish('规划成功：可播放预览，再点击“执行已规划轨迹”', False)
                    self.on_plan(copy.deepcopy(self.plan))
            else:
                self.phase = 'done'
                self._finish('轨迹执行完成', False)
        except Exception as exc:
            self.plan = None
            self.phase = 'error'
            self._finish(str(exc), True)

    def _valid_trajectory(self, trajectory):
        jt = trajectory.joint_trajectory
        expected = ARM if self.plan_group == 'arm' else GRIPPER
        if not jt.points or set(jt.joint_names) != set(expected) or len(jt.joint_names) != len(expected):
            return False
        if trajectory.multi_dof_joint_trajectory.points:
            return False
        previous = -1.0
        for point in jt.points:
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            if t < 0 or t <= previous or len(point.positions) != len(expected):
                return False
            if not all(math.isfinite(v) for v in (*point.positions, *point.velocities, *point.accelerations)):
                return False
            previous = t
        return all(abs(value-self.request_start[name]) <= (.003 if name in GRIPPER else .01)
                   for name, value in zip(jt.joint_names, jt.points[0].positions))

    def _finish(self, message, error):
        self.busy = False
        self.goal_handle = None
        if error: self.phase = 'error'
        self.on_result(message, error)

    def _stop_execution(self):
        # Same stop path as MoveGroupInterface.stop(). Some MoveIt versions accept
        # ExecuteTrajectory cancellation without stopping the execution manager.
        self.stop_event.publish(String(data='stop'))

    def stop(self):
        self.invalidate_plan('已丢弃轨迹')
        if not self.busy:
            self.phase = 'stopped'
            self.on_result('已停止预览并丢弃轨迹', False)
            return
        self.cancel_requested = True
        if self.phase == 'executing': self._stop_execution()
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

    def pose_target(self, xyz, velocity=0.2, acceleration=0.2, quaternion=None):
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
        if quaternion is None:
            orientation.orientation = self.pose.orientation
        else:
            if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
                raise ValueError('目标姿态无效')
            norm = math.sqrt(sum(v*v for v in quaternion))
            if norm < 1e-8:
                raise ValueError('目标四元数不能为零')
            q = [float(v/norm) for v in quaternion]
            orientation.orientation.x, orientation.orientation.y, orientation.orientation.z, orientation.orientation.w = q
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        orientation.absolute_z_axis_tolerance = 0.01
        orientation.weight = 1.0
        c.position_constraints = [position]
        c.orientation_constraints = [orientation]
        self._send('arm', c, velocity, acceleration)
