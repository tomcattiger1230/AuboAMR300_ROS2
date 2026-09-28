"""Separate chassis transports: SEER TCP hardware and ROS/Nav2 Isaac."""
import math
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from PySide6.QtCore import QObject, QTimer, Signal
from .profile import velocity

class SeerBase(QObject):
    snapshot = Signal(object)
    message = Signal(str, bool)
    completed = Signal(object)

    def __init__(self, profile, log_dir, parent=None):
        super().__init__(parent)
        from .seer.client import AgvClient
        self.client = AgvClient(profile.seer_ip, timeout=.7, log_dir=Path(log_dir))
        self.manual_allowed = profile.real_manual
        self.poll_pool = ThreadPoolExecutor(max_workers=1)
        self.control_pool = ThreadPoolExecutor(max_workers=1)
        self.poll_future = self.control_future = None
        self.active = None
        self.last_feedback = 0
        self.position = {}
        self.nav_busy = False
        self.estop = True
        self.closing = False
        self.closed = False
        self.completed.connect(self._received)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self.last_poll = 0

    def _received(self, result):
        if self.closed:
            return
        kind, value = result
        if kind == 'poll':
            if value is None:
                self.last_feedback = 0
                self.stop()
                self.snapshot.emit({'connected': False, 'text': 'SEER 查询失败 / 反馈过期'})
                return
            self.last_feedback = time.monotonic()
            self.position = value['position']
            nav = value['navigation'].get('task_status')
            self.nav_busy = nav in (1, 2, 3)
            # Treat an unrecognised emergency-stop response as unavailable.
            from .seer.status_semantics import estop_is_triggered, ESTOP_TRIGGER_FIELDS
            self.estop = not any(k in value['estop'] for k in ESTOP_TRIGGER_FIELDS) or estop_is_triggered(value['estop'])
            self.snapshot.emit({'connected': True, 'text': '急停 / 急停反馈不可用' if self.estop else '', **value})
        else:
            if not value.ok and self.active:
                self.stop()
            if not value.ok:
                self.nav_busy = False
            self.message.emit(value.error or str(value.response), not value.ok)

    def _poll(self):
        results = [self.client.get_position(), self.client.get_speed(),
                   self.client.get_navigation_status(), self.client.get_estop_status()]
        if not all(r.ok for r in results):
            return None
        return dict(zip(('position', 'speed', 'navigation', 'estop'), (r.response for r in results)))

    def _submit(self, fn, kind='control'):
        pool = self.poll_pool if kind == 'poll' else self.control_pool
        future = pool.submit(fn)
        def done(f):
            if f.cancelled() or self.closed:
                return
            try:
                self.completed.emit((kind, f.result()))
            except Exception as exc:
                self.message.emit(str(exc), True)
        future.add_done_callback(done)
        return future

    def tick(self):
        now = time.monotonic()
        if not self.closing and now - self.last_poll > 1 and (self.poll_future is None or self.poll_future.done()):
            self.last_poll = now
            self.poll_future = self._submit(self._poll, 'poll')
        if self.active:
            if now - self.last_feedback > 4 or self.estop or self.nav_busy:
                self.stop()
                self.message.emit('底盘反馈过期、急停或导航占用，已请求停止', True)
            elif self.control_future is None or self.control_future.done():
                vx, w = self.active
                self.control_future = self._submit(lambda: self.client.open_loop_motion(vx, 0, w))

    def drive(self, linear, angular):
        if self.closing or not self.manual_allowed:
            raise RuntimeError('实机手动驾驶需要在连接配置中确认开环接口已完成现场验证')
        if time.monotonic() - self.last_feedback > 4 or self.nav_busy or self.estop:
            raise RuntimeError('需要新鲜底盘反馈、无急停且无导航任务')
        if self.control_future is not None and not self.control_future.done():
            raise RuntimeError('等待上一条控制请求完成')
        self.active = velocity(linear, angular)
        self.tick()

    def stop(self):
        was_active = self.active is not None
        self.active = None
        if was_active:
            self.control_future = self._submit(self.client.stop_open_loop_motion)

    def navigate_station(self, source, target):
        if not target.strip():
            raise ValueError('请填写目标站点 ID')
        source = source.strip() or str(self.position.get('current_station') or self.position.get('last_station') or '').strip()
        if not source:
            raise ValueError('当前位置没有站点 ID，请填写起点 ID')
        if self.active or self.nav_busy or self.estop or time.monotonic() - self.last_feedback > 4:
            raise RuntimeError('底盘未就绪或已有运动任务')
        if self.control_future is not None and not self.control_future.done():
            raise RuntimeError('等待上一条控制请求完成')
        self.nav_busy = True
        self.control_future = self._submit(lambda: self.client.path_navigation(source, target.strip(), 'unified-'+uuid.uuid4().hex[:12]))

    def pause(self):
        if self.nav_busy:
            self.control_future = self._submit(self.client.pause_navigation)

    def resume(self):
        if self.nav_busy:
            self.control_future = self._submit(self.client.continue_navigation)

    def cancel(self):
        self.stop()
        self.control_future = self._submit(self.client.cancel_navigation)

    def ready_to_close(self):
        if not self.closing:
            self.closing = True
            self.stop()
            if self.nav_busy:
                self.control_future = self._submit(self.client.cancel_navigation)
        return (self.control_future is None or self.control_future.done()) and (self.poll_future is None or self.poll_future.done())

    def close(self):
        self.closed = True
        self.timer.stop()
        self.poll_pool.shutdown(wait=False, cancel_futures=True)
        self.control_pool.shutdown(wait=False, cancel_futures=True)

class IsaacBase(QObject):
    snapshot = Signal(object)
    message = Signal(str, bool)
    map_received = Signal(object)
    path_received = Signal(object)
    map_status = Signal(str)
    map_applied = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
        from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid, Odometry, Path
        from nav2_msgs.action import NavigateToPose
        from nav2_msgs.srv import LoadMap
        from tf2_ros import Buffer, TransformListener
        self.ros = rclpy
        self.node = Node('unified_chassis_client', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.twist_type = Twist
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.nav = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.map_client = self.node.create_client(LoadMap, '/map_server/load_map')
        self.initial_pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.map_future = None
        self.map_deadline = 0
        self.map_ready = False
        self.localization_required = False
        self.localization_pending = False
        self.initial_stamp = None
        self.localize_deadline = 0
        self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._localized, 10)
        self.node.create_subscription(OccupancyGrid, '/map', self._map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(Path, '/plan', lambda msg: self.path_received.emit(msg), 10)
        self.node.create_subscription(Odometry, '/odom', self._odom, qos_profile_sensor_data)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self.node)
        self.last_feedback = 0
        self.position = {}
        self.speed = {}
        self.active = None
        self.goal = None
        self.nav_pending = False
        self.cancel_requested = False
        self.last_send = 0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(20)

    def _odom(self, msg):
        self.last_feedback = time.monotonic()
        self.speed = {'vx': msg.twist.twist.linear.x, 'w': msg.twist.twist.angular.z}

    def _map(self, msg):
        if msg.header.frame_id.strip('/') != 'map' or not msg.info.width or len(msg.data) != msg.info.width*msg.info.height:
            return
        self.map_ready = True
        self.map_received.emit(msg)

    def load_map(self, remote_path):
        from nav2_msgs.srv import LoadMap
        if not remote_path.strip().startswith('/'):
            raise ValueError('请填写 ROS 主机上的 YAML 绝对路径')
        if self.active or self.nav_pending or self.map_future or self.localization_pending:
            raise RuntimeError('请先停止运动，并等待当前地图 / 定位请求结束')
        if any('slam_toolbox' in item.node_name for item in self.node.get_publishers_info_by_topic('/map')):
            raise RuntimeError('请先停止 SLAM 建图，使用仿真定位启动器，避免两套地图和 TF 冲突')
        if not self.map_client.service_is_ready():
            raise RuntimeError('地图服务未就绪：请在 ROS 主机启动 isaac_localization.launch.py')
        request = LoadMap.Request(); request.map_url = remote_path.strip()
        self.localization_required = True
        self.initial_stamp = None
        self.map_deadline = time.monotonic()+15
        self.map_future = self.map_client.call_async(request)
        self.map_status.emit('正在应用远程地图…')
        self.map_future.add_done_callback(self._loaded)

    def _loaded(self, future):
        if future is not self.map_future: return
        self.map_future = None
        try:
            response = future.result()
            if response.result != 0 or not response.map.info.width:
                raise RuntimeError(f'远程地图加载失败（代码 {response.result}）；请核对 ROS 主机路径')
            self.position = {}
            self._map(response.map)
            self.path_received.emit(self._empty_path())
            self.map_applied.emit()
            self.map_status.emit('远程地图已应用 · 请设置初始位置')
        except Exception as exc:
            self.map_status.emit('地图应用失败 · 请查看日志')
            self.message.emit(str(exc), True)

    def _empty_path(self):
        from nav_msgs.msg import Path
        return Path()

    def set_initial_pose(self, x, y, yaw):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        if not all(math.isfinite(v) for v in (x,y,yaw)): raise ValueError('初始位姿无效')
        if not self.map_ready or self.map_future or self.active or self.nav_pending or self.localization_pending:
            raise RuntimeError('需要远程地图就绪，且没有运动、地图或定位请求')
        if any('slam_toolbox' in item.node_name for item in self.node.get_publishers_info_by_topic('/map')):
            raise RuntimeError('初始位置用于 AMCL 定位；请先停止 SLAM 并启动仿真定位')
        if not self.initial_pub.get_subscription_count():
            raise RuntimeError('没有定位节点接收初始位置，请检查 AMCL 和局域网连接')
        msg = PoseWithCovarianceStamped(); msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = .25
        msg.pose.covariance[35] = math.radians(15)**2
        self.initial_stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        self.localization_required = self.localization_pending = True
        self.localize_deadline = time.monotonic()+15
        self.position = {}
        self.initial_pub.publish(msg)
        self.map_status.emit('初始位置已发送 · 等待 AMCL 定位反馈')

    def _localized(self, msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)): return
        if (self.localization_pending and msg.header.frame_id.strip('/') == 'map'
                and self.initial_stamp is not None and stamp >= self.initial_stamp):
            self.localization_pending = self.localization_required = False
            self.map_status.emit('AMCL 已返回定位结果 · 导航还需 Nav2 和实时 TF 就绪')

    def tick(self):
        for _ in range(6):
            self.ros.spin_once(self.node, timeout_sec=0)
        now = time.monotonic()
        if self.map_future and now > self.map_deadline:
            future = self.map_future; self.map_future = None
            self.map_client.remove_pending_request(future)
            future.cancel()
            self.map_status.emit('地图请求超时 · 请确认远程地图状态')
        if self.localization_pending and now > self.localize_deadline:
            self.localization_pending = False
            self.map_status.emit('定位反馈超时 · 请检查 AMCL 后重新设置初始位置')
        connected = now - self.last_feedback < 1.5
        if self.active and not connected:
            self.stop()
        try:
            stamped = self.tf.lookup_transform('map', 'base_footprint', self.ros.time.Time())
            stamp = stamped.header.stamp.sec*10**9+stamped.header.stamp.nanosec
            if (abs(self.node.get_clock().now().nanoseconds-stamp) > 1.5e9
                    or (self.initial_stamp is not None and stamp < self.initial_stamp)):
                raise RuntimeError('等待实时 map TF')
            t = stamped.transform
            q = t.rotation
            self.position = {'x': t.translation.x, 'y': t.translation.y,
                             'angle': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}
        except Exception:
            self.position = {}
        self.snapshot.emit({'connected': connected, 'position': self.position,
                            'speed': self.speed, 'navigation': {'task_status': 2 if self.nav_pending else 0},
                            'text': 'Nav2 可用' if self.nav.server_is_ready() else '等待 Nav2'})
        if self.active and now - self.last_send >= .1:
            msg = self.twist_type()
            msg.linear.x, msg.angular.z = self.active
            self.pub.publish(msg)
            self.last_send = now

    def drive(self, linear, angular):
        if time.monotonic() - self.last_feedback > 1.5 or self.nav_pending or self.map_future or self.localization_pending:
            raise RuntimeError('等待底盘反馈；导航期间不能手动驾驶')
        self.active = velocity(linear, angular)

    def stop(self):
        if self.active:
            self.active = None
            self.pub.publish(self.twist_type())

    def navigate_pose(self, x, y, yaw, frame='map'):
        from nav2_msgs.action import NavigateToPose
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('目标位姿无效')
        if self.map_future or self.localization_required:
            raise RuntimeError('请等待地图应用完成并设置初始位置，收到定位反馈后再导航')
        if time.monotonic()-self.last_feedback > 1.5 or not self.nav.server_is_ready() or not self.position or self.nav_pending or self.active:
            raise RuntimeError('需要 Nav2、map TF 就绪且无运动任务')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw/2)
        goal.pose.pose.orientation.w = math.cos(yaw/2)
        self.nav_pending = True
        self.cancel_requested = False
        future = self.nav.send_goal_async(goal)
        def accepted(f):
            try:
                self.goal = f.result()
                if not self.goal.accepted:
                    self.nav_pending = False
                    self.goal = None
                    self.message.emit('Nav2 拒绝目标', True)
                    return
                self.goal.get_result_async().add_done_callback(finished)
                if self.cancel_requested:
                    self.goal.cancel_goal_async()
            except Exception as exc:
                self.nav_pending = False
                self.message.emit(str(exc), True)
        def finished(f):
            self.nav_pending = False
            self.goal = None
            try:
                status = f.result().status
                label = {4:'已到达目标',5:'已取消',6:'失败'}.get(status,f'未知状态 {status}')
                self.message.emit('导航：'+label, status not in (4,5))
            except Exception as exc:
                self.message.emit(str(exc), True)
        future.add_done_callback(accepted)

    def cancel(self):
        self.stop()
        self.cancel_requested = True
        if self.goal:
            self.goal.cancel_goal_async()

    def ready_to_close(self):
        self.cancel()
        return not self.nav_pending and self.map_future is None

    def close(self):
        self.timer.stop()
        self.node.destroy_node()
