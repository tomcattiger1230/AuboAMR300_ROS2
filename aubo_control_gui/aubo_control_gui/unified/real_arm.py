"""Real AUBO panel using the student's ROS bridge services, without auto startup."""
import math
import time
from PySide6.QtCore import QTimer, Signal
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QLabel, QDoubleSpinBox, QPushButton, QGroupBox, QScrollArea
from ..resources import get_package_share_directory
from ..urdf_view import UrdfRobotView
from pathlib import Path

class RealArm(QWidget):
    connection = Signal(bool)
    def __init__(self, report, parent=None):
        super().__init__(parent)
        import rclpy
        from sensor_msgs.msg import JointState
        from aubo_bridge_msgs.msg import RobotStatus, TrajectoryCommand
        from aubo_bridge_msgs.srv import MoveJoint, MoveToPose, GetRobotInfo
        from std_msgs.msg import String, Float64
        self.ros = rclpy
        self.node = rclpy.create_node('unified_real_arm')
        self.report = report
        self.joint_type, self.pose_type, self.info_type = MoveJoint, MoveToPose, GetRobotInfo
        self.command_type, self.string_type = TrajectoryCommand, String
        self.move = self.node.create_client(MoveJoint, '/aubo/move_joint')
        self.pose_move = self.node.create_client(MoveToPose, '/aubo/move_to_pose')
        self.info = self.node.create_client(GetRobotInfo, '/aubo/get_robot_info')
        self.commands = self.node.create_publisher(TrajectoryCommand, '/aubo/trajectory_command', 10)
        self.gripper = self.node.create_publisher(String, '/aubo/gripper_command', 10)
        self.gripper_position = None
        self.gripper_time = 0
        self.node.create_subscription(Float64, '/aubo/gripper_position', self.on_gripper, 10)
        self.node.create_subscription(RobotStatus, '/aubo/status', self.on_status, 10)
        self.node.create_subscription(JointState, '/aubo/realtime_joint_states', lambda m: self.on_joints(m, True), 20)
        self.node.create_subscription(JointState, '/joint_states', lambda m: self.on_joints(m, False), 10)
        self.status = None
        self.status_time = self.joint_time = self.realtime_time = 0
        self.actual = None
        self.future = None
        self.stopping = False
        self.stop_requested_at = 0
        root = QVBoxLayout(self)
        self.feedback = QLabel('等待实机 AUBO Bridge：/aubo/status 与关节反馈')
        root.addWidget(self.feedback)
        note = QLabel('实机运动通过 AUBO SDK 桥执行；此页不提供 MoveIt 碰撞规划。目标需在机器人基座坐标系中填写。')
        note.setWordWrap(True); root.addWidget(note)
        row = QHBoxLayout(); root.addLayout(row, 1)
        group = QGroupBox('实机目标'); form = QFormLayout(group)
        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(group); scroll.setMinimumWidth(330); row.addWidget(scroll)
        self.boxes = []
        for i in range(6):
            b = QDoubleSpinBox(); b.setRange(-161 if i == 2 else -360, 161 if i == 2 else 360); b.setSuffix(' °'); b.setDecimals(2)
            self.boxes.append(b); form.addRow(f'J{i+1}', b)
        self.speed = QDoubleSpinBox(); self.speed.setRange(.01, .3); self.speed.setValue(.1); self.speed.setSuffix(' rad/s')
        form.addRow('关节速度', self.speed)
        self.gripper_feedback = QLabel('夹爪反馈：--（设备原始位置单位）'); form.addRow(self.gripper_feedback)
        self.actions = []
        def button(text, fn, motion=False):
            b = QPushButton(text); b.clicked.connect(fn); form.addRow(b)
            if motion: self.actions.append(b)
            return b
        button('当前关节 → 目标', self.copy_actual)
        button('执行关节目标', self.execute_joints, True)
        self.pose_boxes = []
        for name in ('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'):
            b = QDoubleSpinBox(); b.setDecimals(4 if name in ('X','Y','Z') else 2)
            b.setRange(-3 if name in ('X','Y','Z') else -360, 3 if name in ('X','Y','Z') else 360)
            b.setSuffix(' m' if name in ('X','Y','Z') else ' °')
            self.pose_boxes.append(b); form.addRow(name, b)
        button('当前末端 → 位姿目标', self.copy_pose)
        button('执行末端位姿', self.execute_pose, True).setToolTip('当前 SDK 桥将速度参数 0.02 用作关节限速，不代表 TCP 线速度')
        button('夹爪打开', lambda: self.command_gripper('open'), True)
        button('夹爪闭合', lambda: self.command_gripper('close'), True)
        button('停止机械臂', self.stop)
        share = Path(get_package_share_directory('aubo_control_gui'))
        urdf = Path(get_package_share_directory('aubo_student_description'))/'urdf/aubo_i16.urdf'
        self.view = UrdfRobotView(urdf, share/'qml/RobotView.qml')
        self.view.rootObject().setProperty('targetEnabled', False)
        row.addWidget(self.view, 1)
        self.timer = QTimer(self); self.timer.timeout.connect(self.tick); self.timer.start(30)
        self.tick()

    def on_gripper(self, msg):
        self.gripper_position = msg.data; self.gripper_time = time.monotonic()

    def on_status(self, msg):
        self.status = msg; self.status_time = time.monotonic()

    def on_joints(self, msg, realtime):
        now = time.monotonic()
        if not realtime and now-self.realtime_time < .5:
            return
        names = ('shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint')
        values = dict(zip(msg.name, msg.position))
        if not all(n in values for n in names):
            return
        self.actual = [values[n] for n in names]
        self.joint_time = now
        if realtime: self.realtime_time = now
        self.view.set_joint_positions(self.actual)

    def ready(self):
        now = time.monotonic()
        return bool(self.status and now-self.status_time < 1.5 and now-self.joint_time < 1.5
                    and self.status.robot_state == self.status.IDLE and self.status.power_on
                    and self.status.brakes_released and self.future is None and not self.stopping)

    def tick(self):
        for _ in range(6): self.ros.spin_once(self.node, timeout_sec=0)
        fresh = self.status and time.monotonic()-self.status_time < 1.5
        self.connection.emit(bool(fresh and self.actual and time.monotonic()-self.joint_time < 1.5 and self.status.robot_state != self.status.DISCONNECTED))
        self.gripper_feedback.setText('夹爪反馈：'+(str(self.gripper_position) if time.monotonic()-self.gripper_time < 1.5 else '-- / 过期')+'（设备原始位置单位）')
        if self.stopping and fresh and self.status_time > self.stop_requested_at and self.status.robot_state == self.status.IDLE and self.future is None:
            self.stopping = False
        self.feedback.setText('实机反馈：'+(', '.join(f'J{i+1} {math.degrees(v):.1f}°' for i,v in enumerate(self.actual)) if self.actual and fresh else '等待 / 过期'))
        if fresh:
            self.feedback.setText(self.feedback.text()+f'\nTCP / m：{self.status.tool_position_x:.4f}, {self.status.tool_position_y:.4f}, {self.status.tool_position_z:.4f}')
        for b in self.actions: b.setEnabled(self.ready())

    def copy_actual(self):
        if self.actual:
            for b, v in zip(self.boxes, self.actual): b.setValue(math.degrees(v))

    def copy_pose(self):
        if self.future is not None or not self.info.service_is_ready():
            self.report('等待当前请求或机器人信息服务', True); return
        self.future = self.info.call_async(self.info_type.Request())
        def done(f):
            self.future = None
            try:
                result = f.result()
                if not result.success: raise RuntimeError(result.message)
                from ..view_math import rpy_from_quaternion
                w,x,y,z = result.current_ori
                values = list(result.current_pos)+[math.degrees(v) for v in rpy_from_quaternion((x,y,z,w))]
                for b,v in zip(self.pose_boxes,values): b.setValue(v)
            except Exception as exc: self.report(str(exc),True)
        self.future.add_done_callback(done)

    def send(self, client, request):
        if not self.ready() or not client.service_is_ready():
            self.report('机械臂未就绪、反馈过期或运动服务不可用', True); return
        self.future = client.call_async(request)
        self.report('已发送实机运动请求，等待桥接服务结果', False)
        def done(f):
            self.future = None
            try:
                result = f.result(); self.report(result.message, not result.success)
            except Exception as exc: self.report(str(exc), True)
        self.future.add_done_callback(done)

    def execute_joints(self):
        req = self.joint_type.Request(); req.target_joint = [math.radians(b.value()) for b in self.boxes]
        req.max_vel = [self.speed.value()]*6; req.max_acc = [.1]*6; req.enable_move = True
        self.send(self.move, req)

    def execute_pose(self):
        from ..view_math import quaternion_from_rpy
        req = self.pose_type.Request()
        req.target_pose.position.x, req.target_pose.position.y, req.target_pose.position.z = [b.value() for b in self.pose_boxes[:3]]
        q = quaternion_from_rpy(*(math.radians(b.value()) for b in self.pose_boxes[3:]))
        req.target_pose.orientation.x, req.target_pose.orientation.y, req.target_pose.orientation.z, req.target_pose.orientation.w = q
        req.max_velocity = .02; req.max_acceleration = .02; req.blocking = True
        self.send(self.pose_move, req)

    def command_gripper(self, value):
        if not self.ready() or self.node.count_subscribers('/aubo/gripper_command') == 0:
            self.report('机械臂未就绪或夹爪桥未连接', True); return
        msg = self.string_type(); msg.data = value; self.gripper.publish(msg)
        self.report(f'已发送夹爪 {value}，等待设备反馈', False)

    def stop(self):
        if self.future is not None or (self.status and self.status.robot_state == self.status.RUNNING):
            if not self.stopping: self.stop_requested_at = time.monotonic()
            self.stopping = True
            msg = self.command_type(); msg.command = self.command_type.STOP_TRAJECTORY; self.commands.publish(msg)
            self.report('已请求机械臂停止，等待反馈终态', False)

    def ready_to_close(self):
        self.stop()
        return self.future is None and not self.stopping

    def close(self):
        self.timer.stop(); self.node.destroy_node()
