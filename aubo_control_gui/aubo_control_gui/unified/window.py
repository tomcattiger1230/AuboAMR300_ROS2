"""Desktop shell; opening it never connects to or starts a robot."""
import math
import re
import sys
import time
from pathlib import Path
from PySide6.QtCore import QEvent, QTimer, Qt
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QFormLayout, QGroupBox, QLabel, QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit,
    QPushButton, QCheckBox, QTabWidget, QSplitter, QPlainTextEdit, QFileDialog, QScrollArea)
from .profile import Profile
from .map_canvas import MapCanvas
from .seer.map_view import MapView
from .seer.smap import load_smap_file, extract_station_like_points
from .ros_map import load_yaml_map
from .isaac_video import IsaacVideo

NAV_STATES = {0:'待命', 1:'等待', 2:'导航中', 3:'已暂停', 4:'完成', 5:'失败', 6:'已取消'}

class UnifiedWindow(QMainWindow):
    def __init__(self, profile=None):
        super().__init__()
        from .seer.styles import apply_app_style
        apply_app_style(QApplication.instance())
        self.setWindowTitle('SEER + AUBO · 复合机器人控制台')
        self.resize(1440, 960)
        self.base = self.arm = None
        self.profile = None
        self.ros = None
        self.closing = False
        self.preview_map = False
        self.remote_grid = None
        self.remote_map_status = '等待 ROS /map'
        root = QWidget(); outer = QVBoxLayout(root); self.setCentralWidget(root)
        header = QHBoxLayout()
        title = QLabel('SEER + AUBO'); title.setStyleSheet('font-size:26px;font-weight:700;color:#16365b')
        header.addWidget(title); header.addStretch()
        self.mode_badge = QLabel(); header.addWidget(self.mode_badge)
        self.stop_all = QPushButton('停止底盘与机械臂'); self.stop_all.setObjectName('stopAll'); self.stop_all.clicked.connect(self.stop_motion)
        header.addWidget(self.stop_all); outer.addLayout(header)
        config = QGroupBox('机器人连接 · 本机与设备处于同一局域网')
        form = QHBoxLayout(config)
        self.mode = QComboBox(); self.mode.addItem('仿真 · ROS 2 + Isaac Sim', 'isaac'); self.mode.addItem('真实 · SEER + AUBO', 'real')
        self.domain = QSpinBox(); self.domain.setRange(0,232)
        self.peer = QLineEdit(); self.peer.setMaximumWidth(180)
        self.seer_ip = QLineEdit(); self.seer_ip.setMaximumWidth(180)
        for label, widget in [('模式',self.mode),('ROS 域',self.domain),('ROS 桥主机 IP',self.peer),('SEER IP',self.seer_ip)]:
            form.addWidget(QLabel(label)); form.addWidget(widget)
        self.manual_verified = QCheckBox('实机开环接口已现场验证')
        form.addWidget(self.manual_verified)
        self.connect_button = QPushButton('连接机器人'); self.connect_button.clicked.connect(self.connect_robot); form.addWidget(self.connect_button)
        self.disconnect_button = QPushButton('断开'); self.disconnect_button.clicked.connect(self.disconnect_robot); self.disconnect_button.setEnabled(False); form.addWidget(self.disconnect_button)
        outer.addWidget(config)
        status = QHBoxLayout()
        self.chassis_status = QLabel('底盘：未连接'); self.arm_status = QLabel('机械臂：未连接'); self.motion_status = QLabel('导航：待命')
        for label in (self.chassis_status,self.arm_status,self.motion_status):
            label.setStyleSheet('background:#eaf0f7;padding:12px;border-radius:6px;font-weight:600'); status.addWidget(label)
        outer.addLayout(status)
        self.tabs = QTabWidget(); outer.addWidget(self.tabs,1)
        self.base_page = QWidget(); self.tabs.addTab(self.base_page,'底盘与导航')
        split_layout = QVBoxLayout(self.base_page); split = QSplitter(); split_layout.addWidget(split)
        self.maps = QTabWidget(); self.ros_map = MapCanvas(); self.seer_map = MapView()
        self.maps.addTab(self.ros_map,'ROS 实时地图'); self.maps.addTab(self.seer_map,'SEER 站点地图'); split.addWidget(self.maps)
        controls = QWidget(); ctl = QVBoxLayout(controls)
        self.pose_label = QLabel('位置：--\n速度：--'); ctl.addWidget(self.pose_label)
        manual = QGroupBox('手动驾驶 · 按住运动，松开停止'); mf = QFormLayout(manual)
        self.linear = QDoubleSpinBox(); self.linear.setRange(.01,.3); self.linear.setValue(.1); self.linear.setSuffix(' m/s')
        self.angular = QDoubleSpinBox(); self.angular.setRange(.01,.6); self.angular.setValue(.2); self.angular.setSuffix(' rad/s')
        mf.addRow('线速度',self.linear); mf.addRow('角速度',self.angular)
        self.drive_buttons = []
        for label, direction in [('前进',(1,0)),('后退',(-1,0)),('左转',(0,1)),('右转',(0,-1))]:
            b = QPushButton(label); b.pressed.connect(lambda d=direction:self.drive(*d)); b.released.connect(self.stop_base)
            mf.addRow(b); self.drive_buttons.append(b)
        b = QPushButton('停止底盘'); b.clicked.connect(self.stop_base); mf.addRow(b)
        self.sim_nav = QGroupBox('仿真导航 · map 坐标'); nf = QFormLayout(self.sim_nav)
        self.map_notice = QLabel('等待 ROS /map'); self.map_notice.setWordWrap(True); nf.addRow(self.map_notice)
        b = QPushButton('载入仿真地图 · YAML'); b.clicked.connect(self.load_sim_map); nf.addRow(b)
        self.remote_map_path = QLineEdit()
        self.remote_map_path.setPlaceholderText('ROS 主机上的 YAML 绝对路径')
        self.remote_map_path.setToolTip('本地文件用于预览；应用时由 ROS 主机读取此路径')
        nf.addRow('远程地图路径',self.remote_map_path)
        b = QPushButton('应用到仿真'); b.clicked.connect(self.apply_sim_map); nf.addRow(b)
        b = QPushButton('显示 ROS 实时地图'); b.clicked.connect(self.show_live_map); nf.addRow(b)
        self.initial_select = QCheckBox('在地图上选择初始位置与朝向')
        self.initial_select.toggled.connect(lambda selected:setattr(self.ros_map,'selection_mode','initial' if selected else 'goal'))
        nf.addRow(self.initial_select)
        self.initial_boxes = []
        for name in ('初始 X / m','初始 Y / m','初始朝向 / °'):
            b = QDoubleSpinBox(); b.setRange(-180,180) if '°' in name else b.setRange(-1000,1000); b.setDecimals(3)
            self.initial_boxes.append(b); nf.addRow(name,b)
        b = QPushButton('设置初始位置'); b.clicked.connect(self.send_initial_pose); nf.addRow(b)
        self.goal_boxes = []
        for name in ('X / m','Y / m','朝向 / °'):
            b = QDoubleSpinBox(); b.setRange(-1000,1000) if '°' not in name else b.setRange(-180,180); b.setDecimals(3)
            self.goal_boxes.append(b); nf.addRow(name,b)
        b = QPushButton('发送导航目标'); b.clicked.connect(self.navigate_pose); nf.addRow(b); ctl.addWidget(self.sim_nav)
        self.real_nav = QGroupBox('实机导航 · SEER 地图站点'); rf = QFormLayout(self.real_nav)
        self.source = QLineEdit(); self.source.setPlaceholderText('留空从当前站点反馈获取')
        self.target = QLineEdit(); self.target.setPlaceholderText('例如 LM1；双击地图站点可填入')
        self.station_choices = QComboBox()
        self.station_choices.addItem('请先载入 .smap 地图', None)
        self.station_choices.setEnabled(False)
        self.station_choices.currentIndexChanged.connect(self.choose_station)
        self.target.textChanged.connect(self.sync_station_selection)
        rf.addRow('LM 点位选择',self.station_choices)
        rf.addRow('起点 ID',self.source); rf.addRow('目标 ID',self.target)
        b = QPushButton('导航到站点'); b.clicked.connect(self.navigate_station); rf.addRow(b)
        b = QPushButton('暂停导航'); b.clicked.connect(lambda:self.invoke(self.base.pause) if self.base and self.profile.mode=='real' else None); rf.addRow(b)
        b = QPushButton('继续导航'); b.clicked.connect(lambda:self.invoke(self.base.resume) if self.base and self.profile.mode=='real' else None); rf.addRow(b)
        b = QPushButton('载入本地 .smap 地图'); b.clicked.connect(self.load_seer_map); rf.addRow(b); ctl.addWidget(self.real_nav)
        ctl.addWidget(manual)
        b = QPushButton('取消导航'); b.clicked.connect(self.cancel_navigation); ctl.addWidget(b)
        ctl.addStretch(); scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(controls); split.addWidget(scroll); split.setSizes([920,400])
        self.arm_page = QWidget(); self.arm_layout = QVBoxLayout(self.arm_page); self.tabs.addTab(self.arm_page,'机械臂与夹爪')
        self.arm_placeholder = QLabel('连接机器人后载入机械臂控制页\n仿真：MoveIt 规划 → 预览 → 执行\n实机：AUBO ROS 桥 → 关节 / 末端运动与夹爪')
        self.arm_placeholder.setAlignment(Qt.AlignCenter)
        self.arm_preview = QWidget(); preview_layout = QHBoxLayout(self.arm_preview)
        preview_controls = QGroupBox('机械臂 · 离线布局预览'); preview_form = QFormLayout(preview_controls)
        preview_form.addRow(self.arm_placeholder)
        for i in range(6):
            value = QDoubleSpinBox(); value.setRange(-360,360); value.setSuffix(' °'); value.setEnabled(False)
            preview_form.addRow(f'J{i+1} 目标',value)
        for label in ('同步当前目标','规划 / 设置目标','预览轨迹','执行运动','夹爪打开 / 闭合'):
            button = QPushButton(label); button.setEnabled(False); preview_form.addRow(button)
        preview_layout.addWidget(preview_controls)
        try:
            from ..urdf_view import UrdfRobotView
            from ..resources import get_package_share_directory
            self.preview_view = UrdfRobotView(Path(get_package_share_directory('aubo_student_description'))/'urdf/aubo_i16.urdf', Path(get_package_share_directory('aubo_control_gui'))/'qml/RobotView.qml')
            self.preview_view.rootObject().setProperty('targetEnabled',False)
            self.preview_view.rootObject().setProperty('offlinePreview',True)
            self.preview_view.set_target_status('未连接机器人 · 模型仅用于界面预览')
            preview_layout.addWidget(self.preview_view,1)
        except Exception as exc:
            preview_layout.addWidget(QLabel('三维模型预览不可用：'+str(exc)),1)
        self.arm_layout.addWidget(self.arm_preview)
        self.video=IsaacVideo(); self.tabs.addTab(self.video,'Isaac 实时场景')
        self.logs = QPlainTextEdit(); self.logs.setReadOnly(True); self.logs.setMaximumBlockCount(1000); self.logs.setMaximumHeight(120); outer.addWidget(self.logs)
        self.statusBar().showMessage('离线界面预览 · 尚未建立设备连接')
        self.setStyleSheet('QWidget{font-size:14px} QMainWindow{background:#f4f7fb} QGroupBox{font-weight:600;border:1px solid #c9d5e3;border-radius:7px;margin-top:12px;padding-top:15px} QGroupBox::title{subcontrol-origin:margin;left:12px} QPushButton{min-height:30px;padding:4px 10px} QPushButton#stopAll{background:#b52f3b;color:white;font-weight:700}')
        p = profile or Profile(); self.mode.setCurrentIndex(0 if p.mode=='isaac' else 1)
        self.domain.setValue(p.domain); self.peer.setText(p.peer); self.seer_ip.setText(p.seer_ip); self.manual_verified.setChecked(p.real_manual)
        self.mode.currentIndexChanged.connect(self.mode_changed)
        self.tabs.currentChanged.connect(lambda _:self.stop_base())
        self.ros_map.goal_selected.connect(self.select_goal)
        self.ros_map.initial_pose_selected.connect(self.select_initial_pose)
        self.seer_map.stationDoubleClicked.connect(lambda station,x,y:self.target.setText(station))
        QApplication.instance().installEventFilter(self)
        self.mode_changed(initial=True)

    def mode_changed(self, *args, initial=False):
        real = self.mode.currentData() == 'real'
        if not initial: self.domain.setValue(20 if real else 133)
        self.sim_nav.setVisible(not real); self.real_nav.setVisible(real)
        self.tabs.setTabVisible(2,not real)
        self.seer_ip.setEnabled(real); self.manual_verified.setEnabled(real)
        self.maps.setCurrentIndex(1 if real else 0)
        self.mode_badge.setText('真实机器人 · 直接设备控制' if real else 'Isaac Sim · 仿真机器人')
        self.mode_badge.setStyleSheet('color:white;padding:8px 14px;border-radius:5px;background:'+('#b76c16' if real else '#1769b5'))
        for b in self.drive_buttons: b.setEnabled(self.base is not None)

    def log(self, text, error=False):
        self.logs.appendPlainText(time.strftime('%H:%M:%S')+('  错误  ' if error else '  ')+str(text))

    def invoke(self, fn):
        try: fn()
        except Exception as exc: self.log(str(exc),True)

    def connect_robot(self):
        if self.base or self.arm: return
        self.remote_grid = None
        self.remote_map_status = '等待 ROS /map'
        if not self.preview_map: self.show_live_map()
        try:
            p = Profile(self.mode.currentData(),self.domain.value(),self.peer.text().strip(),self.seer_ip.text().strip(),self.manual_verified.isChecked()).validate()
        except ValueError as exc: self.log(str(exc),True); return
        self.profile = p
        # A running ROS context cannot be silently rebound to another robot/domain.
        p.configure_ros()
        try:
            import rclpy
            rclpy.init(args=['--ros-args','-p',f'use_sim_time:={str(p.mode == "isaac").lower()}','-p','enable_motion:=true'])
            self.ros = rclpy
        except Exception as exc:
            self.log(f'ROS 未能初始化：{exc}。实机底盘仍可单独使用 TCP。',True)
        from .base import SeerBase, IsaacBase
        try:
            if p.mode == 'real':
                from PySide6.QtCore import QStandardPaths
                self.base = SeerBase(p,Path(QStandardPaths.writableLocation(QStandardPaths.AppLocalDataLocation))/'seer_logs',self)
            elif self.ros:
                self.base = IsaacBase(self)
                self.base.map_received.connect(self.receive_ros_map)
                self.base.path_received.connect(lambda msg:self.ros_map.set_path(msg) if not self.preview_map else None)
                self.base.map_status.connect(self.update_map_status)
                self.base.map_applied.connect(self.show_live_map)
            if self.base:
                self.base.snapshot.connect(self.update_base); self.base.message.connect(self.log)
        except Exception as exc: self.log(f'底盘连接失败：{exc}',True)
        if self.ros:
            try:
                if p.mode == 'isaac':
                    from ..app import MainWindow
                    from ..resources import get_package_share_directory
                    urdf = Path(get_package_share_directory('aubo_student_description'))/'urdf/aubo_i16.urdf'
                    self.arm = MainWindow(urdf)
                    widget = self.arm.takeCentralWidget()
                    self.arm.bridge.result.connect(self.log); self.arm.bridge.connection.connect(self.update_arm)
                else:
                    from .real_arm import RealArm
                    self.arm = RealArm(self.log); self.arm.connection.connect(self.update_arm); widget = self.arm
                self.arm_preview.hide(); self.arm_layout.addWidget(widget)
                self.arm_widget = widget
                self.arm_status.setText('机械臂：等待 ROS 反馈')
            except Exception as exc: self.log(f'机械臂模块不可用：{exc}',True)
        if self.base or self.arm:
            if p.mode == 'isaac': self.video.start(p.peer)
            for w in (self.mode,self.domain,self.peer,self.seer_ip,self.manual_verified): w.setEnabled(False)
            self.connect_button.setEnabled(False); self.disconnect_button.setEnabled(True)
            for b in self.drive_buttons: b.setEnabled(p.mode == 'isaac' or p.real_manual)
            self.log(f'已启用 {p.mode} 连接，ROS Domain={p.domain}，等待设备反馈')
            self.statusBar().showMessage('仿真 · Isaac / MoveIt / Nav2' if p.mode=='isaac' else f'实机 · SEER {p.seer_ip} / AUBO ROS 桥 {p.peer}')
        elif self.ros:
            self.ros.try_shutdown(); self.ros = None

    def update_arm(self, connected):
        self.arm_status.setText('机械臂：已连接' if connected else '机械臂：等待 / 反馈过期')

    def update_base(self, data):
        connected = data.get('connected',False)
        self.chassis_status.setText('底盘：已连接' if connected else '底盘：等待 / 反馈过期')
        pos = data.get('position',{}); speed = data.get('speed',{})
        def number(value):
            return f'{value:.3f}' if isinstance(value,(int,float)) and math.isfinite(value) else '--'
        self.pose_label.setText(f"位置 / m：X {number(pos.get('x'))}  Y {number(pos.get('y'))}\n速度：{number(speed.get('vx'))} m/s · {number(speed.get('w'))} rad/s")
        self.motion_status.setText('导航：'+NAV_STATES.get(data.get('navigation',{}).get('task_status'), '未知')+'  '+data.get('text',''))
        if not connected:
            self.ros_map.robot = None; self.ros_map.update(); self.seer_map.set_agv_pose(None,None,None)
        elif all(k in pos for k in ('x','y','angle')):
            self.ros_map.robot = None if self.preview_map else (pos['x'],pos['y'],pos['angle']); self.ros_map.update()
            self.seer_map.set_agv_pose(pos['x'],pos['y'],pos['angle'])

    def drive(self, linear, angular):
        if self.base: self.invoke(lambda:self.base.drive(linear*self.linear.value(),angular*self.angular.value()))

    def stop_base(self):
        if self.base: self.invoke(self.base.stop)

    def stop_motion(self):
        self.stop_base(); self.cancel_navigation()
        if self.arm: self.invoke(self.arm.stop)
        self.log('已请求停止；请以设备反馈确认运动终态')

    def select_goal(self,x,y,yaw):
        for b,v in zip(self.goal_boxes,(x,y,math.degrees(yaw))): b.setValue(v)

    def navigate_pose(self):
        if self.preview_map:
            self.log('当前是本地预览；请先应用地图，或切回 ROS 实时地图',True)
            return
        if self.base and self.profile.mode == 'isaac':
            x,y,yaw = [b.value() for b in self.goal_boxes]
            self.invoke(lambda:self.base.navigate_pose(x,y,math.radians(yaw)))
        else: self.log('请先连接 Isaac 仿真',True)

    def load_sim_map(self):
        directory = Path(__file__).resolve().parents[3]/'maps'/'sim'
        path,_ = QFileDialog.getOpenFileName(self,'载入仿真地图',str(directory),'ROS 地图 (*.yaml *.yml)')
        if path: self.invoke(lambda:self.set_sim_map(path))

    def set_sim_map(self, path):
        grid = load_yaml_map(path)
        self.preview_map = True
        self.ros_map.path = []; self.ros_map.robot = self.ros_map.goal = self.ros_map.initial_pose = None
        self.ros_map.set_map(grid)
        self.maps.setCurrentIndex(0)
        # The map built in this session already exists at this remote location.
        known = Path(path).parent.name == 'rebar_lab_20260928' and Path(path).name == 'rebar_lab.yaml'
        self.remote_map_path.setText('/home/arnold/.local/share/aubo-mapping-runs/rebar_lab_20260928/export/rebar_lab.yaml' if known else '')
        self.remote_map_path.setCursorPosition(0)
        self.map_notice.setText(f'本地预览：{Path(path).name} · {grid.info.width}×{grid.info.height} · {grid.info.resolution:g} m/像素\n应用时读取 ROS 主机上的地图文件')

    def receive_ros_map(self, msg):
        self.remote_grid = msg
        if not self.preview_map:
            self.ros_map.set_map(msg)
            self.map_notice.setText('ROS 实时地图 · '+self.remote_map_status)

    def update_map_status(self, text):
        self.remote_map_status = text
        self.map_notice.setText(('本地预览 · ' if self.preview_map else 'ROS 实时地图 · ')+text)

    def show_live_map(self):
        self.preview_map = False
        self.ros_map.path = []; self.ros_map.goal = self.ros_map.initial_pose = None
        if self.remote_grid is not None: self.ros_map.set_map(self.remote_grid)
        else:
            self.ros_map.grid = self.ros_map.image = self.ros_map.robot = None
            self.ros_map.update()
        self.map_notice.setText('ROS 实时地图 · '+self.remote_map_status)

    def apply_sim_map(self):
        if self.base and self.profile.mode == 'isaac':
            self.invoke(lambda:self.base.load_map(self.remote_map_path.text()))
        else: self.log('请先连接 Isaac 仿真',True)

    def select_initial_pose(self, x, y, yaw):
        for box,value in zip(self.initial_boxes,(x,y,math.degrees(yaw))): box.setValue(value)

    def send_initial_pose(self):
        if self.preview_map:
            self.log('请先应用远程地图，或切回 ROS 实时地图，再设置初始位置',True); return
        if self.base and self.profile.mode == 'isaac':
            x,y,yaw = [box.value() for box in self.initial_boxes]
            if not self.ros_map.is_free(x,y):
                self.log('初始位置必须位于地图已知空闲区域',True); return
            self.invoke(lambda:self.base.set_initial_pose(x,y,math.radians(yaw)))
        else: self.log('请先连接 Isaac 仿真',True)

    def navigate_station(self):
        if self.base and self.profile.mode == 'real': self.invoke(lambda:self.base.navigate_station(self.source.text(),self.target.text()))
        else: self.log('请先连接真实 SEER 底盘',True)

    def cancel_navigation(self):
        if self.base: self.invoke(self.base.cancel)

    def load_seer_map(self):
        path,_ = QFileDialog.getOpenFileName(self,'载入 SEER 地图','','SEER 地图 (*.smap)')
        if path: self.invoke(lambda:self.set_seer_map(path))

    def set_seer_map(self, path):
        smap = load_smap_file(path)
        stations = {point['id']: point for point in extract_station_like_points(smap)
                    if re.fullmatch(r'LM\d+', point['id'], re.IGNORECASE)}
        self.seer_map.set_map(smap)
        self.station_choices.blockSignals(True)
        self.station_choices.clear()
        self.station_choices.addItem('请选择 LM 点位' if stations else '地图中没有 LM 点位', None)
        for station_id in sorted(stations, key=lambda value: (int(value[2:]), value)):
            point = stations[station_id]
            self.station_choices.addItem(
                f"{station_id} · ({point['x']:.3f}, {point['y']:.3f}) m", station_id)
        self.station_choices.setEnabled(bool(stations))
        self.station_choices.blockSignals(False)
        self.target.clear()
        self.log(f'已载入地图 {smap.name}：{len(stations)} 个 LM 点位；请选择目标后点击导航')

    def choose_station(self, index):
        station_id = self.station_choices.itemData(index)
        if station_id: self.target.setText(station_id)
        else: self.target.clear()

    def sync_station_selection(self, station_id):
        station_id = station_id.strip()
        index = self.station_choices.findData(station_id)
        self.station_choices.blockSignals(True)
        self.station_choices.setCurrentIndex(max(0, index))
        self.station_choices.blockSignals(False)
        self.seer_map.selected_station_id = station_id
        self.seer_map.update()

    def eventFilter(self,obj,event):
        if event.type() == QEvent.ApplicationDeactivate: self.stop_base()
        return super().eventFilter(obj,event)

    def disconnect_robot(self):
        base_ready = self.base is None or self.base.ready_to_close()
        arm_ready = True
        if self.arm:
            if self.profile.mode == 'isaac':
                if self.arm.bridge.node.busy: self.arm.stop(); arm_ready = False
            else: arm_ready = self.arm.ready_to_close()
        if not base_ready or not arm_ready:
            self.log('等待停止 / 取消终态后断开',False)
            QTimer.singleShot(500,self.disconnect_robot); return False
        if self.base: self.base.close(); self.base.deleteLater(); self.base = None
        if self.arm:
            if self.profile.mode == 'isaac':
                self.arm.bridge.close(); self.arm.preview_timer.stop(); self.arm.status_timer.stop()
            else: self.arm.close()
            self.arm_widget.deleteLater(); self.arm.deleteLater(); self.arm = None
        if self.ros: self.ros.try_shutdown(); self.ros = None
        self.video.stop()
        self.remote_grid = None
        self.remote_map_status = '连接已断开 · 等待 ROS /map'
        if not self.preview_map: self.show_live_map()
        self.arm_preview.show()
        self.chassis_status.setText('底盘：未连接'); self.arm_status.setText('机械臂：未连接')
        for w in (self.mode,self.domain,self.peer): w.setEnabled(True)
        self.connect_button.setEnabled(True); self.disconnect_button.setEnabled(False); self.mode_changed(initial=True)
        self.log('连接已断开')
        if self.closing: QTimer.singleShot(0,self.close)
        return True

    def closeEvent(self,event):
        self.closing = True
        if self.disconnect_robot():
            QApplication.instance().removeEventFilter(self)
            event.accept()
        else: event.ignore()


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description='SEER + AUBO 统一 GUI；启动仅预览，点击连接后通信')
    parser.add_argument('--mode',choices=['isaac','real'],default='isaac')
    parser.add_argument('--domain-id',type=int)
    parser.add_argument('--peer',default='192.168.0.103')
    parser.add_argument('--seer-ip',default='192.168.3.250')
    parser.add_argument('--page',choices=['base','arm','scene'],default='base',help='初始显示页面')
    parser.add_argument('--connect',action='store_true',help='启动后自动连接选定的机器人')
    parser.add_argument('--screenshot',help='保存离线界面截图后退出')
    parser.add_argument('--map',help='启动时预览本地仿真 YAML 地图，不连接设备')
    args = parser.parse_args(argv)
    profile = Profile(args.mode,args.domain_id if args.domain_id is not None else (133 if args.mode=='isaac' else 20),args.peer,args.seer_ip).validate()
    app = QApplication.instance() or QApplication([sys.argv[0]])
    app.setApplicationName('SEER AUBO Console'); app.setOrganizationName('Hongshi')
    window = UnifiedWindow(profile); window.tabs.setCurrentIndex({'base':0,'arm':1,'scene':2}[args.page]); window.show()
    if args.map: window.invoke(lambda:window.set_sim_map(args.map))
    if args.connect: QTimer.singleShot(0,window.connect_robot)
    if args.screenshot:
        def capture():
            path = Path(args.screenshot); path.parent.mkdir(parents=True,exist_ok=True)
            window.grab().save(str(path)); window.close()
        QTimer.singleShot(800,lambda: window.preview_view.reset_view() if hasattr(window,'preview_view') else None)
        QTimer.singleShot(1800,capture)
    return app.exec()
