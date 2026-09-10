from __future__ import annotations
import math
import sys
import time
from pathlib import Path
from PySide6.QtCore import QStandardPaths, Qt, QTimer
from PySide6.QtWidgets import (QApplication, QDoubleSpinBox, QFormLayout, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QMainWindow, QPushButton, QScrollArea, QSlider,
    QSplitter, QTabWidget, QVBoxLayout, QWidget)
from .resources import get_package_share_directory
from .presets import PRESET_NAMES, PresetStore
from .ros_bridge import RosBridge
from .urdf_view import UrdfRobotView
from .motion_client import ARM, GRIPPER
from .view_math import quaternion_from_rpy, rpy_from_quaternion, interpolate_trajectory

PHASES = {'idle':'待规划', 'planning':'规划中', 'ready':'可执行', 'executing':'执行中',
          'done':'执行完成', 'stopped':'已停止', 'error':'失败'}


def number(low, high, suffix, decimals=2):
    box=QDoubleSpinBox();box.setRange(low,high);box.setSuffix(suffix)
    box.setDecimals(decimals);box.setMinimumWidth(75)
    return box


class MainWindow(QMainWindow):
    def __init__(self, urdf_path):
        super().__init__()
        self.setWindowTitle('AUBO · 目标 / 规划 / 执行');self.resize(1440,900)
        self.actual=None;self.actual_pose=None;self.actual_tcp=None;self.target_pose=None
        self._updating=False;self.preview=None;self.preview_initial={};self.preview_duration=0
        self.preview_started=None;self.plan_buttons=[]
        cfg=Path(QStandardPaths.writableLocation(QStandardPaths.AppConfigLocation))/'quick_positions.json'
        self.presets=PresetStore(cfg)
        self.bridge=RosBridge(self)
        self._build(urdf_path);self._wire()
        self.preview_timer=QTimer(self);self.preview_timer.setInterval(33)
        self.preview_timer.timeout.connect(self._preview_tick)
        self.status_timer=QTimer(self);self.status_timer.setInterval(100)
        self.status_timer.timeout.connect(self.refresh_controls);self.status_timer.start()
        self.log('拖动目标或输入数值 → 规划并预览 → 单独点击执行。',False)

    def button(self, text, fn, planning=False):
        b=QPushButton(text);b.clicked.connect(fn)
        if planning:self.plan_buttons.append(b)
        return b

    def _build(self, urdf_path):
        self.view=UrdfRobotView(urdf_path,Path(get_package_share_directory('aubo_control_gui'))/'qml/RobotView.qml')
        left=QWidget();layout=QVBoxLayout(left)
        group=QGroupBox('实时反馈');form=QFormLayout(group)
        self.conn=QLabel('等待连接');self.state=QLabel('待规划');self.joint_labels=[QLabel('--') for _ in ARM]
        row=QHBoxLayout()
        for label in self.joint_labels:row.addWidget(label)
        self.tcp=QLabel('--');self.grip=QLabel('--');self.actual_speed=QLabel('--');self.actual_linear_speed=QLabel('--')
        form.addRow('连接',self.conn);form.addRow('任务状态',self.state);form.addRow('J1—J6',row)
        form.addRow('腕部位置 / m',self.tcp);form.addRow('夹指位移',self.grip)
        form.addRow('关节速度',self.actual_speed);form.addRow('末端速度',self.actual_linear_speed)
        layout.addWidget(group)
        profile=QGroupBox('规划参数');pf=QFormLayout(profile)
        self.velocity=number(.01,1,' ×');self.velocity.setValue(.2)
        self.acceleration=number(.01,1,' ×');self.acceleration.setValue(.2)
        pf.addRow('速度比例',self.velocity);pf.addRow('加速度比例',self.acceleration)
        layout.addWidget(profile)
        self.tabs=QTabWidget();self.tabs.setMaximumHeight(300)
        joint=QWidget();jl=QVBoxLayout(joint);grid=QGridLayout()
        self.target_boxes=[number(-161 if i==2 else -360,161 if i==2 else 360,'°') for i in range(6)]
        for i,box in enumerate(self.target_boxes):
            grid.addWidget(QLabel(f'J{i+1}'),0,i);grid.addWidget(box,1,i)
        jl.addLayout(grid)
        jl.addWidget(self.button('将当前关节角设为目标',self.copy_actual))
        jl.addWidget(self.button('规划关节目标',self.plan_manual,True))
        self.tabs.addTab(joint,'关节目标')
        pose=QWidget();pl=QVBoxLayout(pose);grid=QGridLayout()
        self.tcp_target_boxes=[number(-5,5,' m',4) for _ in range(3)]
        self.orientation_boxes=[number(-360,360,'°',2) for _ in range(3)]
        for i,(name,box) in enumerate(zip(('X','Y','Z','Roll','Pitch','Yaw'),self.tcp_target_boxes+self.orientation_boxes)):
            grid.addWidget(QLabel(name),i//3*2,i%3);grid.addWidget(box,i//3*2+1,i%3)
        pl.addLayout(grid)
        note=QLabel('坐标：base_footprint；目标：wrist3_Link 腕部法兰。\n拖动彩色轴平移，拖动圆环旋转。拖动不执行运动。');note.setWordWrap(True);pl.addWidget(note)
        pl.addWidget(self.button('同步当前末端目标',self.copy_actual_tcp))
        pl.addWidget(self.button('规划末端目标',self.plan_tcp,True))
        self.tabs.addTab(pose,'拖动末端 / 位姿')
        layout.addWidget(self.tabs)
        group=QGroupBox('快捷位');grid=QGridLayout(group);self.preset_status={}
        for i,name in enumerate(PRESET_NAMES):
            grid.addWidget(QLabel(name),i,0)
            label=QLabel('已保存' if self.presets.get(name) else '未保存');self.preset_status[name]=label;grid.addWidget(label,i,1)
            grid.addWidget(self.button('规划',lambda _,n=name:self.run_preset(n),True),i,2)
            grid.addWidget(self.button('保存当前位置',lambda _,n=name:self.save_preset(n)),i,3)
        layout.addWidget(group)
        row=QHBoxLayout()
        row.addWidget(self.button('规划夹爪打开',lambda:self.bridge.command_gripper('open',self.velocity.value(),self.acceleration.value()),True))
        row.addWidget(self.button('规划夹爪闭合',lambda:self.bridge.command_gripper('close',self.velocity.value(),self.acceleration.value()),True))
        layout.insertLayout(2,row);layout.addStretch()
        scroll=QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(left)
        viewport=QWidget();view_layout=QVBoxLayout(viewport);view_layout.setContentsMargins(0,0,0,0)
        view_buttons=QHBoxLayout()
        self.camera_source=QLabel('相机：本地模型（等待远端模型）')
        view_buttons.addWidget(self.camera_source)
        view_buttons.addStretch()
        view_buttons.addWidget(self.button('相机特写',self.view.focus_camera))
        view_buttons.addWidget(self.button('整体视图',self.view.reset_view))
        view_layout.addLayout(view_buttons);view_layout.addWidget(self.view,1)
        splitter=QSplitter();splitter.addWidget(scroll);splitter.addWidget(viewport);splitter.setSizes([620,820])
        central=QWidget();outer=QVBoxLayout(central);outer.addWidget(splitter,1)
        self.preview_label=QLabel('尚无规划轨迹；实体模型始终显示实时反馈')
        outer.addWidget(self.preview_label)
        row=QHBoxLayout();self.play_button=self.button('播放预览',self.toggle_preview)
        self.slider=QSlider(Qt.Horizontal);self.slider.setRange(0,1000)
        row.addWidget(self.play_button);row.addWidget(self.slider,1)
        self.execute_button=self.button('执行已规划轨迹',self.execute_saved);self.execute_button.setObjectName('execute')
        self.execute_button.setToolTip('需要有效规划、最新反馈及 MoveIt 执行/停止接口；仅规划模式下禁用')
        self.stop_button=self.button('停止 / 丢弃轨迹',self.stop);self.stop_button.setObjectName('stop')
        row.addWidget(self.execute_button);row.addWidget(self.stop_button);outer.addLayout(row)
        self.message=QLabel();self.message.setWordWrap(True);outer.addWidget(self.message)
        self.setCentralWidget(central)
        self.setStyleSheet('QWidget{font-size:14px} QGroupBox{font-weight:600;border:1px solid #bbc6d3;border-radius:6px;margin-top:10px;padding-top:12px} QGroupBox::title{subcontrol-origin:margin;left:10px} QPushButton{min-height:32px;padding:2px 9px} QPushButton#execute{background:#1976d2;color:white} QPushButton#stop{background:#b71c1c;color:white} QPushButton#execute:disabled, QPushButton:disabled{background:#d5dae1;color:#737c87}')

    def _wire(self):
        b=self.bridge
        b.camera_mount.connect(self.view.set_camera_mount)
        b.camera_mount.connect(lambda _: self.camera_source.setText('相机：远端模型 · MV-CH100-60UM / 12 mm'))
        b.joints.connect(self.on_joints);b.gripper_joints.connect(self.view.set_gripper_positions)
        b.pose_changed.connect(self.on_pose)
        b.gripper.connect(lambda v:self.grip.setText(f'{v:.2f} mm（模型单指）'))
        b.joint_speed.connect(lambda v:self.actual_speed.setText(f'{v:.3f} rad/s'))
        b.tool_speed.connect(lambda v:self.actual_linear_speed.setText(f'{v:.4f} m/s'))
        b.connection.connect(self.on_connection);b.result.connect(self.log)
        b.plan_changed.connect(self.on_plan);b.ik_result.connect(self.on_ik)
        self.view.target_changed.connect(self.on_drag)
        for box in self.target_boxes+[self.velocity,self.acceleration]:box.valueChanged.connect(self.inputs_changed)
        for box in self.tcp_target_boxes+self.orientation_boxes:box.valueChanged.connect(self.pose_inputs_changed)
        self.tabs.currentChanged.connect(self.inputs_changed)
        self.slider.valueChanged.connect(self.scrub_preview)
        self.slider.sliderPressed.connect(self.pause_preview)

    def inputs_changed(self,*args):
        if not self._updating:self.bridge.node.invalidate_plan()

    def pose_inputs_changed(self,*args):
        if self._updating:return
        q=quaternion_from_rpy(*[math.radians(b.value()) for b in self.orientation_boxes])
        self.target_pose=[b.value() for b in self.tcp_target_boxes]+q
        self.view.set_target_pose(self.target_pose)
        self.inputs_changed();self.bridge.request_ik(self.target_pose)

    def on_drag(self,pose):
        if self.bridge.node.busy:return
        self._updating=True;self.tabs.setCurrentIndex(1);self._updating=False
        self.set_pose_inputs(pose);self.inputs_changed()
        self.view.set_target_status('目标已改变；正在检查 IK…')
        self.bridge.request_ik(pose)

    def set_pose_inputs(self,pose):
        self.target_pose=list(pose);self._updating=True
        for box,value in zip(self.tcp_target_boxes,pose[:3]):box.setValue(value)
        for box,value in zip(self.orientation_boxes,rpy_from_quaternion(pose[3:])):box.setValue(math.degrees(value))
        self._updating=False
        self.view.set_target_pose(pose)

    def on_pose(self,pose):
        self.actual_pose=list(pose);self.actual_tcp=pose[:3]
        self.tcp.setText('  '.join(f'{axis} {v:.4f}' for axis,v in zip('XYZ',pose)))
        if self.target_pose is None:self.set_pose_inputs(pose)

    def on_joints(self,joints):
        first=self.actual is None;self.actual=list(joints);self.view.set_joint_positions(joints)
        for label,value in zip(self.joint_labels,joints):label.setText(f'{math.degrees(value):.2f}°')
        if first:self.copy_actual()

    def on_connection(self,ok):
        self.conn.setText('反馈正常 / MoveIt 可用' if ok else '反馈过期或 MoveIt 不可用')
        self.conn.setStyleSheet('color:#198754' if ok else 'color:#c62828')

    def refresh_controls(self):
        n=self.bridge.node;ready=n.plan_ready()
        self.state.setText(PHASES.get(n.phase,n.phase))
        self.execute_button.setEnabled(ready and n.get_parameter('enable_motion').value and n.execute.server_is_ready() and n.stop_event.get_subscription_count()>0)
        self.play_button.setEnabled(ready);self.slider.setEnabled(ready)
        for b in self.plan_buttons:b.setEnabled(not n.busy and n.fresh(ARM+GRIPPER))
        self.view.rootObject().setProperty('targetEnabled',self.target_pose is not None and not n.busy)

    def copy_actual(self):
        if self.actual is None:return
        self._updating=True
        for box,value in zip(self.target_boxes,self.actual):box.setValue(math.degrees(value))
        self._updating=False;self.inputs_changed()

    def copy_actual_tcp(self):
        if self.actual_pose is None:self.log('末端反馈尚未就绪',True);return
        self.set_pose_inputs(self.actual_pose);self.inputs_changed();self.bridge.request_ik(self.target_pose)

    def plan_manual(self):
        self.bridge.plan_joints(self.actual,[math.radians(b.value()) for b in self.target_boxes],self.velocity.value(),self.acceleration.value())

    def plan_tcp(self):
        if self.target_pose is None:self.log('请先同步当前末端目标',True);return
        self.bridge.plan_pose(self.actual_tcp,self.target_pose[:3],self.velocity.value(),self.acceleration.value(),self.target_pose[3:])

    def save_preset(self,name):
        if not self.bridge.node.fresh():self.log('没有新鲜关节反馈可保存',True);return
        self.presets.set(name,[math.degrees(v) for v in self.actual]);self.preset_status[name].setText('已保存')
        self.log(f'已保存{name}',False)

    def run_preset(self,name):
        saved=self.presets.get(name)
        if saved is None:self.log(f'{name}尚未保存',True);return
        if not self.bridge.node.fresh():self.log('关节反馈过期，拒绝规划',True);return
        self.bridge.plan_joints(self.actual,[math.radians(v) for v in saved],self.velocity.value(),self.acceleration.value())

    def on_ik(self,joints,message):
        if self.preview is not None or self.bridge.node.busy:return
        self.view.set_ghost(joints);self.view.set_target_status(message)

    def on_plan(self,trajectory):
        self.pause_preview() if hasattr(self,'preview_timer') else None
        self.preview=trajectory;self.preview_started=None
        self.view.set_ghost(None)
        if trajectory is None:
            self.preview_label.setText('尚无有效轨迹；目标、参数或起始状态变化后须重新规划')
            return
        self.view.set_target_status('轨迹已生成；检查预览后单独点击执行')
        self.preview_initial=dict(self.bridge.node.plan_start)
        p=trajectory.joint_trajectory.points[-1].time_from_start
        self.preview_duration=p.sec+p.nanosec*1e-9
        self.preview_label.setText(f'已规划 {len(trajectory.joint_trajectory.points)} 个轨迹点，{self.preview_duration:.2f} s；半透明模型为预览')
        self.slider.setValue(0);self.play_preview()

    def play_preview(self):
        if self.preview is None:return
        if self.slider.value()>=1000:self.slider.setValue(0)
        self.preview_started=time.monotonic()-self.slider.value()/1000*self.preview_duration
        self.preview_timer.start();self.play_button.setText('暂停预览')

    def pause_preview(self):
        self.preview_timer.stop();self.play_button.setText('播放预览')

    def toggle_preview(self):
        if self.preview_timer.isActive():self.pause_preview()
        else:self.play_preview()

    def _preview_tick(self):
        elapsed=time.monotonic()-self.preview_started
        self.slider.setValue(round(min(1,elapsed/max(.001,self.preview_duration))*1000))
        if elapsed>=self.preview_duration:self.pause_preview()

    def scrub_preview(self,value):
        if self.preview is None:return
        state=interpolate_trajectory(self.preview,value/1000*self.preview_duration,self.preview_initial)
        self.view.set_ghost([state[n] for n in ARM],[state[n] for n in GRIPPER])

    def execute_saved(self):self.pause_preview();self.bridge.execute_plan()
    def stop(self):self.pause_preview();self.view.set_ghost(None);self.bridge.stop()
    def log(self,text,error):self.message.setText(text);self.message.setStyleSheet('color:#c62828' if error else 'color:#198754')
    def closeEvent(self,event):
        if self.bridge.close():self.preview_timer.stop();self.status_timer.stop();event.accept()
        else:self.log('已请求停止；等待执行终态后再关闭窗口',True);event.ignore()


def main():
    app=QApplication(sys.argv);app.setApplicationName('AUBO Control GUI');app.setOrganizationName('AUBO')
    urdf=Path(get_package_share_directory('aubo_student_description'))/'urdf/aubo_i16.urdf'
    window=MainWindow(urdf);window.show();return app.exec()
