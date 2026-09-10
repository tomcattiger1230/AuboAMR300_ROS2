from __future__ import annotations
import math, os, sys
from pathlib import Path
from PySide6.QtCore import QStandardPaths, Qt
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import (QApplication,QDoubleSpinBox,QFormLayout,QGridLayout,QGroupBox,QHBoxLayout,QLabel,QMainWindow,QMessageBox,QPushButton,QScrollArea,QSplitter,QTabWidget,QVBoxLayout,QWidget)
from .resources import get_package_share_directory
from .planner import PlanningError, start_matches_actual
from .presets import PRESET_NAMES, PresetStore

from .ros_bridge import RosBridge
from .urdf_view import UrdfRobotView

STATE_NAMES={0:"未连接",1:"启动中",2:"空闲",3:"运行中",4:"错误",5:"急停"}

def _spin(value=0.0,suffix="°"):
    box=QDoubleSpinBox(); box.setRange(-360,360); box.setDecimals(2); box.setValue(value); box.setSuffix(suffix); box.setMinimumWidth(88); return box
def _position_spin():
    box=QDoubleSpinBox(); box.setRange(-5,5); box.setDecimals(4); box.setSingleStep(.001); box.setSuffix(" m"); box.setMinimumWidth(120); return box

class MainWindow(QMainWindow):
    def __init__(self,urdf_path):
        super().__init__(); self.setWindowTitle("AUBO i16 · MoveIt 控制台"); self.resize(1440,900); self.actual=None; self.actual_tcp=None
        cfg=Path(QStandardPaths.writableLocation(QStandardPaths.AppConfigLocation))/"quick_positions.json"; self.presets=PresetStore(cfg)
        self.bridge=RosBridge(self); self._build(urdf_path); self._wire()
        mode="规划并执行" if self.bridge.node.get_parameter('enable_motion').value else "仅规划"
        self.setWindowTitle(f"AUBO i16 · MoveIt 控制台 · {mode}")
        self.log(f"{mode}模式，等待机械臂状态",False)
    def _build(self,urdf_path):
        splitter=QSplitter(); splitter.setChildrenCollapsible(False); left=QWidget(); root=QVBoxLayout(left)
        status=QGroupBox("机械臂状态"); form=QFormLayout(status); self.conn=QLabel("● 未连接"); self.state=QLabel("未知"); self.joint_labels=[QLabel("--") for _ in range(6)]
        form.addRow("连接",self.conn); form.addRow("本界面任务状态",self.state)
        row=QHBoxLayout(); [row.addWidget(w) for w in self.joint_labels]; form.addRow("J1—J6",row)
        self.tcp=QLabel("X --  Y --  Z -- m"); self.grip=QLabel("-- mm"); self.actual_linear_speed=QLabel("0.0000 m/s"); self.actual_speed=QLabel("0.000 rad/s")
        form.addRow("末端位置",self.tcp); form.addRow("机械爪位置",self.grip); form.addRow("末端移动速度",self.actual_linear_speed); form.addRow("当前关节角速度",self.actual_speed); root.addWidget(status)
        profile=QGroupBox("运动参数（可调整）"); pf=QFormLayout(profile); self.velocity=QDoubleSpinBox(); self.velocity.setRange(.01,1); self.velocity.setValue(.2); self.velocity.setSuffix(" ×")
        self.acceleration=QDoubleSpinBox(); self.acceleration.setRange(.01,1); self.acceleration.setValue(.2); self.acceleration.setSuffix(" ×"); self.linear_velocity=QDoubleSpinBox(); self.linear_velocity.setRange(.01,1); self.linear_velocity.setValue(.05); self.linear_velocity.setSuffix(" ×"); self.linear_acceleration=QDoubleSpinBox(); self.linear_acceleration.setRange(.01,1); self.linear_acceleration.setValue(.05); self.linear_acceleration.setSuffix(" ×"); pf.addRow("关节速度比例",self.velocity); pf.addRow("关节加速度比例",self.acceleration); pf.addRow("位姿规划速度比例",self.linear_velocity); pf.addRow("位姿规划加速度比例",self.linear_acceleration); root.addWidget(profile)
        motion=QGroupBox("初始位置 → 目标位置"); motion_layout=QVBoxLayout(motion); tabs=QTabWidget(); joint_page=QWidget(); grid=QGridLayout(joint_page); grid.addWidget(QLabel(""),0,0)
        for i in range(6): grid.addWidget(QLabel(f"J{i+1}"),0,i+1)
        self.motion_joint_labels=[QLabel("--") for _ in range(6)]; grid.addWidget(QLabel("当前"),1,0)
        for i,label in enumerate(self.motion_joint_labels): grid.addWidget(label,1,i+1)
        self.start_boxes=[_spin() for _ in range(6)]; self.target_boxes=[_spin() for _ in range(6)]
        self.start_boxes[2].setRange(-161,161); self.target_boxes[2].setRange(-161,161)
        for r,(name,boxes) in enumerate((("初始",self.start_boxes),("目标",self.target_boxes)),2):
            grid.addWidget(QLabel(name),r,0)
            for i,box in enumerate(boxes): grid.addWidget(box,r,i+1)
        copy=QPushButton("当前关节角 → 初始"); copy.clicked.connect(self.copy_actual); execute=QPushButton("按关节角求解并运行"); execute.setProperty("primary",True); execute.clicked.connect(self.execute_manual); grid.addWidget(copy,4,0,1,3); grid.addWidget(execute,4,3,1,4)
        tcp_page=QWidget(); tg=QGridLayout(tcp_page); tg.addWidget(QLabel(""),0,0)
        for i,name in enumerate(("X","Y","Z")): tg.addWidget(QLabel(name),0,i+1)
        self.motion_tcp_labels=[QLabel("--") for _ in range(3)]; tg.addWidget(QLabel("当前"),1,0)
        for i,label in enumerate(self.motion_tcp_labels): tg.addWidget(label,1,i+1)
        self.tcp_start_boxes=[_position_spin() for _ in range(3)]; self.tcp_target_boxes=[_position_spin() for _ in range(3)]
        for r,(name,boxes) in enumerate((("初始",self.tcp_start_boxes),("目标",self.tcp_target_boxes)),2):
            tg.addWidget(QLabel(name),r,0)
            for i,box in enumerate(boxes): tg.addWidget(box,r,i+1)
        copy_tcp=QPushButton("当前末端位置 → 初始"); copy_tcp.clicked.connect(self.copy_actual_tcp); execute_tcp=QPushButton("按末端位置求解并运行"); execute_tcp.setProperty("primary",True); execute_tcp.clicked.connect(self.execute_tcp); tg.addWidget(copy_tcp,4,0,1,2); tg.addWidget(execute_tcp,4,2,1,2)
        tabs.addTab(joint_page,"关节角度运动"); tabs.addTab(tcp_page,"末端位置运动"); motion_layout.addWidget(tabs); root.addWidget(motion)
        quick=QGroupBox("快捷位（MoveIt 规划到已保存关节位置）"); qg=QGridLayout(quick); self.preset_status={}
        for r,name in enumerate(PRESET_NAMES):
            qg.addWidget(QLabel(name),r,0); label=QLabel("已保存" if self.presets.get(name) else "未保存"); self.preset_status[name]=label; qg.addWidget(label,r,1)
            go=QPushButton("运行"); go.clicked.connect(lambda _,n=name:self.run_preset(n)); save=QPushButton("保存/更新当前位置"); save.clicked.connect(lambda _,n=name:self.save_preset(n)); qg.addWidget(go,r,2); qg.addWidget(save,r,3)
        root.addWidget(quick)
        actions=QHBoxLayout(); stop=QPushButton("停止运动"); stop.setObjectName("stop"); stop.clicked.connect(self.bridge.stop); op=QPushButton("夹爪打开"); op.clicked.connect(lambda:self.bridge.command_gripper("open")); cl=QPushButton("夹爪闭合"); cl.clicked.connect(lambda:self.bridge.command_gripper("close"))
        for b in (stop,op,cl): actions.addWidget(b)
        self.message=QLabel(); self.message.setWordWrap(True); root.addStretch()
        scroll=QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(left); qml=Path(get_package_share_directory("aubo_control_gui"))/"qml"/"RobotView.qml"; self.view=UrdfRobotView(urdf_path,qml); splitter.addWidget(scroll); splitter.addWidget(self.view); splitter.setSizes([670,770]); central=QWidget(); outer=QVBoxLayout(central); outer.addWidget(splitter,1); outer.addLayout(actions); outer.addWidget(self.message); self.setCentralWidget(central)
        self.setStyleSheet("QWidget{font-size:18px} QGroupBox{font-weight:600;border:1px solid #cad4df;border-radius:6px;margin-top:10px;padding-top:10px} QGroupBox::title{subcontrol-origin:margin;left:10px} QPushButton{min-height:38px;padding:2px 10px} QPushButton[primary='true']{background:#1976d2;color:white;font-weight:bold} QPushButton#stop{background:#b71c1c;color:white;font-weight:bold}")
    def _wire(self):
        self.bridge.gripper_joints.connect(self.view.set_gripper_positions)
        self.bridge.joints.connect(self.on_joints); self.bridge.joint_speed.connect(lambda v:self.actual_speed.setText(f"{v:.3f} rad/s")); self.bridge.tool_speed.connect(lambda v:self.actual_linear_speed.setText(f"{v:.4f} m/s")); self.bridge.status.connect(self.on_status); self.bridge.gripper.connect(lambda v:self.grip.setText(f"{v:.2f} mm（模型单指位移）")); self.bridge.connection.connect(self.on_connection); self.bridge.result.connect(self.log)
    def on_joints(self,joints): self.actual=list(joints); self.view.set_joint_positions(joints); [label.setText(f"{math.degrees(v):.2f}°") for label,v in zip(self.joint_labels,joints)]; [label.setText(f"{math.degrees(v):.2f}°") for label,v in zip(self.motion_joint_labels,joints)]
    def on_status(self,s):
        self.state.setText(STATE_NAMES.get(s.robot_state,"未知")); self.actual_tcp=(s.tool_position_x,s.tool_position_y,s.tool_position_z); self.tcp.setText(f"X {s.tool_position_x:.4f}  Y {s.tool_position_y:.4f}  Z {s.tool_position_z:.4f} m")
        for label,value in zip(self.motion_tcp_labels,self.actual_tcp): label.setText(f"{value:.4f} m")
    def on_connection(self,ok):
        self.conn.setText("● 反馈正常 / MoveIt 可用" if ok else "● 反馈过期或 MoveIt 不可用")
        self.conn.setStyleSheet("color:#198754;font-weight:bold" if ok else "color:#c62828;font-weight:bold")
        if not ok: self.state.setText("不可用")
    def copy_actual(self):
        if self.actual is None: self.log("尚未收到关节状态",True); return
        for box,value in zip(self.start_boxes,self.actual): box.setValue(math.degrees(value))
    def execute_manual(self): self._execute([math.radians(b.value()) for b in self.start_boxes],[math.radians(b.value()) for b in self.target_boxes])
    def copy_actual_tcp(self):
        if self.actual_tcp is None: self.log("尚未收到末端位置",True); return
        for box,value in zip(self.tcp_start_boxes,self.actual_tcp): box.setValue(value)
    def execute_tcp(self):
        if self.actual_tcp is None: self.log("没有实际末端位置，拒绝运动",True); return
        start=tuple(b.value() for b in self.tcp_start_boxes); target=tuple(b.value() for b in self.tcp_target_boxes)
        if max(abs(a-b) for a,b in zip(start,self.actual_tcp))>.005: QMessageBox.warning(self,"初始位不匹配","输入初始末端位置与实际位置相差超过 5 mm，请先复制当前位置。"); return
        self.bridge.execute_pose_with_fallback(start,target,self.linear_velocity.value(),self.linear_acceleration.value())
    def _execute(self,start,target):
        if self.actual is None: self.log("没有实际关节状态，拒绝运动",True); return
        try:
            if not start_matches_actual(start,self.actual,math.radians(3)): QMessageBox.warning(self,"初始位不匹配","输入初始位与实际位置相差超过 3°，请先复制当前位置。"); return
            self.bridge.execute_with_fallback(start,target,self.velocity.value(),self.acceleration.value())
        except PlanningError as exc: self.log(str(exc),True)
    def save_preset(self,name):
        if not self.bridge.node.fresh(): self.log("没有新鲜关节反馈可保存",True); return
        self.presets.set(name,[math.degrees(v) for v in self.actual]); self.preset_status[name].setText("已保存"); self.log(f"已更新{name}",False)
    def run_preset(self,name):
        saved=self.presets.get(name)
        if saved is None: self.log(f"{name}尚未保存",True); return
        if not self.bridge.node.fresh(): self.log("关节反馈过期，拒绝运动",True); return
        self.bridge.execute_with_fallback(self.actual,[math.radians(v) for v in saved],self.velocity.value(),self.acceleration.value())
    def log(self,text,error): self.message.setText(text); self.message.setStyleSheet("color:#c62828" if error else "color:#198754")
    def closeEvent(self,event):
        if self.bridge.close(): event.accept()
        else:
            self.log("已请求停止；等待运动结束后再关闭窗口",True)
            event.ignore()

def main():
    app=QApplication(sys.argv); app.setApplicationName("AUBO Control GUI"); app.setOrganizationName("AUBO")
    urdf=Path(get_package_share_directory("aubo_student_description"))/"urdf"/"aubo_i16.urdf"
    if not urdf.is_file(): raise FileNotFoundError(f"URDF 不存在: {urdf}")
    window=MainWindow(urdf); window.show(); return app.exec()
