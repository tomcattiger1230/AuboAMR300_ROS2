"""Qt adapter: both backend profiles use exactly the same MoveIt client."""
import time
from types import SimpleNamespace
from PySide6.QtCore import QObject, QTimer, Signal, QCoreApplication
import rclpy
from .motion_client import MotionClient, ARM, GRIPPER

class RosBridge(QObject):
    joints=Signal(object); joint_speed=Signal(float); tool_speed=Signal(float)
    status=Signal(object); gripper=Signal(float); connection=Signal(bool); result=Signal(str,bool)
    def __init__(self,parent=None):
        super().__init__(parent)
        rclpy.init(args=None)
        self.node=MotionClient()
        self.node.on_result=self.result.emit
        self.previous=None
        self.previous_tcp=None
        self.timer=QTimer(self)
        self.timer.timeout.connect(self.spin)
        self.timer.start(20)
    def spin(self):
        if not rclpy.ok():
            self.timer.stop()
            QCoreApplication.instance().quit()
            return
        rclpy.spin_once(self.node,timeout_sec=0.0)
        fresh=self.node.fresh()
        self.connection.emit(fresh and self.node.move.server_is_ready())
        if fresh:
            values=[self.node.state[n] for n in ARM]
            self.joints.emit(values)
            stamp=max(self.node.received[n] for n in ARM)
            if self.previous and stamp>self.previous[0]:
                self.joint_speed.emit(max(abs(a-b) for a,b in zip(values,self.previous[1]))/(stamp-self.previous[0]))
            self.previous=(stamp,values)
        if self.node.pose is not None and time.monotonic()-self.node.pose_received<1.5:
            p=self.node.pose.position
            stamp=self.node.pose_received
            if self.previous_tcp and stamp>self.previous_tcp[0]:
                import math
                self.tool_speed.emit(math.dist((p.x,p.y,p.z),self.previous_tcp[1])/(stamp-self.previous_tcp[0]))
            self.previous_tcp=(stamp,(p.x,p.y,p.z))
            self.status.emit(SimpleNamespace(robot_state=3 if self.node.busy else 2,
                tool_position_x=p.x,tool_position_y=p.y,tool_position_z=p.z))
        if self.node.fresh(GRIPPER):
            # Display model joint displacement, not an assumed hardware jaw gap.
            self.gripper.emit(self.node.state[GRIPPER[0]]*1000.0)
    def _call(self,fn,*args):
        try: fn(*args)
        except (ValueError,RuntimeError) as exc: self.result.emit(str(exc),True)
    def execute_with_fallback(self,start,target,velocity,acceleration):
        self._call(self.node.joint_target,target,velocity,acceleration)
    def execute_pose_with_fallback(self,start,target,velocity,acceleration):
        self._call(self.node.pose_target,target,velocity,acceleration)
    def stop(self): self.node.stop()
    def command_gripper(self,command):
        self._call(self.node.joint_target,[0.0 if command=='open' else .04]*2,.2,.2,'gripper')
    def clear_error(self): self.result.emit('请在对应控制器端检查并复位故障',True)
    def close(self):
        if self.node.busy:
            self.node.stop()
            return False
        self.timer.stop();self.node.destroy_node();rclpy.try_shutdown()
        return True
