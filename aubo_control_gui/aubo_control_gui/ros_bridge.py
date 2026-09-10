"""Qt adapter: both backend profiles use exactly the same MoveIt client."""
import time
from types import SimpleNamespace
from PySide6.QtCore import QObject, QTimer, Signal, QCoreApplication
import rclpy
from .motion_client import MotionClient, ARM, GRIPPER
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from .tool_preview import camera_mount_from_description

class RosBridge(QObject):
    plan_changed=Signal(object); pose_changed=Signal(object); ik_result=Signal(object, str)
    camera_mount=Signal(object)
    gripper_joints=Signal(object)
    joints=Signal(object); joint_speed=Signal(float); tool_speed=Signal(float)
    status=Signal(object); gripper=Signal(float); connection=Signal(bool); result=Signal(str,bool)
    def __init__(self,parent=None):
        super().__init__(parent)
        rclpy.init(args=None)
        self.node=MotionClient()
        self.last_camera_description=None
        self.camera_subscription=self.node.create_subscription(String,'robot_description',self._camera_description,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.on_result=self.result.emit
        self.node.on_plan=self.plan_changed.emit
        self.ik=self.node.create_client(GetPositionIK,'compute_ik')
        self.ik_pending=None
        self.ik_revision=0
        self.ik_target=None
        self.ik_timer=QTimer(self); self.ik_timer.setSingleShot(True)
        self.ik_timer.timeout.connect(self._send_ik)
        self.connection_started=time.monotonic()
        self.last_connection_report=None
        self.previous=None
        self.previous_tcp=None
        self.timer=QTimer(self)
        self.timer.timeout.connect(self.spin)
        self.timer.start(20)
    def _camera_description(self,msg):
        if msg.data == self.last_camera_description:
            return
        try:
            mount=camera_mount_from_description(msg.data)
        except (ValueError, KeyError) as exc:
            self.result.emit(f'远端相机安装变换不可用：{exc}',True)
            return
        self.camera_mount.emit(mount)
        self.last_camera_description=msg.data
        print(f"[AUBO GUI] camera mount from remote robot_description: {mount.tolist()}",flush=True)

    def spin(self):
        if not rclpy.ok():
            self.timer.stop()
            QCoreApplication.instance().quit()
            return
        # Drain clock, feedback and action callbacks without starving the Qt event loop.
        deadline=time.monotonic()+.008
        for _ in range(16):
            rclpy.spin_once(self.node,timeout_sec=0.0)
            if time.monotonic()>=deadline:break
        fresh=self.node.fresh()
        moveit=self.node.move.server_is_ready()
        self.connection.emit(fresh and moveit)
        missing=tuple(name for name in ARM+GRIPPER if not self.node.fresh((name,)))
        report=(missing,moveit)
        if report != self.last_connection_report and (fresh or time.monotonic()-self.connection_started>10):
            print(f"[AUBO GUI] feedback={fresh} moveit={moveit} missing={','.join(missing) or 'none'}", flush=True)
            self.last_connection_report=report
        if fresh:
            values=[self.node.state[n] for n in ARM]
            self.joints.emit(values)
            stamp=max(self.node.received[n] for n in ARM)
            if self.previous and stamp>self.previous[0]:
                self.joint_speed.emit(max(abs(a-b) for a,b in zip(values,self.previous[1]))/(stamp-self.previous[0]))
            self.previous=(stamp,values)
        if self.node.pose is not None and time.monotonic()-self.node.pose_received<1.5:
            p=self.node.pose.position
            q=self.node.pose.orientation
            self.pose_changed.emit([p.x,p.y,p.z,q.x,q.y,q.z,q.w])
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
            self.gripper_joints.emit([self.node.state[name] for name in GRIPPER])
    def _call(self,fn,*args):
        try: fn(*args)
        except (ValueError,RuntimeError) as exc: self.result.emit(str(exc),True)
    def plan_joints(self,start,target,velocity,acceleration):
        self._call(self.node.joint_target,target,velocity,acceleration)
    def plan_pose(self,start,target,velocity,acceleration,quaternion=None):
        self._call(self.node.pose_target,target,velocity,acceleration,quaternion)
    def execute_plan(self): self._call(self.node.execute_planned)
    def stop(self): self.node.stop()
    def command_gripper(self,command,velocity=.2,acceleration=.2):
        self._call(self.node.joint_target,[0.0 if command=='open' else .04]*2,velocity,acceleration,'gripper')
    def clear_error(self): self.result.emit('请在对应控制器端检查并复位故障',True)
    def close(self):
        if self.node.busy:
            self.node.stop()
            return False
        self.timer.stop();self.ik_timer.stop();self.node.destroy_node();rclpy.try_shutdown()
        return True

    def request_ik(self, pose):
        self.ik_target=list(pose); self.ik_revision+=1
        self.ik_timer.start(180)

    def _send_ik(self):
        if self.ik_pending is not None or self.ik_target is None: return
        if not self.node.fresh(ARM+GRIPPER) or not self.ik.service_is_ready():
            self.ik_result.emit(None,'IK 不可用或关节反馈过期')
            return
        revision=self.ik_revision
        req=GetPositionIK.Request(); r=req.ik_request
        r.group_name='arm';r.ik_link_name=self.node.get_parameter('tool_link').value
        r.pose_stamped.header.frame_id=self.node.get_parameter('planning_frame').value
        p=r.pose_stamped.pose.position;q=r.pose_stamped.pose.orientation
        p.x,p.y,p.z=map(float,self.ik_target[:3]);q.x,q.y,q.z,q.w=map(float,self.ik_target[3:])
        r.robot_state.joint_state.name=list(self.node.state);r.robot_state.joint_state.position=list(self.node.state.values())
        r.robot_state.is_diff=True;r.avoid_collisions=True;r.timeout.nanosec=200000000
        self.ik_pending=self.ik.call_async(req)
        def done(f):
            self.ik_pending=None
            if revision!=self.ik_revision:
                self.ik_timer.start(1);return
            try:
                response=f.result()
                if response.error_code.val==1:
                    state=dict(zip(response.solution.joint_state.name,response.solution.joint_state.position))
                    self.ik_result.emit([state[n] for n in ARM],'IK 有解；点击“规划末端目标”生成完整轨迹')
                else:
                    self.ik_result.emit(None,f'IK 无有效解（{response.error_code.val}）；目标可能不可达或碰撞')
            except Exception as exc: self.ik_result.emit(None,str(exc))
        self.ik_pending.add_done_callback(done)
