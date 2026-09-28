"""Optional localhost-only ROS checks using real generated interface types."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace
import pytest
from PySide6.QtWidgets import QApplication
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
rclpy = pytest.importorskip('rclpy')
msgs = pytest.importorskip('aubo_bridge_msgs.srv')

@pytest.fixture
def local_ros(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','221')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
    monkeypatch.delenv('ROS_STATIC_PEERS',raising=False)
    monkeypatch.delenv('FASTDDS_DEFAULT_PROFILES_FILE',raising=False)
    app = QApplication.instance() or QApplication([])
    rclpy.init(args=[])
    yield app
    rclpy.try_shutdown()


def test_real_arm_uses_generated_types_without_device(local_ros):
    from aubo_control_gui.unified.real_arm import RealArm
    arm = RealArm(lambda *args:None)
    assert not arm.ready()
    assert all(not b.isEnabled() for b in arm.actions)
    assert arm.joint_type.Request().enable_move is False
    arm.close()


def test_pose_handler_matches_service_response():
    source = Path(__file__).resolve().parents[2]/'aubo_bridge/scripts/aubo_bridge_node.py'
    tree = ast.parse(source.read_text())
    method = next(n for c in tree.body if isinstance(c,ast.ClassDef) for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='on_move_to_pose')
    namespace = {}; exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
    fake = SimpleNamespace(get_logger=lambda:SimpleNamespace(info=lambda *a:None),default_line_acc=.3,default_line_vel=.3,
        _move_line_to_pose_impl=lambda *a,**kw:(True,'local SDK stub success',[0]*6,[0]*3,[1,0,0,0]))
    response = namespace['on_move_to_pose'](fake,msgs.MoveToPose.Request(),msgs.MoveToPose.Response())
    assert response.success and response.message == 'local SDK stub success'
    def fail(*args,**kw): raise RuntimeError('SDK stub failure')
    fake._move_line_to_pose_impl = fail
    response = namespace['on_move_to_pose'](fake,msgs.MoveToPose.Request(),msgs.MoveToPose.Response())
    assert not response.success and 'SDK stub failure' in response.message


def test_isaac_base_has_no_drive_without_odometry(local_ros):
    pytest.importorskip('nav2_msgs.action')
    from aubo_control_gui.unified.base import IsaacBase
    base = IsaacBase()
    with pytest.raises(RuntimeError): base.drive(.1,0)
    assert base.ready_to_close()
    base.close()


def test_map_service_and_initial_pose_require_localization_feedback(local_ros):
    import time
    from nav2_msgs.srv import LoadMap
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from aubo_control_gui.unified.base import IsaacBase
    server = rclpy.create_node('offline_map_server')
    requests = []; initial = []
    def load(request,response):
        requests.append(request.map_url)
        if request.map_url.endswith('missing.yaml'):
            response.result = 3
            return response
        response.result = 0
        response.map.header.frame_id = 'map'
        response.map.info.width = response.map.info.height = 2
        response.map.info.resolution = .05
        response.map.info.origin.orientation.w = 1.
        response.map.data = [0]*4
        return response
    service = server.create_service(LoadMap,'/map_server/load_map',load)
    sub = server.create_subscription(PoseWithCovarianceStamped,'/initialpose',initial.append,10)
    pub = server.create_publisher(PoseWithCovarianceStamped,'/amcl_pose',10)
    base = IsaacBase(); base.timer.stop()
    def spin_until(condition):
        deadline = time.monotonic()+5
        while not condition() and time.monotonic()<deadline:
            rclpy.spin_once(server,timeout_sec=.01); base.tick()
        assert condition()
    try:
        spin_until(lambda:base.map_client.service_is_ready() and base.initial_pub.get_subscription_count()>0)
        base.load_map('/tmp/simulation.yaml')
        with pytest.raises(RuntimeError): base.drive(.1,0)
        spin_until(lambda:base.map_future is None)
        assert requests == ['/tmp/simulation.yaml'] and base.map_ready
        with pytest.raises(RuntimeError,match='初始位置'): base.navigate_pose(1,1,0)
        base.set_initial_pose(.1,.2,.3)
        spin_until(lambda:bool(initial))
        assert initial[0].header.frame_id == 'map'
        assert initial[0].pose.pose.position.x == .1
        assert base.localization_required
        stale = PoseWithCovarianceStamped(); stale.header.frame_id='map'; stale.header.stamp.sec=-1
        base._localized(stale)
        assert base.localization_required
        pub.publish(initial[0])
        spin_until(lambda:not base.localization_required)
        assert not base.localization_pending
        statuses=[];base.map_status.connect(statuses.append)
        with pytest.raises(ValueError):base.load_map('relative.yaml')
        base.load_map('/tmp/missing.yaml')
        spin_until(lambda:base.map_future is None)
        assert base.localization_required and '失败' in statuses[-1]
        base.load_map('/tmp/slow.yaml');base.map_deadline=time.monotonic()-1
        base.tick()
        assert base.map_future is None and base.localization_required
        assert '超时' in statuses[-1]
    finally:
        base.close(); server.destroy_node()


def test_existing_moveit_arm_can_share_initialized_context(local_ros):
    pytest.importorskip('moveit_msgs.action')
    from aubo_control_gui.app import MainWindow
    from aubo_control_gui.resources import get_package_share_directory
    arm = MainWindow(Path(get_package_share_directory('aubo_student_description'))/'urdf/aubo_i16.urdf')
    assert not arm.bridge.node.fresh()
    arm.close()


@pytest.mark.parametrize('mode',['isaac','real'])
def test_shell_connect_disconnect_both_subsystems_locally(monkeypatch,mode):
    import os
    from aubo_control_gui.unified.profile import Profile
    from aubo_control_gui.unified.base import SeerBase
    from aubo_control_gui.unified.window import UnifiedWindow
    def localhost(self):
        os.environ['ROS_DOMAIN_ID']='221'
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
        os.environ.pop('ROS_STATIC_PEERS',None)
        os.environ.pop('FASTDDS_DEFAULT_PROFILES_FILE',None)
    monkeypatch.setattr(Profile,'configure_ros',localhost)
    monkeypatch.setattr(SeerBase,'_poll',lambda self:None)
    app = QApplication.instance() or QApplication([])
    window = UnifiedWindow(Profile(mode=mode,domain=221,peer='127.0.0.1',seer_ip='127.0.0.1'))
    window.connect_robot()
    assert window.base is not None and window.arm is not None, window.logs.toPlainText()
    assert not window.mode.isEnabled()
    assert window.disconnect_robot()
    assert window.base is None and window.arm is None and window.mode.isEnabled()
    window.close()
