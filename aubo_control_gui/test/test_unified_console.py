"""Offline transport/state tests; no robot addresses are contacted."""
import json
import math
import socket
import struct
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
import pytest
from PySide6.QtWidgets import QApplication

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from aubo_control_gui.unified.profile import Profile, velocity
from aubo_control_gui.unified.seer.protocol import pack_message, unpack_message
from aubo_control_gui.unified.seer.client import AgvClient
from aubo_control_gui.unified.base import SeerBase
from aubo_control_gui.unified.map_canvas import MapCanvas
from aubo_control_gui.unified.window import UnifiedWindow

@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])

@pytest.mark.parametrize('kwargs',[{'mode':'demo'},{'domain':233},{'peer':'bad'},{'seer_ip':'300.1.1.1'}])
def test_profile_rejects_invalid_connection(kwargs):
    with pytest.raises(ValueError): Profile(**kwargs).validate()

@pytest.mark.parametrize('values',[(math.nan,0),(0,math.inf),(.31,0),(0,-.61)])
def test_velocity_rejects_invalid_commands(values):
    with pytest.raises(ValueError): velocity(*values)


def test_preview_and_mode_switch_never_open_network(app, monkeypatch):
    monkeypatch.setattr(socket.socket,'connect',lambda *args:pytest.fail('preview opened network'))
    window = UnifiedWindow()
    assert window.base is None and window.arm is None
    assert not window.video.timer.isActive() and window.video.url is None
    window.mode.setCurrentIndex(1)
    assert window.domain.value() == 20
    assert not window.sim_nav.isVisible()
    assert all(not b.isEnabled() for b in window.drive_buttons)
    window.mode.setCurrentIndex(0)
    assert window.domain.value() == 133
    window.close()


def test_lm_dropdown_syncs_map_and_clears_target_on_reload(app, monkeypatch):
    monkeypatch.setattr(socket.socket,'connect',lambda *args:pytest.fail('map selection opened network'))
    window = UnifiedWindow(Profile(mode='real'))
    window.navigate_station = lambda: pytest.fail('selection started navigation')
    maps = Path(__file__).resolve().parents[2] / 'maps' / 'realworld'
    try:
        window.set_seer_map(maps / 'HSJC01.smap')
        assert window.station_choices.count() == 4
        assert window.target.text() == ''
        window.station_choices.setCurrentIndex(2)
        assert window.target.text() == 'LM2'
        assert window.seer_map.selected_station_id == 'LM2'
        window.seer_map.stationDoubleClicked.emit('LM3', 35.211, -13.614)
        assert window.station_choices.currentData() == 'LM3'
        window.target.setText('custom_station')
        assert window.station_choices.currentIndex() == 0
        assert window.target.text() == 'custom_station'
        window.set_seer_map(maps / 'HSJC04.smap')
        assert window.station_choices.count() == 6
        assert window.target.text() == ''
        assert window.seer_map.selected_station_id == ''
        assert window.base is None
    finally:
        window.close()


def test_seer_request_handles_fragmented_response(tmp_path):
    server = socket.socket(); server.bind(('127.0.0.1',0)); server.listen(1)
    port = server.getsockname()[1]
    received = []
    def serve():
        with server:
            conn,_ = server.accept()
            with conn:
                data,meta = unpack_message(conn); received.append((data,meta))
                packet = pack_message(1,13051,{'ret_code':0,'task_status':2})
                for byte in packet: conn.sendall(bytes([byte]))
    thread = threading.Thread(target=serve); thread.start()
    result = AgvClient('127.0.0.1',1,tmp_path).request(port,3051,{'source_id':'LM1','id':'LM2','task_id':'gui-test'})
    thread.join(2)
    assert result.ok and result.response['task_status'] == 2
    assert received[0][0] == {'source_id':'LM1','id':'LM2','task_id':'gui-test'}
    assert received[0][1]['msg_type'] == 3051


def test_real_drive_guard_and_release_stop_order(app,tmp_path):
    base = SeerBase(Profile(mode='real',real_manual=True),tmp_path)
    base.timer.stop()
    events = []; started = threading.Event(); release = threading.Event()
    response = SimpleNamespace(ok=True,response={'ret_code':0},error='')
    def drive(*args):
        events.append(('drive',args)); started.set(); release.wait(1); return response
    base.client.open_loop_motion = drive
    base.client.stop_open_loop_motion = lambda:(events.append(('stop',())) or response)
    base.last_feedback = time.monotonic(); base.estop = False; base.last_poll = time.monotonic()
    base.drive(.1,0); assert started.wait(1)
    base.stop()
    with pytest.raises(RuntimeError): base.drive(.1,0)
    release.set(); base.control_future.result(2)
    assert [kind for kind,args in events] == ['drive','stop']
    assert base.active is None
    base.close()


def test_estop_unknown_and_stale_poll_prevent_drive(app,tmp_path):
    base = SeerBase(Profile(mode='real',real_manual=True),tmp_path); base.timer.stop()
    base._received(('poll',{'position':{},'speed':{},'navigation':{},'estop':{}}))
    with pytest.raises(RuntimeError): base.drive(.1,0)
    base._received(('poll',None))
    assert base.last_feedback == 0
    base.close()


def test_real_manual_requires_profile_opt_in(app,tmp_path):
    base = SeerBase(Profile(mode='real'),tmp_path); base.timer.stop()
    base.last_feedback = time.monotonic(); base.estop = False
    with pytest.raises(RuntimeError): base.drive(.1,0)
    base.close()


def test_rotated_map_round_trip_and_frame_filter(app):
    from PySide6.QtCore import QPointF
    canvas = MapCanvas(); canvas.resize(800,600)
    origin = SimpleNamespace(position=SimpleNamespace(x=2,y=3),orientation=SimpleNamespace(x=0,y=0,z=math.sin(math.pi/4),w=math.cos(math.pi/4)))
    msg = SimpleNamespace(header=SimpleNamespace(frame_id='map'),info=SimpleNamespace(origin=origin,resolution=.1,width=10,height=20),data=[0]*200)
    canvas.set_map(msg)
    point = canvas.screen(1,3.5)
    assert canvas.world(point) == pytest.approx((1,3.5))
    canvas.set_path(SimpleNamespace(header=SimpleNamespace(frame_id='odom'),poses=[SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1,y=2)))]))
    assert canvas.path == []


def test_station_navigation_uses_feedback_source_and_unique_task(app,tmp_path):
    base = SeerBase(Profile(mode='real'),tmp_path); base.timer.stop()
    base.last_feedback = time.monotonic(); base.estop = False
    with pytest.raises(ValueError): base.navigate_station('', 'LM2')
    base.position = {'current_station':'LM1'}
    sent = []
    def navigate(*args):
        sent.append(args); return SimpleNamespace(ok=True,response={'ret_code':0},error='')
    base.client.path_navigation = navigate
    base.navigate_station('', 'LM2'); base.control_future.result(2)
    assert sent[0][:2] == ('LM1','LM2') and sent[0][2].startswith('unified-')
    base.nav_busy = False
    base.close()


def test_nav2_late_acceptance_is_cancelled_and_waits_for_terminal(app,monkeypatch):
    from concurrent.futures import Future
    from PySide6.QtCore import QObject
    from aubo_control_gui.unified.base import IsaacBase
    def make_goal():
        return SimpleNamespace(pose=SimpleNamespace(header=SimpleNamespace(frame_id='',stamp=None),pose=SimpleNamespace(
            position=SimpleNamespace(x=0,y=0),orientation=SimpleNamespace(z=0,w=1))))
    module = SimpleNamespace(NavigateToPose=SimpleNamespace(Goal=make_goal))
    monkeypatch.setitem(sys.modules,'nav2_msgs',SimpleNamespace(action=module))
    monkeypatch.setitem(sys.modules,'nav2_msgs.action',module)
    base = IsaacBase.__new__(IsaacBase); QObject.__init__(base)
    accepted = Future(); terminal = Future(); cancels = []
    base.nav = SimpleNamespace(server_is_ready=lambda:True,send_goal_async=lambda goal:accepted)
    base.node = SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:None)))
    base.position = {'x':0}; base.last_feedback = time.monotonic(); base.active = None
    base.goal = None; base.nav_pending = False; base.cancel_requested = False
    base.map_future = None; base.localization_required = False
    base.navigate_pose(1,2,0)
    base.cancel()
    handle = SimpleNamespace(accepted=True,get_result_async=lambda:terminal,cancel_goal_async=lambda:cancels.append(True))
    accepted.set_result(handle)
    assert cancels == [True] and base.nav_pending
    terminal.set_result(SimpleNamespace(status=5))
    assert not base.nav_pending and base.goal is None
    base.deleteLater()


def test_yaml_map_preview_orientation_and_occupancy(app,tmp_path):
    from aubo_control_gui.unified.ros_map import load_yaml_map
    (tmp_path/'map.pgm').write_bytes(b'P5\n2 2\n255\n'+bytes([0,254,205,254]))
    path = tmp_path/'map.yaml'
    path.write_text('image: map.pgm\nresolution: 0.2\norigin: [3, 4, 1.5707963267948966]\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    grid = load_yaml_map(path)
    assert grid.data == [-1,0,100,0]
    canvas = MapCanvas(); canvas.set_map(grid)
    assert canvas.world(canvas.screen(2.9,4.3)) == pytest.approx((2.9,4.3))
    assert not canvas.is_free(2.9,4.1)
    assert canvas.is_free(2.9,4.3)
    path.write_text(path.read_text()+'mode: scaled\n')
    with pytest.raises(ValueError,match='trinary'): load_yaml_map(path)


def test_local_preview_does_not_apply_map_or_send_pose(app,monkeypatch):
    monkeypatch.setattr(socket.socket,'connect',lambda *args:pytest.fail('preview opened network'))
    path = Path(__file__).resolve().parents[2]/'maps/sim/rebar_lab_20260928/rebar_lab.yaml'
    window = UnifiedWindow()
    try:
        window.set_sim_map(path)
        assert window.preview_map and window.ros_map.grid.info.width == 492
        assert window.remote_map_path.text().startswith('/home/arnold/')
        grid = window.ros_map.grid
        window.receive_ros_map(SimpleNamespace(header=SimpleNamespace(frame_id='map')))
        assert window.ros_map.grid is grid
        window.navigate_pose(); window.send_initial_pose()
        assert window.base is None
        assert '本地预览' in window.logs.toPlainText()
        window.remote_grid = None
        window.show_live_map()
        assert window.ros_map.grid is None
    finally: window.close()


def test_initial_pose_map_drag_does_not_select_navigation_goal(app):
    from PySide6.QtCore import Qt
    from PySide6.QtTest import QTest
    canvas = MapCanvas(); canvas.resize(600,400)
    canvas.set_map(SimpleNamespace(header=SimpleNamespace(frame_id='map'),
        info=SimpleNamespace(width=10,height=10,resolution=1.,origin=SimpleNamespace(
            position=SimpleNamespace(x=0,y=0),orientation=SimpleNamespace(x=0,y=0,z=0,w=1))),data=[0]*100))
    canvas.selection_mode='initial'; initial=[]; goals=[]
    canvas.initial_pose_selected.connect(lambda *pose:initial.append(pose))
    canvas.goal_selected.connect(lambda *pose:goals.append(pose))
    QTest.mousePress(canvas,Qt.LeftButton,pos=canvas.screen(2,3).toPoint())
    QTest.mouseRelease(canvas,Qt.LeftButton,pos=canvas.screen(3,3).toPoint())
    assert initial[0] == pytest.approx((2,3,0))
    assert goals == [] and canvas.goal is None
