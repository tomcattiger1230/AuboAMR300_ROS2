"""Verify Qt-displayed feedback, not just DDS discovery. Never request motion."""
import json
import time
from pathlib import Path
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer
from aubo_control_gui.app import MainWindow
from aubo_control_gui.resources import get_package_share_directory
from aubo_control_gui.motion_client import ARM, GRIPPER

app = QApplication([])
app.setApplicationName('AUBO Control GUI')
app.setOrganizationName('AUBO')
window = MainWindow(Path(get_package_share_directory('aubo_student_description'))/'urdf/aubo_i16.urdf')
assert not window.bridge.node.get_parameter('enable_motion').value
window.show()
started = time.monotonic()
successes = 0
result = {}

def check():
    global successes, result
    n = window.bridge.node
    ready = n.fresh(ARM+GRIPPER) and n.move.server_is_ready() and window.actual is not None and window.actual_pose is not None
    successes = successes+1 if ready else 0
    if successes >= 5 or time.monotonic()-started >= 30:
        result = dict(ok=successes>=5, connection=window.conn.text(), joints=[label.text() for label in window.joint_labels], executed=False)
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
        timer.stop()
        window.close()
        app.quit()

timer = QTimer()
timer.timeout.connect(check)
timer.start(1000)
app.exec()
if not result.get('ok'):
    raise SystemExit('GUI feedback check failed; compare --check in the same terminal')
