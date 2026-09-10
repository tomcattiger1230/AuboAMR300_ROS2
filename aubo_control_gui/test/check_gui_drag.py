#!/usr/bin/env python3
"""Opt-in visible Qt/MoveIt check; use a collision-free near-home Isaac pose.

Only requests IK and planning. Mouse dragging never sends ExecuteTrajectory.
Run with the same ROS environment as the GUI and --ros-args -p enable_motion:=true.
"""
import sys,time,json,math
from pathlib import Path
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt,QObject,QPoint
from PySide6.QtTest import QTest
from aubo_control_gui.app import MainWindow
from aubo_control_gui.resources import get_package_share_directory
app=QApplication(sys.argv);w=MainWindow(Path(get_package_share_directory('aubo_student_description'))/'urdf/aubo_i16.urdf');w.show()
def until(fn,timeout=35):
    end=time.monotonic()+timeout
    while not fn() and time.monotonic()<end:QTest.qWait(50)
    assert fn(),w.message.text()
try:
    until(lambda:w.actual_pose is not None)
    gizmo=w.view.rootObject().findChild(QObject,'endEffectorGizmo')
    QTest.qWait(200)
    segments=gizmo.property('segments').toVariant()
    print('SEGMENTS',len(segments),str(segments[:1]),flush=True)
    # Z translation: drag on the outermost quarter of its handle, beyond rotation rings.
    s=next(s for s in segments if s['axis']==2 and not s['rotation'])
    a,b=s['a'],s['b']
    if hasattr(a,'x'):ax,ay,bx,by=a.x(),a.y(),b.x(),b.y()
    else:ax,ay,bx,by=a['x'],a['y'],b['x'],b['y']
    start=QPoint(round(ax+.9*(bx-ax)),round(ay+.9*(by-ay)))
    before=list(w.target_pose)
    QTest.mousePress(w.view,Qt.LeftButton,Qt.NoModifier,start)
    QTest.mouseMove(w.view,start+QPoint(0,2),100)
    QTest.mouseRelease(w.view,Qt.LeftButton,Qt.NoModifier,start+QPoint(0,2))
    QTest.qWait(500)
    print('DRAG',before,w.target_pose,flush=True)
    assert w.target_pose[2] < before[2] and abs(w.target_pose[2]-before[2])<.02
    assert w.bridge.node.phase=='idle' and not w.bridge.node.busy
    w.plan_tcp();until(lambda:not w.bridge.node.busy)
    print('PLAN_CHECK',w.bridge.node.revision,w.bridge.node.request_revision,{n:w.bridge.node.state[n]-v for n,v in w.bridge.node.request_start.items()},flush=True)
    if not w.bridge.node.plan_ready():
        QTest.qWait(1000);w.plan_tcp();until(lambda:not w.bridge.node.busy)
    assert w.bridge.node.plan_ready(),w.message.text()
    QTest.qWait(300)
    print('PREVIEW',w.preview is not None,'ENABLED',w.execute_button.isEnabled(),'SERVER',w.bridge.node.execute.server_is_ready(),'MSG',w.message.text(),flush=True)
    until(lambda:w.execute_button.isEnabled(),15)
    assert w.preview is not None and w.execute_button.isEnabled()
    w.pause_preview();w.slider.setValue(1000)
    w.grab().save('/tmp/aubo-drag-planned.png')
    print('PLAN_READY',w.message.text(),'IK',w.view.rootObject().property('targetStatus'),flush=True)
    # Verify a rotation-ring mouse drag invalidates the previous trajectory.
    QTest.qWait(100)
    segments=gizmo.property('segments').toVariant()
    def clearance(s):
        x=(s['a'].x()+s['b'].x())/2;y=(s['a'].y()+s['b'].y())/2
        distances=[]
        for other in segments:
            if other['rotation'] and other['axis']==s['axis']:continue
            a,b=other['a'],other['b'];dx=b.x()-a.x();dy=b.y()-a.y()
            t=max(0,min(1,((x-a.x())*dx+(y-a.y())*dy)/max(.0001,dx*dx+dy*dy)))
            distances.append(math.hypot(x-a.x()-t*dx,y-a.y()-t*dy))
        return min(distances)
    s=max((s for s in segments if s['rotation']),key=clearance)
    print('RING',s,'CLEARANCE',clearance(s),flush=True)
    a,b=s['a'],s['b'];start=QPoint(round((a.x()+b.x())/2),round((a.y()+b.y())/2))
    dx,dy=b.x()-a.x(),b.y()-a.y();length=math.hypot(dx,dy)
    end=start+QPoint(round(dx/length*4),round(dy/length*4));before=list(w.target_pose)
    QTest.mousePress(w.view,Qt.LeftButton,Qt.NoModifier,start);QTest.mouseMove(w.view,end,100);QTest.mouseRelease(w.view,Qt.LeftButton,Qt.NoModifier,end)
    QTest.qWait(500)
    print('ROTATION_POSE',before,w.target_pose,flush=True)
    assert math.dist(before[3:],w.target_pose[3:])>.001
    assert not w.bridge.node.plan_ready() and not w.execute_button.isEnabled()
    print('ROTATION_INVALIDATED',before[3:],w.target_pose[3:],flush=True)
finally:
    w.close()
