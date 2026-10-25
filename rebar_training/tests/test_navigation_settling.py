import math
import pytest
from rebar_training.mobile import SettledPrecisionDock,PrecisionDock,TransportGate,TransportFeedback


def test_recorded_boundary_cannot_reverse_yaw_alignment_after_acquisition():
    goal=(9.23,1.3,-2.432966381462128)
    candidate=SettledPrecisionDock(goal,.015,.02,.15,.2)
    candidate.step((9.23,1.309,1.9),(0.,0.),1/30)
    assert candidate.phase=='yaw'
    for distance in (.0142,.01514,.0148,.01508)*10:
        pose=(goal[0],goal[1]+distance,1.9)
        assert candidate.step(pose,(0.,0.),1/30)[1]>0
        assert not candidate.ready
    assert candidate.phase_changes==1
    # Legacy behavior on the real threshold crossing is retained separately.
    assert PrecisionDock(goal,.015,.02,.15,.2).step((9.23,1.31514,1.9),(0.,0.),1/30)[1]<0


def test_margin_loss_reacquires_position_and_cannot_false_accept():
    c=SettledPrecisionDock((0.,0.,0.),.015,.02,.15,.2)
    c.step((0.,.009,1.),(0.,0.),1/30);assert c.phase=='yaw'
    c.step((0.,.020,1.),(0.,0.),1/30);assert c.phase=='position'
    for _ in range(60):c.step((0.,.016,0.),(0.,0.),1/30)
    assert not c.ready
    for _ in range(60):c.step((0.,.009,0.),(.02,0.),1/30)
    assert not c.ready
    for _ in range(16):assert c.step((0.,.009,0.),(0.,0.),1/30)==(0.,0.)
    assert c.ready
    c.step((0.,.009,.025),(0.,0.),1/30);assert not c.ready


def test_acquired_yaw_with_invalid_position_still_reacquires():
    c=SettledPrecisionDock((0.,0.,0.))
    c.step((0.,.004,1.),(0.,0.),1/30);assert c.phase=='yaw'
    c.step((0.,.009,0.),(0.,0.),1/30);assert c.phase=='position' and not c.ready
    with pytest.raises(ValueError):c.step((0.,0.,math.nan),(0.,0.),1/30)


@pytest.mark.parametrize('fault,reason',[
    ('camera','stale camera'),('tray','steel not supported by tray'),('arm','arm outside transport pose')])
def test_settling_commands_remain_subject_to_immediate_transport_faults(fault,reason):
    c=SettledPrecisionDock((0.,0.,1.));gate=TransportGate()
    for k in range(20):
        t=(k+1)/30;gate.command_twist(*c.step((0.,0.,0.),(0.,0.),1/30),t)
        assert gate.tick(TransportFeedback(t,t,t,True,True,True))[1]>0
    t+=1/30;gate.command_twist(0.,.1,t)
    assert gate.tick(TransportFeedback(t,t-1 if fault=='camera' else t,t,fault!='tray',fault!='arm',True))==(0.,0.)
    assert gate.reason==reason
