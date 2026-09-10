import math
import pytest
from aubo_control_gui.planner import PlanningError, plan_cartesian_path, plan_joint_path, start_matches_actual
def test_direct_path_contains_target_only():
    plan=plan_joint_path([0]*6,[1]*6); assert plan.waypoints==((1.0,)*6,)
def test_fallback_interpolates_requested_points():
    plan=plan_joint_path([0]*6,[1]*6,3); assert len(plan.waypoints)==4; assert plan.waypoints[0]==(.25,)*6; assert plan.waypoints[-1]==(1.0,)*6
def test_limits_and_start_tolerance():
    with pytest.raises(PlanningError): plan_joint_path([0]*6,[2*math.pi+.01]*6)
    assert start_matches_actual([0]*6,[.01]*6,.02)
def test_cartesian_fallback_interpolation():
    points=plan_cartesian_path((0,0,0),(.4,.8,1.2),3)
    assert len(points)==4 and points[0]==(.1,.2,.3) and points[-1]==(.4,.8,1.2)
