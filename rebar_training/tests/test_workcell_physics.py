import math
from pathlib import Path
import numpy as np
import pytest
from rebar_training.workcells import WorkcellLayout,Pose
from rebar_training.workcell_physics import machine_cell_pose,base_reset_pose,WorkcellResetSampler,plan_chassis_path
from rebar_training.navigation_map import chassis_clear

LAYOUT=Path(__file__).parents[1]/'config/multi_workcell_offline.json'


def test_fixture_cell_transform_matches_legacy_and_rotations():
    # Original fixture local goal is (-.87,-.08,1.508) under the 90-degree cell.
    for yaw in (0,math.pi/2,-math.pi/2):
        p=Pose(6,4,0,yaw);cell=machine_cell_pose(p)
        assert cell.point((-.87,-.08,1.508))==pytest.approx(p.point((0,0,1.508)))
    assert machine_cell_pose(Pose(6,4,0,0)).values()==pytest.approx((5.92,4.87,0,math.pi/2))


def test_reset_sampler_rotates_physical_pose_only_at_reset():
    import torch
    from rebar_training.reset import ResetSampler
    layout=WorkcellLayout.load(LAYOUT)
    for name in ('source_A','source_B'):
        original=ResetSampler(2525100);expected=ResetSampler(2525100).sample('loading',1)
        wrapped=WorkcellResetSampler(original,layout,name);got=wrapped.sample('loading',1)
        p=layout.station(name).pose_map
        for a,b in ((got.bar_pose,expected.bar_pose),(got.station_pose,expected.station_pose)):
            local=[float(b[0,0])+1.15,float(b[0,1]),float(b[0,2])]
            assert a[0,:3].tolist()==pytest.approx(p.point(local),abs=1e-6)
        assert torch.equal(got.joints,expected.joints)
        assert wrapped.generator is original.generator
        dock=layout.navigation_goal(name);r=base_reset_pose(layout,name,.15)
        assert r[:3]==pytest.approx(dock.point((.15,0,0)))
    with pytest.raises(ValueError):base_reset_pose(layout,'source_A',.5)


def test_collision_route_avoids_obstacle_and_retains_final_yaw():
    obstacle=dict(minimum=[1.,-.5,0],maximum=[2.,.5,1.])
    goals=plan_chassis_path((0.,0.,0.),(4.,0.,0.),[obstacle]);assert goals[-1]==(4.,0.,0.)
    assert any(abs(y)>.5 for x,y,yaw in goals)
    previous=(0.,0.,0.)
    for point in goals:
        yaw=math.atan2(point[1]-previous[1],point[0]-previous[0])
        for x,y in np.linspace(previous[:2],point[:2],100):assert chassis_clear((x,y,yaw),[obstacle])
        previous=point
    with pytest.raises(ValueError):plan_chassis_path((1.5,0.,0.),(4.,0.,0.),[obstacle])


def test_simulation_candidate_has_no_training_or_truth_control_api():
    import ast
    src=Path(__file__).parents[1]/'src/rebar_training'
    tree=ast.parse((src/'workcell_sim_check.py').read_text())
    calls={n.func.attr if isinstance(n.func,ast.Attribute) else n.func.id if isinstance(n.func,ast.Name) else '' for n in ast.walk(tree) if isinstance(n,ast.Call)}
    assert not {'ResidualPPO','backward','step_optimizer'}&calls
    # Original validated implementation and checkpoint are kept separately.
    import hashlib
    assert hashlib.sha256((src/'mobile_env.py').read_bytes()).hexdigest()=='d2985ce3200eea0529bd3963c933683faefed41d836e7293b0da0fe60a82c3af'


def test_independent_review_never_accepts_bounded_or_runtime_failure():
    from rebar_training.workcell_sim_review import review
    layout=WorkcellLayout.load(LAYOUT)
    result=review(dict(completed=False,results=[],error='sensor stale'),layout)
    assert not result['physical_candidate_passed']
    assert not result['task_interface_accepted']
    assert any('runtime error' in x for x in result['errors'])
