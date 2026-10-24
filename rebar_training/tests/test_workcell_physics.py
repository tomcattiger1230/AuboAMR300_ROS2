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


def test_actual_candidate_loading_geometry_requires_explicit_scene_survey():
    # Execute the actual method without importing the Isaac-only environment.
    import ast
    from types import SimpleNamespace
    from rebar_training.route import LoadingRoute
    from rebar_training.vision.frames import BaseFrame
    src=Path(__file__).parents[1]/'src/rebar_training/workcell_sim_env.py'
    tree=ast.parse(src.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='WorkcellControl')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='loading_geometry')
    namespace={'__package__':'rebar_training','np':np};exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),str(src),'exec'),namespace)
    layout=WorkcellLayout.load(LAYOUT)
    for name in ('machine_1','machine_2','machine_3'):
        base=layout.navigation_goal(name);c,s=math.cos(base.yaw),math.sin(base.yaw)
        frame=BaseFrame((base.x,base.y,base.z),(0.,0.,math.sin(base.yaw/2),math.cos(base.yaw/2)))
        controller=SimpleNamespace(env=SimpleNamespace(layout=layout,cfg=SimpleNamespace(workcell_machine=name,workcell_ideal_machine_survey=True)),frame=lambda:frame)
        g=namespace['loading_geometry'](controller)
        assert g['refined'] is True and g['frame']=='base'
        assert g['goal']==pytest.approx((-.3,-.77,1.508))
        # The unchanged route must accept these actual candidate targets.
        checked=LoadingRoute(1,'cpu')._loading_geometry(g['goal'],g)
        assert checked is not None
        controller.env.cfg.workcell_ideal_machine_survey=False
        with pytest.raises(ValueError,match='explicit exact-scene survey'):namespace['loading_geometry'](controller)


def test_measured_physical_pitch_retains_exact_route_goal_guard():
    from rebar_training.workcell_physics import ideal_scene_loading_geometry
    from rebar_training.vision.frames import BaseFrame
    from rebar_training.route import LoadingRoute
    layout=WorkcellLayout.load(LAYOUT)
    # Recorded v4 final base feedback: planar projection differs by 1.477 um.
    frame=BaseFrame([6.295770168304443,4.770079612731934,-9.93837829810218e-8],
        [-2.3684656014211214e-7,4.913092084279924e-7,-.00666304724290967,.9999777674674988])
    geometry=ideal_scene_loading_geometry(layout,'machine_1',frame,enabled=True)
    actual=frame.point(layout.machine_geometry('machine_1')['goal'])
    assert geometry['goal']==pytest.approx(actual,abs=1e-12)
    assert LoadingRoute(1,'cpu')._loading_geometry(actual,geometry) is not None


def test_loading_orientation_advances_route_without_corrupting_grasp_evidence():
    import torch
    from types import SimpleNamespace
    from rebar_training.vision.frames import BaseFrame
    from rebar_training.workcell_physics import ideal_scene_loading_geometry
    from rebar_training.route import LoadingRoute
    layout=WorkcellLayout.load(LAYOUT)
    # Actual v6 terminal measured TCP/steel estimate, not scoring truth.
    frame=BaseFrame([6.306112289428711,4.769908905029297,9.416947222007366e-9],
        [2.9727766559517477e-7,-7.228599940845015e-8,-.011750463396310806,.9999309778213501])
    g=ideal_scene_loading_geometry(layout,'machine_1',frame,enabled=True)
    state=SimpleNamespace(bar=torch.tensor([[-.288665235,-.726651788,1.508043885]]),
        tcp=torch.tensor([[-.290029317,-.717264473,1.507939816]]),
        fingers=torch.tensor([[.017839693,.015851855]]),finger_forces=torch.tensor([[53.30795,53.43282]]),jaw_forces=torch.zeros(1,4))
    logic=SimpleNamespace(contact_dwell=torch.tensor([0]),pick_complete=torch.tensor([True]),phase=torch.tensor([0]))
    route=LoadingRoute(1,'cpu');route.stage[:]=6;route.grasp_alignment_error_deg[:]=.104678
    q=torch.tensor([[.706952271,.007641190,.007156748,.707183767]])
    route.advance(state,logic,torch.zeros(1,6),g['goal'],loading_geometry=g)
    assert route.stage.item()==6
    route.advance(state,logic,torch.zeros(1,6),g['goal'],loading_geometry=g,loading_quaternion=q)
    assert route.stage.item()==7
    assert route.grasp_alignment_error_deg.item()==pytest.approx(.104678)
    route.stage[:]=6
    with pytest.raises(ValueError,match='Invalid measured loading quaternion'):
        route.advance(state,logic,torch.zeros(1,6),g['goal'],loading_geometry=g,loading_quaternion=torch.zeros(1,4))
