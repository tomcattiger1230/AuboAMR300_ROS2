"""Reset-only scene transforms and static CAD navigation for simulation candidates.

No environment or hardware construction on import. The reset sampler is never
available to the controller: blinded_control keeps its existing tripwire.
"""
import math
import heapq
import numpy as np
from .workcells import Pose, wrap
from .navigation_map import chassis_clear


def machine_cell_pose(pose):
    # Legacy fixture goal (6,4,1.508), cell translation (5.92,4.87,0), yaw pi/2.
    xyz=pose.point((-.08,.87,0.))
    return Pose(*xyz,pose.yaw+math.pi/2)


def base_reset_pose(layout,source_id,offset):
    offset=float(offset)
    if not 0<=offset<=.3:raise ValueError('Source approach offset must be within 0..0.3 m')
    dock=layout.navigation_goal(source_id)
    return (*dock.point((offset,0,0)),dock.yaw)


class WorkcellResetSampler:
    def __init__(self,original,layout,source_id):
        self.original=original;self.layout=layout;self.source=layout.station(source_id,'source')
    @property
    def generator(self):return self.original.generator
    def seed(self,value):self.original.seed(value)
    def sample(self,task,count,randomization='baseline'):
        from .reset import quat_multiply
        import torch
        if task!='loading':raise ValueError('Workcell runtime only supports loading')
        batch=self.original.sample(task,count,randomization)
        p=self.source.pose_map
        for tensor in (batch.station_pose,batch.bar_pose):
            x=tensor[:,0]+1.15;y=tensor[:,1].clone()
            tensor[:,0]=p.x+math.cos(p.yaw)*x-math.sin(p.yaw)*y
            tensor[:,1]=p.y+math.sin(p.yaw)*x+math.cos(p.yaw)*y
            tensor[:,2]+=p.z
            rotation=tensor.new_tensor([0.,0.,math.sin(p.yaw/2),math.cos(p.yaw/2)]).expand(count,-1)
            tensor[:,3:]=quat_multiply(rotation,tensor[:,3:])
        return batch


def configure_scene(env):
    """Called only inside _setup_scene, before physics starts."""
    from pxr import UsdGeom,Gf
    import omni.usd
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation,RigidObject
    from .spec import asset_directory
    stage=omni.usd.get_context().get_stage()
    cell=UsdGeom.Xform(stage.GetPrimAtPath('/World/envs/env_0/Cell'))
    p=machine_cell_pose(env.layout.station(env.cfg.workcell_machine,'machine').pose_map)
    cell.ClearXformOpOrder();cell.AddTranslateOp().Set(Gf.Vec3d(p.x,p.y,p.z));cell.AddRotateZOp().Set(math.degrees(p.yaw))
    for s in env.layout.stations.values():
        if s.id in (env.cfg.workcell_source,env.cfg.workcell_machine):continue
        if env.layout.profiles[s.profile_id].kind=='source':
            cfg=env.cfg.station.replace(prim_path='/World/envs/env_0/OtherSource_'+s.id)
            cfg.init_state.pos=s.pose_map.values()[:3]
            cfg.init_state.rot=(0.,0.,math.sin(s.pose_map.yaw/2),math.cos(s.pose_map.yaw/2))
            env.scene.rigid_objects['source_'+s.id]=RigidObject(cfg)
        else:
            p=machine_cell_pose(s.pose_map);path='/World/envs/env_0/OtherCell_'+s.id
            node=UsdGeom.Xform.Define(stage,path);node.AddTranslateOp().Set(Gf.Vec3d(p.x,p.y,p.z));node.AddRotateZOp().Set(math.degrees(p.yaw))
            cfg=env.cfg.fixture.replace(prim_path=path+'/Fixture')
            env.scene.articulations['fixture_'+s.id]=Articulation(cfg)
            machine=sim_utils.UsdFileCfg(usd_path=str(asset_directory()/'machine.usda'))
            machine.func(path+'/Machine',machine)


def all_machine_obstacles(layout,assets):
    from pxr import Usd,UsdGeom,UsdPhysics,Gf
    from pathlib import Path
    stage=Usd.Stage.CreateInMemory()
    for s in layout.stations.values():
        if layout.profiles[s.profile_id].kind!='machine':continue
        p=machine_cell_pose(s.pose_map);path='/'+s.id
        node=UsdGeom.Xform.Define(stage,path);node.AddTranslateOp().Set(Gf.Vec3d(p.x,p.y,p.z));node.AddRotateZOp().Set(math.degrees(p.yaw))
        for name in ('machine','fixture'):
            stage.DefinePrim(path+'/'+name,'Xform').GetReferences().AddReference(str((Path(assets)/(name+'.usda')).resolve()))
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy','guide']);result=[]
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI) or prim.GetAttribute('physics:collisionEnabled').Get() is False:continue
        box=cache.ComputeWorldBound(prim).ComputeAlignedBox();lo,hi=np.asarray(box.GetMin()),np.asarray(box.GetMax())
        if lo[2]<.60 and hi[2]>.02:result.append(dict(path=str(prim.GetPath()),minimum=lo.tolist(),maximum=hi.tolist()))
    if not result:raise ValueError('No machine collision CAD found')
    # Source supports are additional obstacles in the same map.
    for s in layout.stations.values():
        if layout.profiles[s.profile_id].kind!='source':continue
        path='/'+s.id;node=UsdGeom.Xform.Define(stage,path)
        node.AddTranslateOp().Set(Gf.Vec3d(*s.pose_map.values()[:3]));node.AddRotateZOp().Set(math.degrees(s.pose_map.yaw))
        node.GetPrim().GetReferences().AddReference(str((Path(assets)/'station.usda').resolve()))
    cache.Clear()
    for prim in stage.Traverse():
        if not any(str(prim.GetPath()).startswith('/'+s.id+'/') for s in layout.stations.values() if layout.profiles[s.profile_id].kind=='source'):continue
        if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
        box=cache.ComputeWorldBound(prim).ComputeAlignedBox();lo,hi=np.asarray(box.GetMin()),np.asarray(box.GetMax())
        if lo[2]<.60 and hi[2]>.02:result.append(dict(path=str(prim.GetPath()),minimum=lo.tolist(),maximum=hi.tolist()))
    # Merge each table's contained support/leg colliders into a conservative AABB.
    merged=[]
    for station in layout.stations.values():
        rows=[o for o in result if o['path'].startswith('/'+station.id+'/')]
        if layout.profiles[station.profile_id].kind=='source' and rows:
            merged.append(dict(path='/'+station.id,minimum=np.min([o['minimum'] for o in rows],axis=0).tolist(),
                maximum=np.max([o['maximum'] for o in rows],axis=0).tolist()))
        else:merged.extend(rows)
    return merged


def plan_chassis_path(start,goal,obstacles,resolution=.25):
    """Conservative 2-D grid route; full yaw sweep at turns, checked final approach.

    Final docking retains its required orientation; free-space path nodes must
    clear a complete chassis turn. This is a static-CAD planner, not Nav2.
    """
    start=tuple(start);goal=tuple(goal)
    def segment(a,b,yaw=None):
        d=math.dist(a[:2],b[:2]);heading=math.atan2(b[1]-a[1],b[0]-a[0]) if yaw is None else yaw
        return all(chassis_clear((float(x),float(y),heading),obstacles) for x,y in np.linspace(a[:2],b[:2],max(2,math.ceil(d/.04)+1)))
    def turn(p):
        radius=math.hypot(.52,.42)
        for o in obstacles:
            lo=o['minimum'];hi=o['maximum']
            dx=max(lo[0]-p[0],0,p[0]-hi[0]);dy=max(lo[1]-p[1],0,p[1]-hi[1])
            if math.hypot(dx,dy)<radius:return False
        return True
    if not chassis_clear(start,obstacles) or not chassis_clear(goal,obstacles):raise ValueError('Source/destination dock intersects CAD')
    # Approach the dock backwards along its local -X, retaining the dock yaw.
    approaches=[(goal[0]-d*math.cos(goal[2]),goal[1]-d*math.sin(goal[2]),goal[2]) for d in (.75,1.,1.5,2.,2.5,3.)]
    approaches=[p for p in approaches if turn(p) and segment(p,goal,goal[2])]
    if not approaches:raise ValueError('No collision-free final docking corridor')
    xs=[start[0],goal[0]]+[o[k][0] for o in obstacles for k in ('minimum','maximum')]
    ys=[start[1],goal[1]]+[o[k][1] for o in obstacles for k in ('minimum','maximum')]
    lo=(min(xs)-2,min(ys)-2);hi=(max(xs)+2,max(ys)+2)
    def xy(n):return (lo[0]+n[0]*resolution,lo[1]+n[1]*resolution)
    def node(p):return (round((p[0]-lo[0])/resolution),round((p[1]-lo[1])/resolution))
    origin=node(start);cache={}
    def free(n):
        if n not in cache:
            p=xy(n);cache[n]=lo[0]<=p[0]<=hi[0] and lo[1]<=p[1]<=hi[1] and turn(p)
        return cache[n]
    # The source dock must also have clear yaw sweep before a grid turn.
    if not turn(start):raise ValueError('Source dock lacks turn clearance')
    near=[(origin[0]+dx,origin[1]+dy) for dx in range(-3,4) for dy in range(-3,4)]
    near=sorted(near,key=lambda n:math.dist(xy(n),start[:2]))
    connected=[n for n in near if free(n) and segment(start,xy(n))]
    if not connected:raise ValueError('Source dock lacks a collision-free grid connection')
    origin=connected[0]
    candidates=[]
    for approach in approaches:
        end=node(approach)
        if free(end) and segment(xy(end),approach):candidates.append((end,approach))
    if not candidates:raise ValueError('No dock grid connection')
    targets={n:p for n,p in candidates};cost={origin:0};parents={};queue=[(0,origin)];found=None
    while queue:
        _,n=heapq.heappop(queue)
        if n in targets:found=n;break
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            m=(n[0]+dx,n[1]+dy)
            if not free(m) or not segment(xy(n),xy(m)):continue
            new=cost[n]+1
            if new<cost.get(m,math.inf):
                cost[m]=new;parents[m]=n;h=min(abs(m[0]-t[0])+abs(m[1]-t[1]) for t in targets)
                heapq.heappush(queue,(new+h,m))
    if found is None:raise ValueError('No static CAD route connects the stations')
    nodes=[found]
    while nodes[-1]!=origin:nodes.append(parents[nodes[-1]])
    points=[start[:2]]+[xy(n) for n in reversed(nodes)]+[targets[found][:2]]
    # Shortcut only through segments checked against the same CAD. Every
    # retained turn remains inside its full-rotation collision-free envelope.
    compact=[points[0]];index=0
    while index<len(points)-1:
        next_index=next((j for j in range(len(points)-1,index,-1)
                         if turn(points[j]) and segment(points[index],points[j])),None)
        if next_index is None:raise ValueError('No verified shortcut to next route point')
        compact.append(points[next_index]);index=next_index
    goals=[]
    for i,point in enumerate(compact[1:],1):
        if math.dist(compact[i-1],point)<1e-8:continue
        # Turn once into the outgoing corridor at a checked free-space point.
        # The former incoming-yaw + same-position outgoing-yaw pair forced
        # needless small-segment heading changes and boundary reacquisition.
        following=compact[i+1] if i+1<len(compact) else goal[:2]
        yaw=math.atan2(following[1]-point[1],following[0]-point[0])
        if i==len(compact)-1:yaw=goal[2]
        goals.append((point[0],point[1],yaw))
    goals.append(goal)
    return goals


def simulation_navigation(layout,source_id,machine_id,assets):
    obstacles=all_machine_obstacles(layout,assets)
    a=layout.navigation_goal(source_id);b=layout.navigation_goal(machine_id)
    return plan_chassis_path((a.x,a.y,a.yaw),(b.x,b.y,b.yaw),obstacles),obstacles


def ideal_scene_loading_geometry(layout,machine_id,base_frame,*,enabled):
    """Exact configured scene survey, transformed by full measured base SE(3).

    This is for the explicit ideal-localization physics probe. It provides no
    camera-based machine pose or identity evidence for MissionFlow acceptance.
    """
    if enabled is not True:raise ValueError('Physical probe requires an explicit exact-scene survey')
    from .vision.capture import optical_transform
    from .vision.observation import matrix_quaternion
    station=layout.station(machine_id,'machine')
    world=layout.machine_geometry(machine_id,refined_pose=station.pose_map)
    T=np.asarray(base_frame.base_from_world)
    if T.shape!=(4,4) or not np.isfinite(T).all():raise ValueError('Invalid measured base rigid transform')
    R=T[:3,:3]
    if not np.allclose(R.T@R,np.eye(3),atol=1e-5) or np.linalg.det(R)<.99999:
        raise ValueError('Invalid measured base rigid transform')
    geometry=dict(world,frame='base',pose_source='exact registered scene survey; no visual refinement')
    geometry['goal']=tuple(base_frame.point(world['goal']))
    geometry['standoff']=tuple(base_frame.point(world['standoff']))
    for key in ('retreat_vector','bar_axis'):geometry[key]=tuple(R@world[key])
    orientation=optical_transform([0.,0.,0.],world['tcp_quaternion_xyzw'])
    geometry['tcp_quaternion_xyzw']=tuple(matrix_quaternion(R@np.asarray(orientation)[:3,:3]))
    return geometry
