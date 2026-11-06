"""Machine-fixed ChArUco mount geometry and image-only pose estimation.

Board centre axes: X across the printed board, Y up, Z out of its front face.
The OpenCV board object's top-left/down coordinates are explicitly converted.
No simulator, policy, robot or network is imported by this module.
"""
import cv2
import numpy as np

from .fiducial_localization import MarkerAnchor, WristMarkerEstimator, rigid
from .workcells import identifier

DICTIONARY='DICT_5X5_100'
SQUARE_M=.05
MARKER_M=.035
COLUMNS=7
ROWS=5
WIDTH=COLUMNS*SQUARE_M
HEIGHT=ROWS*SQUARE_M
BOARD_POSITION_STATION=(.55,.14,1.78)
CAMERA_POSITION_STATION=(.55,.70,1.62)
CAMERA_AIM_BELOW_CENTRE_M=.070


def make_board(machine_id):
    identifier(machine_id)
    if machine_id not in ('machine_1','machine_2','machine_3'):
        raise ValueError('Explicit registered machine board required')
    offset=20*(int(machine_id.rsplit('_',1)[1])-1)
    dictionary=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,DICTIONARY))
    count=(COLUMNS*ROWS)//2
    return cv2.aruco.CharucoBoard((COLUMNS,ROWS),SQUARE_M,MARKER_M,dictionary,
                                 np.arange(offset,offset+count,dtype=np.int32))


def board_from_cv():
    t=np.diag([1.,-1.,-1.,1.]);t[:3,3]=[-WIDTH/2,HEIGHT/2,0.]
    return t


def station_from_board():
    t=np.eye(4);t[:3,3]=BOARD_POSITION_STATION
    # Board front points toward the approach side (+station Y).
    t[:3,:3]=[[-1.,0.,0.],[0.,0.,1.],[0.,1.,0.]]
    return t


def fixture_from_board(layout,machine_id):
    station=layout.station(machine_id,'machine');profile=layout.profiles[station.profile_id]
    t=station_from_board();t[:3,3]-=np.array(profile.fixture_goal)
    return t


class CharucoFixtureEstimator(WristMarkerEstimator):
    """All 24 chess corners and all registered marker IDs, same original gates."""
    def __init__(self,calibration,layout,machine_id,limits=None):
        self.board=make_board(machine_id)
        self.ids=self.board.getIds().reshape(-1).tolist()
        self.cv_to_board=board_from_cv()
        raw=np.asarray(self.board.getChessboardCorners(),np.float64)
        self.points=raw@self.cv_to_board[:3,:3].T+self.cv_to_board[:3,3]
        # Here the pose reference is the board centre, explicitly not marker 0.
        anchor=MarkerAnchor(machine_id,DICTIONARY,self.ids[0],MARKER_M,
                            fixture_from_board(layout,machine_id),'machine_fixed_charuco_v1')
        super().__init__(calibration,anchor,limits)
        params=cv2.aruco.DetectorParameters();params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        self.charuco=cv2.aruco.CharucoDetector(self.board,cv2.aruco.CharucoParameters(),params)

    @property
    def marker_ids(self):return list(self.ids)

    def detect_corners(self,image):
        corners,ids,_,marker_ids=self.charuco.detectBoard(image)
        got=[] if marker_ids is None else marker_ids.reshape(-1).tolist()
        if any(got.count(marker_id)!=1 for marker_id in self.ids):
            raise ValueError('Requested machine board IDs incomplete or ambiguous')
        if ids is None or len(ids)!=len(self.points) or set(ids.reshape(-1))!=set(range(len(self.points))):
            raise ValueError('Full ChArUco chess-corner observation required')
        order=np.argsort(ids.reshape(-1))
        return np.asarray(corners,np.float64).reshape(-1,2)[order]

    def solve_corners(self,pixels):
        pixels=np.asarray(pixels,np.float64)
        if pixels.shape!=(len(self.points),2) or not np.isfinite(pixels).all():
            raise ValueError('Full ordered board corners required')
        return self._solve_pose(self.points,pixels,cv2.SOLVEPNP_IPPE)

    def estimate(self,image,**kwargs):
        result=super().estimate(image,**kwargs)
        result.update(pose_reference='charuco_board_centre_x_right_y_up_z_front',
                      charuco_corner_ids=list(range(len(self.points))),
                      fixture_from_board=rigid(self.anchor.fixture_from_marker).tolist())
        return result


def attach_board(stage,parent_path,layout,machine_id):
    """Scene construction only: fixed child of actual static Machine CAD prim."""
    from pxr import Gf,Sdf,UsdGeom,UsdShade,UsdPhysics
    parent=stage.GetPrimAtPath(parent_path)
    if not parent:raise ValueError('Actual testing-machine CAD parent missing')
    station=layout.station(machine_id,'machine');profile=layout.profiles[station.profile_id]
    # Construct desired world pose from the immutable registry, not camera pose.
    c,s=np.cos(station.pose_map.yaw),np.sin(station.pose_map.yaw)
    world_station=np.eye(4);world_station[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]]
    world_station[:3,3]=station.pose_map.values()[:3]
    world_board=world_station@station_from_board()
    cache=UsdGeom.XformCache();world_parent=np.asarray(cache.GetLocalToWorldTransform(parent)).T
    parent_board=np.linalg.inv(world_parent)@world_board
    root=parent_path+'/FixedFiducialBoard'
    node=UsdGeom.Xform.Define(stage,root);node.AddTransformOp().Set(Gf.Matrix4d(parent_board.T.tolist()))
    def material(name,color):
        mat=UsdShade.Material.Define(stage,root+'/'+name);shader=UsdShade.Shader.Define(stage,root+'/'+name+'/Shader')
        shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(1.)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');return mat
    white=material('White',(1.,1.,1.));black=material('Black',(0.,0.,0.))
    backing=UsdGeom.Cube.Define(stage,root+'/RigidBacking');backing.CreateSizeAttr(1.)
    backing.AddTranslateOp().Set(Gf.Vec3d(0,0,-.002));backing.AddScaleOp().Set(Gf.Vec3f(WIDTH+.03,HEIGHT+.03,.003))
    UsdShade.MaterialBindingAPI.Apply(backing.GetPrim()).Bind(white);UsdPhysics.CollisionAPI.Apply(backing.GetPrim())
    # Rigid cantilever to the actual upper frame, rather than a visual board
    # suspended in space. The surveyed attachment is inside TopBeam CAD.
    steel=material('MountSteel',(.28,.31,.34))
    attachment=np.array([.53,.60,2.22])
    mounting_path=[np.array([.53,.105,1.90]),np.array([.53,.105,2.22]),attachment]
    board_station=np.linalg.inv(station_from_board())
    for i,(start,end) in enumerate(zip(mounting_path,mounting_path[1:])):
        a=board_station[:3,:3]@start+board_station[:3,3]
        b=board_station[:3,:3]@end+board_station[:3,3]
        member=UsdGeom.Cube.Define(stage,root+f'/FrameBracket_{i}');member.CreateSizeAttr(1.)
        member.AddTranslateOp().Set(Gf.Vec3d(*((a+b)/2)))
        member.AddScaleOp().Set(Gf.Vec3f(*(np.abs(b-a)+.025)))
        UsdShade.MaterialBindingAPI.Apply(member.GetPrim()).Bind(steel)
        UsdPhysics.CollisionAPI.Apply(member.GetPrim())
    clamp=UsdGeom.Cube.Define(stage,root+'/BoardBackClamp');clamp.CreateSizeAttr(1.)
    center=board_station[:3,:3]@np.array([.53,.1225,1.90])+board_station[:3,3]
    clamp.AddTranslateOp().Set(Gf.Vec3d(*center));clamp.AddScaleOp().Set(Gf.Vec3f(.05,.05,.032))
    UsdShade.MaterialBindingAPI.Apply(clamp.GetPrim()).Bind(steel);UsdPhysics.CollisionAPI.Apply(clamp.GetPrim())
    node.GetPrim().CreateRelationship('mountedTo').SetTargets([parent_path+'/TopBeam'])
    def quad(name,points,mat):
        mesh=UsdGeom.Mesh.Define(stage,root+'/'+name);mesh.CreatePointsAttr([Gf.Vec3f(*map(float,v)) for v in points])
        mesh.CreateFaceVertexCountsAttr([4]);mesh.CreateFaceVertexIndicesAttr([0,1,2,3]);mesh.CreateDoubleSidedAttr(True)
        mesh.CreateSubdivisionSchemeAttr('none');UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    def rectangle(name,x0,y0,x1,y1,mat):quad(name,[(x0,y0,0),(x1,y0,0),(x1,y1,0),(x0,y1,0)],mat)
    cvt=board_from_cv();board=make_board(machine_id)
    pattern=board.generateImage((1400,1000))
    for row in range(ROWS):
        for col in range(COLUMNS):
            x0=col*SQUARE_M-WIDTH/2;y1=HEIGHT/2-row*SQUARE_M
            color=black if pattern[row*200+3,col*200+3]<128 else white
            rectangle(f'Square_{row}_{col}',x0,y1-SQUARE_M,x0+SQUARE_M,y1,color)
    for marker_id,raw in zip(board.getIds().reshape(-1),board.getObjPoints()):
        points=np.asarray(raw,float)@cvt[:3,:3].T+cvt[:3,3]
        image=cv2.aruco.generateImageMarker(board.getDictionary(),int(marker_id),560)
        bits=image.reshape(7,80,7,80).mean(axis=(1,3))
        x0,y1=points[0,:2]
        for row in range(7):
            for col in range(7):
                x=x0+col*MARKER_M/7;y=y1-row*MARKER_M/7
                # Tiny positive offset avoids coplanar square/marker z-fighting.
                quad(f'Marker_{marker_id}_{row}_{col}',[(x,y,.00002),(x+MARKER_M/7,y,.00002),
                     (x+MARKER_M/7,y-MARKER_M/7,.00002),(x,y-MARKER_M/7,.00002)],white if bits[row,col]>127 else black)
    return dict(machine_id=machine_id,parent_prim=parent_path,board_prim=root,
                parent_from_board=parent_board.tolist(),fixture_from_board=fixture_from_board(layout,machine_id).tolist(),
                station_from_board=station_from_board().tolist(),marker_ids=board.getIds().reshape(-1).tolist(),
                squares=[COLUMNS,ROWS],square_m=SQUARE_M,marker_m=MARKER_M,
                mount_source='fixed static testing-machine CAD child; camera-independent scene registry',
                collision_backing=True,rigid_bracket=True,attached_frame_prim=parent_path+'/TopBeam',
                frame_attachment_station_m=attachment.tolist(),geometry_status='simulation_known_survey')
