"""Offline wrist-camera fiducial geometry. No network, robot commands or policy imports.

T_a_b maps column points in b to a. OpenCV optical axes are +X right, +Y down,
+Z forward; marker axes/corner order follow IPPE_SQUARE. All lengths are metres.
"""
from dataclasses import dataclass
import hashlib
import math

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from .workcells import identifier


def rigid(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-6, rtol=0)
            or abs(np.linalg.det(t[:3, :3])-1) > 1e-6):
        raise ValueError('Expected a finite proper rigid transform')
    return t.copy()


def inverse(value):
    t = rigid(value); out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ t[:3, 3]
    return out


def rotation_distance(a, b):
    return float(Rotation.from_matrix(rigid(a)[:3, :3].T @ rigid(b)[:3, :3]).magnitude())


def square_points(length_m):
    if isinstance(length_m, bool) or not math.isfinite(length_m) or not 0 < length_m < 2:
        raise ValueError('Measured outer black-square side length must be in metres')
    s = length_m / 2
    return np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)


@dataclass(frozen=True)
class Limits:
    """Provisional research limits, not validated engineering accuracy claims."""
    max_age_s: float = .2
    max_fk_skew_s: float = .005
    max_session_s: float = 10.
    max_reprojection_px: float = 1.
    min_edge_px: float = 60.
    min_distance_m: float = .15
    max_distance_m: float = 2.
    ambiguity_error_gap_px: float = .2
    ambiguity_translation_m: float = .005
    ambiguity_rotation_rad: float = math.radians(2)
    stable_translation_m: float = .003
    stable_rotation_rad: float = math.radians(.5)
    max_correction_m: float = .10
    max_correction_rad: float = math.radians(10)
    dock_position_m: float = .008
    dock_yaw_rad: float = math.radians(.5)
    max_base_linear_m_s: float = .005
    max_base_angular_rad_s: float = .005
    min_stable_frames: int = 3

    def __post_init__(self):
        values = [v for k, v in vars(self).items() if k != 'min_stable_frames']
        if any(isinstance(v, bool) or not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError('Positive finite gate limits required')
        if (type(self.min_stable_frames) is not int or self.min_stable_frames < 3
                or self.max_distance_m <= self.min_distance_m):
            raise ValueError('Invalid frame/distance limits')


@dataclass(frozen=True)
class MarkerAnchor:
    machine_id: str
    dictionary: str
    marker_id: int
    length_m: float
    fixture_from_marker: tuple
    survey_id: str

    def __post_init__(self):
        identifier(self.machine_id); identifier(self.survey_id)
        if not self.dictionary.startswith('DICT_') or not hasattr(cv2.aruco, self.dictionary):
            raise ValueError('Unsupported ArUco dictionary')
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary))
        if type(self.marker_id) is not int or not 0 <= self.marker_id < len(dictionary.bytesList):
            raise ValueError('Invalid marker ID')
        square_points(self.length_m)
        object.__setattr__(self, 'fixture_from_marker', tuple(map(tuple, rigid(self.fixture_from_marker))))


@dataclass(frozen=True)
class CameraCalibration:
    camera_id: str
    calibration_id: str
    hand_eye_id: str
    image_size: tuple
    K: tuple
    distortion: tuple
    flange_from_optical: tuple
    base_frame_id: str
    flange_frame_id: str
    optical_frame_id: str
    clock_id: str

    def __post_init__(self):
        for v in (self.camera_id, self.calibration_id, self.hand_eye_id, self.base_frame_id,
                  self.flange_frame_id, self.optical_frame_id, self.clock_id):
            identifier(v)
        if len(self.image_size) != 2 or any(type(v) is not int or v <= 0 for v in self.image_size):
            raise ValueError('Exact calibrated image size required')
        k = np.asarray(self.K, float); d = np.asarray(self.distortion, float)
        if (k.shape != (3, 3) or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0
                or not np.allclose(k[2], [0, 0, 1]) or abs(k[0, 1])+abs(k[1, 0]) > 1e-9
                or d.ndim != 1 or len(d) not in (4, 5, 8, 12, 14) or not np.isfinite(d).all()):
            raise ValueError('Invalid pinhole K/distortion; fisheye needs a separate model')
        object.__setattr__(self, 'K', tuple(map(tuple, k)))
        object.__setattr__(self, 'distortion', tuple(d))
        object.__setattr__(self, 'image_size', tuple(self.image_size))
        object.__setattr__(self, 'flange_from_optical', tuple(map(tuple, rigid(self.flange_from_optical))))


class WristMarkerEstimator:
    def __init__(self, calibration, anchor, limits=None):
        self.calibration = calibration; self.anchor = anchor; self.limits = limits or Limits()
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, anchor.dictionary))
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)

    def estimate(self, image, *, machine_id, frame_id, image_stamp_s, fk_stamp_s, now_s,
                 base_from_flange, camera_id, calibration_id, hand_eye_id,
                 fk_base_frame_id, fk_flange_frame_id, optical_frame_id, clock_id):
        c = self.calibration; a = self.anchor; lim = self.limits
        identifier(frame_id)
        if machine_id != a.machine_id:
            raise ValueError('Marker anchor is not bound to requested machine')
        if (camera_id, calibration_id, hand_eye_id) != (c.camera_id, c.calibration_id, c.hand_eye_id):
            raise ValueError('Camera/intrinsics/hand-eye identity mismatch')
        if (fk_base_frame_id, fk_flange_frame_id, optical_frame_id, clock_id) != (
                c.base_frame_id, c.flange_frame_id, c.optical_frame_id, c.clock_id):
            raise ValueError('FK/optical coordinate frames or clock domain mismatch')
        stamps = (image_stamp_s, fk_stamp_s, now_s)
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in stamps):
            raise ValueError('Finite exposure/FK/current timestamps required')
        if not 0 <= now_s-image_stamp_s <= lim.max_age_s or not 0 <= now_s-fk_stamp_s <= lim.max_age_s:
            raise ValueError('Stale or future image/FK')
        if abs(image_stamp_s-fk_stamp_s) > lim.max_fk_skew_s:
            raise ValueError('Wrist FK not synchronized to exposure time')
        base_from_optical = rigid(base_from_flange) @ rigid(c.flange_from_optical)
        if (not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 2
                or (image.shape[1], image.shape[0]) != c.image_size):
            raise ValueError('Native calibrated Mono8 image required; no implicit resize/crop')
        pixels = self.detect_corners(image)
        if (min(np.min(np.linalg.norm(block-np.roll(block,-1,axis=0),axis=1))
                for block in pixels.reshape(-1,4,2)) < lim.min_edge_px
                or np.min(pixels) < 2 or np.max(pixels[:, 0]) >= image.shape[1]-2
                or np.max(pixels[:, 1]) >= image.shape[0]-2):
            raise ValueError('Marker too small or clipped at image boundary')
        pose, candidates = self.solve_corners(pixels)
        base_from_fixture = rigid(base_from_optical @ pose @ inverse(a.fixture_from_marker))
        return dict(machine_id=machine_id, marker_id=a.marker_id, dictionary=a.dictionary,
                    survey_id=a.survey_id, frame_id=frame_id, image_stamp_s=image_stamp_s,
                    fk_stamp_s=fk_stamp_s, camera_id=c.camera_id, calibration_id=c.calibration_id,
                    hand_eye_id=c.hand_eye_id, image_sha256=hashlib.sha256(image.tobytes()).hexdigest(),
                    base_frame_id=c.base_frame_id, flange_frame_id=c.flange_frame_id,
                    optical_frame_id=c.optical_frame_id, clock_id=c.clock_id,
                    base_from_fixture=base_from_fixture.tolist(),
                    fixture_from_base=inverse(base_from_fixture).tolist(),
                    marker_ids=self.marker_ids, corners_px=pixels.tolist(), candidates=candidates,
                    global_map_pose_corrected=False, physical_acceptance=False, actuator_dispatch=False)

    @property
    def marker_ids(self):
        return [self.anchor.marker_id]

    def detect_corners(self,image):
        corners,ids,_=self.detector.detectMarkers(image)
        ids=[] if ids is None else ids.reshape(-1).tolist()
        pixels=[]
        for marker_id in self.marker_ids:
            matches=[i for i,v in enumerate(ids) if v==marker_id]
            if len(matches)!=1:
                raise ValueError('Requested marker absent or duplicate ID visible')
            pixels.extend(np.asarray(corners[matches[0]],np.float64).reshape(4,2))
        return np.asarray(pixels,np.float64)

    def solve_corners(self, pixels):
        pixels = np.asarray(pixels, np.float64)
        if pixels.shape != (4, 2) or not np.isfinite(pixels).all():
            raise ValueError('Four finite ordered corners required')
        return self._solve_pose(square_points(self.anchor.length_m),pixels,cv2.SOLVEPNP_IPPE_SQUARE)

    def _solve_pose(self,points,pixels,method):
        """Return all positive-depth, front-facing hypotheses; refuse material ambiguity."""
        c = self.calibration; lim = self.limits
        ok, rotations, translations, _ = cv2.solvePnPGeneric(
            points, pixels, np.array(c.K), np.array(c.distortion), flags=method)
        choices = []
        for rv, tv in zip(rotations, translations):
            rotation = cv2.Rodrigues(rv)[0]; translation = tv.reshape(3)
            p = points @ rotation.T + translation
            distance = np.linalg.norm(translation)
            if (not np.isfinite(p).all() or np.any(p[:, 2] <= 0)
                    or not lim.min_distance_m <= distance <= lim.max_distance_m
                    or float(rotation[:, 2] @ translation) >= 0):
                continue
            projected = cv2.projectPoints(points, rv, tv, np.array(c.K), np.array(c.distortion))[0].reshape(-1, 2)
            error = float(np.sqrt(np.mean(np.sum((projected-pixels)**2, axis=1))))
            t = np.eye(4); t[:3, :3] = rotation; t[:3, 3] = translation
            choices.append((error, rigid(t)))
        choices.sort(key=lambda v: v[0])
        if not ok or not choices or choices[0][0] > lim.max_reprojection_px:
            raise ValueError('No acceptable front-facing positive-depth PnP hypothesis')
        for error, t in choices[1:]:
            if error-choices[0][0] < lim.ambiguity_error_gap_px and (
                    np.linalg.norm(t[:3, 3]-choices[0][1][:3, 3]) > lim.ambiguity_translation_m
                    or rotation_distance(t, choices[0][1]) > lim.ambiguity_rotation_rad):
                raise ValueError('Ambiguous planar pose; require another view or multi-marker board')
        return choices[0][1], [dict(rms_px=e, optical_from_marker=t.tolist()) for e, t in choices]


class WristMarkerBoardEstimator(WristMarkerEstimator):
    """Coplanar rigid multi-marker board, surveyed entirely in fixture coordinates.

    All configured IDs must be visible. No single-marker fallback or weakening
    of the original per-frame/multi-frame gates is performed.
    """
    def __init__(self,calibration,anchors,limits=None):
        anchors=tuple(anchors)
        if len(anchors)<2:
            raise ValueError('At least two surveyed markers required')
        first=anchors[0]
        if len({a.marker_id for a in anchors})!=len(anchors) or any(
                (a.machine_id,a.dictionary,a.survey_id)!=(first.machine_id,first.dictionary,first.survey_id)
                for a in anchors):
            raise ValueError('Board IDs must be unique and share machine/dictionary/survey')
        super().__init__(calibration,first,limits)
        self.anchors=anchors
        points=[]
        for a in anchors:
            relative=inverse(first.fixture_from_marker)@rigid(a.fixture_from_marker)
            p=square_points(a.length_m)@relative[:3,:3].T+relative[:3,3]
            if np.max(np.abs(p[:,2]))>1e-6 or np.linalg.norm(relative[:3,2]-[0,0,1])>1e-6:
                raise ValueError('Board markers must lie in one front-facing plane')
            points.extend(p)
        self.points=np.asarray(points,np.float64)

    @property
    def marker_ids(self):
        return [a.marker_id for a in self.anchors]

    def solve_corners(self,pixels):
        pixels=np.asarray(pixels,np.float64)
        if pixels.shape!=(len(self.points),2) or not np.isfinite(pixels).all():
            raise ValueError('All board corners in configured marker order required')
        return self._solve_pose(self.points,pixels,cv2.SOLVEPNP_IPPE)


class PlanarWristBoardEstimator(WristMarkerBoardEstimator):
    """Explicit level-floor/known-height geometry variant, not unconstrained SE(3).

    Height is a fixed survey/calibration input, never the current simulated root
    position. This variant requires independently validated level base/fixture
    axes. Real deployment needs a tilt gate or an IMU-aware extension.
    """
    def __init__(self,calibration,anchors,*,fixture_from_base_z_m,limits=None):
        super().__init__(calibration,anchors,limits)
        if isinstance(fixture_from_base_z_m,bool) or not math.isfinite(fixture_from_base_z_m):
            raise ValueError('Fixed surveyed base height in fixture coordinates required')
        self.height=float(fixture_from_base_z_m)

    def estimate(self,image,**kwargs):
        from scipy.optimize import least_squares
        obs=super().estimate(image,**kwargs)
        initial=rigid(obs['fixture_from_base'])
        p0=np.array([initial[0,3],initial[1,3],math.atan2(initial[1,0],initial[0,0])])
        base_camera=rigid(kwargs['base_from_flange'])@rigid(self.calibration.flange_from_optical)
        fixture_points=self.points@rigid(self.anchor.fixture_from_marker)[:3,:3].T+rigid(self.anchor.fixture_from_marker)[:3,3]
        pixels=np.asarray(obs['corners_px']);k=np.asarray(self.calibration.K);d=np.asarray(self.calibration.distortion)
        def base_pose(values):
            t=np.eye(4);t[:3,:3]=Rotation.from_euler('z',values[2]).as_matrix()
            t[:3,3]=[values[0],values[1],self.height];return t
        def residual(values):
            optical_fixture=inverse(base_camera)@inverse(base_pose(values))
            predicted=cv2.projectPoints(fixture_points,cv2.Rodrigues(optical_fixture[:3,:3])[0],
                optical_fixture[:3,3],k,d)[0].reshape(-1,2)
            return (predicted-pixels).reshape(-1)
        result=least_squares(residual,p0,method='trf',max_nfev=50,
            bounds=(p0-[.1,.1,math.radians(10)],p0+[.1,.1,math.radians(10)]),
            ftol=1e-10,xtol=1e-10,gtol=1e-10)
        rms=float(np.sqrt(np.mean(np.sum(residual(result.x).reshape(-1,2)**2,axis=1))))
        singular=np.linalg.svd(result.jac,compute_uv=False)
        pose=base_pose(result.x);camera_fixture=inverse(base_camera)@inverse(pose)
        depths=(fixture_points@camera_fixture[:3,:3].T+camera_fixture[:3,3])[:,2]
        if (not result.success or not math.isfinite(rms) or rms>self.limits.max_reprojection_px
                or np.any(depths<=0) or singular[-1]<1e-6 or singular[0]/singular[-1]>1e5):
            raise ValueError(f'Planar constrained image fit rejected: converged={result.success}, '
                             f'rms_px={rms:.6f}, jacobian_condition={singular[0]/singular[-1]:.3f}')
        obs.update(unconstrained_base_from_fixture=obs['base_from_fixture'],
                   unconstrained_fixture_from_base=obs['fixture_from_base'],
                   base_from_fixture=inverse(pose).tolist(),fixture_from_base=pose.tolist(),
                   planar_constraints=dict(base_roll_pitch_assumed_rad=[0.,0.],
                       surveyed_fixture_from_base_height_m=self.height,rms_px=rms,
                       validated_level_floor_required=True,solver='three-variable image reprojection least squares',
                       height_or_tilt_uncertainty_validated=False))
        return obs


class RelocalizationSession:
    """Stationary multi-frame offline gate. Recreate after *any* base correction.

    A ready result is a geometry candidate only. It neither sends wheel commands
    nor authorizes loading, changes navigation localization or bypasses load guards.
    """
    def __init__(self, estimator, *, session_id, issued_s, fixture_from_base_goal):
        identifier(session_id)
        if not math.isfinite(issued_s):
            raise ValueError('Finite session start required')
        self.estimator = estimator; self.session_id = session_id; self.issued_s = issued_s
        self.goal = rigid(fixture_from_base_goal); self.frames = []; self.seen = set()
        self.invalidated = False

    def observe(self, image, *, measured_base_twist, base_stopped_confirmed, **kwargs):
        lim = self.estimator.limits
        try:
            if self.invalidated:
                raise ValueError('Session invalidated; start a fresh observation session')
            if not self.issued_s <= kwargs['now_s'] <= self.issued_s+lim.max_session_s:
                raise ValueError('Observation session deadline expired or not yet started')
            twist = np.asarray(measured_base_twist, float)
            if (twist.shape != (2,) or not np.isfinite(twist).all() or base_stopped_confirmed is not True
                    or abs(twist[0]) >= lim.max_base_linear_m_s or abs(twist[1]) >= lim.max_base_angular_rad_s):
                raise ValueError('Base must be measured stopped; wrist may move only with synchronized FK')
            if kwargs['image_stamp_s'] < self.issued_s:
                raise ValueError('Pre-session observation')
            if (kwargs['frame_id'] in self.seen or self.frames and
                    kwargs['image_stamp_s'] <= self.frames[-1]['image_stamp_s']):
                raise ValueError('Repeated or out-of-order exposure')
            obs = self.estimator.estimate(image, **kwargs)
            current = rigid(obs['fixture_from_base'])
            # All retained frames must agree; pairwise check avoids slow drift.
            for prior in self.frames:
                old = rigid(prior['fixture_from_base'])
                if (np.linalg.norm(current[:3, 3]-old[:3, 3]) > lim.stable_translation_m
                        or rotation_distance(current, old) > lim.stable_rotation_rad):
                    raise ValueError('Multi-frame fixture-relative pose is unstable')
            self.seen.add(obs['frame_id']); self.frames.append(obs)
            self.frames = self.frames[-lim.min_stable_frames:]
            # Common sensor timestamps use integer nanoseconds; avoid rejecting
            # an exactly 200ms window because binary subtraction rounds upward.
            if round(kwargs['now_s']*1e9)-round(self.frames[0]['image_stamp_s']*1e9) > round(lim.max_age_s*1e9):
                raise ValueError('Stable evidence window expired')
            delta = inverse(current) @ self.goal
            yaw = math.atan2(delta[1, 0], delta[0, 0])
            if (np.linalg.norm(delta[:2, 3]) > lim.max_correction_m or abs(yaw) > lim.max_correction_rad
                    or abs(delta[2, 3]) > .02 or rotation_distance(delta, _planar(delta)) > math.radians(2)):
                raise ValueError('Correction outside local planar envelope; reacquire/replan required')
            state = 'collecting'
            if len(self.frames) >= lim.min_stable_frames:
                state = ('docked_geometry_candidate' if np.linalg.norm(delta[:2, 3]) <= lim.dock_position_m
                         and abs(yaw) <= lim.dock_yaw_rad else 'precision_docking_required')
            return dict(state=state, session_id=self.session_id, observation=obs,
                        delta_base_xy_yaw=[float(delta[0, 3]), float(delta[1, 3]), yaw],
                        evidence_frame_ids=[f['frame_id'] for f in self.frames],
                        actuator_dispatch=False, loading_authorized=False,
                        requires_fresh_session_after_base_motion=True)
        except Exception:
            # An invalid observation invalidates readiness rather than retaining a previous pass.
            self.frames.clear()
            self.invalidated = True
            raise


def _planar(t):
    yaw = math.atan2(t[1, 0], t[0, 0]); out = np.eye(4)
    out[:3, :3] = Rotation.from_euler('z', yaw).as_matrix(); out[:3, 3] = t[:3, 3]
    return out


def fixture_loading_geometry(layout, machine_id, base_from_fixture):
    """Full SE(3) candidate targets for existing local route geometry, never a receipt."""
    s = layout.station(machine_id, 'machine'); p = layout.profiles[s.profile_id]
    t = rigid(base_from_fixture); r = t[:3, :3]
    return dict(station_id=s.id, profile_id=p.id, frame='base', refined=True,
                goal=t[:3, 3].tolist(), standoff=(t[:3, 3]+r @ p.standoff_offset).tolist(),
                retreat_vector=(r @ p.retreat_offset).tolist(), bar_axis=(r @ p.bar_axis).tolist(),
                tcp_quaternion_xyzw=Rotation.from_matrix(r @ Rotation.from_euler('x', math.pi/2).as_matrix()).as_quat().tolist(),
                evidence_scope='offline_fiducial_geometry_candidate', policy_compatibility_accepted=False,
                actuator_dispatch=False)
