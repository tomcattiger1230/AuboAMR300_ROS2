"""RGB-only pose observer with measured robot kinematics; no simulator access."""
from dataclasses import dataclass
import numpy as np

from .cylinder import resting_pose, constrained_pose


def rotate(q, v):
    t = 2*np.cross(np.asarray(q[:3]), v)
    return v+q[3]*t+np.cross(np.asarray(q[:3]), t)


def inverse(q):
    return np.r_[-np.asarray(q[:3]), q[3]]


@dataclass(frozen=True)
class CameraPacket:
    mono8: np.ndarray
    K: np.ndarray
    T_scene_optical: np.ndarray
    timestamp_ns: int
    frame: int


@dataclass(frozen=True)
class RobotPacket:
    tcp: np.ndarray
    quaternion_xyzw: np.ndarray
    fingers: np.ndarray
    finger_forces: np.ndarray
    jaw_forces: np.ndarray
    timestamp_ns: int


class ImageObserver:
    """The only detector call receives mono8. Ground-truth providers cannot enter.

    Both bar ends initialize position on the surveyed support plane. During a
    confirmed physical grasp, FK propagates that vision-initialized transform;
    every accepted camera frame corrects the lateral pose. This is an estimate,
    not a constraint or pose write to the real/simulated steel.
    """
    def __init__(self, detector, max_age_ns=200_000_000, plane_height=.522, require_full_support=False,
                 machine_load_min=40.,prefer_full_support=False,track_support_texture=False,
                 fixed_support_prior=False):
        self.detector = detector
        self.max_age_ns, self.plane_height = max_age_ns, plane_height
        self.require_full_support=require_full_support
        self.prefer_full_support=prefer_full_support
        self.track_support_texture=track_support_texture
        self.fixed_support_prior=fixed_support_prior
        self.machine_load_min=machine_load_min
        self.reset()

    def reset(self):
        self.acquired = self.anchored = self.released = False
        self.position = np.full(3, np.nan); self.axis = np.full(3, np.nan)
        self.linear_velocity = np.zeros(3); self.angular_velocity = np.zeros(3)
        self.local_center = self.local_axis = None
        self.local_correction = np.zeros(3); self.axis_correction = None
        self.last_good_ns = self.last_tick_ns = self.last_frame = -1
        self.initial_candidates = []
        self.survey_prior=None
        self.bilateral_dwell = 0
        self.last_error = 'awaiting full-bar observation'
        self.quality = {}
        self.last_probability=None
        self.measurements = self.rejected_measurements = 0
        self.texture_measurements=0
        if self.track_support_texture:
            from .support_texture import SupportTexture
            self.texture=SupportTexture()

    def step(self, robot, camera=None, *, survey_allowed=False, grasp_allowed=False, release_requested=False):
        if robot.timestamp_ns <= self.last_tick_ns:
            raise ValueError('Robot timestamps must increase')
        dt = (robot.timestamp_ns-self.last_tick_ns)*1e-9 if self.last_tick_ns >= 0 else 1/30
        self.last_tick_ns = robot.timestamp_ns
        old_p, old_a = self.position.copy(), self.axis.copy()
        bilateral = bool(np.all(robot.finger_forces >= 15)) and grasp_allowed
        self.bilateral_dwell = self.bilateral_dwell+1 if bilateral else 0
        if (self.anchored and release_requested and np.all(robot.jaw_forces >= .5)
                and np.sum(robot.jaw_forces)>=self.machine_load_min):
            # The machine already carries the specimen before the fingers
            # finish opening. Continuing the robot-grasp prior during opening
            # invents axial steel motion from small wrist tracking errors.
            self.anchored = False; self.released = True
            self.local_correction[:] = 0
            self.release_prior_position = self.position.copy()
            self.release_prior_axis = self.axis.copy()
        if self.anchored:
            prior_p = robot.tcp+rotate(robot.quaternion_xyzw, self.local_center)
            prior_a = rotate(robot.quaternion_xyzw, self.local_axis)
            self.position = prior_p+rotate(robot.quaternion_xyzw, self.local_correction)
            self.axis = rotate(robot.quaternion_xyzw, self.axis_correction if self.axis_correction is not None else self.local_axis)
        else:
            if self.released:
                prior_p,prior_a=self.release_prior_position.copy(),self.release_prior_axis.copy()
            elif self.acquired and self.fixed_support_prior and self.local_center is None:
                # A stationary source bar has no robot-grasp motion model.
                # Keep its independent full-cap survey as the reference. A
                # succession of individually small side-edge errors must not
                # rotate/translate the next prior until valid pixels are
                # rejected. reframe() propagates this survey using odometry.
                # The first measured grasp retires it for this observation
                # lifetime, including a subsequent release onto another rack.
                prior_p,prior_a=self.initial_position.copy(),self.initial_axis.copy()
            else:
                prior_p,prior_a=self.position.copy(),self.axis.copy()
        if camera is not None:
            if camera.frame <= self.last_frame or camera.timestamp_ns > robot.timestamp_ns:
                raise ValueError('Stale/future camera packet')
            self.last_frame = camera.frame
            try:
                probability = self.detector.probabilities(camera.mono8)
                self.last_probability=probability
                if not self.acquired:
                    if not survey_allowed:
                        raise ValueError('Move to the surveyed observation view first')
                    p, a, quality = resting_pose(camera.mono8, probability, camera.K, camera.T_scene_optical,
                        plane_height=self.plane_height,
                        **(dict(prior_center=self.survey_prior[0],prior_axis=self.survey_prior[1]) if self.survey_prior is not None else {}))
                    if a[np.argmax(np.abs(a[:2]))] < 0:
                        a = -a
                    self.initial_candidates.append((p, a))
                    self.initial_candidates = self.initial_candidates[-3:]
                    if len(self.initial_candidates) < 3:
                        raise ValueError('Waiting for three independent full-bar frames')
                    centers = np.stack([value[0] for value in self.initial_candidates])
                    if np.linalg.norm(np.ptp(centers, axis=0)) > .004:
                        raise ValueError('Initial visual position is not yet stable')
                    self.position = centers.mean(0)
                    self.axis = np.mean([value[1] for value in self.initial_candidates], axis=0)
                    self.axis /= np.linalg.norm(self.axis)
                    self.acquired = True
                    self.initial_position = self.position.copy()
                    self.initial_axis = self.axis.copy()
                    self.survey_prior=None
                    if self.track_support_texture:
                        try:self.texture.reset(camera.mono8,probability,camera.K,camera.T_scene_optical,self.position,self.axis)
                        except ValueError as error:quality['texture_status']=str(error)
                elif (self.require_full_support or self.prefer_full_support) and not (self.anchored or self.released):
                    try:
                        p,a,quality=resting_pose(camera.mono8,probability,camera.K,camera.T_scene_optical,
                            plane_height=self.plane_height,prior_center=prior_p,prior_axis=prior_a)
                    except ValueError:
                        if self.require_full_support:raise
                        p,a,quality=constrained_pose(camera.mono8,probability,camera.K,camera.T_scene_optical,
                            prior_p,prior_a,on_support_plane=True,plane_height=self.plane_height)
                    if a @ self.axis < 0:a=-a
                    if self.track_support_texture:
                        if quality.get('axial_position_measured'):
                            try:self.texture.reset(camera.mono8,probability,camera.K,camera.T_scene_optical,p,a)
                            except ValueError as error:quality['texture_status']=str(error)
                        else:
                            p,texture_quality=self.texture.update(camera.mono8,probability,camera.K,camera.T_scene_optical,p,a)
                            quality.update(texture_quality)
                            self.texture_measurements+=1
                    if np.linalg.norm(p-prior_p)>.025:
                        raise ValueError('Tray steel moved beyond the visual support envelope')
                    self.position,self.axis=p,a
                else:
                    p, a, quality = constrained_pose(camera.mono8, probability, camera.K, camera.T_scene_optical,
                        prior_p, prior_a, on_support_plane=not (self.anchored or self.released), plane_height=self.plane_height)
                    if a @ self.axis < 0:
                        a = -a
                    self.position, self.axis = p, a
                    if self.anchored:
                        self.local_correction = rotate(inverse(robot.quaternion_xyzw), p-prior_p)
                        self.axis_correction = rotate(inverse(robot.quaternion_xyzw), a)
                self.last_good_ns = camera.timestamp_ns
                self.last_error = ''
                if self.released:
                    quality={**quality,'prior':'visually_aligned_pose_at_measured_machine_load_transfer',
                             'axial_position_measured':False}
                self.quality = quality
                self.measurements += 1
            except (ValueError, np.linalg.LinAlgError) as error:
                self.last_error = str(error)
                self.rejected_measurements += 1
        if self.acquired and np.isfinite(old_p).all():
            v = (self.position-old_p)/dt
            w = np.cross(old_a, self.axis)/dt
            self.linear_velocity = .75*self.linear_velocity+.25*v
            self.angular_velocity = .75*self.angular_velocity+.25*w
        return self.valid(robot.timestamp_ns)

    def valid(self, now_ns):
        return self.acquired and 0 <= now_ns-self.last_good_ns <= self.max_age_ns

    def reframe(self, new_from_old):
        """Re-express a world-supported estimate after chassis odometry moves.

        The caller must not use this for a bar supported by the moving tray.
        Local grasp transforms remain robot-relative and are never modified.
        """
        T=np.asarray(new_from_old,float);R=T[:3,:3];offset=T[:3,3]
        for position,axis in (('position','axis'),('initial_position','initial_axis'),
                              ('release_prior_position','release_prior_axis')):
            if hasattr(self,position):setattr(self,position,R@getattr(self,position)+offset)
            if hasattr(self,axis):setattr(self,axis,R@getattr(self,axis))
        self.initial_candidates=[(R@p+offset,R@a) for p,a in self.initial_candidates]
        if self.survey_prior is not None:
            p,a=self.survey_prior;self.survey_prior=(R@p+offset,R@a)
        self.linear_velocity=R@self.linear_velocity;self.angular_velocity=R@self.angular_velocity
        if self.track_support_texture:self.texture.reframe(T)

    def latch_grasp(self, robot):
        if not self.valid(robot.timestamp_ns) or not np.all(robot.finger_forces >= 15):
            raise ValueError('A visual pose and measured bilateral grasp are required')
        self.local_center = rotate(inverse(robot.quaternion_xyzw), self.position-robot.tcp)
        self.local_axis = rotate(inverse(robot.quaternion_xyzw), self.axis)
        self.local_correction[:] = 0
        self.axis_correction = None
        self.anchored = True
