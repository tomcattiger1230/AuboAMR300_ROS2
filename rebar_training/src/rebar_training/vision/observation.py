"""Coarse, surveyed observation pose using only the robot and camera mount."""
import numpy as np
import torch

from ..reset import quat_apply, quat_multiply

PICKUP_SEARCH_VIEWS = (
    ((-.50,-.10,1.25),(0.,1.,0.)),
    ((-.55,-.10,1.25),(0.,1.,0.)),
    ((-.50,-.12,1.25),(0.,1.,0.)),
)


def matrix_quaternion(matrix):
    """Proper rotation matrix to a normalized XYZW quaternion."""
    m = np.asarray(matrix, float)
    if m.shape != (3, 3) or not np.allclose(m.T @ m, np.eye(3), atol=1e-5) or np.linalg.det(m) < .99:
        raise ValueError('Expected a proper rotation')
    # Symmetric eigenproblem avoids branch instability near a 180 degree view.
    K = np.array([[m[0,0]-m[1,1]-m[2,2], m[1,0]+m[0,1], m[2,0]+m[0,2], m[2,1]-m[1,2]],
                  [m[1,0]+m[0,1], m[1,1]-m[0,0]-m[2,2], m[2,1]+m[1,2], m[0,2]-m[2,0]],
                  [m[2,0]+m[0,2], m[2,1]+m[1,2], m[2,2]-m[0,0]-m[1,1], m[1,0]-m[0,1]],
                  [m[2,1]-m[1,2], m[0,2]-m[2,0], m[1,0]-m[0,1], np.trace(m)]]) / 3
    _, vectors = np.linalg.eigh(K)
    q = vectors[:, -1]
    return q if q[-1] >= 0 else -q


def look_at(position, target, reference=(1.,0.,0.)):
    z = np.asarray(target, float) - np.asarray(position, float)
    z /= np.linalg.norm(z)
    x = np.cross(z, reference)
    if np.linalg.norm(x) < .1:
        raise ValueError('Observation axis is parallel to the reference direction')
    x /= np.linalg.norm(x)
    return matrix_quaternion(np.column_stack((x, np.cross(z, x), z)))


class ObservationMotion:
    """Feedback motion to look at the pickup zone, never an actual bar pose."""
    def __init__(self, env, camera_position=(-.5, 0., 1.25), survey_target=(-1.15, 0., .522), reference=(1.,0.,0.)):
        wrist_p = env.robot.data.body_pos_w.torch[:, env.wrist_id]
        wrist_q = env.robot.data.body_quat_w.torch[:, env.wrist_id]
        mount=env.vision_mounts['wrist']
        body_id=env.robot.find_bodies(mount['body'])[0][0]
        body_p=env.robot.data.body_pos_w.torch[:,body_id]
        body_q=env.robot.data.body_quat_w.torch[:,body_id]
        mount_p=wrist_p.new_tensor(mount['position']).expand(env.num_envs,-1)
        mount_q=wrist_q.new_tensor(mount['quaternion_xyzw']).expand(env.num_envs,-1)
        camera_position_w=body_p+quat_apply(body_q,mount_p)
        camera_quaternion_w=quat_multiply(body_q,mount_q)
        inverse = wrist_q.clone(); inverse[:, :3] *= -1
        # Fixed hand-eye calibration comes from same-time rigid-link FK and
        # source mount extrinsics. Camera.data can still cache the pre-reset
        # frame when a new observation motion is constructed in the same tick.
        offset = quat_apply(inverse, camera_position_w - wrist_p)
        camera_local_q = quat_multiply(inverse, camera_quaternion_w)
        inverse_local = camera_local_q.clone(); inverse_local[:, :3] *= -1
        camera_q = torch.tensor(look_at(camera_position, survey_target,reference), device=env.device, dtype=wrist_q.dtype).expand(env.num_envs, -1)
        self.quaternion = quat_multiply(camera_q, inverse_local)
        camera_p = torch.tensor(camera_position, device=env.device, dtype=wrist_p.dtype).expand(env.num_envs, -1)
        tcp_offset = offset.clone(); tcp_offset[:, 2] -= .16
        self.target = camera_p - quat_apply(self.quaternion, tcp_offset)
        self.dwell = torch.zeros(env.num_envs, device=env.device, dtype=torch.long)
        self.ready = torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
        self.env = env
        if hasattr(env,'control_tcp_task'):
            initial_p,initial_q=env.control_tcp_task()
        else:
            tcp_offset_world=wrist_p.new_tensor([0.,0.,.16]).expand(env.num_envs,-1)
            initial_p=wrist_p+quat_apply(wrist_q,tcp_offset_world)-env.scene.env_origins
            initial_q=wrist_q
        self.position_error=torch.linalg.vector_norm(self.target-initial_p,dim=-1)
        self.quaternion_error=1-(initial_q*self.quaternion).sum(-1).abs()

    def reset(self, ids):
        self.dwell[ids] = 0
        self.ready[ids] = False

    def actions(self, cartesian_controller):
        e = self.env
        p, q = e._wrist_tcp()
        p = p - e.scene.env_origins
        if hasattr(e,'control_tcp_task'):
            p,q=e.control_tcp_task()
        position_error = torch.linalg.vector_norm(self.target - p, dim=-1)
        quaternion_error = 1 - (q * self.quaternion).sum(-1).abs()
        self.position_error = position_error
        self.quaternion_error = quaternion_error
        # This is a coarse camera placement, not the 3 mm insertion criterion.
        # Actual full-bar visibility must subsequently pass image-space checks.
        good = (position_error < .020) & (quaternion_error < .0005)
        self.dwell = torch.where(good, self.dwell + 1, 0)
        self.ready |= self.dwell >= 24
        return cartesian_controller.cartesian_actions(self.target, self.quaternion,
                                                     -torch.ones(e.num_envs, device=e.device))


class ObserveThenReturn:
    """Retrace measured, collision-checked observation joint motion before picking.

    A direct Cartesian descent from the oblique observation pose changes elbow
    branch and can collide with the robot. Replay records robot joints only.
    The observed steel estimate remains in its scene frame throughout return.
    """
    def __init__(self, env, motion=None, return_speed_scale=1.):
        if not 0<return_speed_scale<=1:raise ValueError('Invalid observation return speed scale')
        self.env = env
        self.return_speed_scale=return_speed_scale
        self.motion = motion if motion is not None else ObservationMotion(env)
        self.ready = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        self.history = [[] for _ in range(env.num_envs)]
        self.index = [-1] * env.num_envs
        self.allow_return = torch.ones(env.num_envs, dtype=torch.bool, device=env.device)

    def reset(self, ids):
        self.motion.reset(ids)
        self.ready[ids] = False
        for i in ids.tolist():
            self.history[i] = []; self.index[i] = -1

    def actions(self, controller):
        e = self.env
        q = e.robot.data.joint_pos.torch[:, e.control_ids[:6]]
        before = self.motion.ready.clone()
        action = self.motion.actions(controller)
        for i in range(e.num_envs):
            if not before[i]:
                self.history[i].append(q[i].clone())
                if self.motion.ready[i]:
                    self.index[i] = len(self.history[i]) - 1
            elif not self.ready[i] and self.allow_return[i]:
                target = self.history[i][self.index[i]]
                while self.index[i] > 0 and (target-q[i]).abs().max() < .018:
                    self.index[i] -= 1
                    target = self.history[i][self.index[i]]
                velocity = 5 * (target-q[i])
                action[i, :6] = (velocity / e.task_spec.arm_speed).clamp(-1, 1)*self.return_speed_scale
                action[i, 6] = -1
                self.ready[i] = self.index[i] == 0 and (target-q[i]).abs().max() < .008
        return action


class JointObservationMotion(ObservationMotion):
    """Coarse empty-hand joint branch chosen by offline USD IK, then FK dwell.

    PhysX contact/force checks remain active. IK alone is not collision proof.
    The actual outbound joint history is retraced before the next pickup.
    """
    def __init__(self,env,*args,**kwargs):
        super().__init__(env,*args,**kwargs)
        from ..kinematics import UsdArm
        from ..spec import asset_directory,GRASP_JOINTS
        from ..route import CARRY_JOINTS
        from .capture import optical_transform
        if env.num_envs!=1:raise ValueError('Joint observation branch currently requires one environment')
        current=env.robot.data.joint_pos.torch[0,env.control_ids[:6]].detach().cpu().numpy()
        model=UsdArm(asset_directory()/'robot.usda')
        target=np.asarray(optical_transform(self.target[0].cpu().numpy(),self.quaternion[0].cpu().numpy()))
        candidates=model.solve(target,[current,CARRY_JOINTS,GRASP_JOINTS,(0.,-.35,.6,0.,.35,0.)])
        for q in candidates:
            wrapped=q+2*np.pi*np.round((current-q)/(2*np.pi))
            q[:]=np.where((wrapped>model.lower+.03)&(wrapped<model.upper-.03),wrapped,q)
        if not candidates:raise RuntimeError('No USD IK branch for the surveyed tray camera view')
        selected=min(candidates,key=lambda q:np.linalg.norm(q-current))
        self.joint_target=torch.tensor(selected,device=env.device,dtype=torch.float32).reshape(1,6)

    def actions(self,controller):
        action=super().actions(controller)
        measured=self.env.robot.data.joint_pos.torch[:,self.env.control_ids[:6]]
        velocity=1.5*(self.joint_target-measured)
        action[:,:6]=(velocity/self.env.task_spec.arm_speed).clamp(-1,1)
        return action
