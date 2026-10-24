"""Opt-in adapter for pinned Lab 3 Camera/ProxyArray; imported only with cameras enabled."""
import hashlib
import ast
import json
from pathlib import Path

import numpy as np

from .dataset import DatasetWriter

SEMANTIC_CLASSES = ('BACKGROUND', 'UNLABELLED', 'robot', 'rebar', 'station', 'fixture', 'machine')


def load_config(path):
    config = json.loads(Path(path).read_text())
    if config.get('schema_version') != 'rebar_capture_v1':
        raise ValueError('Unsupported capture configuration')
    if not isinstance(config['stride'], int) or config['stride'] < 1:
        raise ValueError('Capture stride must be a positive control-step count')
    if not config['env_ids'] or len(set(config['env_ids'])) != len(config['env_ids']) or any(i < 0 for i in config['env_ids']):
        raise ValueError('Unique, non-negative env IDs required')
    names = [c['name'] for c in config['cameras']]
    if not names or len(set(names)) != len(names) or any(not n.isidentifier() for n in names):
        raise ValueError('Unique camera names required')
    for c in config['cameras']:
        outputs = c.get('data_types', ['rgb','distance_to_image_plane','semantic_segmentation'])
        if 'rgb' not in outputs or len(outputs) != len(set(outputs)) or set(outputs)-{'rgb','distance_to_image_plane','semantic_segmentation'}:
            raise ValueError('Unsupported camera output selection')
        if c['width'] < 16 or c['height'] < 16 or c['mode'] not in ('existing', 'spawn'):
            raise ValueError('Invalid camera geometry')
        if not c['prim_path'].startswith('/World/envs/env_.*/'):
            raise ValueError('Camera path must be scoped to cloned environments')
        if c['mode'] == 'spawn' and c['prim_path'].count('/') != 4:
            raise ValueError('Spawn diagnostic cameras directly under each env')
    return config


def tensor(value):
    return value.torch if hasattr(value, 'torch') else value


def array(value):
    value = tensor(value)
    return value.detach().cpu().numpy() if hasattr(value, 'detach') else np.asarray(value)


def optical_transform(position, quaternion):
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    if abs(np.linalg.norm(quaternion) - 1) > .002:
        raise ValueError('Camera quaternion must be normalized XYZW')
    out = np.eye(4)
    out[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                   [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                   [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    out[:3, 3] = position
    return out.tolist()


def canonical_semantics(values, mapping):
    """Normalize RTX packed RGBA IDs to explicit integer class IDs.

    Lab 3 RTX may return packed int32 colors despite colorize=False. Never
    write those as numeric IDs paired with a tuple-keyed color lookup table.
    """
    values = np.asarray(values)
    labels = mapping.get('idToLabels', {})
    if not labels:
        raise RuntimeError('Semantic renderer returned no label mapping')
    if all(str(k).lstrip('-').isdigit() for k in labels):
        if any(str(int(value)) not in labels for value in np.unique(values)):
            raise RuntimeError('Unmapped semantic pixels')
        return values.astype(np.int32), mapping
    classes = SEMANTIC_CLASSES
    if any(label['class'] not in classes for label in labels.values()):
        raise ValueError('Unknown semantic class; extend the explicit class catalog')
    result = np.full(values.shape, -1, np.int32)
    for key, label in labels.items():
        rgba = ast.literal_eval(key)
        if len(rgba) != 4 or any(not isinstance(v, int) or not 0 <= v <= 255 for v in rgba):
            raise ValueError('Invalid RTX semantic color key')
        packed = sum(v << (8 * i) for i, v in enumerate(rgba))
        native = packed if packed < 2**31 else packed - 2**32
        result[values == native] = classes.index(label['class'])
    if (result < 0).any():
        raise RuntimeError('Unmapped RTX semantic pixels')
    return result, {'idToLabels': {str(i): {'class': label} for i, label in enumerate(classes)},
                    'nativeIdToLabels': labels, 'native_encoding': 'little_endian_packed_rgba_int32'}


def synchronize_mounts(env):
    """Drive moving camera poses from measured rigid links, not stale USD FK."""
    import torch
    from ..reset import quat_apply, quat_multiply
    for name, mount in getattr(env, 'vision_mounts', {}).items():
        body_id = env.robot.find_bodies(mount['body'])[0][0]
        q = tensor(env.robot.data.body_quat_w)[:, body_id]
        p = tensor(env.robot.data.body_pos_w)[:, body_id]
        offset = torch.tensor(mount['position'], device=env.device, dtype=p.dtype).expand(env.num_envs, -1)
        rotation = torch.tensor(mount['quaternion_xyzw'], device=env.device, dtype=q.dtype).expand(env.num_envs, -1)
        env.vision_cameras[name].set_world_poses(p + quat_apply(q, offset), quat_multiply(q, rotation), convention='ros')


def setup_cameras(env):
    from isaaclab.sensors import Camera, CameraCfg
    import isaaclab.sim as sim_utils
    import omni.usd
    from pxr import Gf, UsdGeom, UsdSemantics

    config = load_config(env.cfg.camera_capture_config)
    if max(config['env_ids']) >= env.num_envs or env.num_envs > config.get('max_envs', 4):
        raise ValueError('Capture env IDs/count exceed the configured small-batch limit')
    stage = omni.usd.get_context().get_stage()
    paths = {'Robot': 'robot', 'Rebar/body': 'rebar'}
    if env.cfg.task in ('grasp', 'loading'):
        paths['Station'] = 'station'
    if env.cfg.task != 'grasp':
        prefix = 'Cell/' if env.cfg.task == 'loading' else ''
        paths[prefix + 'Fixture'] = 'fixture'
        paths[prefix + 'Machine'] = 'machine'
    for path, label in (paths.items() if config.get('label_scene', True) else []):
        prim = stage.GetPrimAtPath('/World/envs/env_0/' + path)
        if not prim:
            raise RuntimeError('Cannot label missing prim: ' + path)
        UsdSemantics.LabelsAPI.Apply(prim, 'class').CreateLabelsAttr([label])
    env.vision_cameras = {}
    env.vision_mounts = {}
    for c in config['cameras']:
        spawn = None
        offset = CameraCfg.OffsetCfg()
        if c['mode'] == 'spawn':
            spawn = sim_utils.PinholeCameraCfg(focal_length=c['focal_length_mm'],
                         horizontal_aperture=c['horizontal_aperture_mm'], clipping_range=(.01, 20.))
            offset = CameraCfg.OffsetCfg(pos=tuple(c['position_env_m']), rot=tuple(c['quaternion_xyzw']), convention='ros')
        if c.get('mount_body'):
            body_path = '/World/envs/env_0/Robot/' + c['mount_body']
            camera_path = c['prim_path'].replace('env_.*', 'env_0')
            body, prim = stage.GetPrimAtPath(body_path), stage.GetPrimAtPath(camera_path)
            if not body or not prim:
                raise RuntimeError('Missing physical camera mount or optical prim')
            cache = UsdGeom.XformCache()
            relative = cache.GetLocalToWorldTransform(prim) * cache.GetLocalToWorldTransform(body).GetInverse()
            # USD cameras use OpenGL; optical/ROS axes differ by Rx(pi).
            q = relative.ExtractRotationQuat() * Gf.Quatd(0, Gf.Vec3d(1, 0, 0))
            env.vision_mounts[c['name']] = {'body': c['mount_body'],
                'position': list(relative.ExtractTranslation()), 'quaternion_xyzw': [*q.GetImaginary(), q.GetReal()]}
        cfg = CameraCfg(prim_path=c['prim_path'], spawn=spawn, offset=offset, width=c['width'], height=c['height'],
                        update_period=0., update_latest_camera_pose=True,
                        data_types=c.get('data_types', ['rgb', 'distance_to_image_plane', 'semantic_segmentation']),
                        colorize_semantic_segmentation=False)
        camera = Camera(cfg)
        env.vision_cameras[c['name']] = camera
        env.scene.sensors['capture_' + c['name']] = camera
    env.vision_config = config


class CaptureRecorder:
    def __init__(self, env):
        from ..spec import asset_directory
        self.env = env
        self.config = env.vision_config
        self.writer = DatasetWriter(env.cfg.camera_dataset_root, {
            'task': env.cfg.task, 'seed': env.cfg.seed, 'observation_contract': env.observation_contract,
            'control_dt_s': env.task_spec.control_dt, 'capture_stride': self.config['stride'],
            'nominal_fps': 1 / (env.task_spec.control_dt * self.config['stride']),
            'source': 'isaac_lab', 'fixture': False, 'runtime_validated': False,
            'camera_config': self.config, 'units': {'position': 'm', 'time': 'ns', 'force': 'N', 'arm_joints': 'rad', 'finger_joints': 'm'},
            'moving_camera_pose_source': 'measured_rigid_link_fk_with_usd_fixed_extrinsics',
        })
        manifest = asset_directory() / 'manifest.json'
        if manifest.is_file():
            self.writer.metadata['asset_manifest_sha256'] = hashlib.sha256(manifest.read_bytes()).hexdigest()
            self.writer.metadata['fixture_model'] = json.loads(manifest.read_text()).get('fixture_model', 'pads_v1')
        self.pending = {i: [] for i in self.config['env_ids']}
        self.last_sensor_frame = {}
        self.tick = 0

    def reset(self, ids):
        for i in ids.tolist():
            if i not in self.pending:
                continue
            if i in self.writer.active:
                self.writer.finish(i)  # Explicit reset is an interruption, never an inferred success.
            self.writer.start(i, self.env.cfg.seed)
            self.pending[i] = []
            for name in self.env.vision_cameras:
                self.last_sensor_frame.pop((i, name), None)

    def record(self, state, result):
        e = self.env
        self.tick += 1
        end = round(self.tick * e.task_spec.control_dt * 1e9)
        start = round((self.tick - 1) * e.task_spec.control_dt * 1e9)
        actions = array(e.actions.clamp(-1, 1))
        q = array(e.robot.data.joint_pos)[:, e.control_ids]
        qd = array(e.robot.data.joint_vel)[:, e.control_ids]
        # Reading Camera.data renders the current physics step in this pinned API.
        camera_data = None
        for i in self.pending:
            self.pending[i].append({'t_start_ns': start, 't_end_ns': end, 'action_applied': actions[i].tolist()})
            terminal = bool(result.terminated[i] | result.truncated[i])
            if self.tick % self.config['stride'] and not terminal:
                continue
            if camera_data is None:
                synchronize_mounts(e)
                camera_data = {n: c.data for n, c in e.vision_cameras.items()}
            cameras = {}
            for c in self.config['cameras']:
                name = c['name']
                data = camera_data[name]
                frame = int(array(e.vision_cameras[name].frame)[i])
                previous = self.last_sensor_frame.get((i, name), -1)
                if frame <= previous:
                    raise RuntimeError('Stale camera frame; refusing image/state pairing')
                self.last_sensor_frame[i, name] = frame
                image = array(data.output['rgb'])[i]
                if c.get('mono', False):
                    image = np.round(image[..., :3] @ np.array([.299, .587, .114])).clip(0, 255).astype(np.uint8)
                mapping = data.info.get('semantic_segmentation') or {}
                if isinstance(mapping, list):
                    mapping = mapping[i]
                semantic, mapping = canonical_semantics(array(data.output['semantic_segmentation'])[i].squeeze(-1), mapping)
                cameras[name] = {'image': image, 'sensor_frame': frame,
                    'depth_m': array(data.output['distance_to_image_plane'])[i].squeeze(-1),
                    'semantic_ids': semantic,
                    'semantic_mapping': mapping, 'policy_observable': c.get('policy_observable', True),
                    'calibration': {'K': array(data.intrinsic_matrices)[i].tolist(), 'distortion': [0.] * 5,
                        'calibration_source': 'ideal_simulation_not_real_calibration',
                        'T_env_optical': optical_transform(array(data.pos_w)[i] - array(e.scene.env_origins)[i],
                                                          array(data.quat_w_ros)[i])}}
            measured = {key: array(getattr(state, key))[i].tolist() for key in state.__dataclass_fields__}
            measured.update(arm_joint_position_rad=q[i, :6].tolist(), arm_joint_velocity_rad_s=qd[i, :6].tolist(),
                            finger_position_m=q[i, 6:].tolist(), finger_velocity_m_s=qd[i, 6:].tolist(),
                            joint_target_position=array(e.targets)[i].tolist(),
                            bar_pose_env_xyzw=np.concatenate((array(state.bar)[i], array(e.bar.data.root_quat_w)[i])).tolist(),
                            logic_phase=int(e.logic.phase[i]), route_stage=int(e.route.stage[i]) if e.route else None)
            if getattr(e, 'carriage_ids', []):
                measured['carriage_displacement_m'] = array(e.fixture.data.joint_pos)[i, e.carriage_ids].tolist()
            self.writer.append(i, {'t_start_ns': start, 't_end_ns': end, 'action_applied': actions[i].tolist(),
                               'action_history': self.pending[i], 'state': measured, 'terminal': terminal,
                               'on_capture_grid': self.tick % self.config['stride'] == 0}, cameras)
            self.pending[i] = []
            if terminal:
                self.writer.finish(i, {'success': bool(result.success[i]), 'failure': bool(result.failure[i]),
                                     'timeout': bool(result.truncated[i]), 'picked': bool(e.logic.pick_complete[i]),
                                     'steps': int(e.logic.elapsed[i])})

    def close(self):
        self.writer.close()
