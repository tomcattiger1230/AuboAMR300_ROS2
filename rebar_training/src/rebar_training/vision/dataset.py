"""Append-only, per-environment episodes captured before automatic reset.

Each row describes state AFTER an action interval (t_start, t_end]. It contains
the action actually applied during that interval, not a claimed action label
for the resulting image. Sparse camera sampling retains the intervening actions.
"""
import hashlib
import json
import math
from pathlib import Path
import uuid

import numpy as np
from PIL import Image

from . import CONTRACT


def dump(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def safe_path(root, relative):
    root = Path(root).resolve()
    path = (root / relative).resolve()
    if not path.is_relative_to(root):
        raise ValueError('Path escapes dataset root')
    return path


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def finite(value):
    if isinstance(value, dict):
        return all(finite(v) for v in value.values())
    if isinstance(value, (list, tuple)):
        return all(finite(v) for v in value)
    return not isinstance(value, (float, int)) or math.isfinite(value)


class DatasetWriter:
    def __init__(self, root, metadata):
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        if (self.root / 'dataset.json').exists():
            raise FileExistsError('Use a new capture directory; existing datasets are immutable')
        self.metadata = dict(metadata, schema_version=CONTRACT, quaternion_order='xyzw',
                             frame_convention='env_local_z_up; camera optical x-right/y-down/z-forward',
                             action_alignment='applied over (t_start_ns,t_end_ns]; image/state at t_end_ns',
                             generated_by_cosmos=False, status='recording')
        dump(self.root / 'dataset.json', self.metadata)
        self.active = {}
        self.count = 0

    def start(self, env_id, seed):
        if env_id in self.active:
            raise RuntimeError('Finish previous episode before reset')
        episode_id = uuid.uuid4().hex
        path = self.root / 'episodes' / episode_id
        path.mkdir(parents=True)
        record = {'episode_id': episode_id, 'env_id': env_id, 'seed': seed,
                  'status': 'recording', 'frame_count': 0, 'physics_outcome': None}
        dump(path / 'episode.json', record)
        self.active[env_id] = (path, record, 0)
        return episode_id

    def append(self, env_id, sample, cameras):
        path, record, last_time = self.active[env_id]
        sample = dict(sample)
        if not finite(sample) or sample['t_end_ns'] <= last_time or sample['t_start_ns'] >= sample['t_end_ns']:
            raise ValueError('Finite, strictly increasing capture times required')
        if len(sample['action_applied']) != 7 or any(abs(v) > 1.000001 for v in sample['action_applied']):
            raise ValueError('action_applied must contain seven clipped values')
        index = record['frame_count']
        sample.update(schema_version=CONTRACT, frame_index=index, cameras={})
        for name, camera in cameras.items():
            if not name.isidentifier():
                raise ValueError('Invalid camera name')
            folder = path / name
            folder.mkdir(exist_ok=True)
            rgb = np.asarray(camera['image'])
            if rgb.dtype != np.uint8 or rgb.ndim not in (2, 3) or (rgb.ndim == 3 and rgb.shape[-1] != 3):
                raise ValueError('Expected uint8 mono or RGB image')
            height, width = rgb.shape[:2]
            calibration = camera['calibration']
            if not finite(calibration) or np.asarray(calibration['K']).shape != (3, 3):
                raise ValueError('Invalid intrinsic matrix')
            if np.asarray(calibration['T_env_optical']).shape != (4, 4):
                raise ValueError('Expected env-from-optical 4x4 transform')
            if calibration['K'][0][0] <= 0 or calibration['K'][1][1] <= 0:
                raise ValueError('Focal lengths must be positive')
            prefix = folder / f'{index:06d}'
            image_path = prefix.with_suffix('.png')
            Image.fromarray(rgb).save(image_path)
            item = {'image': str(image_path.relative_to(self.root)), 'image_sha256': sha256(image_path),
                    'width': width, 'height': height, 'encoding': 'mono8' if rgb.ndim == 2 else 'rgb8',
                    'calibration': calibration, 'sensor_frame': int(camera['sensor_frame']),
                    'sensor_t_end_ns': sample['t_end_ns'], 'pose_source': 'simulator',
                    'policy_observable': camera.get('policy_observable', True)}
            for kind, dtype in (('depth_m', np.float32), ('semantic_ids', np.int32)):
                if kind not in camera:
                    continue
                array = np.asarray(camera[kind], dtype=dtype)
                if array.shape != (height, width):
                    raise ValueError(f'{kind} shape mismatch')
                target = folder / f'{index:06d}_{kind}.npy'
                if kind == 'depth_m':
                    valid = np.isfinite(array) & (array > 0)
                    array = np.where(valid, array, 0).astype(np.float32)
                    valid_path = folder / f'{index:06d}_depth_valid.npy'
                    np.save(valid_path, valid, allow_pickle=False)
                    item['depth_valid'] = str(valid_path.relative_to(self.root))
                    item['depth_valid_sha256'] = sha256(valid_path)
                np.save(target, array, allow_pickle=False)
                item[kind] = str(target.relative_to(self.root))
                item[kind + '_sha256'] = sha256(target)
            item['auxiliary_labels_simulation_only'] = True
            item['semantic_mapping'] = camera.get('semantic_mapping', {})
            sample['cameras'][name] = item
        if not sample['cameras']:
            raise ValueError('At least one camera required')
        with (path / 'frames.jsonl').open('a') as stream:
            stream.write(json.dumps(sample, allow_nan=False) + '\n')
        record['frame_count'] += 1
        self.count += 1
        self.active[env_id] = (path, record, sample['t_end_ns'])

    def finish(self, env_id, outcome=None):
        path, record, _ = self.active[env_id]
        if outcome and outcome.get('success') and (outcome.get('failure') or outcome.get('timeout')):
            raise ValueError('Contradictory episode result')
        self.active.pop(env_id)
        record.update(status='complete' if outcome else 'interrupted', physics_outcome=outcome)
        dump(path / 'episode.json', record)

    def close(self):
        for env_id in list(self.active):
            self.finish(env_id)
        self.metadata.update(status='closed', frame_count=self.count)
        dump(self.root / 'dataset.json', self.metadata)


def validate(root):
    root = Path(root).resolve()
    metadata = json.loads((root / 'dataset.json').read_text())
    if metadata.get('schema_version') != CONTRACT:
        raise ValueError('Unsupported camera contract')
    from jsonschema import Draft202012Validator
    schema_path = Path(__file__).with_name('camera_frame.schema.json')
    schema = Draft202012Validator(json.loads(schema_path.read_text()))
    frames = episodes = 0
    for episode_path in sorted((root / 'episodes').glob('*/episode.json')):
        episode = json.loads(episode_path.read_text())
        rows_path = episode_path.with_name('frames.jsonl')
        rows = [json.loads(line) for line in rows_path.read_text().splitlines()] if rows_path.exists() else []
        last = 0
        for index, row in enumerate(rows):
            schema.validate(row)
            if row['frame_index'] != index or row['t_end_ns'] <= last or not finite(row):
                raise ValueError('Invalid frame ordering or values')
            if row['schema_version'] != CONTRACT or row['t_start_ns'] >= row['t_end_ns']:
                raise ValueError('Invalid action alignment')
            if not row.get('action_history') or row['action_history'][-1]['t_end_ns'] != row['t_end_ns']:
                raise ValueError('Intervening control actions must be retained')
            history = row['action_history']
            if row['t_start_ns'] != history[-1]['t_start_ns'] or row['action_applied'] != history[-1]['action_applied']:
                raise ValueError('Final action does not match the recorded interval')
            if last and history[0]['t_start_ns'] != last:
                raise ValueError('Missing actions between captured frames')
            for i, action in enumerate(history):
                if action['t_start_ns'] >= action['t_end_ns'] or (i and action['t_start_ns'] != history[i-1]['t_end_ns']):
                    raise ValueError('Control history has a gap or invalid duration')
            for camera in row['cameras'].values():
                for key in ('image', 'depth_m', 'semantic_ids', 'depth_valid'):
                    if key in camera and sha256(safe_path(root, camera[key])) != camera[key + '_sha256']:
                        raise ValueError('Corrupted camera asset')
                with Image.open(safe_path(root, camera['image'])) as image:
                    if image.size != (camera['width'], camera['height']):
                        raise ValueError('Image/calibration geometry mismatch')
                for key in ('depth_m', 'semantic_ids', 'depth_valid'):
                    if key in camera and np.load(safe_path(root, camera[key]), allow_pickle=False).shape != (camera['height'], camera['width']):
                        raise ValueError('Label geometry mismatch')
                if camera['sensor_t_end_ns'] != row['t_end_ns']:
                    raise ValueError('Unsynchronized image/state')
            last = row['t_end_ns']
        if episode['frame_count'] != len(rows):
            raise ValueError('Episode did not close cleanly')
        frames += len(rows)
        episodes += 1
    return {'schema_version': CONTRACT, 'episodes': episodes, 'frames': frames,
            'physics_validated': False, 'note': 'File integrity only; GPU synchronization and physics require runtime validation'}
