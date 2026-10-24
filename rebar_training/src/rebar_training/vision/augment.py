"""Offline Cosmos 3 NIM Transfer jobs. No HTTP calls, weights, or implicit generation."""
import base64
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

import numpy as np
from PIL import Image

from .dataset import dump, safe_path, sha256, validate

NIM_REFERENCE = 'https://github.com/NVIDIA/cosmos/blob/main/cookbooks/cosmos3/nim/transfer.md'


def plan(dataset, output, camera, prompts, frames=121, control='edge', model='Cosmos3-Nano', allow_fixture=False):
    dataset, output = Path(dataset).resolve(), Path(output).resolve()
    validate(dataset)
    metadata = json.loads((dataset / 'dataset.json').read_text())
    if metadata.get('status') != 'closed' or (metadata.get('fixture') and not allow_fixture):
        raise ValueError('Use a closed real capture dataset; CPU fixtures require explicit test opt-in')
    review_path = dataset / 'quality_review.json'
    review = json.loads(review_path.read_text()) if review_path.is_file() else {}
    if review.get('status') == 'rejected':
        raise ValueError('Capture quality review rejected this dataset')
    if model not in ('Cosmos3-Nano', 'Cosmos3-Super'):
        raise ValueError('Transfer is prepared only for Nano/Super, not Edge')
    if control not in ('edge', 'blur', 'depth', 'seg') or not 5 <= frames <= 300:
        raise ValueError('Unsupported control or frame count')
    if metadata['nominal_fps'] not in (10, 30):
        raise ValueError('This NIM export supports 10/30 FPS; explicitly resample other rates before planning')
    if not prompts or any(not isinstance(p, str) or not p.strip() for p in prompts):
        raise ValueError('Appearance prompts are required')
    output.mkdir(parents=True, exist_ok=True)
    target = output / 'jobs.json'
    if target.exists():
        raise FileExistsError('Use a new augmentation plan directory')
    jobs = []
    period = round(1e9 / metadata['nominal_fps'])
    for episode_path in sorted((dataset / 'episodes').glob('*/episode.json')):
        episode = json.loads(episode_path.read_text())
        if episode['status'] != 'complete' or not episode.get('physics_outcome', {}).get('success'):
            continue
        rows = [json.loads(line) for line in episode_path.with_name('frames.jsonl').read_text().splitlines()]
        # Off-grid terminal frames remain in the dataset, but are not silently
        # retimed into a constant-FPS control video.
        rows = [r for r in rows if r.get('on_capture_grid', True)]
        starts = list(range(0, len(rows) - frames + 1, frames))
        # Include the end of a successful rollout. Otherwise a non-divisible
        # episode silently loses the clamp/release tail, the task's key event.
        if len(rows) >= frames and len(rows) - frames not in starts:
            starts.append(len(rows) - frames)
        for start in starts:
            clip = rows[start:start + frames]
            if any(clip[i]['t_end_ns'] - clip[i-1]['t_end_ns'] != period for i in range(1, frames)):
                raise ValueError('Control video must have uniformly sampled source frames')
            if any(camera not in row['cameras'] for row in clip):
                raise ValueError('Requested camera absent from a source frame')
            if control in ('depth', 'seg'):
                key = 'depth_m' if control == 'depth' else 'semantic_ids'
                if any(key not in row['cameras'][camera] for row in clip):
                    raise ValueError('Precomputed control requires raw simulation labels')
            for variant, prompt in enumerate(prompts):
                jobs.append({'job_id': f'{episode["episode_id"]}_{start:06d}_{variant:02d}',
                    'episode_id': episode['episode_id'], 'camera': camera, 'frames': clip,
                    'fixture': bool(metadata.get('fixture')),
                    'episode_path': str(episode_path.relative_to(dataset)),
                    'episode_metadata_sha256': sha256(episode_path),
                    'episode_frames_sha256': sha256(episode_path.with_name('frames.jsonl')),
                    'appearance_prompt': prompt, 'model': model, 'control': control,
                    'num_frames': frames, 'fps': metadata['nominal_fps'], 'seed': 2026 + variant,
                    'resolution': '480_16_9', 'output_size': [832, 480], 'depth_range_m': [.01, 3.],
                    'status': 'planned_not_generated', 'labels_reusable': False,
                    'geometry_review_required': True})
    if not jobs:
        raise ValueError('No complete successful episodes contain a full uniform clip')
    result = {'schema_version': 'rebar_cosmos_jobs_v1', 'backend': 'cosmos3_nim_v1_infer',
              'reference': NIM_REFERENCE, 'dataset_manifest_sha256': sha256(dataset / 'dataset.json'),
              'fixture': bool(metadata.get('fixture')), 'jobs': jobs}
    dump(target, result)
    return result


def letterbox_geometry(width, height, target):
    tw, th = target
    scale = min(tw / width, th / height)
    w, h = round(width * scale), round(height * scale)
    x, y = (tw - w) // 2, (th - h) // 2
    # Use actual rounded scales, not an ideal scale that disagrees with pixels.
    transform = np.array([[w / width, 0, x], [0, h / height, y], [0, 0, 1.]])
    return (w, h), (x, y), transform


def prepare_media(dataset, plan_path, job_id, output, ffmpeg='ffmpeg'):
    dataset, output = Path(dataset).resolve(), Path(output).resolve()
    spec = json.loads(Path(plan_path).read_text())
    if sha256(dataset / 'dataset.json') != spec['dataset_manifest_sha256']:
        raise ValueError('Source dataset changed since planning')
    matches = [j for j in spec['jobs'] if j['job_id'] == job_id]
    if len(matches) != 1:
        raise ValueError('Unknown or duplicate augmentation job ID')
    job = matches[0]
    if sha256(safe_path(dataset, job['episode_path'])) != job['episode_metadata_sha256']:
        raise ValueError('Episode outcome changed since planning')
    if sha256(safe_path(dataset, job['episode_path']).with_name('frames.jsonl')) != job['episode_frames_sha256']:
        raise ValueError('Episode trajectory changed since planning')
    if not shutil.which(ffmpeg):
        raise RuntimeError('Install ffmpeg on the new machine to encode control media')
    output.mkdir(parents=True, exist_ok=True)
    if (output / 'prepared.json').exists():
        raise FileExistsError('Prepared media are immutable')
    calibration = []
    with tempfile.TemporaryDirectory(prefix='rebar-control-') as folder:
        folder = Path(folder)
        for index, row in enumerate(job['frames']):
            camera = row['cameras'][job['camera']]
            key = {'depth': 'depth_m', 'seg': 'semantic_ids'}.get(job['control'], 'image')
            source = safe_path(dataset, camera[key])
            if sha256(source) != camera[key + '_sha256']:
                raise ValueError('Source frame changed')
            if key == 'depth_m':
                data = np.load(source, allow_pickle=False)
                near, far = job['depth_range_m']
                values = np.where(data > 0, 255 * (1 - np.clip((data - near) / (far - near), 0, 1)), 0)
                image = Image.fromarray(values.astype(np.uint8)).convert('RGB')
            elif key == 'semantic_ids':
                from .capture import SEMANTIC_CLASSES
                data = np.load(source, allow_pickle=False)
                canonical = np.zeros_like(data, dtype=np.uint32)
                labels = camera['semantic_mapping'].get('idToLabels', {})
                if any(str(v) not in labels for v in np.unique(data)):
                    raise ValueError('Semantic IDs lack an integer class mapping')
                for identifier, label in labels.items():
                    canonical[data == int(identifier)] = SEMANTIC_CLASSES.index(label['class'])
                data = canonical
                image = Image.fromarray(np.stack(((data*53)%256, (data*97)%256, (data*193)%256), -1).astype(np.uint8))
            else:
                image = Image.open(source).convert('RGB')
            size, offset, affine = letterbox_geometry(image.width, image.height, job['output_size'])
            resized = image.resize(size, Image.Resampling.NEAREST if key == 'semantic_ids' else Image.Resampling.BILINEAR)
            canvas = Image.new('RGB', tuple(job['output_size']))
            canvas.paste(resized, offset)
            canvas.save(folder / f'{index:06d}.png')
            calibration.append({'t_end_ns': row['t_end_ns'], 'source_to_control_pixel': affine.tolist(),
                                'K_control': (affine @ np.asarray(camera['calibration']['K'])).tolist(),
                                'T_env_optical': camera['calibration']['T_env_optical']})
        media = output / 'control.mp4'
        subprocess.run([ffmpeg, '-v', 'error', '-n', '-framerate', str(job['fps']), '-i', str(folder / '%06d.png'),
                        '-frames:v', str(job['num_frames']), '-c:v', 'libx264', '-crf', '12', '-pix_fmt', 'yuv420p', str(media)],
                       check=True, timeout=120)
    prepared = {k: v for k, v in job.items() if k != 'frames'}
    prepared.update(media='control.mp4', media_sha256=sha256(media), calibration=calibration,
                    control_is_visualization_not_metric_labels=True, labels_reusable=False,
                    source_dataset_sha256=spec['dataset_manifest_sha256'], status='media_prepared_not_generated')
    dump(output / 'prepared.json', prepared)
    return prepared


def nim_request(prepared_dir, output):
    folder = Path(prepared_dir).resolve()
    spec = json.loads((folder / 'prepared.json').read_text())
    if spec['model'] not in ('Cosmos3-Nano', 'Cosmos3-Super'):
        raise ValueError('Unsupported Transfer model')
    media = safe_path(folder, spec['media'])
    if sha256(media) != spec['media_sha256']:
        raise ValueError('Control media hash mismatch')
    encoded = 'data:video/mp4;base64,' + base64.b64encode(media.read_bytes()).decode('ascii')
    if len(encoded) > 100_000_000:
        raise ValueError('Control exceeds documented NIM media limit')
    transfer = {'control_guidance': 1.5, 'num_conditional_frames': 1,
                'num_first_chunk_conditional_frames': 0, 'num_video_frames_per_chunk': spec['num_frames']}
    request = {'model_mode': 'video2video', 'prompt': spec['appearance_prompt'],
        'negative_prompt': 'Bent or duplicated rebar, changing gripper geometry, displaced jaws, altered camera trajectory, object morphing.',
        'transfer': transfer, 'resolution': spec['resolution'], 'num_frames': spec['num_frames'],
        'fps': spec['fps'], 'num_inference_steps': 50, 'guidance_scale': 3., 'seed': spec['seed']}
    if spec['control'] in ('edge', 'blur'):
        request['input_reference'] = encoded
        transfer[spec['control']] = {'preset_edge_threshold' if spec['control'] == 'edge' else 'preset_blur_strength': 'medium'}
    else:
        transfer[spec['control']] = {'video': encoded}
    dump(output, request)
    return {'request': str(Path(output).resolve()), 'method': 'POST', 'endpoint': '/v1/infer',
            'sent': False, 'fixture': spec.get('fixture', False), 'model_selection': 'Choose matching compatible NIM deployment; model is not a request field',
            'labels_reusable': False, 'reference': NIM_REFERENCE}
