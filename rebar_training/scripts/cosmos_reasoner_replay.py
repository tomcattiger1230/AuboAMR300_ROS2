"""Offline Cosmos Reasoner replay only. Never imports Isaac, a policy, or optimizer.

prepare uses ffmpeg on an immutable copy of recorded video. infer runs a local
Reasoner in an isolated environment and writes advisory analysis, never actions.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import math
import os
import subprocess
import time

MODEL_ID = 'nvidia/Cosmos3-Edge'
MODEL_REVISION = '344d602b128d1bbdacb43b08d0a3626f46343e29'
VIDEO_SHA = '7d85144259e009b40d14c1fc7924c1718caee48264b6792938f0eb1a0703f2ef'
POLICY_SHA = '393b47f8385a9163e3c9f88b53375923327aa3c80239a3af105e700cf1f93a82'


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def save(path, value):
    path = Path(path)
    pending = path.with_suffix(path.suffix + '.pending')
    pending.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    os.replace(pending, path)


def now():
    return datetime.now(timezone.utc).isoformat()


def intervals(duration, clip_seconds):
    if not math.isfinite(duration) or duration <= 0 or clip_seconds <= 0:
        raise ValueError('Invalid video interval')
    return [(start, min(duration, start + clip_seconds))
            for start in range(0, math.ceil(duration), clip_seconds)]


def verify_baseline(root):
    manifest = json.loads((root / 'manifest.json').read_text())
    assert sha256(manifest['source_video']) == manifest['source_video_sha256'] == VIDEO_SHA
    assert sha256(manifest['frozen_policy']) == manifest['frozen_policy_sha256'] == POLICY_SHA
    return manifest


def prepare(args):
    video, policy, root = args.video.resolve(), args.policy.resolve(), args.output.resolve()
    assert not root.exists(), 'Choose a new replay output directory; no overwriting old evidence'
    assert sha256(video) == VIDEO_SHA and sha256(policy) == POLICY_SHA
    probe = json.loads(subprocess.check_output(
        [args.ffprobe, '-v', 'error', '-show_format', '-show_streams', '-of', 'json', str(video)],
        timeout=30, text=True))
    duration = float(probe['format']['duration'])
    assert abs(duration - 316.4) < .001
    root.mkdir(parents=True)
    (root / 'clips').mkdir()
    (root / 'responses').mkdir()
    clips = []
    for index, (start, end) in enumerate(intervals(duration, args.clip_seconds)):
        target = root / 'clips' / f'clip_{index:03d}.mp4'
        # Remove all top phase/time/control text. Original video stays untouched.
        filters = f'crop=1280:630:0:90,fps={args.fps},scale=768:-2'
        subprocess.run([args.ffmpeg, '-v', 'error', '-n', '-ss', str(start), '-i', str(video),
                        '-t', str(end-start), '-vf', filters, '-an', '-c:v', 'libx264',
                        '-crf', '18', '-pix_fmt', 'yuv420p', str(target)], check=True, timeout=60)
        clips.append({'id': index, 'path': str(target.relative_to(root)),
                      'source_start_s': start, 'source_end_s': end,
                      'sha256': sha256(target)})
    manifest = {'at': now(), 'schema': 'rebar_cosmos_replay_v1',
                'source_video': str(video), 'source_video_sha256': VIDEO_SHA,
                'frozen_policy': str(policy), 'frozen_policy_sha256': POLICY_SHA,
                'source_duration_seconds': duration, 'sampling_fps': args.fps,
                'phase_overlay_removed': True, 'crop_original_pixels': [0, 90, 1280, 630],
                'derived_media_only': True, 'native_report_or_metrics_given_to_model': False,
                'clips': clips, 'model_id': MODEL_ID, 'model_revision': MODEL_REVISION,
                'training': False, 'simulation': False, 'control_output_connected': False}
    save(root / 'manifest.json', manifest)
    verify_baseline(root)
    print(json.dumps({'prepared_clips': len(clips), 'output': str(root)}), flush=True)


def prompt_for(clip):
    duration = clip['source_end_s'] - clip['source_start_s']
    return (
        'Analyze ONLY the visible content in this recorded robotics simulation video. '
        'It shows a mobile manipulator handling a metal rod. The inset is a wrist camera. '
        'No control commands will be executed. Do not infer millimetre accuracy, contact '
        'forces, sensor validity, or final task success from appearance alone. '
        'Distinguish what is visible from what is uncertain or occluded. '
        f'The clip is {duration:.3f} seconds. All event times must be relative to this clip, '
        'starting at zero. Give a concise final answer only, without a reasoning trace. '
        'Return one JSON object, no markdown, with exactly these keys: '
        '"summary" (string), "events" (array of objects with "start_s" (number), '
        '"end_s" (number), "description" (string)), '
        '"visible_concerns" (array of strings), "uncertain" (array of strings). '
        'Use English. Report observable movements, rod support, gripper changes and '
        'camera viewpoint changes where visible. A nearby gripper does not prove a '
        'secure grasp. Use empty arrays when appropriate. Follow this valid JSON '
        'template exactly (replace the text and numbers with your observations): '
        '{"summary":"Visible scene and movement.","events":['
        '{"start_s":0,"end_s":1,"description":"Visible movement."}],'
        '"visible_concerns":[],"uncertain":["Occluded details."]}. '
        'Do not rename the key uncertain. End with a closing brace.'
    )


def validate_answer(answer, duration):
    expected = {'summary', 'events', 'visible_concerns', 'uncertain'}
    if not isinstance(answer, dict) or set(answer) != expected or not isinstance(answer['summary'], str):
        raise ValueError('Wrong response keys or summary')
    for key in ['visible_concerns', 'uncertain']:
        if not isinstance(answer[key], list) or not all(isinstance(v, str) for v in answer[key]):
            raise ValueError('Invalid advisory list')
    if not isinstance(answer['events'], list):
        raise ValueError('Invalid events')
    for event in answer['events']:
        if set(event) != {'start_s', 'end_s', 'description'} or not isinstance(event['description'], str):
            raise ValueError('Invalid event')
        start, end = event['start_s'], event['end_s']
        if (isinstance(start, bool) or isinstance(end, bool)
                or not isinstance(start, (float, int)) or not isinstance(end, (float, int))
                or not math.isfinite(start) or not math.isfinite(end)
                or not 0 <= start <= end <= duration + .05):
            raise ValueError('Event timestamp outside clip')
    return answer


def replay_inputs(processor, video, prompt, fps):
    # Decode explicitly: torchvision's legacy PyAV wrapper is incompatible with
    # PyAV 19. Preserve the prepared clip's exact frame/time mapping.
    import av
    import numpy as np
    with av.open(str(video)) as container:
        stream = container.streams.video[0]
        assert abs(float(stream.average_rate) - fps) < .001
        frames = np.stack([frame.to_ndarray(format='rgb24')
                           for frame in container.decode(stream)])
    metadata = {'total_num_frames': len(frames), 'fps': fps,
                'frames_indices': list(range(len(frames))),
                'duration': len(frames)/fps, 'height': frames.shape[1],
                'width': frames.shape[2], 'video_backend': 'pyav_explicit'}
    messages = [{'role': 'user', 'content': [
        {'type': 'video'}, {'type': 'text', 'text': prompt}]}]
    text = processor.apply_chat_template(messages, tokenize=False,
        add_generation_prompt=True, enable_thinking=False)
    return processor(text=[text], videos=[frames], return_tensors='pt',
        videos_kwargs={'video_metadata': [metadata], 'do_sample_frames': False,
                       'size': {'shortest_edge': 64*64,
                                'longest_edge': len(frames)*384*192}})


def infer(args):
    root = args.output.resolve()
    manifest = verify_baseline(root)
    model_path = args.model_path.resolve()
    model_manifest = json.loads((model_path / 'download_manifest.json').read_text())
    assert model_manifest['revision'] == MODEL_REVISION
    for item in model_manifest['files']:
        assert sha256(model_path / item['name']) == item['actual_sha256'], item['name']
    import torch
    import transformers
    from transformers import AutoProcessor, AutoModelForImageTextToText
    assert torch.cuda.is_available()
    torch.manual_seed(0)
    processor = AutoProcessor.from_pretrained(str(model_path), local_files_only=True)
    model = AutoModelForImageTextToText.from_pretrained(
        str(model_path), dtype=torch.bfloat16, device_map='cuda',
        attn_implementation='sdpa', local_files_only=True)
    model.eval()
    torch.set_grad_enabled(False)
    execution = {'at': now(), 'torch': torch.__version__, 'transformers': transformers.__version__,
                 'model_id': MODEL_ID, 'model_revision': MODEL_REVISION,
                 'download_manifest_sha256': sha256(model_path / 'download_manifest.json'),
                 'gpu': torch.cuda.get_device_name(0), 'dtype': 'bfloat16',
                 'training': False, 'optimizer_loaded': False, 'simulation_started': False,
                 'control_output_connected': False, 'model_class': type(model).__name__,
                 'input_source_video_sha256': VIDEO_SHA}
    save(root / 'execution.json', execution)
    processed = 0
    for clip in manifest['clips']:
        video = root / clip['path']
        assert sha256(video) == clip['sha256']
        prompt = prompt_for(clip)
        prompt_sha = hashlib.sha256(prompt.encode()).hexdigest()
        target = root / 'responses' / f"clip_{clip['id']:03d}.json"
        if target.exists():
            previous = json.loads(target.read_text())
            assert previous['clip_sha256'] == clip['sha256']
            assert previous['prompt_sha256'] == prompt_sha
            assert previous['model_revision'] == MODEL_REVISION
            continue
        if args.limit is not None and processed >= args.limit:
            break
        start = time.monotonic()
        inputs = replay_inputs(processor, video, prompt, manifest['sampling_fps'])
        inputs = inputs.to(model.device)
        inputs = {k: (v.to(torch.bfloat16) if torch.is_tensor(v) and v.is_floating_point() else v)
                  for k, v in inputs.items()}
        input_tokens = inputs['input_ids'].shape[-1]
        assert input_tokens + args.max_tokens < model.config.text_config.max_position_embeddings
        torch.cuda.reset_peak_memory_stats()
        with torch.inference_mode():
            generated = model.generate(**inputs, do_sample=False, max_new_tokens=args.max_tokens)
        suffix = generated[:, input_tokens:]
        text = processor.batch_decode(suffix, skip_special_tokens=True)[0].strip()
        answer, error = None, None
        try:
            answer = validate_answer(json.loads(text), clip['source_end_s']-clip['source_start_s'])
        except (ValueError, TypeError, KeyError) as exc:
            error = str(exc)
        response = {'at': now(), 'clip_id': clip['id'], 'clip_sha256': clip['sha256'],
                    'source_start_s': clip['source_start_s'], 'source_end_s': clip['source_end_s'],
                    'prompt': prompt, 'prompt_sha256': prompt_sha,
                    'model_id': MODEL_ID, 'model_revision': MODEL_REVISION,
                    'raw_output': text, 'answer': answer, 'schema_valid': error is None,
                    'validation_error': error, 'input_tokens': input_tokens,
                    'output_tokens': suffix.shape[-1], 'latency_s': time.monotonic()-start,
                    'peak_cuda_memory_bytes': torch.cuda.max_memory_allocated(),
                    'advisory_only': True, 'physical_acceptance': False}
        save(target, response)
        verify_baseline(root)
        processed += 1
        print(json.dumps({k: response[k] for k in ['clip_id', 'schema_valid', 'latency_s',
                                                 'input_tokens', 'output_tokens',
                                                 'peak_cuda_memory_bytes']}), flush=True)
        del inputs, generated, suffix
    responses = list((root / 'responses').glob('clip_*.json'))
    save(root / 'progress.json', {'at': now(), 'clips': len(manifest['clips']),
         'completed': len(responses), 'complete': len(responses) == len(manifest['clips']),
         'schema_valid': sum(json.loads(p.read_text())['schema_valid'] for p in responses),
         'source_video_and_frozen_policy_unchanged': True, 'control_output_connected': False})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('prepare')
    p.add_argument('--video', type=Path, required=True)
    p.add_argument('--policy', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--clip-seconds', type=int, default=20)
    p.add_argument('--fps', type=float, default=4.)
    p.add_argument('--ffmpeg', default='ffmpeg')
    p.add_argument('--ffprobe', default='ffprobe')
    p = sub.add_parser('infer')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--model-path', type=Path, required=True)
    p.add_argument('--limit', type=int)
    p.add_argument('--max-tokens', type=int, default=1536)
    args = parser.parse_args()
    (prepare if args.command == 'prepare' else infer)(args)


if __name__ == '__main__':
    main()
