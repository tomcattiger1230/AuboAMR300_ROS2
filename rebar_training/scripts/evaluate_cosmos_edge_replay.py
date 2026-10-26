"""Frozen offline Edge evaluation; imports no simulator, policy, or optimizer."""
import argparse
import hashlib
import json
import math
import time
import traceback
from pathlib import Path

from cosmos_generation_audit import GenerationScoreAudit, generation_receipt
from cosmos_reasoner_replay import MODEL_REVISION, POLICY_SHA, VIDEO_SHA, sha256


def sample_indices(frame_count, source_fps, target_fps):
    if frame_count < 1 or not 0 < target_fps <= source_fps:
        raise ValueError('Sampling must select existing frames, without upsampling')
    count = math.ceil(frame_count / source_fps * target_fps - 1e-9)
    return sorted({min(frame_count - 1, math.floor(i * source_fps / target_fps + .5))
                   for i in range(count)})


def first_json(text):
    decoder = json.JSONDecoder()
    for index, char in enumerate(text):
        if char in '[{':
            try:
                return decoder.raw_decode(text[index:])[0]
            except ValueError:
                pass
    return None


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as stream:
        json.dump(data, stream, indent=2, ensure_ascii=False)
        stream.write('\n')


def frozen_files(base):
    acceptance = base / 'rebar_training/artifacts/mobile_curriculum_20261020/source_return_v2_frozen_acceptance_20261022'
    result = {name: sha256(acceptance / name) for name in ['frozen_policy.pt', 'full_policy_demo.mp4']}
    assert result == {'frozen_policy.pt': POLICY_SHA, 'full_policy_demo.mp4': VIDEO_SHA}
    model = base / '.cosmos_reasoner_replay/models' / MODEL_REVISION
    manifest = json.loads((model / 'download_manifest.json').read_text())
    result['cosmos_model_files'] = {item['name']: sha256(model / item['name']) for item in manifest['files']}
    assert all(result['cosmos_model_files'][item['name']] == item['actual_sha256'] for item in manifest['files'])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import av
    import numpy as np
    import torch
    import transformers
    from PIL import Image, ImageOps

    base = Path(__file__).resolve().parents[2]
    root = args.plan.resolve().parent
    output = args.output.resolve()
    assert not output.exists(), 'Preserve all previous outcomes; never reuse an output directory'
    output.mkdir(parents=True)
    plan = json.loads(args.plan.read_text())
    assert plan['model_revision'] == MODEL_REVISION and plan['source_video_sha256'] == VIDEO_SHA
    save(output / 'plan.json', plan)
    before = frozen_files(base)
    for relative, expected in plan['input_files'].items():
        path = root / 'inputs' / relative
        assert sha256(path) == expected['sha256'], relative
    save(output / 'frozen_before.json', before)

    assert torch.cuda.is_available()
    model_path = base / '.cosmos_reasoner_replay/models' / MODEL_REVISION
    processor = transformers.AutoProcessor.from_pretrained(str(model_path), local_files_only=True)
    model, loading = transformers.AutoModelForImageTextToText.from_pretrained(
        str(model_path), dtype=torch.bfloat16, attn_implementation='sdpa',
        local_files_only=True, output_loading_info=True)
    save(output / 'loading_info.json', {key: list(value) for key, value in loading.items()})
    assert all(not value for value in loading.values())
    model.to('cuda').eval()
    torch.set_grad_enabled(False)
    save(output / 'execution.json', dict(
        at_unix=time.time(), script_sha256=sha256(Path(__file__)), plan_sha256=sha256(args.plan),
        model_revision=MODEL_REVISION, model_class=type(model).__name__, device='cuda',
        dtype='bfloat16', attention='sdpa', torch=torch.__version__, transformers=transformers.__version__,
        parameters=sum(v.numel() for v in model.parameters()), policy_loaded=False,
        optimizer_loaded=False, training=False, simulation=False, tracker_labels_supplied=False,
        generation_config=model.generation_config.to_dict()))
    outcomes = []
    for case in plan['cases']:
        torch.manual_seed(0)
        torch.cuda.reset_peak_memory_stats()
        result = dict(case=case, at_unix=time.time(), status='started')
        x = None
        ids = []
        audit = None
        started = time.monotonic()
        try:
            path = root / 'inputs' / case['media']
            assert sha256(path) == case['media_sha256']
            content = [{'type': case['kind'], 'path': str(path)}, {'type': 'text', 'text': case['prompt']}]
            template = dict(add_generation_prompt=True)
            if case['thinking'] is not None:
                template['enable_thinking'] = case['thinking']
            messages = [{'role': 'user', 'content': content}]
            if case.get('official_api_reference'):
                # The exact public Transformers notebook API and native processor defaults.
                kw = dict(tokenize=True, return_dict=True, return_tensors='pt', **template)
                if case['kind'] == 'video':
                    kw['fps'] = case['fps']
                x = processor.apply_chat_template(messages, **kw)
                result['preprocessing'] = dict(api='official_apply_chat_template', kwargs=kw)
            else:
                text = processor.apply_chat_template(messages, tokenize=False, **template)
                result['prompt_rendered'] = text
                if case['kind'] == 'image':
                    image = Image.open(path).convert('RGB')
                    if case.get('blank'):
                        image = Image.new('RGB', image.size, (128, 128, 128))
                    target = output / 'actual_inputs' / (case['id'] + '.png')
                    target.parent.mkdir(exist_ok=True)
                    image.save(target)
                    budget = case.get('max_image_pixels')
                    kw = {'size': {'shortest_edge': 4096, 'longest_edge': budget}} if budget else {}
                    x = processor(text=[text], images=[image], return_tensors='pt', images_kwargs=kw)
                    result['preprocessing'] = dict(api='explicit_image', actual_size=list(image.size),
                                                   sha256=sha256(target), kwargs=kw)
                else:
                    with av.open(str(path)) as container:
                        fps = float(container.streams.video[0].average_rate)
                        frames = [frame.to_image().convert('RGB') for frame in container.decode(video=0)]
                    indices = sample_indices(len(frames), fps, case['fps'])
                    selected = [frames[i] for i in indices]
                    if case.get('roi'):
                        selected = [im.crop(case['roi']) for im in selected]
                    if case.get('duplicate'):
                        selected = [selected[0].copy() for _ in selected]
                    if case.get('fixed_video_size'):
                        selected = [ImageOps.pad(im, tuple(case['fixed_video_size']),
                                    method=Image.Resampling.LANCZOS, color=(220, 220, 220))
                                    for im in selected]
                    hashes = []
                    for index, image in enumerate(selected):
                        target = output / 'actual_inputs' / case['id'] / f'{index:03d}.png'
                        target.parent.mkdir(parents=True, exist_ok=True)
                        image.save(target)
                        hashes.append(dict(path=str(target.relative_to(output)), sha256=sha256(target)))
                    metadata = dict(total_num_frames=len(frames), fps=fps, frames_indices=indices,
                                    duration=len(frames)/fps, height=selected[0].height,
                                    width=selected[0].width, video_backend='pyav_explicit')
                    kw = dict(video_metadata=[metadata], do_sample_frames=False)
                    if case.get('fixed_video_size'):
                        kw['do_resize'] = False
                    if case['max_video_pixels_total'] is not None:
                        kw['size'] = {'shortest_edge': 4096, 'longest_edge': case['max_video_pixels_total']}
                    x = processor(text=[text], videos=[np.stack([np.asarray(im) for im in selected])],
                                  return_tensors='pt', videos_kwargs=kw)
                    result['preprocessing'] = dict(api='explicit_video', metadata=metadata,
                        actual_sample_times_s=[i/fps for i in indices], frames=hashes, kwargs=kw)
            grid = x.get('video_grid_thw', x.get('image_grid_thw'))
            patches = int(grid.prod(-1).sum().item())
            tokens = int(x['input_ids'].shape[-1])
            result['prefill'] = dict(grid=grid.tolist(), vision_patches=patches, language_tokens=tokens)
            save(output / 'prefill' / (case['id'] + '.json'), result['prefill'])
            if patches > case['max_vision_patches'] or tokens > case['max_language_tokens']:
                raise ValueError('Predeclared case prefill budget exceeded; inference not started')
            result['pixel_tensors'] = {key: dict(shape=list(value.shape), finite=bool(value.isfinite().all()),
                sha256=hashlib.sha256(value.contiguous().numpy().tobytes()).hexdigest())
                for key, value in x.items() if value.is_floating_point()}
            assert all(value['finite'] for value in result['pixel_tensors'].values())
            x = {key: value.to('cuda', torch.bfloat16) if value.is_floating_point() else value.to('cuda')
                 for key, value in x.items()}
            audit = GenerationScoreAudit(tokens)
            generated_at = time.monotonic()
            with torch.inference_mode():
                generated = model.generate(**x, do_sample=False, max_new_tokens=case['max_new_tokens'],
                                           max_time=60, logits_processor=[audit])
            ids = generated[0, tokens:].tolist()
            elapsed = time.monotonic() - generated_at
            receipt = generation_receipt(ids, processor.tokenizer,
                bool(case['thinking']), case['max_new_tokens'], elapsed, 60)
            result.update(receipt)
            result['generation_latency_s'] = elapsed
            result['parsed_json'] = first_json(result['final_answer'] or '')
            result['status'] = 'complete' if receipt['answer_complete'] else 'incomplete'
            del generated
        except Exception as exc:
            result.update(status='error', error=repr(exc), traceback=traceback.format_exc())
        finally:
            result['score_audit'] = audit.steps if audit else []
            result['elapsed_s'] = time.monotonic() - started
            result['peak_cuda_allocated_bytes'] = torch.cuda.max_memory_allocated()
            save(output / 'responses' / (case['id'] + '.json'), result)
            outcomes.append(dict(id=case['id'], status=result['status'], error=result.get('error')))
            print(json.dumps(dict(id=case['id'], status=result['status'], answer=result.get('final_answer'),
                                  error=result.get('error')), ensure_ascii=False), flush=True)
            del x
            torch.cuda.empty_cache()
    after = frozen_files(base)
    assert before == after
    save(output / 'frozen_after.json', dict(unchanged=True, files=after))
    save(output / 'summary.json', dict(outcomes=outcomes, all_cases_attempted=True,
        runtime_complete=sum(v['status']=='complete' for v in outcomes), total_cases=len(outcomes),
        semantic_acceptance_passed=False, policy_optimizer_unchanged=True,
        training=False, simulation=False))


if __name__ == '__main__':
    main()
