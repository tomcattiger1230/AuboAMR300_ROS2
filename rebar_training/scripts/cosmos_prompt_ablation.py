"""Prepare/assess frozen-video prompt contrasts, without importing a model or simulator."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import re
import shutil

from cosmos_reasoner_replay import MODEL_REVISION, VIDEO_SHA, sha256
from evaluate_cosmos_edge_replay import save


def prepare(previous, output):
    if output.exists():
        raise ValueError('New experiment directory required; preserve previous outcomes')
    old = json.loads((previous / 'fixed_resolution_plan.json').read_text())
    templates = {c['id'].split('_')[3]: c for c in old['cases'] if c['fps'] == 4}
    recognition = json.loads((previous / 'plan.json').read_text())
    wrist = {c['id'].split('_')[1]: c for c in recognition['cases']
             if c['id'] in ('rebar_e_4fps', 'rebar_f_4fps', 'rebar_g_4fps')}
    cases, references = [], {}
    targets = [('a', 'rotation', 'yes', False), ('b', 'translation', 'yes', False),
               ('c', 'translation', 'no', False), ('c', 'rotation', 'no', False),
               ('b', 'translation', 'no', True), ('b', 'rotation', 'no', True)]
    for clip, axis, expected, duplicate in targets:
        stem = f'{clip}_{axis}' + ('_duplicate' if duplicate else '')
        templates_by_format = {
            'baseline': templates[clip]['prompt'],
            'single_json': (
                'Observe the wheeled base supporting the robot arm, separately from the arm itself. '
                + ('Does that base change position' if axis == 'translation' else 'Does that base change heading')
                + ' relative to the stationary surroundings during this clip? '
                + f'Return one JSON object with only the key "{axis}" and a value of "yes", "no", or "uncertain". '
                + 'Use uncertain if the visible evidence is insufficient.'),
            'single_prose': (
                'Observe the wheeled base supporting the robot arm, separately from the arm itself. '
                + ('Does that base change position' if axis == 'translation' else 'Does that base change heading')
                + ' relative to the stationary surroundings during this clip? '
                + 'Begin with Yes, No, or Uncertain, then give one sentence of visible evidence. '
                + 'Use Uncertain if the visible evidence is insufficient.')}
        for mode, prompt in templates_by_format.items():
            c = deepcopy(templates[clip])
            c.update(id=f'{stem}_{mode}', prompt=prompt, duplicate=duplicate,
                     task='motion_contrast', target_axis=axis, output_contract=mode)
            cases.append(c)
            references[c['id']] = {'expected': expected, 'axis': axis,
                                  'reference': 'exact-frame review from prior experiment; duplicate is static by construction'}
    questions = [
        ('e', 'rod_identity', 'Describe any elongated object actually visible and its texture. '
         'If none is visible say none. Do not infer what the object is used for.'),
        ('f', 'gripper_identity', 'Describe the visible foreground mechanical parts and whether opposed jaws '
         'are distinguishable. Do not infer a secure grasp or contact force.'),
        ('f', 'holding_relationship', 'What visible spatial relationship exists between the foreground jaws '
         'and any elongated object? Distinguish a visible gap from contact, and state uncertainty when occluded. '
         'Do not infer holding force or task success.'),
        ('g', 'event_time', 'Describe any change in separation between the foreground mechanical parts during '
         'this clip. If an event is visible, give the clip-relative time interval in seconds when it first '
         'becomes visible; otherwise say no observable event. Do not infer force or task success.')]
    for clip, question, prompt in questions:
        for mode in ('prose', 'json'):
            c = deepcopy(wrist[clip])
            c.update(id=f'{clip}_{question}_{mode}', task=question, output_contract=mode,
                     fixed_video_size=[384, 256], max_video_pixels_total=None,
                     prompt=prompt + (' Return concise prose.' if mode == 'prose' else
                                     ' Return one JSON object with observation and uncertainty fields.'))
            cases.append(c)
    output.mkdir(parents=True)
    media = {}
    for c in cases:
        c.update(thinking=False, max_new_tokens=768, official=False)
        path = previous / 'inputs' / c['media']
        if sha256(path) != c['media_sha256']:
            raise ValueError('Original input SHA mismatch')
        dest = output / 'inputs' / c['media']
        if not dest.exists():
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, dest)
        media[c['media']] = {'sha256': sha256(dest), 'bytes': dest.stat().st_size}
    plan = dict(schema='cosmos_edge_prompt_ablation_v1', model_revision=MODEL_REVISION,
                source_video_sha256=VIDEO_SHA, input_files=media, cases=cases,
                training=False, simulation=False, policy_optimizer_load=False,
                intervention='question scope/output format only within each identical-input group',
                independent_tracker_used=False)
    save(output / 'plan.json', plan)
    save(output / 'assessment_reference.json', dict(
        model_answers_read=False, never_supplied_to_model=True, motion=references,
        semantic_scope='Motion labels only; recognition/event-time require separate human review',
        event_reference={'g': {'source_interval_s': [312, 316],
                               'first_visible_change_relative_s': [1, 2],
                               'event': 'foreground jaw gap widening; force not observable'}}))
    return {'cases': len(cases), 'root': str(output), 'plan_sha256': sha256(output / 'plan.json')}


def strict_motion(answer, axis, mode):
    if mode == 'single_prose':
        found = re.match(r'^\s*(yes|no|uncertain)\b', answer, re.I)
        return found.group(1).lower() if found else None
    try:
        d = json.loads(answer)
    except (ValueError, TypeError):
        return None
    if not isinstance(d, dict) or d.get(axis) not in ('yes', 'no', 'uncertain'):
        return None
    if mode == 'single_json' and set(d) != {axis}:
        return None
    if mode == 'baseline' and (set(d) != {'translation', 'rotation', 'visible_evidence'}
            or any(d.get(k) not in ('yes', 'no', 'uncertain') for k in ('translation', 'rotation'))
            or not isinstance(d.get('visible_evidence'), str)):
        return None
    return d[axis]


def assess(root):
    reference = json.loads((root / 'assessment_reference.json').read_text())
    rows = []
    for path in sorted((root / 'outputs/responses').glob('*.json')):
        r = json.loads(path.read_text()); c = r['case']; answer = r.get('final_answer') or ''
        row = dict(id=c['id'], status=r['status'], answer=answer, semantic_passed=None)
        if c['id'] in reference['motion']:
            ref = reference['motion'][c['id']]
            label = strict_motion(answer, ref['axis'], c['output_contract'])
            row.update(expected=ref['expected'], label=label, contract_passed=label is not None,
                       semantic_passed=r['status'] == 'complete' and label == ref['expected'])
        rows.append(row)
    paired = []
    plan = json.loads((root / 'plan.json').read_text())
    for target in reference['motion']:
        if not target.endswith('_baseline'):
            continue
        stem = target[:-len('_baseline')]
        members = [json.loads((root / 'outputs/responses' / (stem+'_'+mode+'.json')).read_text())
                   for mode in ('baseline', 'single_json', 'single_prose')]
        paired.append(dict(group=stem, pixel_tensors_identical=all(
            m.get('pixel_tensors') == members[0].get('pixel_tensors') and bool(m.get('pixel_tensors'))
            for m in members)))
    return dict(cases_expected=len(plan['cases']), cases_retained=len(rows), rows=rows,
                paired_inputs=paired, raw_outputs_preserved=True, semantic_acceptance_passed=False,
                note='Strict enum scoring does not recover labels from invalid/ambiguous coordinates; manual recognition review pending')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('operation', choices=('prepare', 'assess'))
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--previous', type=Path)
    args = p.parse_args()
    if args.operation == 'prepare':
        print(json.dumps(prepare(args.previous, args.root)))
    else:
        save(args.root / 'assessment.json', assess(args.root))
