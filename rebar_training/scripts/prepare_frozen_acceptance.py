"""Copy a verified fixed policy/source into a fresh evaluation campaign."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import shutil


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_sha(directory):
    digest = hashlib.sha256()
    for file in sorted(directory.rglob('*.py')):
        digest.update(str(file.relative_to(directory)).encode())
        digest.update(file.read_bytes())
    return digest.hexdigest()


def prepare(file, dry_run=False):
    plan = json.loads(file.read_text())
    repo = Path(plan['repository'])
    root = repo / 'rebar_training/artifacts/mobile_curriculum_20261020' / plan['campaign_name']
    gate = repo / plan['gate_input']
    proof = json.loads(gate.read_text())
    audit = repo / 'rebar_training/artifacts/mobile_curriculum_20261020/review_20261020/resume_0103_20261022/new_frozen_acceptance_seed_audit.json'
    assert sha(gate) == plan['prerequisite_11_gate_sha256']
    assert proof['passed'] and proof['episodes'] == proof['successes'] == 11
    assert not proof['failures'] and not proof['evidence_errors']
    assert sha(audit) == plan['seed_audit_sha256']
    seeds = [s for phase in plan['phases'] for s in phase['seeds']]
    assert seeds == json.loads(audit.read_text())['proposed_seeds']
    assert len(seeds) == len(set(seeds)) == 13
    assert not json.loads(audit.read_text())['overlap']
    assert source_sha(repo / plan['source_input']) == proof['source_sha256'] == plan['source_sha256']
    assert sha(repo / plan['checkpoint_input']) == proof['checkpoint_sha256'] == plan['checkpoint_sha256']
    assert not plan['new_training'] and not plan['optimizer_reset'] and not plan['new_policy_version']
    assert [p['name'] for p in plan['phases']] == ['tray', 'dock_clamp', 'full', 'demo']
    assert len(plan['phases'][2]['seeds']) * plan['phases'][2]['episodes_per_seed'] == 64
    if root.exists():
        assert sha(root / 'plan.json') == sha(file)
        assert source_sha(root / 'campaign_source') == plan['source_sha256']
        assert sha(root / 'frozen_policy.pt') == plan['checkpoint_sha256']
        return dict(root=str(root),state='already_prepared_no_overwrite')
    if dry_run:
        return dict(root=str(root),state='validated_not_created')
    root.mkdir(parents=True)
    shutil.copyfile(file, root / 'plan.json')
    shutil.copyfile(gate, root / 'candidate_11_gate_independently_verified.json')
    shutil.copyfile(audit, root / 'seed_audit.json')
    shutil.copytree(repo / plan['source_input'], root / 'campaign_source',
                    ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copyfile(repo / plan['checkpoint_input'], root / 'frozen_policy.pt')
    assert source_sha(root / 'campaign_source') == plan['source_sha256']
    assert sha(root / 'frozen_policy.pt') == plan['checkpoint_sha256']
    provenance = dict(created_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        plan_sha256=sha(root / 'plan.json'), source_sha256=plan['source_sha256'],
        checkpoint_sha256=plan['checkpoint_sha256'], counters=plan['retained_counters'],
        candidate_11_gate_sha256=sha(gate), seed_audit_sha256=sha(audit),
        launcher_sha256=sha(repo / 'rebar_training/scripts/launch_frozen_acceptance.py'),
        new_learning_updates=0, formal_full_gate_passed=False)
    (root / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
    (root / 'acceptance.status.json').write_text(json.dumps(dict(state='prepared',
        phase='tray', active_jobs=[], new_learning_updates=0, formal_full_gate_passed=False), indent=2) + '\n')
    return dict(root=str(root), state='prepared', **provenance)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    print(json.dumps(prepare(args.plan, args.dry_run)))
