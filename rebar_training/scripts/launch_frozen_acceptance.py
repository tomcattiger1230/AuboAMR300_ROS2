"""Launch at most two predeclared evaluation jobs, without training or retries.

This one-shot scheduler lives outside the frozen controller source. Promotion
requires an independently reviewed phase gate; repeated calls reuse job records.
"""
import argparse
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_sha(directory):
    digest = hashlib.sha256()
    for path in sorted(Path(directory).rglob('*.py')):
        digest.update(str(path.relative_to(directory)).encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def mission_args(plan, phase, seed, root):
    args = ['--until', phase['until'], '--randomization', 'expanded',
            '--residual-scale', '.08', '--seed', str(seed), '--episodes',
            str(phase['episodes_per_seed']), '--residual-checkpoint',
            str(root / 'frozen_policy.pt')]
    if phase.get('start_on_tray'):
        args.append('--start-on-tray')
    if phase.get('start_at_dock'):
        args.append('--start-at-dock')
    if phase.get('initial_joints'):
        args += ['--initial-joints', *map(str, phase['initial_joints'])]
    if phase['name'] == 'demo':
        args += ['--regression', '--video', str(root / 'full_policy_demo.mp4')]
    assert '--learn-residual' not in args and '--save-policy' not in args
    assert '--survey-from-packet' not in args
    if phase['name'] == 'full':
        assert not any(x in args for x in ['--start-on-tray', '--start-at-dock', '--regression'])
    return args


def require_previous_gate(root, plan, phase_name):
    names = [p['name'] for p in plan['phases']]
    index = names.index(phase_name)
    if not index:
        proof = json.loads((root / 'candidate_11_gate_independently_verified.json').read_text())
        assert proof['passed'] and proof['episodes'] == proof['successes'] == 11
        assert proof['failures'] == 0 and not proof['evidence_errors']
        assert proof['source_sha256'] == plan['source_sha256']
        assert proof['checkpoint_sha256'] == plan['checkpoint_sha256']
        return
    previous = plan['phases'][index - 1]
    review = json.loads((root / (previous['name'] + '_gate.independently_verified.json')).read_text())
    assert review['passed'] and review['gate']['passed'] and not review['gate']['evidence_errors']
    assert review['source_sha256'] == plan['source_sha256']
    assert review['checkpoint_sha256'] == plan['checkpoint_sha256']
    expected = {previous['name'] + '_holdout_' + str(seed) + '.json'
                for seed in previous['seeds']}
    assert set(review['report_sha256']) == expected
    assert all(sha(root / name) == value for name, value in review['report_sha256'].items())
    assert review['gate']['episodes'] == len(previous['seeds']) * previous['episodes_per_seed']


def inventory():
    groups = set()
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        try:
            args = directory.joinpath('cmdline').read_bytes().split(b'\0')
            if '\nState:\tZ' in directory.joinpath('status').read_text():
                continue
            if b'-m' in args and (b'rebar_training.mobile_mission_check' in args or
                    (b'rebar_training.mobile_jobs' in args and b'--run' in args)):
                groups.add(os.getpgid(int(directory.name)))
        except OSError:
            pass
    gpu = subprocess.check_output(['nvidia-smi', '--query-compute-apps=pid,used_memory',
                                   '--format=csv,noheader'], text=True).splitlines()
    return dict(groups=sorted(groups), gpu=[s for s in gpu if s.strip()])


def ensure_idle_hold(root):
    """Bounded, campaign-only idle inhibition; does not change power settings."""
    script = root / 'review_tools/hold_campaign_idle.py'
    script.parent.mkdir(exist_ok=True)
    if not script.exists():
        script.write_text("from pathlib import Path\nimport json,time,sys\n"
            "root=Path(sys.argv[1]);deadline=time.monotonic()+48*3600\n"
            "while time.monotonic()<deadline:\n"
            " state=json.loads((root/'acceptance.status.json').read_text()).get('state')\n"
            " if state in ('paused_user','needs_analysis','completed'):break\n"
            " time.sleep(10)\n")
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():
            continue
        try:
            if str(script).encode() in p.joinpath('cmdline').read_bytes().split(b'\0'):
                return dict(pid=int(p.name), pgid=os.getpgid(int(p.name)), script=str(script))
        except OSError:
            pass
    command = ['systemd-inhibit', '--what=idle', '--mode=block',
               '--who=rebar-frozen-acceptance', '--why=Finite frozen-policy evaluation',
               'python3', str(script), str(root)]
    with (root / 'idle_inhibitor.log').open('a') as log:
        process = subprocess.Popen(command, start_new_session=True, stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=subprocess.STDOUT)
    time.sleep(.1)
    if process.poll() is not None:
        raise RuntimeError('Campaign idle inhibitor failed; inspect its log')
    return dict(pid=process.pid, pgid=process.pid, script=str(script))


def run(root, phase_name, dry_run=False):
    root = Path(root).resolve()
    plan = json.loads((root / 'plan.json').read_text())
    provenance = json.loads((root / 'provenance.json').read_text())
    assert sha(root / 'plan.json') == provenance['plan_sha256']
    assert source_sha(root / 'campaign_source') == plan['source_sha256']
    assert sha(root / 'frozen_policy.pt') == plan['checkpoint_sha256']
    repo = Path(plan['repository'])
    assert sha(repo / 'rebar_training/artifacts/visual_control_20261019/segment_v1.pt') == plan['detector_sha256']
    assert sha(repo / 'rebar_training/artifacts/assets_mobile_v2/manifest.json') == plan['asset_manifest_sha256']
    assert sha(repo / 'rebar_training/config/vision_runtime.json') == plan['vision_runtime_sha256']
    require_previous_gate(root, plan, phase_name)
    phase = next(p for p in plan['phases'] if p['name'] == phase_name)
    if dry_run:
        return dict(dry_run=True, phase=phase_name,
                    commands=[mission_args(plan, phase, s, root) for s in phase['seeds']],
                    inventory=inventory())
    lock = (root / 'launch.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    status_path = root / 'acceptance.status.json'
    state = json.loads(status_path.read_text()) if status_path.exists() else {}
    if state.get('state') in ('paused_user', 'needs_analysis'):
        raise RuntimeError('Campaign is paused or requires analysis; do not dispatch')
    launched = []
    # A known physical failure blocks further dispatch, including queued seeds.
    for path in root.glob('*.progress.json'):
        data = json.loads(path.read_text())
        if data['successes'] != data['episodes']:
            raise RuntimeError('Preserve and analyze physical failure: ' + path.name)
    for seed in phase['seeds']:
        for previous_status in root.glob('*.job.status.json'):
            if json.loads(previous_status.read_text())['state'] == 'failed':
                raise RuntimeError('A prior job failed; stop further dispatch')
        for progress in root.glob('*.progress.json'):
            data = json.loads(progress.read_text())
            if data['successes'] != data['episodes']:
                raise RuntimeError('A physical failure requires analysis before further dispatch')
        name = phase_name + '_holdout_' + str(seed)
        job_file = root / (name + '.job.json')
        if job_file.exists():
            job_state = json.loads(job_file.with_suffix('.status.json').read_text())
            if job_state['state'] == 'failed':
                raise RuntimeError('Existing failed job; no automatic retry: ' + name)
            continue
        current = inventory()
        if len(current['groups']) >= 2 or len(current['gpu']) >= 2:
            break
        if plan.get('hold_idle_between_gates'):
            state['idle_inhibitor'] = ensure_idle_hold(root)
        env = {**os.environ, 'PYTHONPATH': str(root / 'campaign_source'),
               'REBAR_REPO_ROOT': str(repo)}
        argv = [str(repo / 'runtime_python_0103'), '-m', 'rebar_training.mobile_jobs',
                '--python', str(repo / 'runtime_python_0103'), '--artifact-dir', str(root),
                '--name', name, '--', *mission_args(plan, phase, seed, root)]
        result = json.loads(subprocess.check_output(argv, cwd=repo, env=env, text=True))
        assert result['source_sha256'] == plan['source_sha256']
        launched.append(dict(name=name, **result))
    active = []
    for path in root.glob('*.job.status.json'):
        value = json.loads(path.read_text())
        if value['state'] in ('queued', 'running'):
            active.append(dict(name=path.name.removesuffix('.job.status.json'), **value))
    pending = any(not (root / (phase_name + '_holdout_' + str(s) + '.job.json')).exists()
                  for s in phase['seeds'])
    state.update(state=('evaluation_running' if active else 'waiting_for_simulation_capacity'
                        if pending else 'awaiting_independent_phase_gate_review'),
                 phase=phase_name, active_jobs=active, last_launch=launched,
                 updated_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                 new_learning_updates=0, formal_full_gate_passed=False)
    temporary = status_path.with_suffix('.tmp')
    temporary.write_text(json.dumps(state, indent=2) + '\n')
    temporary.replace(status_path)
    return dict(phase=phase_name, launched=launched, active_jobs=active, inventory=inventory())


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path)
    parser.add_argument('--phase', choices=['tray', 'dock_clamp', 'full', 'demo'], required=True)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    print(json.dumps(run(args.root, args.phase, args.dry_run)))
