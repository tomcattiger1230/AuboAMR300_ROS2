"""Resume only the two user-interrupted frozen jobs, retaining prior evidence."""
import argparse
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

from launch_frozen_acceptance import inventory, ensure_idle_hold, source_sha, require_previous_gate


NAMES = ('full_holdout_9301042', 'full_holdout_9311042')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def write(path, value):
    temporary = path.with_suffix('.resume.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def check_command(job, name):
    args = job['command']
    forbidden = ('--learn-residual', '--save-policy', '--regression', '--start-at-dock',
                 '--start-on-tray', '--visual-hint-packet', '--survey-from-packet')
    assert not any(a == flag or a.startswith(flag + '=') for a in args for flag in forbidden)
    assert args[args.index('--seed') + 1] == name.rsplit('_', 1)[1]
    assert args[args.index('--episodes') + 1] == '8'
    assert args[args.index('--until') + 1] == 'clamp'


def run(root, action, name=None):
    root = Path(root).resolve()
    with (root / 'launch.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return run_locked(root, action, name)


def run_locked(root, action, name=None):
    plan = read(root / 'plan.json')
    auth_path = root / 'resume_authorization_0103_20261023.json'
    auth = read(auth_path)
    assert auth['user_authorization'] == '回到同一个网络 继续训练'
    assert auth['new_learning_updates'] == 0 and not auth['optimizer_reset']
    assert not auth['control_source_change'] and not plan['new_training']
    assert not plan['optimizer_reset'] and plan['max_simulations'] == 2
    assert plan['retained_counters'] == dict(updates=466, actor_updates=170, policy_steps=36602)
    assert auth['plan_sha256'] == sha(root / 'plan.json')
    assert auth['source_sha256'] == plan['source_sha256'] == source_sha(root / 'campaign_source')
    assert auth['policy_optimizer_sha256'] == plan['checkpoint_sha256'] == sha(root / 'frozen_policy.pt')
    assert all(sha(root / n) == h for n, h in auth['complete_group_report_sha256'].items())
    repo = Path(plan['repository'])
    for relative, key in [('artifacts/visual_control_20261019/segment_v1.pt', 'detector_sha256'),
                          ('artifacts/assets_mobile_v2/manifest.json', 'asset_manifest_sha256'),
                          ('config/vision_runtime.json', 'vision_runtime_sha256')]:
        assert sha(repo / 'rebar_training' / relative) == plan[key]
    require_previous_gate(root, plan, 'full')
    pause_path = root / 'pause_user.latest.json'
    pause = read(pause_path)
    assert datetime.datetime.fromisoformat(auth['at']) > datetime.datetime.fromisoformat(pause['at'])
    receipt_path = root / 'resume_after_user_pause_20261023.receipt.json'
    if action == 'prepare':
        if receipt_path.exists():
            receipt = read(receipt_path)
            assert receipt['prepared'] and receipt['authorization_sha256'] == sha(auth_path)
            return receipt
        current = inventory()
        assert not current['groups'] and not current['gpu'], current
        assert read(root / 'acceptance.status.json')['state'] == 'paused_user'
        for item in NAMES:
            assert read(root / (item + '.job.status.json'))['state'] == 'paused_user'
            assert not (root / (item + '.json')).exists()
            progress = root / (item + '.progress.json')
            if progress.exists():
                data = read(progress)
                assert data['episodes'] == data['successes'] == 6
                assert item == NAMES[1]
            check_command(read(root / (item + '.job.json')), item)
        archive = root / 'interrupted_attempts' / 'user_pause_20261023_002631'
        archive.mkdir(parents=True, exist_ok=False)
        receipt = dict(prepared=False, authorization_sha256=sha(auth_path),
                       pause_sha256=sha(pause_path), archive=str(archive), launched={},
                       job_sha256={n: sha(root / (n + '.job.json')) for n in NAMES},
                       retained_complete_native_episodes=48, archived_partial_episodes=6,
                       new_learning_updates=0, optimizer_reset=False, policy_version_changed=False)
        write(receipt_path, receipt)
        for item in NAMES:
            shutil.copytree(root / 'source_snapshots' / item, archive / 'source_snapshots' / item)
            for path in sorted(root.glob(item + '*')):
                target = archive / path.name
                if path.name == item + '.job.json':
                    shutil.copy2(path, target)
                elif path.is_file() or path.name == item:
                    assert not target.exists()
                    path.rename(target)
        manifest = {str(p.relative_to(archive)): sha(p)
                    for p in archive.rglob('*') if p.is_file()}
        write(archive / 'files.sha256.json', manifest)
        receipt.update(prepared=True, archive_files=len(manifest),
                       archive_manifest_sha256=sha(archive / 'files.sha256.json'))
        write(receipt_path, receipt)
        state = read(root / 'acceptance.status.json')
        state.update(state='resume_authorized_waiting_startup', active_jobs=[],
                     user_authorized_resume=True, requires_explicit_resume=False,
                     resume_receipt=str(receipt_path), user_pause_reason=None)
        write(root / 'acceptance.status.json', state)
        return receipt
    receipt = read(receipt_path)
    assert receipt['prepared'] and receipt['authorization_sha256'] == sha(auth_path)
    assert receipt['pause_sha256'] == sha(pause_path), 'A newer user pause supersedes this resume'
    state = read(root / 'acceptance.status.json')
    assert state['state'] not in ('paused_user', 'needs_analysis')
    if name in receipt['launched']:
        return dict(already_launched=receipt['launched'][name])
    assert name in NAMES and not (root / (name + '.log')).exists()
    assert not (root / (name + '.progress.json')).exists()
    assert not (root / (name + '.json')).exists()
    if name == NAMES[1]:
        assert NAMES[0] in receipt['launched']
        ready = False
        for line in reversed((root / (NAMES[0] + '.log')).read_text(errors='replace')[-30000:].splitlines()):
            try:
                trace = json.loads(line)
            except ValueError:
                continue
            if 'step' in trace:
                ready = trace.get('step', 0) >= 60 and trace.get('valid') and trace.get('observation_ready')
                break
        assert ready, 'Start the second job only after the first has fresh camera observations'
    current = inventory()
    assert len(current['groups']) < 2 and len(current['gpu']) < 2, current
    job_path = root / (name + '.job.json')
    assert sha(job_path) == receipt['job_sha256'][name], 'The frozen job command changed'
    job = read(job_path)
    check_command(job, name)
    assert source_sha(Path(job['source_snapshot'])) == plan['source_sha256']
    state['idle_inhibitor'] = ensure_idle_hold(root)
    write(root / (name + '.job.status.json'), dict(state='queued', job=str(job_path)))
    process = subprocess.Popen([str(repo / 'runtime_python_0103'), '-m', 'rebar_training.mobile_jobs',
                                '--run', str(job_path)], cwd=repo,
                               env={**os.environ, 'PYTHONPATH': job['source_snapshot']},
                               start_new_session=True, stdin=subprocess.DEVNULL,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    result = dict(name=name, runner_pid=process.pid, source_sha256=job['source_sha256'],
                  at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  prior_attempt_archive=receipt['archive'], physical_state_resumed=False)
    receipt['launched'][name] = result
    write(receipt_path, receipt)
    state.update(state='evaluation_running', phase='full', last_resume_launch=result,
                 user_authorized_resume=True, requires_explicit_resume=False,
                 new_learning_updates=0, formal_full_gate_passed=False)
    write(root / 'acceptance.status.json', state)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path)
    parser.add_argument('--action', required=True, choices=['prepare', 'launch'])
    parser.add_argument('--name', choices=NAMES)
    args = parser.parse_args()
    if args.action == 'launch' and not args.name:
        parser.error('--name is required for launch')
    print(json.dumps(run(args.root, args.action, args.name)))
