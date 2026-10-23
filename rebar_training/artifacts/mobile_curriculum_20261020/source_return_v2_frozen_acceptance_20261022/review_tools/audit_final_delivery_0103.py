"""Mac-only read-only audit of frozen delivery artifacts on both hosts.

Uses existing raw manifests as expectations, then rehashes every listed file.
Does not start, stop, train, or alter a simulation or original evidence.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import os
import shlex
import signal
import subprocess

BASE = Path('/Users/weiluo/Develop/github/AuboAMR300_ROS2')
REL = Path('rebar_training/artifacts/mobile_curriculum_20261020/source_return_v2_frozen_acceptance_20261022')
ROOT = BASE / REL
REMOTE = '/home/arnold/Develop/rebar-training-resume-0103-20261022'
HOST = 'arnold@192.168.0.103'
parser = argparse.ArgumentParser()
parser.add_argument('--include-final-documents', action='store_true')
args = parser.parse_args()


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


manifest = {}
counts = {}


def add(path, expected=None):
    path = path.resolve()
    assert path.is_relative_to(BASE) and path.is_file(), path
    actual = digest(path)
    if expected is not None:
        assert actual == expected, str(path)
    name = str(path.relative_to(BASE))
    assert name not in manifest or manifest[name] == actual
    manifest[name] = actual


plan = json.loads((ROOT / 'plan.json').read_text())
for phase in plan['phases']:
    count = 0
    for seed in phase['seeds']:
        proof_path = ROOT / (phase['name'] + '_holdout_' + str(seed) + '.all_files.both_hosts_verified.json')
        proof = json.loads(proof_path.read_text())
        assert not proof['mismatches']
        for name, expected in proof['files'].items():
            add(ROOT / name, expected)
        count += len(proof['files'])
        add(proof_path)
    counts[phase['name']] = count
assert counts == {'tray': 419, 'dock_clamp': 419, 'full': 7542, 'demo': 172}, counts

candidate = BASE / 'rebar_training/artifacts/mobile_curriculum_20261020/review_20261020/source_return_candidate_v2_regression_resume_0103_20261022_run2'
counts['development'] = 0
for suffix in ['reset8', 'reset2', 'supported_outage']:
    path = candidate / ('completed_' + suffix + '_all_files.both_hosts_verified.json')
    proof = json.loads(path.read_text())
    assert not proof['mismatches']
    for name, expected in proof['files'].items():
        add(candidate / name, expected)
    counts['development'] += len(proof['files'])
    add(path)
assert counts['development'] == 1557

archive = ROOT / 'interrupted_attempts/user_pause_20261023_002631'
archived = json.loads((archive / 'files.sha256.json').read_text())
for name, expected in archived.items():
    add(archive / name, expected)
add(archive / 'files.sha256.json')
counts['interrupted_archive'] = len(archived)
assert counts['interrupted_archive'] == 883

for name in ['frozen_policy.pt', 'plan.json', 'provenance.json', 'seed_audit.json',
             'candidate_11_gate_independently_verified.json',
             'tray_gate.independently_verified.json', 'tray_gate.json',
             'dock_clamp_gate.independently_verified.json', 'dock_clamp_gate.json',
             'full_gate.independently_verified.json', 'full_gate.json',
             'completed_full_groups.latest_independently_verified.json',
             'completed_groups_full_gate.json',
             'frozen_model_optimizer_final_cpu_verified_20261024.json',
             'demo_independently_verified_20261024.json', 'demo_video_validation_notes_20261024.json',
             'full_policy_demo.launch_20261024.receipt.json', 'demo_preflight_20261024.json',
             'resume_interrupted_archive_20261023.both_hosts_verified.json',
             'network_unavailable_20261024_0030.json',
             'network_unavailable_20261024_0030.metadata_sync_pending_history.json',
             'network_recovered_demo_completed_20261024.json', 'full_policy_demo.mp4']:
    add(ROOT / name)
for directory in ['campaign_source', 'demo_video_review_20261024']:
    for path in sorted((ROOT / directory).rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            add(path)
for relative in ['rebar_training/config/source_return_v2_frozen_acceptance_20261022.json',
                 'rebar_training/config/vision_runtime.json',
                 'rebar_training/artifacts/visual_control_20261019/segment_v1.pt',
                 'rebar_training/scripts/run_isaac_python.sh']:
    add(BASE / relative)
assets = BASE / 'rebar_training/artifacts/assets_mobile_v2'
for path in sorted(assets.iterdir()):
    if path.is_file():
        add(path)
asset_manifest = json.loads((assets / 'manifest.json').read_text())
for name, expected in asset_manifest['source_sha256'].items():
    add(BASE / name, expected)
add(ROOT / 'review_tools/audit_final_delivery_0103.py')
assert manifest[str(REL / 'frozen_policy.pt')] == plan['checkpoint_sha256']
assert manifest['rebar_training/artifacts/visual_control_20261019/segment_v1.pt'] == plan['detector_sha256']
assert manifest['rebar_training/config/vision_runtime.json'] == plan['vision_runtime_sha256']
assert manifest['rebar_training/artifacts/assets_mobile_v2/manifest.json'] == plan['asset_manifest_sha256']

if args.include_final_documents:
    for relative in ['rebar_training/README.md', 'rebar_training/TRAINING_HANDOFF.md',
                     'rebar_training/reports/source_return_frozen_acceptance_0103_20261022.json',
                     'rebar_training/artifacts/mobile_curriculum_20261020/review_20261020/status.json']:
        add(BASE / relative)
    for name in ['acceptance.status.json', 'delivery_complete_20261024.json',
                 'campaign_completed_process_inventory_20261024.json']:
        add(ROOT / name)

code = '''import pathlib,hashlib,json,sys
r=pathlib.Path(ROOT)
m=json.load(sys.stdin)
errors=[]
for name,expected in m.items():
 p=r/name
 if not p.is_file():
  errors.append({'file':name,'error':'missing'})
  continue
 h=hashlib.sha256()
 with p.open('rb') as stream:
  for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
 if h.hexdigest()!=expected:errors.append({'file':name,'error':'sha256 mismatch'})
print(json.dumps({'files':len(m),'mismatches':errors}))
assert not errors
'''.replace('ROOT', repr(REMOTE))
process = subprocess.Popen(['ssh', '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=5',
                            '-o', 'ServerAliveCountMax=2', HOST, 'python3 -c ' + shlex.quote(code)],
                           stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           text=True, start_new_session=True)
try:
    output, error = process.communicate(json.dumps(manifest), timeout=240)
except subprocess.TimeoutExpired:
    os.killpg(process.pid, signal.SIGTERM)
    process.communicate(timeout=5)
    raise
assert process.returncode == 0, (output, error)
node = json.loads(output)
assert not node['mismatches'] and node['files'] == len(manifest)
receipt = {
    'at': datetime.now(timezone.utc).isoformat(), 'passed': True,
    'scope': 'Every listed artifact independently rehashed on Mac and node; immutable raw expectations retained.',
    'final_documents_included': args.include_final_documents,
    'files': manifest, 'files_count': len(manifest), 'raw_counts': counts,
    'node_check': node, 'mac_mismatches': [],
    'source_sha256': plan['source_sha256'], 'checkpoint_sha256': plan['checkpoint_sha256'],
    'historical_migration_archive': {
        'bytes': 12698828724,
        'sha256': 'd8f0a99269cad0a58a953ccdf03ee3118b21f8d4087d598c0ee8aa8d7fe4f753',
        'previously_verified_both_hosts': True, 'retransferred_or_rehashed_this_audit': False,
        'mac_payload_entries_previously_verified': 12000,
    },
    'simulation_process_commands_issued': 0,
}
name = 'final_delivery_manifest.both_hosts_verified.json' if args.include_final_documents else 'delivery_core_artifacts.both_hosts_verified.json'
target = ROOT / name
pending = target.with_suffix('.pending.json')
pending.write_text(json.dumps(receipt, indent=2) + '\n')
os.replace(pending, target)
print(json.dumps({'passed': True, 'files': len(manifest), 'raw_counts': counts, 'receipt': str(target)}))
