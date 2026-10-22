"""Safety of frozen evaluation admission and idempotent dispatch, without GPUs."""
import importlib.util
import json
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location('frozen_scheduler',
    Path(__file__).parents[1] / 'scripts/launch_frozen_acceptance.py')
scheduler = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scheduler)


@pytest.fixture
def campaign(tmp_path):
    root = tmp_path / 'campaign'
    root.mkdir()
    (root / 'campaign_source').mkdir()
    (root / 'campaign_source/controller.py').write_text('frozen = True\n')
    (root / 'frozen_policy.pt').write_bytes(b'unchanged-policy-and-optimizer')
    files = ['rebar_training/artifacts/visual_control_20261019/segment_v1.pt',
             'rebar_training/artifacts/assets_mobile_v2/manifest.json',
             'rebar_training/config/vision_runtime.json']
    for file in files:
        p = tmp_path / file
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(b'frozen')
    plan = dict(repository=str(tmp_path), source_sha256=scheduler.source_sha(root / 'campaign_source'),
                checkpoint_sha256=scheduler.sha(root / 'frozen_policy.pt'),
                detector_sha256=scheduler.sha(tmp_path / files[0]),
                asset_manifest_sha256=scheduler.sha(tmp_path / files[1]),
                vision_runtime_sha256=scheduler.sha(tmp_path / files[2]),
                phases=[dict(name='tray', until='tray', seeds=[101, 102], episodes_per_seed=4),
                        dict(name='full', until='clamp', seeds=[201, 202], episodes_per_seed=8)])
    (root / 'plan.json').write_text(json.dumps(plan))
    (root / 'provenance.json').write_text(json.dumps(dict(plan_sha256=scheduler.sha(root / 'plan.json'))))
    (root / 'candidate_11_gate_independently_verified.json').write_text(json.dumps(dict(
        passed=True, episodes=11, successes=11, failures=0, evidence_errors=[],
        source_sha256=plan['source_sha256'], checkpoint_sha256=plan['checkpoint_sha256'])))
    return root, plan


def test_full_evaluation_cannot_train_initialize_or_use_hint(campaign):
    root, plan = campaign
    args = scheduler.mission_args(plan, plan['phases'][1], 201, root)
    assert not {'--learn-residual', '--save-policy', '--regression', '--start-on-tray',
                '--start-at-dock', '--survey-from-packet'}.intersection(args)


def test_no_promotion_without_independent_gate(campaign):
    root, plan = campaign
    with pytest.raises(FileNotFoundError):
        scheduler.require_previous_gate(root, plan, 'full')


def test_changed_evidence_blocks_promotion(campaign):
    root, plan = campaign
    reports = {}
    for seed in [101, 102]:
        name = 'tray_holdout_' + str(seed) + '.json'
        (root / name).write_text('{}')
        reports[name] = scheduler.sha(root / name)
    review = dict(passed=True, gate=dict(passed=True, episodes=8, evidence_errors=[]),
                  source_sha256=plan['source_sha256'], checkpoint_sha256=plan['checkpoint_sha256'],
                  report_sha256=reports)
    (root / 'tray_gate.independently_verified.json').write_text(json.dumps(review))
    scheduler.require_previous_gate(root, plan, 'full')
    (root / 'tray_holdout_101.json').write_text('{"changed":true}')
    with pytest.raises(AssertionError):
        scheduler.require_previous_gate(root, plan, 'full')


def test_two_existing_simulations_block_dispatch(campaign, monkeypatch):
    root, plan = campaign
    monkeypatch.setattr(scheduler, 'inventory', lambda: dict(groups=[10, 20], gpu=[]))
    def forbidden(*a, **k):
        raise AssertionError('Must not launch a third simulation')
    monkeypatch.setattr(scheduler.subprocess, 'check_output', forbidden)
    result = scheduler.run(root, 'tray')
    assert not result['launched']
    assert json.loads((root / 'acceptance.status.json').read_text())['state'] == 'waiting_for_simulation_capacity'


def test_existing_job_is_never_started_again(campaign, monkeypatch):
    root, plan = campaign
    for seed in [101, 102]:
        (root / f'tray_holdout_{seed}.job.json').write_text('{}')
        (root / f'tray_holdout_{seed}.job.status.json').write_text('{"state":"completed"}')
    monkeypatch.setattr(scheduler, 'inventory', lambda: dict(groups=[], gpu=[]))
    monkeypatch.setattr(scheduler.subprocess, 'check_output', lambda *a, **k: pytest.fail('Duplicate launch'))
    assert not scheduler.run(root, 'tray')['launched']


def test_existing_failure_blocks_further_dispatch(campaign, monkeypatch):
    root, plan = campaign
    (root / 'tray_holdout_101.job.status.json').write_text('{"state":"failed"}')
    monkeypatch.setattr(scheduler, 'inventory', lambda: dict(groups=[], gpu=[]))
    monkeypatch.setattr(scheduler.subprocess, 'check_output', lambda *a, **k: pytest.fail('Dispatch after failure'))
    with pytest.raises(RuntimeError):
        scheduler.run(root, 'tray')
