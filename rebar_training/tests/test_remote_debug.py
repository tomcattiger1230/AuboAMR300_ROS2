import json

import pytest

from rebar_training.remote_debug import dispatch


def test_remote_status_remains_paused_without_gpu_or_network(tmp_path):
    result=dispatch(tmp_path,{'operation':'status','request_id':'test'})
    assert result['training_paused'] and not result['remote_deployment_verified']


@pytest.mark.parametrize('payload', [
    {'operation':'train'}, {'operation':'status','command':'echo hello'},
    {'operation':'tail_log','path':'../../etc/passwd'},
    {'operation':'tail_log','path':'rebar_training/artifacts/a.log','lines':10000},
])
def test_remote_interface_cannot_start_training_or_execute_arbitrary_commands(tmp_path, payload):
    with pytest.raises(ValueError):dispatch(tmp_path,payload)


def test_symlink_escape_and_bounded_log_reads(tmp_path):
    folder=tmp_path/'rebar_training/artifacts';folder.mkdir(parents=True)
    log=folder/'test.log';log.write_text('first\nlast\n')
    assert dispatch(tmp_path,{'operation':'tail_log','path':'rebar_training/artifacts/test.log','lines':1})['text']=='last'
    (folder/'escape.log').symlink_to(tmp_path/'secret.log')
    with pytest.raises(ValueError):dispatch(tmp_path,{'operation':'tail_log','path':'rebar_training/artifacts/escape.log'})


def test_entire_artifact_root_symlink_cannot_expand_read_access(tmp_path):
    root = tmp_path / 'repo'
    (root / 'rebar_training').mkdir(parents=True)
    outside = tmp_path / 'outside'
    outside.mkdir()
    (outside / 'report.json').write_text('{"private": true}')
    (root / 'rebar_training/artifacts').symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match='Only training'):
        dispatch(root, {'operation': 'inspect_manifest', 'path': 'rebar_training/artifacts/report.json'})


@pytest.mark.parametrize('bad_report', ['symlink', 'oversized', 'malformed'])
def test_saved_status_respects_directory_size_and_structure_limits(tmp_path, bad_report):
    folder = tmp_path / 'rebar_training/artifacts/curriculum_20261019'
    folder.mkdir(parents=True)
    report = folder / 'validation_summary.json'
    if bad_report == 'symlink':
        secret = tmp_path / 'outside.json'
        secret.write_text('{"status": "private"}')
        report.symlink_to(secret)
    elif bad_report == 'oversized':
        report.write_text('{"status":"' + 'x' * 65536 + '"}')
    else:
        report.write_text('{"loading": null}')
    with pytest.raises(ValueError):
        dispatch(tmp_path, {'operation': 'status'})
