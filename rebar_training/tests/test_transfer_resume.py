import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


def script(name):
    file=Path(__file__).resolve().parents[1]/'scripts'/name
    spec=importlib.util.spec_from_file_location(name.removesuffix('.py'),file)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_mobile_transfer_is_closed_and_does_not_repackage_prior_backups(tmp_path):
    module=script('build_transfer.py');root=tmp_path
    def write(name,data='data'):
        p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(data);return p
    write('README.md');write('.gitignore');write('robot.usda')
    digest=hashlib.sha256((root/'robot.usda').read_bytes()).hexdigest()
    a='rebar_training/artifacts'
    for name in ['assets_mobile_v2','assets_full_v2_precision']:
        write(f'{a}/{name}/manifest.json',json.dumps({'source_sha256':{'robot.usda':digest}}))
    write('rebar_training/src/pkg/main.py');write('rebar_training/.venv/bin/python')
    for name in ['rebar_training_recovery_old.tar.gz','rebar_training_transfer_old.tar.gz',
                 'rebar_training_migration_old.tar.gz.sha256']:
        write(f'{a}/{name}')
    write(f'{a}/unrelated_old_run/huge.npy')
    b=f'{a}/mobile_curriculum_20261020'
    write(f'{b}/course_v7_support_lifecycle/dock_clamp_policy.pt')
    write(f'{b}/review_20261020/precision_residual_fix/initial_policy.pt')
    write(f'{b}/review_20261020/checkpoints/optimizer.pt')
    write(f'{a}/visual_control_20261019/segment_v1.pt')
    write(f'{a}/visual_control_20261019/mobile_navigation_packets_v11_cad/wrist_step300.json')
    write(f'{a}/visual_control_20261019/mobile_navigation_packets_v11_cad/wrist_step300.png')
    write(f'{b}/course_v8_precision_envelope/full_policy.pt','policy and optimizer')
    write(f'{b}/course_v8_precision_envelope/partial.progress.json')
    write(f'{b}/review_20261020/migration_demo/current.mp4')
    write(f'{b}/review_20261020/status.json',json.dumps({
        'current_course':'course_v8_precision_envelope',
        'migration_required_directories':['review_20261020/migration_demo']}))
    files={str(p.relative_to(root)) for p in module.collect_files(root,root/'out.tar.gz','mobile-resume')}
    assert 'robot.usda' in files and f'{b}/review_20261020/checkpoints/optimizer.pt' in files
    assert f'{b}/course_v7_support_lifecycle/dock_clamp_policy.pt' in files
    assert f'{a}/visual_control_20261019/mobile_navigation_packets_v11_cad/wrist_step300.png' in files
    assert f'{b}/course_v8_precision_envelope/full_policy.pt' in files
    assert f'{b}/course_v8_precision_envelope/partial.progress.json' in files
    assert f'{b}/review_20261020/migration_demo/current.mp4' in files
    assert not any('old.tar.gz' in name or '/.venv/' in name or 'unrelated_old_run' in name for name in files)
    write(f'{b}/review_20261020/status.json',json.dumps({
        'migration_required_directories':['../../outside']}))
    with pytest.raises(ValueError):
        module.collect_files(root,root/'out.tar.gz','mobile-resume')
    write(f'{b}/review_20261020/status.json','{}')
    (root/'robot.usda').write_text('changed')
    with pytest.raises(ValueError,match='Source asset changed'):
        module.collect_files(root,root/'out.tar.gz','mobile-resume')


def test_prepare_resume_checks_hashes_and_copies_only_fresh_inputs(tmp_path):
    module=script('prepare_resume.py');root=tmp_path/'repo';src=root/'rebar_training/src/pkg'
    src.mkdir(parents=True);(src/'core.py').write_text('x=1\n')
    template=tmp_path/'old';frozen=template/'validation_source/pkg';frozen.mkdir(parents=True)
    (frozen/'core.py').write_text('x=1\n')
    expected={'validate_precision.py':'controller_sha256','plan_v8.json':'plan_sha256',
              'initial_policy.pt':'warm_start_sha256','failed_policy.pt':'failed_policy_sha256'}
    pre={'source_sha256':module.source_sha(root/'rebar_training/src')}
    for name,key in expected.items():
        (template/name).write_bytes(name.encode());pre[key]=hashlib.sha256(name.encode()).hexdigest()
    (template/'predeclaration.json').write_text(json.dumps(pre))
    for name in ['cpu_tests.log','launcher.json','old.job.json','validation.status.json']:
        (template/name).write_text('stale old host')
    out=tmp_path/'new';receipt=module.prepare(root,template,out)
    assert not receipt['training_started'] and receipt['requires_explicit_resume']
    assert {p.name for p in out.iterdir()}=={*expected,'predeclaration.json','validation_source','PREPARED_ONLY.json'}
    with pytest.raises(FileExistsError):module.prepare(root,template,out)
    (template/'failed_policy.pt').write_text('corrupt')
    with pytest.raises(ValueError,match='Validation input changed'):
        module.prepare(root,template,tmp_path/'another')
    assert not (tmp_path/'another').exists()
