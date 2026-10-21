#!/usr/bin/env python3
"""Build and verify a portable training project; excludes venvs and prior archives."""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import tarfile


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def source_assets(root, manifests=None):
    """Include dependencies from every asset variant, including the new rack."""
    expected_files={}
    if manifests is None:manifests=(root/'rebar_training/artifacts').glob('**/manifest.json')
    for manifest in manifests:
        assets=json.loads(manifest.read_text())
        sources=dict(assets.get('source_sha256',{}))
        if assets.get('rack_source'):
            sources[assets['rack_source']]=assets['rack_sha256']
        for relative,expected in sources.items():
            if relative in expected_files and expected_files[relative]!=expected:
                raise ValueError('Conflicting source asset versions: '+relative)
            expected_files[relative]=expected
    if not expected_files:raise ValueError('No asset source manifests found')
    files=[]
    for relative,expected in expected_files.items():
        path=(root/relative).resolve();path.relative_to(root)
        if digest(path)!=expected:raise ValueError('Source asset changed: '+relative)
        files.append(path)
    return files


def collect_files(root, output, profile='all'):
    """Select a closed mobile resume bundle without copying prior backup tarballs."""
    training=root/'rebar_training';artifacts=training/'artifacts';files=[]
    def add_tree(folder):
        for parent,dirs,names in os.walk(folder):
            dirs[:]=[d for d in dirs if d not in ('.venv','__pycache__','.pytest_cache','.ruff_cache')
                     and not d.endswith('.egg-info')]
            for name in names:
                path=Path(parent)/name
                if (path==output or path.is_symlink() or name.endswith('.pyc')
                        or name.startswith(('rebar_training_transfer_','rebar_training_recovery_',
                                            'rebar_training_migration_'))):continue
                files.append(path)
    if profile=='all':
        add_tree(training);files.extend(source_assets(root))
    elif profile=='mobile-resume':
        for path in training.iterdir():
            if path.name in ('artifacts','.venv','__pycache__','.pytest_cache','.ruff_cache'):continue
            if path.is_dir():add_tree(path)
            elif path.is_file() and not path.is_symlink():files.append(path)
        assets=[artifacts/'assets_mobile_v2',artifacts/'assets_full_v2_precision']
        for folder in assets:
            if not (folder/'manifest.json').is_file():raise FileNotFoundError(folder/'manifest.json')
            add_tree(folder)
        files.extend(source_assets(root,[folder/'manifest.json' for folder in assets]))
        base=artifacts/'mobile_curriculum_20261020';review=base/'review_20261020'
        for folder in (base/'course_v7_support_lifecycle',review/'precision_residual_fix',review/'checkpoints'):
            if not folder.is_dir():raise FileNotFoundError(folder)
            add_tree(folder)
        state_file=review/'status.json'
        if state_file.is_file():
            state=json.loads(state_file.read_text())
            directories=list(state.get('migration_required_directories',[]))
            if state.get('current_course'):directories.append(state['current_course'])
            for relative in directories:
                folder=(base/relative).resolve()
                folder.relative_to(base.resolve())
                if folder==base.resolve():raise ValueError('Select a specific migration artifact directory')
                if not folder.is_dir():raise FileNotFoundError(folder)
                add_tree(folder)
        files.extend(base.glob('PAUSED*.json'))
        files.extend(review.glob('*.json'))
        files.extend(review.joinpath('support_prior_lifecycle_fix').glob('*.json'))
        visual=artifacts/'visual_control_20261019'
        detector=visual/'segment_v1.pt'
        if not detector.is_file():raise FileNotFoundError(detector)
        files.append(detector)
        versions=visual/'runtime_versions_0103.json'
        if versions.is_file():files.append(versions)
        hint=visual/'mobile_navigation_packets_v11_cad'
        files.extend(hint.glob('wrist_step300.*'))
        if not (hint/'wrist_step300.json').is_file():raise FileNotFoundError(hint/'wrist_step300.json')
    else:raise ValueError('Unknown transfer profile: '+profile)
    files.extend((root/'seer_description/urdf/textures').glob('rebar_*.png'))
    files.extend([root/'README.md',root/'.gitignore'])
    return sorted(set(files))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', choices=('all','mobile-resume'), default='all',
                        help='mobile-resume includes mobile assets, the status-selected current course and migration evidence; older backup archives stay on the source machine')
    parser.add_argument('--training-state', choices=('paused', 'active', 'completed'), default='paused',
                        help='Describes this snapshot; building an archive does not change a training job')
    args = parser.parse_args()
    root, output = args.root.resolve(), args.output.resolve()
    if output.exists():
        raise FileExistsError('Preserve existing transfer archives; choose a new name')
    files = collect_files(root,output,args.profile)
    manifest = {str(p.relative_to(root)): digest(p) for p in files}
    output.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps({'schema_version': 'rebar_transfer_v1', 'files': manifest}, indent=2)+'\n').encode()
    with tarfile.open(output, 'w:gz', compresslevel=3) as tar:
        info = tarfile.TarInfo('TRANSFER_MANIFEST.json')
        info.size = len(encoded)
        tar.addfile(info, io.BytesIO(encoded))
        for path in files:
            tar.add(path, arcname=str(path.relative_to(root)), recursive=False)
    bad = []
    with tarfile.open(output, 'r:gz') as tar:
        for name, expected in manifest.items():
            stream = tar.extractfile(name)
            result = hashlib.sha256()
            for block in iter(lambda: stream.read(1024*1024), b''):
                result.update(block)
            if result.hexdigest() != expected:
                bad.append(name)
    if bad:
        raise RuntimeError('Archive verification failed: ' + repr(bad))
    sha = digest(output)
    output.with_suffix(output.suffix+'.sha256').write_text(sha+'  '+output.name+'\n')
    report = {'archive': str(output), 'bytes': output.stat().st_size, 'sha256': sha,
              'verified_files': len(files), 'mismatches': bad,
              'training_state': args.training_state,
              'profile': args.profile,
              # A snapshot description never launches or resumes a job.
              'training_resumed': False, 'archive_creation_only': True}
    output.with_suffix(output.suffix+'.verify.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
