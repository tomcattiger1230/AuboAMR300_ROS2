"""Read-only JSON tool entry point for Hermes/OpenClaw over LAN SSH.

No shell, model calls, SSH, training launch, or robot control occurs here.
The outer bot transport authenticates the human and invokes this fixed program.
"""
import argparse
import json
from pathlib import Path
import sys

from .spec import repository_root

OPERATIONS = ('status', 'list_artifacts', 'tail_log', 'inspect_manifest')


def _permitted_path(root, relative):
    if not isinstance(relative, str):
        raise ValueError('Path must be a string')
    path = (root / relative).resolve()
    # Compare the resolved target with literal allowed roots. Resolving the
    # roots too would allow an artifacts-directory symlink to authorize any
    # directory on the host.
    permitted = [root / 'rebar_training/artifacts', root / 'logs/rsl_rl/aubo_rebar']
    if not any(path.is_relative_to(base) for base in permitted):
        raise ValueError('Only training artifact/log directories may be read')
    return path


def dispatch(root, request):
    root = Path(root).resolve()
    if not isinstance(request, dict) or set(request) - {'request_id', 'operation', 'path', 'lines'}:
        raise ValueError('Only the documented structured fields are allowed')
    request_id = request.get('request_id')
    if request_id is not None and (not isinstance(request_id, str) or len(request_id) > 128):
        raise ValueError('request_id must be a string of at most 128 characters')
    operation = request.get('operation')
    if operation not in OPERATIONS:
        raise ValueError('Operation unavailable: training remains paused; no arbitrary command execution')
    response = {'request_id': request.get('request_id'), 'operation': operation, 'training_paused': True}
    if operation == 'status':
        response.update(allowed_operations=list(OPERATIONS), remote_deployment_verified=False,
                        status_source='saved_validation_report', training_process_state_verified=False,
                        note='training_paused is the operator policy, not a live process probe; saved reports only')
        status = root / 'rebar_training/artifacts/curriculum_20261019/validation_summary.json'
        if status.is_file():
            status = _permitted_path(root, str(status.relative_to(root)))
            if status.stat().st_size > 65536:
                raise ValueError('Saved status report must be no larger than 64 KiB')
            saved = json.loads(status.read_text())
            if not isinstance(saved, dict) or not isinstance(saved.get('loading', {}), dict):
                raise ValueError('Invalid saved status report structure')
            response['saved_status'] = {'status': saved.get('status'), 'stable_tasks': list(saved.get('stable_tasks', {})),
                                        'loading_stable': saved.get('loading', {}).get('stable')}
        return response
    relative = request.get('path', 'rebar_training/artifacts')
    path = _permitted_path(root, relative)
    if operation == 'list_artifacts':
        response['files'] = [{'path': str(p.relative_to(root)), 'bytes': p.stat().st_size}
                             for p in sorted(path.iterdir())[:100] if p.is_file() and not p.is_symlink()]
    elif operation == 'tail_log':
        lines = request.get('lines', 40)
        if type(lines) is not int or not 1 <= lines <= 200 or path.suffix != '.log':
            raise ValueError('Use a .log file and 1..200 lines')
        with path.open('rb') as stream:
            stream.seek(max(0, path.stat().st_size - 65536))
            response['text'] = '\n'.join(stream.read().decode('utf-8', errors='replace').splitlines()[-lines:])
    else:
        if path.suffix != '.json' or path.stat().st_size > 65536:
            raise ValueError('Manifest must be a JSON file no larger than 64 KiB')
        response['manifest'] = json.loads(path.read_text())
    return response


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=repository_root())
    args = parser.parse_args()
    try:
        payload = sys.stdin.read(8193)
        if len(payload) > 8192:
            raise ValueError('Request exceeds 8 KiB')
        result = {'ok': True, **dispatch(args.root, json.loads(payload))}
    except (OSError, ValueError, TypeError) as exc:
        result = {'ok': False, 'training_paused': True, 'error': str(exc)}
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
