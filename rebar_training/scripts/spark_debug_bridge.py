#!/usr/bin/env python3
"""Hermes/OpenClaw tool adapter; dry-run by default. Sends JSON on SSH stdin.

Copy this standalone script to Spark. Select --execute-readonly only after
installing the fixed receiver and verifying its dedicated SSH key/host key.
"""
import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys

OPERATIONS = {'status', 'list_artifacts', 'tail_log', 'inspect_manifest'}


def run():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--target')
    parser.add_argument('--receiver')
    parser.add_argument('--identity-file')
    parser.add_argument('--execute-readonly', action='store_true')
    args = parser.parse_args()
    config = json.loads(args.config.read_text()) if args.config else {}
    if not isinstance(config, dict) or set(config) - {'target', 'receiver', 'identity_file', 'execute_readonly'}:
        raise ValueError('Unknown bridge configuration field')
    if type(config.get('execute_readonly', False)) is not bool:
        raise ValueError('execute_readonly must be a boolean')
    target = args.target or config.get('target', 'arnold@192.168.3.133')
    receiver = args.receiver or config.get('receiver', '/home/arnold/Develop/ROS_ws/hongshi_mm_ws/src/AuboAMR300_ROS2/rebar_training/scripts/remote_debug_stdio.py')
    identity = args.identity_file or config.get('identity_file')
    execute = args.execute_readonly or config.get('execute_readonly', False)
    payload = sys.stdin.read(8193)
    if len(payload) > 8192:
        raise ValueError('Request too large')
    request = json.loads(payload)
    if not isinstance(request, dict) or request.get('operation') not in OPERATIONS:
        raise ValueError('Read-only diagnostic operation required; training is paused')
    if set(request) - {'operation', 'request_id', 'path', 'lines'}:
        raise ValueError('Unknown request field')
    if not isinstance(target, str) or not re.fullmatch(r'[A-Za-z0-9_.-]+@[A-Za-z0-9_.-]+', target) or not isinstance(receiver, str) or not receiver.startswith('/'):
        raise ValueError('Configure a fixed SSH user/host and absolute receiver path')
    # Ignore general SSH configuration and agent keys. Only this dedicated key
    # may authenticate; its authorized_keys entry must force the receiver.
    command = ['ssh', '-F', '/dev/null', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
               '-o', 'ConnectTimeout=5', '-o', 'ClearAllForwardings=yes', '-o', 'IdentityAgent=none']
    if identity is not None:
        if not isinstance(identity, str):
            raise ValueError('identity_file must be a path string')
        identity = str(Path(identity).expanduser().resolve())
        command.extend(['-o', 'IdentitiesOnly=yes', '-i', identity])
    if execute and not identity:
        raise ValueError('Live read-only calls require an explicit dedicated identity_file')
    command.extend([target, 'python3 ' + shlex.quote(receiver)])
    if not execute:
        print(json.dumps({'dry_run': True, 'training_paused': True, 'argv': command, 'stdin': request}))
        return 0
    result = subprocess.run(command, input=json.dumps(request), text=True, capture_output=True, timeout=15)
    if result.returncode:
        print(json.dumps({'ok': False, 'training_paused': True, 'error': result.stderr[-2000:], 'response': result.stdout[-4000:]}))
        return 1
    print(json.dumps(json.loads(result.stdout), ensure_ascii=False))
    return 0


def main():
    try:
        return run()
    except (OSError, ValueError, TypeError, subprocess.TimeoutExpired) as exc:
        print(json.dumps({'ok': False, 'training_paused': True, 'error': str(exc)}, ensure_ascii=False))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
