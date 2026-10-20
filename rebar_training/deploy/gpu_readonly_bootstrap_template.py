"""Self-contained GPU setup template; filled locally with verified public data."""
import argparse
import base64
import getpass
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile

PUBLIC_KEY = "__VERIFIED_PUBLIC_KEY__"
FILES = {}  # Replaced by the local preparation step, never downloaded remotely.
REPO = Path('/home/arnold/Develop/ROS_ws/hongshi_mm_ws/src/AuboAMR300_ROS2')
HOST_FP = 'SHA256:+OjWUONiEbj5iAtDwhAe8ybSdn23rmxmwqz5RKSQNH0'
KEY_FP = 'SHA256:eEpEm0h+LgacobX4yOiu+GbGtRSU1am/Q1QqpzHwMEk'


def guarded(path):
    for part in (path, *path.parents):
        if part.is_symlink():
            raise ValueError(f'Symbolic link refused: {part}')
    if path.exists() and not path.is_file():
        raise ValueError(f'Expected a regular file: {path}')


def atomic_write(path, data, mode):
    guarded(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix='.rebar-setup-', dir=path.parent)
    try:
        with os.fdopen(descriptor, 'wb') as out:
            os.fchmod(out.fileno(), mode)
            out.write(data)
            out.flush()
            os.fsync(out.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    if getpass.getuser() != 'arnold' or Path.home() != Path('/home/arnold'):
        raise ValueError('Run as arnold on the intended GPU host')
    if not REPO.is_dir() or REPO.resolve() != REPO:
        raise ValueError('Expected GPU repository is missing or redirected')
    fingerprint = subprocess.run(
        ['ssh-keygen', '-lf', '/etc/ssh/ssh_host_ed25519_key.pub'],
        capture_output=True, text=True, check=True,
    ).stdout.split()[1]
    if fingerprint != HOST_FP:
        raise ValueError('GPU host fingerprint does not match')
    public = PUBLIC_KEY.split()
    if len(public) < 2 or public[0] != 'ssh-ed25519' or '\n' in PUBLIC_KEY:
        raise ValueError('Invalid dedicated public key')
    key_fp = 'SHA256:' + base64.b64encode(hashlib.sha256(
        base64.b64decode(public[1], validate=True)).digest()).decode().rstrip('=')
    if key_fp != KEY_FP:
        raise ValueError('Dedicated public-key fingerprint does not match')
    expected = {
        'rebar_training/scripts/remote_debug_stdio.py',
        'rebar_training/src/rebar_training/__init__.py',
        'rebar_training/src/rebar_training/spec.py',
        'rebar_training/src/rebar_training/remote_debug.py',
    }
    if set(FILES) != expected:
        raise ValueError('Unexpected receiver payload')
    auth = Path.home() / '.ssh/authorized_keys'
    guarded(auth)
    receiver = REPO / 'rebar_training/scripts/remote_debug_stdio.py'
    forced = f'/usr/bin/timeout 20s /usr/bin/python3 -I {receiver} --root {REPO}'
    entry = f'from="192.168.3.144",restrict,command="{forced}" {PUBLIC_KEY}'
    old_auth = auth.read_bytes() if auth.exists() else b''
    auth_lines = old_auth.decode('utf-8').splitlines()
    matches = [line for line in auth_lines if not line.lstrip().startswith('#')
               and public[1] in line.split()]
    if matches and matches != [entry]:
        raise ValueError('Dedicated key already has different or duplicate authorization')
    payload = {}
    before = {}
    for relative, record in FILES.items():
        target = REPO / relative
        guarded(target)
        data = base64.b64decode(record['base64'], validate=True)
        if hashlib.sha256(data).hexdigest() != record['sha256']:
            raise ValueError(f'Receiver payload checksum failed: {relative}')
        compile(data, str(target), 'exec')
        payload[target] = data
        before[target] = (target.read_bytes(), stat.S_IMODE(target.stat().st_mode)) if target.exists() else None
    print(json.dumps({'phase': 'plan', 'host_fingerprint': fingerprint,
                      'public_key_fingerprint': key_fp, 'receiver_files': sorted(FILES),
                      'source_ip': '192.168.3.144', 'authorization': 'restricted forced receiver',
                      'apply': args.apply, 'training_policy': 'paused'}), flush=True)
    if not args.apply:
        return
    backup = Path(tempfile.mkdtemp(prefix='rebar-readonly-backup-', dir=Path.home()))
    os.chmod(backup, 0o700)
    for target, previous in before.items():
        if previous is not None:
            atomic_write(backup / target.relative_to(REPO), *previous)
    if auth.exists():
        atomic_write(backup / 'authorized_keys', old_auth, stat.S_IMODE(auth.stat().st_mode))
    changed = []
    auth_changed = False
    try:
        for target, data in payload.items():
            atomic_write(target, data, 0o644)
            changed.append(target)
        test = subprocess.run(
            ['/usr/bin/python3', '-I', str(receiver), '--root', str(REPO)],
            input=json.dumps({'request_id': 'gpu-bootstrap-self-test', 'operation': 'status'}),
            capture_output=True, text=True, timeout=20, check=True,
        )
        response = json.loads(test.stdout)
        if response.get('ok') is not True or response.get('training_paused') is not True:
            raise ValueError('Receiver self-test did not pass')
        if not matches:
            auth.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            updated = old_auth + (b'\n' if old_auth and not old_auth.endswith(b'\n') else b'')
            atomic_write(auth, updated + entry.encode() + b'\n', 0o600)
            auth_changed = True
        else:
            os.chmod(auth, 0o600)
        os.chmod(auth.parent, 0o700)
    except Exception:
        if auth_changed:
            atomic_write(auth, old_auth, 0o600)
        for target in reversed(changed):
            previous = before[target]
            if previous is None:
                target.unlink()
            else:
                atomic_write(target, *previous)
        raise
    print(json.dumps({'ok': True, 'receiver_self_test': True, 'key_authorized': True,
                      'key_added': not bool(matches), 'backup': str(backup),
                      'training_policy': 'paused', 'training_started': False}), flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(json.dumps({'ok': False, 'error': str(exc), 'training_policy': 'paused'}), flush=True)
        sys.exit(1)
