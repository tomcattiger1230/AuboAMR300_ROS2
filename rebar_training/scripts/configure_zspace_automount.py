#!/usr/bin/env python3
"""Install one CIFS fstab automount on the authorized host; never reboot/unmount live NAS.

Run --check as the ordinary user. Installation needs root and prompts for SMB
credentials on the host's terminal only. No simulator or training imports.
"""
import argparse
from datetime import datetime, timezone
import getpass
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import stat
import subprocess
import tempfile

HOST = 'arnold-NUC16JNK9XX11'
TARGET = '/mnt/zspace'
CREDENTIALS = Path('/etc/samba/zspace.credentials')
MARKER = '# Managed zspace CIFS automount'


def run(args, timeout=40, check=True):
    p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, start_new_session=True)
    try:
        out, err = p.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGTERM)
        try:
            p.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.communicate()
        raise RuntimeError('Timed out: ' + args[0]) from None
    if check and p.returncode:
        raise RuntimeError(args[0] + ' failed: ' + err.strip())
    return p.returncode, out, err


def mount_info():
    _, out, _ = run(['findmnt', '-T', TARGET, '-J'])
    entry = json.loads(out)['filesystems'][0]
    options = dict(x.split('=', 1) if '=' in x else (x, True) for x in entry.get('options', '').split(','))
    return entry, options


def escape_fstab(value):
    if any(x in value for x in ('\n', '\r', '\0')):
        raise ValueError('Control character in fstab value')
    return value.replace('\\', '\\134').replace(' ', '\\040').replace('\t', '\\011')


def fstab_line(source, options):
    if not source.startswith('//') or source.count('/') < 3:
        raise ValueError('Expected an existing UNC share')
    values = {}
    for key, default, pattern in [('vers', '3.1.1', r'[0-9]+(?:\.[0-9]+){0,2}'),
                                  ('uid', '0', r'[0-9]+'), ('gid', '0', r'[0-9]+'),
                                  ('file_mode', '0777', r'0?[0-7]{3}'), ('dir_mode', '0777', r'0?[0-7]{3}')]:
        value = str(options.get(key, default))
        if not re.fullmatch(pattern, value):
            raise ValueError('Unexpected mount option: ' + key)
        values[key] = value
    opts = ['credentials=' + str(CREDENTIALS)] + [k + '=' + v for k, v in values.items()]
    opts += ['iocharset=utf8', '_netdev', 'nofail', 'x-systemd.automount', 'x-systemd.mount-timeout=30s']
    return f'{escape_fstab(source)} {TARGET} cifs {",".join(opts)} 0 0'


def updated_fstab(original, line):
    matches = []
    for index, raw in enumerate(original.splitlines(keepends=True)):
        fields = raw.strip().split()
        if fields and not fields[0].startswith('#') and len(fields) > 1 and fields[1] == TARGET:
            matches.append((index, fields))
    if len(matches) > 1:
        raise ValueError('Multiple existing entries for /mnt/zspace; resolve them first')
    lines = original.splitlines(keepends=True)
    if matches:
        index, fields = matches[0]
        if len(fields) < 3 or fields[0] != line.split()[0] or fields[2] != 'cifs':
            raise ValueError('Conflicting existing /mnt/zspace entry; left unchanged')
        lines[index] = line + '\n'
        return ''.join(lines)
    return original + ('' if not original or original.endswith('\n') else '\n') + '\n' + MARKER + '\n' + line + '\n'


def credential_bytes(username, password, domain=''):
    if not username or not password or any(x in username + password + domain for x in ('\r', '\n', '\0')):
        raise ValueError('A nonempty single-line username/password is required')
    text = 'username=' + username + '\npassword=' + password + '\n'
    if domain:
        text += 'domain=' + domain + '\n'
    return text.encode()


def atomic_write(path, data, mode=0o600, owner=(0, 0)):
    fd, name = tempfile.mkstemp(prefix='.' + path.name + '.', dir=path.parent)
    try:
        os.fchmod(fd, mode)
        os.fchown(fd, *owner)
        with os.fdopen(fd, 'wb') as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def auth_probe(source, path, version):
    # A separate read-only connection tests the file without touching live /mnt/zspace.
    probe = Path(tempfile.mkdtemp(prefix='zspace-auth-', dir='/run'))
    try:
        run(['mount', '-t', 'cifs', source, str(probe), '-o',
             f'credentials={path},vers={version},ro,nosuid,nodev,nosharesock'], timeout=35)
    finally:
        if run(['mountpoint', '-q', str(probe)], check=False)[0] == 0:
            run(['umount', str(probe)], timeout=20)
        probe.rmdir()


def install_credentials(source, options, replace):
    CREDENTIALS.parent.mkdir(parents=True, exist_ok=True)
    if CREDENTIALS.exists() and not replace:
        if CREDENTIALS.is_symlink():
            raise RuntimeError('Credential path must not be a symlink')
        content = dict(line.split('=', 1) for line in CREDENTIALS.read_text().splitlines()
                       if '=' in line and not line.lstrip().startswith('#'))
        if not content.get('username') or not content.get('password'):
            raise RuntimeError('Existing credential file is incomplete; use --replace-credentials')
        os.chown(CREDENTIALS, 0, 0)
        os.chmod(CREDENTIALS, 0o600)
        auth_probe(source, CREDENTIALS, options.get('vers', '3.1.1'))
        return
    if not os.isatty(0):
        raise RuntimeError('Run from an interactive terminal on 103; no secrets are read from stdin pipes')
    username = options.get('username') or options.get('user') or input('NAS username: ').strip()
    password = getpass.getpass('NAS password (hidden): ')
    data = credential_bytes(username, password, str(options.get('domain', '')))
    password = None
    fd, temporary = tempfile.mkstemp(prefix='.zspace-auth-', dir=CREDENTIALS.parent)
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, 'wb') as handle:
            handle.write(data)
        auth_probe(source, Path(temporary), options.get('vers', '3.1.1'))
        if CREDENTIALS.exists():
            if CREDENTIALS.is_symlink():
                raise RuntimeError('Credential path must not be a symlink')
            backup = CREDENTIALS.with_name(CREDENTIALS.name + '.previous.' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
            shutil.copyfile(CREDENTIALS, backup)
            os.chmod(backup, 0o600)
        os.replace(temporary, CREDENTIALS)
        os.chown(CREDENTIALS, 0, 0)
        os.chmod(CREDENTIALS, 0o600)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Read-only plan, no root or credentials required')
    parser.add_argument('--replace-credentials', action='store_true', help='Prompt again; root-only previous file retained')
    args = parser.parse_args()
    if socket.gethostname() != HOST:
        parser.error('This installer is scoped to ' + HOST)
    entry, options = mount_info()
    if entry['target'] != TARGET or entry['fstype'] != 'cifs':
        parser.error('Mount the intended NAS at /mnt/zspace once before configuring it')
    line = fstab_line(entry['source'], options)
    fstab = Path('/etc/fstab')
    original = fstab.read_bytes()
    candidate = updated_fstab(original.decode(), line).encode()
    if args.check:
        print(json.dumps(dict(check_passed=True,host=HOST,target=TARGET,source=entry['source'],
                             planned_fstab_line=line,live_mount_will_not_be_unmounted=True,
                             install_requires_root=True,secrets_logged=False)))
        return
    if os.geteuid() != 0:
        parser.error('Run sudo python3 <this-script> in an interactive terminal on 103')
    install_credentials(entry['source'], options, args.replace_credentials)
    state = Path('/var/lib/zspace-automount')
    state.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(state, 0o700)
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    backup = state / ('fstab.before.' + timestamp)
    atomic_write(backup, original)
    trial = state / 'fstab.candidate'
    atomic_write(trial, candidate)
    run(['findmnt', '--verify', '--tab-file', str(trial)])
    oldstat = fstab.stat()
    atomic_write(fstab, candidate, stat.S_IMODE(oldstat.st_mode), (oldstat.st_uid, oldstat.st_gid))
    try:
        run(['systemctl', 'daemon-reload'])
        _, loaded, _ = run(['systemctl', 'show', 'mnt-zspace.automount', '-p', 'LoadState', '--value'])
        if loaded.strip() != 'loaded':
            raise RuntimeError('Generated automount unit did not load')
        links = list(Path('/run/systemd').glob('generator*/remote-fs.target.wants/mnt-zspace.automount'))
        if not links:
            raise RuntimeError('Boot dependency for automount was not generated')
        current, _ = mount_info()
        if current.get('target') != TARGET or current.get('fstype') != 'cifs' or current.get('source') != entry['source']:
            raise RuntimeError('Live mount changed during configuration')
    except Exception:
        atomic_write(fstab, original, stat.S_IMODE(oldstat.st_mode), (oldstat.st_uid, oldstat.st_gid))
        run(['systemctl', 'daemon-reload'])
        raise
    report = dict(configured=True,host=HOST,target=TARGET,source=entry['source'],fstab_sha256=hashlib.sha256(candidate).hexdigest(),
                  fstab_backup=str(backup),credentials_path=str(CREDENTIALS),credentials_mode=oct(stat.S_IMODE(CREDENTIALS.stat().st_mode)),
                  boot_automount_dependency_verified=True,credential_auth_probe_passed=True,live_mount_unchanged=True,
                  reboot_performed=False,simulation_started=False)
    atomic_write(state/'installed.json', (json.dumps(report, indent=2)+'\n').encode(), 0o644)
    atomic_write(Path('/etc/zspace-automount.status.json'), (json.dumps(report, indent=2)+'\n').encode(), 0o644)
    print(json.dumps(report))
    print('Configured: after reboot, accessing /mnt/zspace mounts the NAS automatically. No reboot was performed.')


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, ValueError) as error:
        raise SystemExit(str(error)) from None
