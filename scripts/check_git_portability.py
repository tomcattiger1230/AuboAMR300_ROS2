#!/usr/bin/env python3
"""Check tracked filename portability on case-insensitive filesystems."""
import json
import subprocess
import sys
from pathlib import Path

repo = Path(__file__).resolve().parents[1]
paths = subprocess.check_output(['git', 'ls-files', '-z'], cwd=repo).decode().split('\0')
groups = {}
for path in filter(None, paths):
    groups.setdefault(path.casefold(), []).append(path)
collisions = [group for group in groups.values() if len(group) > 1]
print(json.dumps({'tracked_paths': len(list(filter(None, paths))),
                  'case_collisions': collisions, 'pass': not collisions}, indent=2))
sys.exit(1 if collisions else 0)
