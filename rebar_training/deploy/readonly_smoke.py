"""Portable CPU smoke tests; synthetic data, no SSH or agent installation.

Run: python3 rebar_training/deploy/readonly_smoke.py
"""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

MODULE = Path(__file__).resolve().parents[1]


class ReadOnlySmoke(unittest.TestCase):
    def setUp(self):
        self.scratch = tempfile.TemporaryDirectory(prefix='rebar-debug-test-')
        self.addCleanup(self.scratch.cleanup)
        self.root = Path(self.scratch.name) / 'repo'
        self.artifacts = self.root / 'rebar_training/artifacts'
        self.artifacts.mkdir(parents=True)
        (self.artifacts / 'sample.log').write_text('first\nlast\n')
        (self.artifacts / 'sample.json').write_text('{"synthetic":true}')

    def receiver(self, payload):
        result = subprocess.run(
            [sys.executable, str(MODULE / 'scripts/remote_debug_stdio.py'), '--root', str(self.root)],
            input=json.dumps(payload), capture_output=True, text=True, timeout=5,
        )
        value = json.loads(result.stdout)
        self.assertTrue(value['training_paused'])
        self.assertEqual(result.returncode, 0 if value['ok'] else 1)
        return value

    def test_all_four_read_operations(self):
        status = self.receiver({'operation': 'status'})
        self.assertTrue(status['ok'])
        self.assertFalse(status['training_process_state_verified'])
        self.assertEqual(len(self.receiver({'operation': 'list_artifacts'})['files']), 2)
        self.assertEqual(self.receiver({'operation': 'tail_log', 'path': 'rebar_training/artifacts/sample.log', 'lines': 1})['text'], 'last')
        self.assertTrue(self.receiver({'operation': 'inspect_manifest', 'path': 'rebar_training/artifacts/sample.json'})['manifest']['synthetic'])

    def test_train_and_shell_fields_are_rejected(self):
        for payload in [{'operation': 'train'}, {'operation': 'status', 'command': 'uname -a'}]:
            self.assertFalse(self.receiver(payload)['ok'])

    def test_path_escape_and_log_limit_are_rejected(self):
        for path, lines in [('../../etc/passwd', 1), ('rebar_training/artifacts/sample.log', 201)]:
            self.assertFalse(self.receiver({'operation': 'tail_log', 'path': path, 'lines': lines})['ok'])

    def test_child_symlink_escape_is_rejected(self):
        outside = Path(self.scratch.name) / 'outside.json'
        outside.write_text('{"synthetic_private":true}')
        (self.artifacts / 'escape.json').symlink_to(outside)
        self.assertFalse(self.receiver({'operation': 'inspect_manifest', 'path': 'rebar_training/artifacts/escape.json'})['ok'])

    def test_root_symlink_escape_is_rejected(self):
        self.artifacts.rename(self.root / 'outside-artifacts')
        self.artifacts.symlink_to(self.root / 'outside-artifacts', target_is_directory=True)
        self.assertFalse(self.receiver({'operation': 'inspect_manifest', 'path': 'rebar_training/artifacts/sample.json'})['ok'])

    def test_saved_status_symlink_is_rejected(self):
        folder = self.artifacts / 'curriculum_20261019'
        folder.mkdir()
        outside = Path(self.scratch.name) / 'outside.json'
        outside.write_text('{"status":"synthetic_private"}')
        (folder / 'validation_summary.json').symlink_to(outside)
        self.assertFalse(self.receiver({'operation': 'status'})['ok'])

    def test_bridge_dry_run_does_not_connect(self):
        result = subprocess.run([sys.executable, str(MODULE / 'scripts/spark_debug_bridge.py')],
                                input='{"operation":"status"}', capture_output=True, text=True, timeout=5, check=True)
        value = json.loads(result.stdout)
        self.assertTrue(value['dry_run'])
        self.assertTrue(value['training_paused'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
