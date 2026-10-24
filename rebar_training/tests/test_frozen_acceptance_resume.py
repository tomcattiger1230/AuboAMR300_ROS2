"""Paused outputs and user authorization survive a frozen-job resume."""
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
import resume_paused_frozen_acceptance as resume


class ResumeSafety(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.repo = Path(self.temp.name)
        self.root = self.repo / 'campaign'
        self.root.mkdir()
        (self.root / 'campaign_source').mkdir()
        (self.root / 'campaign_source/controller.py').write_text('frozen = True\n')
        (self.root / 'frozen_policy.pt').write_bytes(b'unchanged-policy-and-optimizer')
        (self.root / 'completed.json').write_text('{"episodes":8}')
        plan = dict(repository=str(self.repo), new_training=False, optimizer_reset=False,
                    max_simulations=2, retained_counters=dict(updates=466, actor_updates=170, policy_steps=36602),
                    source_sha256=resume.source_sha(self.root / 'campaign_source'),
                    checkpoint_sha256=resume.sha(self.root / 'frozen_policy.pt'))
        for relative, key in [('artifacts/visual_control_20261019/segment_v1.pt', 'detector_sha256'),
                              ('artifacts/assets_mobile_v2/manifest.json', 'asset_manifest_sha256'),
                              ('config/vision_runtime.json', 'vision_runtime_sha256')]:
            file = self.repo / 'rebar_training' / relative
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_bytes(b'frozen')
            plan[key] = resume.sha(file)
        resume.write(self.root / 'plan.json', plan)
        resume.write(self.root / 'pause_user.latest.json', dict(at='2026-10-23T00:26:31+00:00'))
        resume.write(self.root / 'acceptance.status.json', dict(state='paused_user'))
        resume.write(self.root / 'resume_authorization_0103_20261023.json', dict(
            at='2026-10-23T12:39:42+00:00', user_authorization='回到同一个网络 继续训练',
            new_learning_updates=0, optimizer_reset=False, control_source_change=False,
            plan_sha256=resume.sha(self.root / 'plan.json'), source_sha256=plan['source_sha256'],
            policy_optimizer_sha256=plan['checkpoint_sha256'],
            complete_group_report_sha256={'completed.json': resume.sha(self.root / 'completed.json')}))
        for name in resume.NAMES:
            source = self.root / 'source_snapshots' / name
            source.parent.mkdir(exist_ok=True)
            shutil.copytree(self.root / 'campaign_source', source)
            resume.write(self.root / (name + '.job.json'), dict(
                command=['python', '--seed', name.rsplit('_', 1)[1], '--episodes', '8', '--until', 'clamp'],
                source_snapshot=str(source), source_sha256=plan['source_sha256']))
            resume.write(self.root / (name + '.job.status.json'), dict(state='paused_user'))
            (self.root / (name + '.log')).write_text('old attempt must remain intact\n')
        resume.write(self.root / (resume.NAMES[1] + '.progress.json'), dict(episodes=6, successes=6))
        self.gate = patch.object(resume, 'require_previous_gate')
        self.gate.start()
        self.addCleanup(self.gate.stop)
        self.inventory = patch.object(resume, 'inventory', return_value=dict(groups=[], gpu=[]))
        self.inventory.start()
        self.addCleanup(self.inventory.stop)

    def test_partial_evidence_is_archived_without_changing_completed_reports_or_policy(self):
        original_policy = (self.root / 'frozen_policy.pt').read_bytes()
        original_job = (self.root / (resume.NAMES[1] + '.job.json')).read_bytes()
        result = resume.run(self.root, 'prepare')
        archive = Path(result['archive'])
        self.assertEqual(resume.read(archive / (resume.NAMES[1] + '.progress.json'))['episodes'], 6)
        self.assertFalse((self.root / (resume.NAMES[1] + '.progress.json')).exists())
        self.assertEqual((self.root / 'frozen_policy.pt').read_bytes(), original_policy)
        self.assertEqual((self.root / (resume.NAMES[1] + '.job.json')).read_bytes(), original_job)
        self.assertEqual((self.root / 'completed.json').read_text(), '{"episodes":8}')
        self.assertEqual(resume.run(self.root, 'prepare'), result)

    def test_existing_simulations_prevent_preparation(self):
        with patch.object(resume, 'inventory', return_value=dict(groups=[10, 20], gpu=[])):
            with self.assertRaises(AssertionError):
                resume.run(self.root, 'prepare')
        self.assertTrue((self.root / (resume.NAMES[1] + '.progress.json')).exists())

    def test_failed_partial_rows_are_not_erased_by_resume(self):
        resume.write(self.root / (resume.NAMES[1] + '.progress.json'), dict(episodes=6, successes=5))
        with self.assertRaises(AssertionError):
            resume.run(self.root, 'prepare')
        self.assertFalse((self.root / 'interrupted_attempts').exists())

    def test_a_new_pause_invalidates_old_resume_authorization(self):
        resume.run(self.root, 'prepare')
        resume.write(self.root / 'pause_user.latest.json', dict(at='2026-10-23T13:00:00+00:00'))
        with patch.object(resume.subprocess, 'Popen', side_effect=AssertionError('Must not start')):
            with self.assertRaises(AssertionError):
                resume.run(self.root, 'launch', resume.NAMES[0])

    def test_already_launched_attempt_is_not_duplicated(self):
        result = resume.run(self.root, 'prepare')
        result['launched'][resume.NAMES[0]] = dict(runner_pid=1234)
        resume.write(self.root / 'resume_after_user_pause_20261023.receipt.json', result)
        with patch.object(resume.subprocess, 'Popen', side_effect=AssertionError('Duplicate start')):
            self.assertEqual(resume.run(self.root, 'launch', resume.NAMES[0]),
                             dict(already_launched=dict(runner_pid=1234)))

    def test_training_initialization_and_hint_flags_are_rejected(self):
        job = resume.read(self.root / (resume.NAMES[0] + '.job.json'))
        for flag in ['--learn-residual', '--save-policy=/tmp/policy.pt', '--regression',
                     '--start-at-dock', '--start-on-tray', '--survey-from-packet=/tmp/hint']:
            with self.subTest(flag=flag), self.assertRaises(AssertionError):
                resume.check_command({**job, 'command': job['command'] + [flag]}, resume.NAMES[0])


if __name__ == '__main__':
    unittest.main()
