"""Replay boundary checks; no model, training runtime, GPU or simulator needed."""
import importlib.util
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location(
    'cosmos_replay', Path(__file__).parents[1] / 'scripts/cosmos_reasoner_replay.py')
replay = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(replay)


class ReplayContractTests(unittest.TestCase):
    def test_intervals_cover_original_end_without_overlap(self):
        spans = replay.intervals(316.4, 20)
        self.assertEqual(spans[0][0], 0)
        self.assertEqual(spans[-1], (300, 316.4))
        self.assertTrue(all(a[1] == b[0] for a, b in zip(spans, spans[1:])))

    def test_reject_model_events_outside_supplied_clip(self):
        answer = {'summary': 'Recorded movement', 'events': [
            {'start_s': 0, 'end_s': 24, 'description': 'Movement'}],
            'visible_concerns': [], 'uncertain': []}
        with self.assertRaises(ValueError):
            replay.validate_answer(answer, 20)
        answer['events'][0]['end_s'] = float('nan')
        with self.assertRaises(ValueError):
            replay.validate_answer(answer, 20)

    def test_model_prompt_excludes_native_metrics_and_rejects_unknown_schema(self):
        prompt = replay.prompt_for({'source_start_s': 300, 'source_end_s': 316.4})
        self.assertNotIn('1.253', prompt)
        self.assertIn('Do not infer millimetre accuracy', prompt)
        with self.assertRaises(ValueError):
            replay.validate_answer({'success': True}, 16.4)


if __name__ == '__main__':
    unittest.main()
