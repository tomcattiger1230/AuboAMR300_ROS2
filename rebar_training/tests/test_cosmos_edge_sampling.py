"""Sampling must preserve source time and never invent extra video frames."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from evaluate_cosmos_edge_replay import first_json, sample_indices


class SamplingContract(unittest.TestCase):
    def test_two_fps_from_ten_fps_preserves_indices(self):
        self.assertEqual(sample_indices(40, 10, 2), list(range(0, 40, 5)))

    def test_four_fps_rounding_keeps_source_time_error_bounded(self):
        indices = sample_indices(40, 10, 4)
        self.assertEqual(len(indices), 16)
        self.assertEqual(len(indices), len(set(indices)))
        self.assertTrue(all(abs(index/10 - n/4) <= .050001 for n, index in enumerate(indices)))

    def test_reject_upsampling_and_empty_input(self):
        for args in [(40, 10, 12), (0, 10, 2), (40, 0, 2)]:
            with self.assertRaises(ValueError):
                sample_indices(*args)

    def test_full_official_clip_is_not_truncated(self):
        self.assertEqual(sample_indices(26, 4, 2), list(range(0, 26, 2)))

    def test_parse_event_array_without_conflating_format_and_accuracy(self):
        self.assertEqual(first_json('```json\n[{"start": 1}]\n```'), [{'start': 1}])
        self.assertEqual(first_json('[]'), [])
        self.assertIsNone(first_json('I cannot identify the object.'))


if __name__ == '__main__':
    unittest.main()
