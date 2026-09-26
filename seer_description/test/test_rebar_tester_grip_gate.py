"""Safety checks for transfer of a rebar from robot fingers to tester jaws."""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from rebar_tester_control import read_command, read_state, rebar_grip_ready  # noqa: E402


class RebarGripGateTest(unittest.TestCase):
    def setUp(self):
        self.jaws = {
            "upper_z": 1.87,
            "lower_z": 1.12,
            "upper_opening": 0.024,
            "lower_opening": 0.024,
        }
        self.expected = (6.0, 3.92, 1.5)

    def test_aligned_bar_can_be_held(self):
        self.assertTrue(rebar_grip_ready(
            self.jaws, (6.0, 3.90, 1.5), self.expected, (0, 0, 1)
        ))

    def test_bar_outside_gap_is_not_taken_from_robot(self):
        self.assertFalse(rebar_grip_ready(
            self.jaws, (6.0, 3.80, 1.5), self.expected, (0, 0, 1)
        ))

    def test_open_jaw_does_not_take_bar(self):
        jaws = {**self.jaws, "lower_opening": 0.16}
        self.assertFalse(rebar_grip_ready(
            jaws, self.expected, self.expected, (0, 0, 1)
        ))

    def test_horizontal_bar_does_not_take_bar(self):
        self.assertFalse(rebar_grip_ready(
            self.jaws, self.expected, self.expected, (1, 0, 0)
        ))

    def test_attachment_command_rejects_non_boolean(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "command.json"
            path.write_text('{"robot_attach": 1}', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "robot_attach must be boolean"):
                read_command(path)
            path.write_text('{"robot_attach": true}', encoding="utf-8")
            self.assertTrue(read_command(path)["robot_attach"])

    def test_attachment_feedback_rejects_non_boolean(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "state.json"
            path.write_text('{"upper_z": 1.87, "lower_z": 1.12, '
                            '"upper_opening": 0.024, "lower_opening": 0.024, '
                            '"robot_attached": "yes"}', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "robot_attached must be boolean"):
                read_state(path)


if __name__ == "__main__":
    unittest.main()
