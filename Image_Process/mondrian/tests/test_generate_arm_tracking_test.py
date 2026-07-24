import unittest
from pathlib import Path

from context import CONFIGS_DIR

from config_loader import load_config
from generate_arm_tracking_test import build_arm_tracking_tests
from path_validation import validate_painting_paths


class TestGenerateArmTrackingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config_path = CONFIGS_DIR / "demo_v1_a4_pen.json"
        cls.config = load_config(cls.config_path)
        cls.fixtures = build_arm_tracking_tests(cls.config, Path(cls.config_path))

    def test_all_fixtures_validate_without_warnings(self):
        self.assertEqual(
            set(self.fixtures),
            {
                "arm_tracking_direction_test",
                "arm_tracking_reversal_test",
                "arm_tracking_curve_test",
            },
        )
        for fixture in self.fixtures.values():
            validation = validate_painting_paths(fixture)
            self.assertTrue(validation["passed"], validation["errors"])
            self.assertEqual(validation["warnings"], [])

    def test_each_path_is_independently_lifted(self):
        for fixture in self.fixtures.values():
            command_types = [command["command"] for command in fixture["commands"]]
            self.assertEqual(command_types[:2], ["select_tool", "dip_paint"])
            for index in range(2, len(command_types), 4):
                self.assertEqual(
                    command_types[index : index + 4],
                    ["move_to", "lower_tool", "paint_path", "lift_tool"],
                )

    def test_direction_controls_retrace_the_same_segments(self):
        paths = [
            command["points_mm"]
            for command in self.fixtures["arm_tracking_direction_test"]["commands"]
            if command["command"] == "paint_path"
        ]
        self.assertEqual(paths[0], list(reversed(paths[1])))
        self.assertEqual(paths[2], list(reversed(paths[3])))

    def test_reversal_paths_change_y_direction(self):
        paths = [
            command["points_mm"]
            for command in self.fixtures["arm_tracking_reversal_test"]["commands"]
            if command["command"] == "paint_path"
        ]
        self.assertGreater(paths[0][1][1] - paths[0][0][1], 0)
        self.assertLess(paths[0][2][1] - paths[0][1][1], 0)
        self.assertLess(paths[1][1][1] - paths[1][0][1], 0)
        self.assertGreater(paths[1][2][1] - paths[1][1][1], 0)

    def test_curve_advances_x_and_alternates_y_direction(self):
        curve = next(
            command["points_mm"]
            for command in self.fixtures["arm_tracking_curve_test"]["commands"]
            if command["command"] == "paint_path"
        )
        x_steps = [end[0] - start[0] for start, end in zip(curve, curve[1:])]
        y_steps = [end[1] - start[1] for start, end in zip(curve, curve[1:])]
        self.assertEqual(len(curve), 41)
        self.assertTrue(all(step > 0 for step in x_steps))
        self.assertTrue(any(step > 0 for step in y_steps))
        self.assertTrue(any(step < 0 for step in y_steps))


if __name__ == "__main__":
    unittest.main()
