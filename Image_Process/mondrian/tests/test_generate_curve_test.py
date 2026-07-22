import math
import unittest
from pathlib import Path

from context import CONFIGS_DIR

from config_loader import load_config
from generate_curve_test import (
    CIRCLE_CENTER,
    CIRCLE_RADIUS_MM,
    build_curve_test_paths,
)
from path_validation import validate_painting_paths


class TestGenerateCurveTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config_path = CONFIGS_DIR / "demo_v1_a4_pen.json"
        cls.config = load_config(cls.config_path)
        cls.paths = build_curve_test_paths(cls.config, Path(cls.config_path))
        cls.path_commands = {
            command["label"]: command
            for command in cls.paths["commands"]
            if command["command"] == "paint_path"
        }

    def test_command_sequence_lifts_between_shapes(self):
        command_types = [command["command"] for command in self.paths["commands"]]
        self.assertEqual(command_types[:2], ["select_tool", "dip_paint"])
        self.assertEqual(len(command_types), 18)
        for index in range(2, len(command_types), 4):
            self.assertEqual(
                command_types[index : index + 4],
                ["move_to", "lower_tool", "paint_path", "lift_tool"],
            )

    def test_expected_shapes_and_sampling_are_present(self):
        self.assertEqual(
            set(self.path_commands),
            {"smooth_s_curve", "closed_circle", "sine_squiggle", "sharp_corners"},
        )
        self.assertEqual(len(self.path_commands["smooth_s_curve"]["points_mm"]), 61)
        self.assertEqual(len(self.path_commands["closed_circle"]["points_mm"]), 49)
        self.assertEqual(len(self.path_commands["sine_squiggle"]["points_mm"]), 97)
        self.assertEqual(len(self.path_commands["sharp_corners"]["points_mm"]), 8)

    def test_each_move_to_matches_its_path_start(self):
        commands = self.paths["commands"]
        for index in range(2, len(commands), 4):
            move = commands[index]
            path = commands[index + 2]
            self.assertEqual([move["x_mm"], move["y_mm"]], path["points_mm"][0])

    def test_circle_is_closed_and_has_constant_radius(self):
        points = self.path_commands["closed_circle"]["points_mm"]
        self.assertEqual(points[0], points[-1])
        for point in points[:-1]:
            self.assertAlmostEqual(math.dist(point, CIRCLE_CENTER), CIRCLE_RADIUS_MM, delta=0.01)

    def test_sharp_path_contains_right_angle_and_acute_corner(self):
        points = self.path_commands["sharp_corners"]["points_mm"]
        angles = []
        for previous, current, following in zip(points, points[1:], points[2:]):
            incoming = (previous[0] - current[0], previous[1] - current[1])
            outgoing = (following[0] - current[0], following[1] - current[1])
            cosine = sum(a * b for a, b in zip(incoming, outgoing)) / (
                math.hypot(*incoming) * math.hypot(*outgoing)
            )
            angles.append(math.degrees(math.acos(max(-1.0, min(1.0, cosine)))))

        self.assertTrue(any(math.isclose(angle, 90.0, abs_tol=0.01) for angle in angles))
        self.assertLess(min(angles), 60.0)

    def test_full_fixture_validates_without_warnings(self):
        validation = validate_painting_paths(self.paths)
        self.assertTrue(validation["passed"], validation["errors"])
        self.assertEqual(validation["warnings"], [])


if __name__ == "__main__":
    unittest.main()
