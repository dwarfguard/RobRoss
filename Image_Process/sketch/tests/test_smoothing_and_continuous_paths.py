import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import context  # noqa: F401

from canny_edges import simplify
from generate_sketch_paths import (
    _map_traced_strokes,
    build_commands,
    build_painting_paths,
    map_points_to_canvas,
    render_svg,
    smooth_canvas_path,
)
from path_validation import validate_painting_paths


class TestArcLengthSmoothing(unittest.TestCase):
    def test_noisy_straight_line_becomes_smoother_and_keeps_endpoints(self):
        points = [(index * 0.1, 0.12 * (-1) ** index) for index in range(101)]
        smoothed, effective = smooth_canvas_path(points, False, 0.5)

        before_roughness = sum(
            abs(points[index - 1][1] - 2 * points[index][1] + points[index + 1][1])
            for index in range(1, len(points) - 1)
        )
        after_roughness = sum(
            abs(smoothed[index - 1][1] - 2 * smoothed[index][1] + smoothed[index + 1][1])
            for index in range(1, len(smoothed) - 1)
        )
        self.assertLess(after_roughness, before_roughness * 0.1)
        self.assertEqual(smoothed[0], points[0])
        self.assertEqual(smoothed[-1], points[-1])
        self.assertEqual(effective, 0.5)

    def test_shared_junction_anchor_remains_identical(self):
        junction = (2.0, 3.0)
        first, _ = smooth_canvas_path(
            [junction, (3.0, 3.2), (4.0, 3.0), (5.0, 3.1)], False, 0.5
        )
        second, _ = smooth_canvas_path(
            [junction, (2.2, 4.0), (2.0, 5.0), (2.1, 6.0)], False, 0.5
        )
        self.assertEqual(first[0], junction)
        self.assertEqual(second[0], junction)

    def test_curve_and_closed_loop_remain_recognizable(self):
        arc = [
            (5.0 * math.cos(theta), 5.0 * math.sin(theta))
            for theta in [index * math.pi / 40 for index in range(41)]
        ]
        smoothed_arc, _ = smooth_canvas_path(arc, False, 0.5)
        self.assertEqual(smoothed_arc[0], arc[0])
        self.assertEqual(smoothed_arc[-1], arc[-1])
        self.assertGreater(max(point[1] for point in smoothed_arc), 4.8)

        circle = [
            (5.0 * math.cos(index * 2 * math.pi / 60),
             5.0 * math.sin(index * 2 * math.pi / 60))
            for index in range(60)
        ]
        smoothed_circle, _ = smooth_canvas_path(circle, True, 0.5)
        radii = [math.hypot(x, y) for x, y in smoothed_circle]
        self.assertEqual(len(smoothed_circle), len(circle))
        self.assertGreater(min(radii), 4.8)
        self.assertLess(math.dist(smoothed_circle[0], smoothed_circle[-1]), 0.7)

    def test_short_facial_mark_caps_radius_without_erasing_points(self):
        points = [(0.0, 0.0), (0.2, 0.08), (0.4, 0.0), (0.6, 0.05)]
        smoothed, effective = smooth_canvas_path(points, False, 2.0)
        self.assertAlmostEqual(effective, sum(
            math.dist(a, b) for a, b in zip(points, points[1:])
        ) / 6.0)
        self.assertEqual(len(smoothed), len(points))
        self.assertEqual(smoothed[0], points[0])
        self.assertEqual(smoothed[-1], points[-1])

    def test_zero_radius_uses_legacy_pixel_geometry(self):
        raw = [([(0, 0), (1, 1), (2, 0), (3, 1), (4, 0)], False)]
        image_size = (10, 10)
        origin = (5.0, 5.0)
        size = (90.0, 90.0)
        traced, stats = _map_traced_strokes(
            raw, image_size, origin, size, 0.01, 1.0, 0.0
        )
        legacy = map_points_to_canvas(
            simplify(raw[0][0], False, 0.01, 1.0), image_size, origin, size
        )
        self.assertEqual(traced, [(legacy, False)])
        self.assertEqual(stats["effective_smoothing_radius_mm"], 0.0)


class TestContinuousCommands(unittest.TestCase):
    def _painting_paths(self, commands, continuous):
        config = {
            "project": "test",
            "profile_name": "test",
            "style": "sketch",
            "canvas": {"width_mm": 20.0, "height_mm": 20.0, "origin": "top-left"},
            "path_generation": {
                "tool_width_mm": 1.0,
                "continuous_paths": continuous,
                "smoothing_radius_mm": 0.5,
            },
        }
        return build_painting_paths(
            commands,
            config,
            Path("config.json"),
            Path("photo.png"),
            {"baseline": 0.0, "optimized": 0.0},
        )

    def test_continuous_mode_emits_one_path_per_line_in_phase_order(self):
        commands = build_commands(
            [[(0, 0), (1, 0), (2, 1)]],
            [[(3, 3), (4, 4)]],
            continuous_paths=True,
        )
        paint_commands = [
            command for command in commands
            if command["command"] in ("paint_path", "paint_stroke")
        ]
        self.assertEqual([command["command"] for command in paint_commands], [
            "paint_path", "paint_path"
        ])
        self.assertEqual(
            [command["label"] for command in paint_commands],
            ["sketch_outline_1", "sketch_detail_1"],
        )
        painting_paths = self._painting_paths(commands, True)
        self.assertTrue(validate_painting_paths(painting_paths)["passed"])
        self.assertEqual(painting_paths["debug"]["num_paint_path_commands"], 2)
        self.assertEqual(painting_paths["debug"]["num_paint_stroke_commands"], 0)

    def test_segmented_mode_remains_available(self):
        commands = build_commands(
            [[(0, 0), (1, 0), (2, 1)]], continuous_paths=False
        )
        self.assertEqual(
            [command["command"] for command in commands].count("paint_stroke"), 2
        )
        self.assertNotIn("paint_path", [command["command"] for command in commands])

    def test_svg_polyline_points_equal_json_geometry(self):
        commands = build_commands(
            [[(0.123, 1.234), (2.345, 3.456), (4.567, 5.678)]],
            continuous_paths=True,
        )
        painting_paths = self._painting_paths(commands, True)
        svg = render_svg(painting_paths)
        root = ET.fromstring(svg)
        polyline = root.find("{http://www.w3.org/2000/svg}polyline")
        json_points = next(
            command["points_mm"] for command in commands
            if command["command"] == "paint_path"
        )
        svg_points = [
            [float(value) for value in pair.split(",")]
            for pair in polyline.attrib["points"].split()
        ]
        self.assertEqual(svg_points, json_points)


if __name__ == "__main__":
    unittest.main()
