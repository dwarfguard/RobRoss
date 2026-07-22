import tempfile
import unittest
from pathlib import Path

import context  # noqa: F401
import cv2
import numpy as np

from generate_line_art_paths import (
    build_canvas_strokes,
    build_commands,
    build_painting_paths,
    map_points_to_canvas,
    render_svg,
    stroke_to_commands,
)
from path_ordering import order_strokes, total_travel_distance
from path_validation import validate_painting_paths

CONFIG = {
    "profile_name": "unittest",
    "project": "R.O.B Ross",
    "style": "line_art",
    "canvas": {"width_mm": 100.0, "height_mm": 100.0, "margin_mm": 5.0, "origin": "top-left"},
    "source_image": {
        "binary_threshold": 128,
        "min_spur_length_px": 2.0,
        "min_stroke_length_mm": 0.5,
        "simplify_epsilon_ratio": 0.01,
    },
    "path_generation": {"tool_width_mm": 1.0, "home_position_mm": [5.0, 5.0]},
}


class TestMapPointsToCanvas(unittest.TestCase):
    def test_centers_a_square_image_in_a_square_box(self):
        points = [(0, 0), (100, 100)]
        mapped = map_points_to_canvas(points, (100, 100), (5.0, 5.0), (90.0, 90.0))
        self.assertAlmostEqual(mapped[0][0], 5.0)
        self.assertAlmostEqual(mapped[0][1], 5.0)
        self.assertAlmostEqual(mapped[1][0], 95.0)
        self.assertAlmostEqual(mapped[1][1], 95.0)


class TestStrokeToCommands(unittest.TestCase):
    def test_emits_one_paint_path_for_the_whole_line(self):
        points = [(0.0, 0.0), (10.0, 0.0), (20.0, 5.0)]
        commands = stroke_to_commands(points, stroke_index=1)

        self.assertEqual([c["command"] for c in commands], ["move_to", "lower_tool", "paint_path", "lift_tool"])
        paint_path_cmd = commands[2]
        self.assertEqual(paint_path_cmd["points_mm"], [[0.0, 0.0], [10.0, 0.0], [20.0, 5.0]])
        self.assertEqual(paint_path_cmd["color"], "black")

    def test_collapses_consecutive_duplicate_points(self):
        points = [(0.0, 0.0), (0.0, 0.0), (10.0, 0.0)]
        commands = stroke_to_commands(points, stroke_index=1)
        paint_path_cmd = next(c for c in commands if c["command"] == "paint_path")
        self.assertEqual(paint_path_cmd["points_mm"], [[0.0, 0.0], [10.0, 0.0]])

    def test_single_point_after_dedupe_emits_nothing(self):
        points = [(0.0, 0.0), (0.0, 0.0)]
        commands = stroke_to_commands(points, stroke_index=1)
        self.assertEqual(commands, [])


class TestBuildCommands(unittest.TestCase):
    def test_emits_one_select_tool_and_dip_paint_up_front(self):
        ordered = [[(0.0, 0.0), (10.0, 0.0)], [(20.0, 20.0), (30.0, 30.0)]]
        commands = build_commands(ordered)
        self.assertEqual(commands[0]["command"], "select_tool")
        self.assertEqual(commands[1]["command"], "dip_paint")
        self.assertEqual(sum(1 for c in commands if c["command"] == "paint_path"), 2)


class TestBuildPaintingPaths(unittest.TestCase):
    def test_debug_counts_match_commands(self):
        ordered = [[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]]
        commands = build_commands(ordered)
        painting_paths = build_painting_paths(
            commands, CONFIG, Path("configs/fake.json"), Path("fake.png"),
            {"baseline": 10.0, "optimized": 5.0},
        )
        self.assertEqual(painting_paths["style"], "line_art")
        self.assertEqual(painting_paths["debug"]["num_paint_path_commands"], 1)
        self.assertEqual(painting_paths["debug"]["num_traced_lines"], 1)
        self.assertAlmostEqual(painting_paths["debug"]["estimated_total_paint_distance_mm"], 20.0)


class TestRenderSvg(unittest.TestCase):
    def test_renders_a_polyline_per_paint_path(self):
        ordered = [[(0.0, 0.0), (10.0, 0.0)]]
        commands = build_commands(ordered)
        painting_paths = build_painting_paths(
            commands, CONFIG, Path("configs/fake.json"), Path("fake.png"),
            {"baseline": 10.0, "optimized": 10.0},
        )
        svg = render_svg(painting_paths)
        self.assertIn("<polyline", svg)
        self.assertIn("0.0,0.0 10.0,0.0", svg)


class TestFullPipelineOnSyntheticImage(unittest.TestCase):
    """Runs the real pipeline (binarize -> skeletonize -> trace -> prune ->
    simplify -> order -> commands -> validate) end to end on a tiny
    synthetic line-art image: a single straight black line on white."""

    def setUp(self):
        self.tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmpdir.cleanup)
        image = np.full((40, 60), 255, dtype=np.uint8)
        image[19:21, 5:55] = 0  # a horizontal black bar, 2px thick
        self.image_path = Path(self.tmpdir.name) / "line.png"
        cv2.imwrite(str(self.image_path), image)

    def test_traces_one_stroke_and_produces_valid_paths(self):
        config = dict(CONFIG)
        config["source_image"] = {**CONFIG["source_image"], "path": str(self.image_path)}

        strokes_data, image_path = build_canvas_strokes(config)
        self.assertEqual(len(strokes_data), 1)

        home_position = tuple(config["path_generation"]["home_position_mm"])
        baseline = total_travel_distance([points for points, _ in strokes_data], home_position)
        ordered_strokes = order_strokes(strokes_data, home_position)
        optimized = total_travel_distance(ordered_strokes, home_position)

        commands = build_commands(ordered_strokes)
        painting_paths = build_painting_paths(
            commands, config, Path("configs/fake.json"), image_path,
            {"baseline": baseline, "optimized": optimized},
        )
        validation = validate_painting_paths(painting_paths)

        self.assertTrue(validation["passed"], validation["errors"])
        self.assertEqual(painting_paths["debug"]["num_paint_path_commands"], 1)


if __name__ == "__main__":
    unittest.main()
