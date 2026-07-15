import unittest
from pathlib import Path

from context import CONFIGS_DIR

from config_loader import load_config
from generate_painting_paths import (
    build_animation_timeline,
    build_commands,
    build_painting_paths,
    compute_stripe_row_centers,
    line_to_commands,
    rectangle_to_commands,
    render_animated_svg,
)
from mondrian_generator import build_painting_plan, generate_mondrian_layout

PATH_SETTINGS = {
    "tool_width_mm": 10.0,
    "stroke_overlap_ratio": 0.25,
    "edge_inset_mm": 3.0,
}


class TestComputeStripeRowCenters(unittest.TestCase):
    def test_short_rectangle_gets_one_center_stripe(self):
        centers = compute_stripe_row_centers(y=20.0, height=8.0, tool_width_mm=10.0, stroke_overlap_ratio=0.0)
        self.assertEqual(centers, [24.0])

    def test_first_and_last_stripes_cover_the_edges(self):
        y, height, tool = 10.0, 100.0, 10.0
        centers = compute_stripe_row_centers(y, height, tool, stroke_overlap_ratio=0.0)
        self.assertAlmostEqual(centers[0], y + tool / 2)
        self.assertAlmostEqual(centers[-1], y + height - tool / 2)

    def test_centers_are_strictly_increasing(self):
        centers = compute_stripe_row_centers(0.0, 100.0, 10.0, stroke_overlap_ratio=0.25)
        for a, b in zip(centers, centers[1:]):
            self.assertLess(a, b)

    def test_overlap_produces_more_stripes(self):
        no_overlap = compute_stripe_row_centers(0.0, 100.0, 10.0, 0.0)
        with_overlap = compute_stripe_row_centers(0.0, 100.0, 10.0, 0.5)
        self.assertGreater(len(with_overlap), len(no_overlap))


class TestLineToCommands(unittest.TestCase):
    def test_command_sequence_and_fields(self):
        op = {
            "operation": "paint_line",
            "label": "grid_line_1",
            "color": "black",
            "from_mm": [10.0, 20.0],
            "to_mm": [200.0, 20.0],
            "stroke_width_mm": 1.0,
        }
        commands = line_to_commands(op)
        self.assertEqual(
            [cmd["command"] for cmd in commands],
            ["select_tool", "dip_paint", "move_to", "lower_tool", "paint_stroke", "lift_tool"],
        )
        stroke = commands[4]
        self.assertEqual(stroke["from_mm"], [10.0, 20.0])
        self.assertEqual(stroke["to_mm"], [200.0, 20.0])
        move = commands[2]
        self.assertEqual((move["x_mm"], move["y_mm"]), (10.0, 20.0))


class TestRectangleToCommands(unittest.TestCase):
    def test_boustrophedon_rows_alternate_direction(self):
        op = {
            "operation": "paint_rectangle",
            "label": "red_block_1",
            "color": "#d62828",
            "x_mm": 20.0,
            "y_mm": 20.0,
            "width_mm": 100.0,
            "height_mm": 50.0,
        }
        commands = rectangle_to_commands(op, PATH_SETTINGS)
        strokes = [cmd for cmd in commands if cmd["command"] == "paint_stroke"]
        self.assertGreater(len(strokes), 1)
        for index, stroke in enumerate(strokes):
            going_right = stroke["from_mm"][0] < stroke["to_mm"][0]
            self.assertEqual(going_right, index % 2 == 0, f"row {index}")

    def test_rectangle_too_small_after_inset_is_skipped(self):
        op = {
            "operation": "paint_rectangle",
            "label": "tiny_block",
            "color": "#d62828",
            "x_mm": 20.0,
            "y_mm": 20.0,
            "width_mm": 5.0,  # smaller than 2 * edge_inset_mm
            "height_mm": 5.0,
        }
        self.assertEqual(rectangle_to_commands(op, PATH_SETTINGS), [])


class TestBuildCommands(unittest.TestCase):
    def test_operation_order_is_preserved(self):
        plan = {
            "operations": [
                {
                    "operation": "paint_rectangle",
                    "label": "red_block_1",
                    "color": "#d62828",
                    "x_mm": 20.0,
                    "y_mm": 20.0,
                    "width_mm": 100.0,
                    "height_mm": 50.0,
                },
                {
                    "operation": "paint_line",
                    "label": "grid_line_1",
                    "color": "black",
                    "from_mm": [10.0, 20.0],
                    "to_mm": [200.0, 20.0],
                    "stroke_width_mm": 1.0,
                },
            ]
        }
        commands = build_commands(plan, PATH_SETTINGS)
        labels = [cmd["label"] for cmd in commands]
        last_rect_index = max(i for i, label in enumerate(labels) if label.startswith("red_block_1"))
        first_line_index = labels.index("grid_line_1")
        self.assertLess(last_rect_index, first_line_index)


class TestBuildAnimationTimeline(unittest.TestCase):
    COMMANDS = [
        {"command": "select_tool", "label": "l1", "color": "black"},
        {"command": "dip_paint", "label": "l1", "color": "black"},
        {"command": "move_to", "label": "l1", "x_mm": 10.0, "y_mm": 10.0},
        {"command": "lower_tool", "label": "l1"},
        {"command": "paint_stroke", "label": "l1", "color": "black", "from_mm": [10.0, 10.0], "to_mm": [110.0, 10.0]},
        {"command": "lift_tool", "label": "l1"},
        {"command": "move_to", "label": "l2", "x_mm": 10.0, "y_mm": 50.0},
        {"command": "lower_tool", "label": "l2"},
        {"command": "paint_stroke", "label": "l2", "color": "black", "from_mm": [10.0, 50.0], "to_mm": [110.0, 50.0]},
        {"command": "lift_tool", "label": "l2"},
    ]

    def test_event_kinds_and_order(self):
        events, total = build_animation_timeline(self.COMMANDS)
        kinds = [ev["type"] for ev in events]
        self.assertEqual(
            kinds,
            [
                "dip_paint", "place", "lower_tool", "stroke", "lift_tool",
                "travel", "lower_tool", "stroke", "lift_tool",
            ],
        )
        self.assertGreater(total, 0.0)

    def test_start_times_never_decrease(self):
        events, total = build_animation_timeline(self.COMMANDS)
        starts = [ev["start"] for ev in events]
        self.assertEqual(starts, sorted(starts))
        self.assertLessEqual(starts[-1], total)

    def test_strokes_keep_command_geometry(self):
        events, _ = build_animation_timeline(self.COMMANDS)
        strokes = [ev for ev in events if ev["type"] == "stroke"]
        self.assertEqual(strokes[0]["from"], (10.0, 10.0))
        self.assertEqual(strokes[0]["to"], (110.0, 10.0))
        self.assertEqual(strokes[1]["from"], (10.0, 50.0))

    def test_travel_connects_stroke_end_to_next_start(self):
        events, _ = build_animation_timeline(self.COMMANDS)
        travel = next(ev for ev in events if ev["type"] == "travel")
        self.assertEqual(travel["from"], (110.0, 10.0))
        self.assertEqual(travel["to"], (10.0, 50.0))

    def test_empty_command_list_gives_empty_timeline(self):
        events, total = build_animation_timeline([])
        self.assertEqual(events, [])
        self.assertEqual(total, 0.0)


class TestRenderAnimatedSvg(unittest.TestCase):
    def test_animation_renders_for_both_repo_profiles(self):
        """Full pipeline through render_animated_svg for each config profile."""
        for name in ("demo_v1_a4_pen.json", "mondrian_12x12_paint.json"):
            with self.subTest(config=name):
                config_path = CONFIGS_DIR / name
                config = load_config(config_path)
                rectangles, lines = generate_mondrian_layout(config, seed=123)
                plan = build_painting_plan(rectangles, lines, config, Path(config_path), seed=123)
                commands = build_commands(plan, config["path_generation"])
                paths = build_painting_paths(plan, commands, config, Path(config_path), Path("plan.json"))
                svg = render_animated_svg(paths)
                self.assertIn("<animate", svg)
                self.assertIn("stroke-dashoffset", svg)
                self.assertIn("<circle", svg)  # tool marker
                # One dashoffset animation per painted segment: a
                # paint_stroke is one segment, a paint_path with N points
                # is N-1 segments.
                num_segments = sum(
                    1 if cmd["command"] == "paint_stroke"
                    else len(cmd["points_mm"]) - 1
                    for cmd in paths["commands"]
                    if cmd["command"] in ("paint_stroke", "paint_path")
                )
                self.assertGreater(num_segments, 0)
                self.assertEqual(svg.count('attributeName="stroke-dashoffset"'), num_segments)


if __name__ == "__main__":
    unittest.main()
