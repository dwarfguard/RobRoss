import unittest

import context  # noqa: F401  (adds scripts/ to sys.path)

from generate_painting_paths import (
    build_commands,
    compute_stripe_row_centers,
    line_to_commands,
    rectangle_to_commands,
)

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


if __name__ == "__main__":
    unittest.main()
