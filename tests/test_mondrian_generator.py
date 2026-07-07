import unittest
from pathlib import Path

from context import CONFIGS_DIR

from config_loader import load_config
from mondrian_generator import build_painting_plan, generate_mondrian_layout

DEMO_CONFIG = CONFIGS_DIR / "demo_v1_a4_pen.json"
SEED_RANGE = range(300)


class TestGenerateMondrianLayout(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(DEMO_CONFIG)
        cls.canvas = cls.config["canvas"]

    def test_artwork_is_never_empty(self):
        """Every seed must produce at least one interior grid line."""
        for seed in SEED_RANGE:
            _, lines = generate_mondrian_layout(self.config, seed=seed)
            interior = [line for line in lines if line.label and line.label.startswith("grid_line")]
            self.assertGreaterEqual(
                len(interior), 1, f"seed {seed} produced a border-only artwork"
            )

    def test_all_lines_respect_the_canvas_margin(self):
        margin = self.canvas["margin_mm"]
        x_max = self.canvas["width_mm"] - margin
        y_max = self.canvas["height_mm"] - margin
        eps = 1e-6
        for seed in SEED_RANGE:
            _, lines = generate_mondrian_layout(self.config, seed=seed)
            for line in lines:
                for x, y in ((line.x1, line.y1), (line.x2, line.y2)):
                    self.assertGreaterEqual(x, margin - eps, f"seed {seed}, {line.label}")
                    self.assertLessEqual(x, x_max + eps, f"seed {seed}, {line.label}")
                    self.assertGreaterEqual(y, margin - eps, f"seed {seed}, {line.label}")
                    self.assertLessEqual(y, y_max + eps, f"seed {seed}, {line.label}")

    def test_monochrome_mode_produces_no_colored_rectangles(self):
        rectangles, _ = generate_mondrian_layout(self.config, seed=7)
        background = self.config["artwork"]["background_color"]
        colored = [rect for rect in rectangles if rect.fill != background]
        self.assertEqual(colored, [])

    def test_border_lines_are_always_present(self):
        _, lines = generate_mondrian_layout(self.config, seed=7)
        labels = {line.label for line in lines}
        for border in ("border_top", "border_bottom", "border_left", "border_right"):
            self.assertIn(border, labels)

    def test_same_seed_gives_identical_plan(self):
        config_path = Path(DEMO_CONFIG)
        plans = []
        for _ in range(2):
            rectangles, lines = generate_mondrian_layout(self.config, seed=123)
            plans.append(build_painting_plan(rectangles, lines, self.config, config_path, seed=123))
        self.assertEqual(plans[0], plans[1])

    def test_pen_stroke_width_matches_tool_width(self):
        """Demo v1: preview line width must match what the 1 mm pen can draw."""
        tool_width = self.config["path_generation"]["tool_width_mm"]
        _, lines = generate_mondrian_layout(self.config, seed=7)
        for line in lines:
            self.assertEqual(line.stroke_width, tool_width, line.label)


class TestColorMode(unittest.TestCase):
    def test_legacy_color_config_still_generates_colored_blocks(self):
        config = load_config(CONFIGS_DIR / "mondrian_12x12_paint.json")
        background = config["artwork"]["background_color"]
        # Not every seed picks accent cells the same way; one known seed is enough.
        rectangles, _ = generate_mondrian_layout(config, seed=123)
        colored = [rect for rect in rectangles if rect.fill != background]
        self.assertGreater(len(colored), 0)


if __name__ == "__main__":
    unittest.main()
