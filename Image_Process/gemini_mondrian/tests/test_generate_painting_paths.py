import tempfile
import unittest
from pathlib import Path

import context  # noqa: F401
import cv2
import numpy as np

from generate_painting_paths import (
    build_border_strokes,
    build_painting_paths,
    build_region_strokes,
    build_regions,
    image_to_canvas_transform,
    order_and_build_commands,
    px_to_mm,
)
from path_validation import validate_painting_paths

PALETTE_COLORS = [
    {"name": "white", "hex": "#ffffff"},
    {"name": "red", "hex": "#d62828"},
    {"name": "black", "hex": "#000000"},
]

CONFIG = {
    "profile_name": "unittest",
    "project": "R.O.B Ross",
    "style": "gemini_mondrian",
    "canvas": {"width_mm": 100.0, "height_mm": 100.0, "margin_mm": 5.0, "origin": "top-left"},
    "source_image": {},
    "gemini": {"model": "gemini-2.5-flash-image", "prompt": "test prompt"},
    "palette": {"colors": PALETTE_COLORS, "color_space": "lab"},
    "segmentation": {"min_region_area_mm2": 1.0, "morph_open_kernel_px": 0, "skip_white_regions": True},
    "path_generation": {
        "tool_width_mm": 2.0,
        "stroke_overlap_ratio": 0.0,
        "mask_erosion_mm": 0.5,
        "home_position_mm": [5.0, 5.0],
    },
    "border_generation": {"draw_borders": True, "simplify_epsilon_ratio": 0.01},
}


class TestImageToCanvasTransform(unittest.TestCase):
    def test_centers_a_square_image_in_a_square_box(self):
        scale, offset = image_to_canvas_transform((100, 100), CONFIG["canvas"])
        self.assertAlmostEqual(scale, 0.9)  # 90mm box / 100px
        self.assertAlmostEqual(offset[0], 5.0)
        self.assertAlmostEqual(offset[1], 5.0)


class TestPxToMm(unittest.TestCase):
    def test_applies_scale_and_offset(self):
        point = px_to_mm((10, 20), scale_mm_per_px=2.0, offset_mm=(1.0, 1.0))
        self.assertEqual(point, (21.0, 41.0))


class TestBuildRegionsAndFullPipeline(unittest.TestCase):
    """Runs the real pipeline (quantize -> segment -> fill -> border-trace ->
    order) end to end on a tiny synthetic "Gemini-like" image - a solid red
    square on a white background - without touching the network."""

    def setUp(self):
        self.tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmpdir.cleanup)
        image = np.full((60, 60, 3), (255, 255, 255), dtype=np.uint8)  # white BGR
        image[15:45, 15:45] = (40, 40, 200)  # a red block, BGR
        self.image_path = Path(self.tmpdir.name) / "fake_gemini_output.png"
        cv2.imwrite(str(self.image_path), image)

    def test_build_regions_finds_the_red_block(self):
        regions, dropped, label_image, image_size, scale, offset = build_regions(CONFIG, self.image_path)
        red_regions = [r for r in regions if r["color_name"] == "red"]
        self.assertEqual(len(red_regions), 1)
        self.assertEqual(image_size, (60, 60))

    def test_full_pipeline_produces_valid_painting_paths(self):
        regions, dropped, label_image, image_size, scale, offset = build_regions(CONFIG, self.image_path)
        path_generation = CONFIG["path_generation"]
        home_position = tuple(path_generation["home_position_mm"])
        skip_white = CONFIG["segmentation"]["skip_white_regions"]

        strokes_by_color = build_region_strokes(regions, path_generation, scale, offset, skip_white)
        border_strokes = build_border_strokes(regions, CONFIG["border_generation"], scale, offset)
        commands = order_and_build_commands(
            strokes_by_color, border_strokes, PALETTE_COLORS, home_position
        )

        self.assertTrue(any(cmd["command"] == "paint_stroke" for cmd in commands))

        region_debug = {
            "num_regions_total": len(regions) + dropped,
            "num_regions_after_filter": len(regions),
            "num_regions_dropped_small": dropped,
            "num_regions_by_color": {},
            "num_border_strokes": len(border_strokes),
        }
        painting_paths = build_painting_paths(
            commands, CONFIG, Path("configs/unittest.json"), Path("fake_source.jpg"),
            self.image_path, region_debug,
        )
        validation = validate_painting_paths(painting_paths)
        self.assertTrue(validation["passed"], validation["errors"])


if __name__ == "__main__":
    unittest.main()
