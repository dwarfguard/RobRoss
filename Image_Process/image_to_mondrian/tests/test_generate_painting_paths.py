import unittest

import context  # noqa: F401

from generate_painting_paths import (
    build_border_strokes,
    build_painting_paths,
    build_region_strokes,
    build_regions,
    order_and_build_commands,
)
from path_validation import validate_painting_paths

FIXTURE_IMAGE = context.FIXTURES_DIR / "tiny_test_image.png"

PALETTE = [
    {"name": "white", "hex": "#ffffff"},
    {"name": "yellow", "hex": "#f1c40f"},
    {"name": "red", "hex": "#d62828"},
    {"name": "blue", "hex": "#1d3557"},
    {"name": "black", "hex": "#000000"},
]


def make_config() -> dict:
    return {
        "profile_name": "test_fixture",
        "project": "R.O.B Ross",
        "style": "image_to_mondrian",
        "canvas": {"width_mm": 100.0, "height_mm": 150.0, "margin_mm": 5.0, "origin": "top-left"},
        "source_image": {
            "path": str(FIXTURE_IMAGE),
            "blur_kernel_size": 1,
            "blur_sigma": 0,
            "downscale_max_dimension_px": None,
        },
        "palette": {"colors": PALETTE, "color_space": "lab"},
        "segmentation": {
            "min_region_area_mm2": 0.01,
            "morph_open_kernel_px": 0,
            "skip_white_regions": True,
        },
        "path_generation": {
            "tool_width_mm": 1.0,
            "stroke_overlap_ratio": 0.1,
            "mask_erosion_mm": 0.3,
            "home_position_mm": [5.0, 5.0],
        },
        "border_generation": {"draw_borders": True, "simplify_epsilon_ratio": 0.01},
        "output": {
            "directory": "output",
            "painting_paths_file": "test_fixture_painting_paths.json",
            "preview_svg_file": "test_fixture_preview.svg",
        },
    }


class TestEndToEnd(unittest.TestCase):
    def test_fixture_image_produces_valid_painting_paths(self):
        config = make_config()

        regions, dropped_count, label_image, image_size, scale_mm_per_px, offset_mm = build_regions(config)
        # White background + red block + blue block = 3 distinct regions.
        self.assertEqual(len(regions), 3)
        self.assertEqual(dropped_count, 0)

        strokes_by_color = build_region_strokes(
            regions, config["path_generation"], scale_mm_per_px, offset_mm,
            skip_white=config["segmentation"]["skip_white_regions"],
        )
        # White is skipped, so only red and blue should have fill strokes.
        self.assertEqual(set(strokes_by_color.keys()), {"red", "blue"})

        border_strokes = build_border_strokes(
            label_image, config["border_generation"], scale_mm_per_px, offset_mm
        )
        self.assertGreater(len(border_strokes), 0)

        commands = order_and_build_commands(
            strokes_by_color, border_strokes, PALETTE, tuple(config["path_generation"]["home_position_mm"])
        )
        self.assertTrue(any(cmd["command"] == "paint_stroke" for cmd in commands))

        painting_paths = build_painting_paths(
            commands, config, "test_config.json", FIXTURE_IMAGE,
            {
                "num_regions_total": len(regions) + dropped_count,
                "num_regions_after_filter": len(regions),
                "num_regions_dropped_small": dropped_count,
                "num_regions_by_color": {},
                "num_border_strokes": len(border_strokes),
            },
        )

        result = validate_painting_paths(painting_paths)
        self.assertTrue(result["passed"], result["errors"])
        self.assertEqual(painting_paths["debug"]["num_regions_after_filter"], 3)


if __name__ == "__main__":
    unittest.main()
