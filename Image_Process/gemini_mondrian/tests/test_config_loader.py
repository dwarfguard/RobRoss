import unittest

from context import CONFIGS_DIR

from config_loader import ConfigError, load_config, validate_config


def make_valid_config() -> dict:
    """A minimal config that passes validation, for mutation in tests."""
    return {
        "profile_name": "gemini_mondrian_test",
        "project": "R.O.B Ross",
        "style": "gemini_mondrian",
        "canvas": {
            "width_mm": 210.0,
            "height_mm": 297.0,
            "margin_mm": 10.0,
            "origin": "top-left",
        },
        "source_image": {
            "path": "Image_Process/assets/sample.jpg",
        },
        "gemini": {
            "model": "gemini-2.5-flash-image",
            "prompt": "Redraw this photo in the style of a Piet Mondrian painting.",
        },
        "palette": {
            "colors": [
                {"name": "white", "hex": "#ffffff"},
                {"name": "yellow", "hex": "#f1c40f"},
                {"name": "red", "hex": "#d62828"},
                {"name": "blue", "hex": "#1d3557"},
                {"name": "black", "hex": "#000000"},
            ],
            "color_space": "lab",
        },
        "segmentation": {
            "min_region_area_mm2": 30.0,
            "morph_open_kernel_px": 3,
            "skip_white_regions": True,
        },
        "path_generation": {
            "tool_width_mm": 3.0,
            "stroke_overlap_ratio": 0.15,
            "mask_erosion_mm": 1.0,
            "home_position_mm": [10.0, 10.0],
        },
        "border_generation": {
            "draw_borders": True,
            "simplify_epsilon_ratio": 0.0015,
        },
        "output": {
            "directory": "output/gemini_mondrian_test",
            "gemini_output_image_file": "gemini_styled.png",
            "painting_paths_file": "gemini_mondrian_painting_paths.json",
            "preview_svg_file": "gemini_mondrian_path_preview.svg",
        },
    }


class TestValidateConfig(unittest.TestCase):
    def test_valid_config_has_no_errors(self):
        self.assertEqual(validate_config(make_valid_config()), [])

    def test_demo_config_is_valid(self):
        config = load_config(CONFIGS_DIR / "gemini_mondrian_demo_a4.json")
        self.assertEqual(validate_config(config), [])

    def test_lenna_config_is_valid(self):
        config = load_config(CONFIGS_DIR / "gemini_mondrian_lenna_a4.json")
        self.assertEqual(validate_config(config), [])

    def test_missing_section_is_reported(self):
        config = make_valid_config()
        del config["gemini"]
        errors = validate_config(config)
        self.assertTrue(any("gemini" in error for error in errors))

    def test_margin_larger_than_half_the_short_side_is_an_error(self):
        config = make_valid_config()
        config["canvas"]["margin_mm"] = 105.0  # 2 * 105 >= 210 (A4 width)
        errors = validate_config(config)
        self.assertTrue(any("margin_mm" in error for error in errors))

    def test_empty_source_image_path_is_an_error(self):
        config = make_valid_config()
        config["source_image"]["path"] = ""
        errors = validate_config(config)
        self.assertTrue(any("source_image.path" in error for error in errors))

    def test_missing_gemini_model_is_an_error(self):
        config = make_valid_config()
        del config["gemini"]["model"]
        errors = validate_config(config)
        self.assertTrue(any("gemini.model" in error for error in errors))

    def test_missing_gemini_prompt_is_an_error(self):
        config = make_valid_config()
        del config["gemini"]["prompt"]
        errors = validate_config(config)
        self.assertTrue(any("gemini.prompt" in error for error in errors))

    def test_empty_palette_is_an_error(self):
        config = make_valid_config()
        config["palette"]["colors"] = []
        errors = validate_config(config)
        self.assertTrue(any("palette.colors" in error for error in errors))

    def test_duplicate_palette_name_is_an_error(self):
        config = make_valid_config()
        config["palette"]["colors"][0]["name"] = "black"
        errors = validate_config(config)
        self.assertTrue(any("duplicated" in error for error in errors))

    def test_bad_hex_is_an_error(self):
        config = make_valid_config()
        config["palette"]["colors"][0]["hex"] = "not-a-color"
        errors = validate_config(config)
        self.assertTrue(any(".hex" in error for error in errors))

    def test_min_region_area_must_be_positive(self):
        config = make_valid_config()
        config["segmentation"]["min_region_area_mm2"] = 0
        errors = validate_config(config)
        self.assertTrue(any("min_region_area_mm2" in error for error in errors))

    def test_negative_morph_open_kernel_is_an_error(self):
        config = make_valid_config()
        config["segmentation"]["morph_open_kernel_px"] = -1
        errors = validate_config(config)
        self.assertTrue(any("morph_open_kernel_px" in error for error in errors))

    def test_overlap_ratio_of_one_is_an_error(self):
        config = make_valid_config()
        config["path_generation"]["stroke_overlap_ratio"] = 1.0
        errors = validate_config(config)
        self.assertTrue(any("stroke_overlap_ratio" in error for error in errors))

    def test_negative_mask_erosion_is_an_error(self):
        config = make_valid_config()
        config["path_generation"]["mask_erosion_mm"] = -1.0
        errors = validate_config(config)
        self.assertTrue(any("mask_erosion_mm" in error for error in errors))

    def test_non_bool_draw_borders_is_an_error(self):
        config = make_valid_config()
        config["border_generation"]["draw_borders"] = "yes"
        errors = validate_config(config)
        self.assertTrue(any("draw_borders" in error for error in errors))

    def test_missing_gemini_output_image_file_is_an_error(self):
        config = make_valid_config()
        del config["output"]["gemini_output_image_file"]
        errors = validate_config(config)
        self.assertTrue(any("output.gemini_output_image_file" in error for error in errors))

    def test_missing_output_directory_is_an_error(self):
        config = make_valid_config()
        del config["output"]["directory"]
        errors = validate_config(config)
        self.assertTrue(any("output.directory" in error for error in errors))


class TestLoadConfig(unittest.TestCase):
    def test_missing_file_raises_config_error(self):
        with self.assertRaises(ConfigError):
            load_config(CONFIGS_DIR / "does_not_exist.json")


if __name__ == "__main__":
    unittest.main()
