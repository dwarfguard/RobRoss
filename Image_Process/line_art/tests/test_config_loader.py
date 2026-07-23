import unittest

from context import CONFIGS_DIR

from config_loader import ConfigError, load_config, validate_config


def make_valid_config() -> dict:
    """A minimal config that passes validation, for mutation in tests."""
    return {
        "profile_name": "line_art_test",
        "project": "R.O.B Ross",
        "style": "line_art",
        "canvas": {
            "width_mm": 210.0,
            "height_mm": 297.0,
            "margin_mm": 10.0,
            "origin": "top-left",
        },
        "source_image": {
            "path": "Image_Process/assets/hinton.png",
            "binary_threshold": 200,
            "morph_close_kernel_px": 2,
            "min_spur_length_px": 4.0,
            "min_stroke_length_mm": 1.0,
            "simplify_epsilon_ratio": 0.002,
        },
        "path_generation": {
            "tool_width_mm": 1.0,
            "home_position_mm": [10.0, 10.0],
        },
        "output": {
            "directory": "output/line_art_test",
            "painting_paths_file": "line_art_painting_paths.json",
            "preview_svg_file": "line_art_path_preview.svg",
        },
    }


class TestValidateConfig(unittest.TestCase):
    def test_valid_config_has_no_errors(self):
        self.assertEqual(validate_config(make_valid_config()), [])

    def test_demo_config_is_valid(self):
        config = load_config(CONFIGS_DIR / "line_art_demo_a4.json")
        self.assertEqual(validate_config(config), [])

    def test_missing_section_is_reported(self):
        config = make_valid_config()
        del config["source_image"]
        errors = validate_config(config)
        self.assertTrue(any("source_image" in error for error in errors))

    def test_margin_larger_than_half_the_short_side_is_an_error(self):
        config = make_valid_config()
        config["canvas"]["margin_mm"] = 105.0  # 2 * 105 >= 210 (A4 width)
        errors = validate_config(config)
        self.assertTrue(any("margin_mm" in error for error in errors))

    def test_wrong_origin_is_an_error(self):
        config = make_valid_config()
        config["canvas"]["origin"] = "bottom-left"
        errors = validate_config(config)
        self.assertTrue(any("origin" in error for error in errors))

    def test_empty_source_image_path_is_an_error(self):
        config = make_valid_config()
        config["source_image"]["path"] = ""
        errors = validate_config(config)
        self.assertTrue(any("source_image.path" in error for error in errors))

    def test_binary_threshold_is_optional(self):
        config = make_valid_config()
        del config["source_image"]["binary_threshold"]
        self.assertEqual(validate_config(config), [])

    def test_binary_threshold_out_of_range_is_an_error(self):
        config = make_valid_config()
        config["source_image"]["binary_threshold"] = 300
        errors = validate_config(config)
        self.assertTrue(any("binary_threshold" in error for error in errors))

    def test_negative_binary_threshold_is_an_error(self):
        config = make_valid_config()
        config["source_image"]["binary_threshold"] = -1
        errors = validate_config(config)
        self.assertTrue(any("binary_threshold" in error for error in errors))

    def test_morph_close_kernel_px_is_optional(self):
        config = make_valid_config()
        del config["source_image"]["morph_close_kernel_px"]
        self.assertEqual(validate_config(config), [])

    def test_negative_morph_close_kernel_px_is_an_error(self):
        config = make_valid_config()
        config["source_image"]["morph_close_kernel_px"] = -1
        errors = validate_config(config)
        self.assertTrue(any("morph_close_kernel_px" in error for error in errors))

    def test_non_integer_morph_close_kernel_px_is_an_error(self):
        config = make_valid_config()
        config["source_image"]["morph_close_kernel_px"] = 2.5
        errors = validate_config(config)
        self.assertTrue(any("morph_close_kernel_px" in error for error in errors))

    def test_min_spur_length_px_is_optional(self):
        config = make_valid_config()
        del config["source_image"]["min_spur_length_px"]
        self.assertEqual(validate_config(config), [])

    def test_negative_min_spur_length_px_is_an_error(self):
        config = make_valid_config()
        config["source_image"]["min_spur_length_px"] = -1
        errors = validate_config(config)
        self.assertTrue(any("min_spur_length_px" in error for error in errors))

    def test_min_stroke_length_mm_is_optional(self):
        config = make_valid_config()
        del config["source_image"]["min_stroke_length_mm"]
        self.assertEqual(validate_config(config), [])

    def test_negative_min_stroke_length_mm_is_an_error(self):
        config = make_valid_config()
        config["source_image"]["min_stroke_length_mm"] = -1
        errors = validate_config(config)
        self.assertTrue(any("min_stroke_length_mm" in error for error in errors))

    def test_simplify_epsilon_ratio_must_be_positive(self):
        config = make_valid_config()
        config["source_image"]["simplify_epsilon_ratio"] = 0
        errors = validate_config(config)
        self.assertTrue(any("simplify_epsilon_ratio" in error for error in errors))

    def test_tool_width_must_be_positive(self):
        config = make_valid_config()
        config["path_generation"]["tool_width_mm"] = 0
        errors = validate_config(config)
        self.assertTrue(any("tool_width_mm" in error for error in errors))

    def test_home_position_must_be_two_numbers(self):
        config = make_valid_config()
        config["path_generation"]["home_position_mm"] = [1.0]
        errors = validate_config(config)
        self.assertTrue(any("home_position_mm" in error for error in errors))

    def test_home_position_is_optional(self):
        config = make_valid_config()
        del config["path_generation"]["home_position_mm"]
        self.assertEqual(validate_config(config), [])

    def test_missing_painting_paths_file_is_an_error(self):
        config = make_valid_config()
        del config["output"]["painting_paths_file"]
        errors = validate_config(config)
        self.assertTrue(any("output.painting_paths_file" in error for error in errors))

    def test_missing_preview_svg_file_is_an_error(self):
        config = make_valid_config()
        del config["output"]["preview_svg_file"]
        errors = validate_config(config)
        self.assertTrue(any("output.preview_svg_file" in error for error in errors))


class TestLoadConfig(unittest.TestCase):
    def test_missing_file_raises_config_error(self):
        with self.assertRaises(ConfigError):
            load_config(CONFIGS_DIR / "does_not_exist.json")


if __name__ == "__main__":
    unittest.main()
