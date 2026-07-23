import unittest

from context import CONFIGS_DIR

from config_loader import ConfigError, load_config, validate_config


def make_valid_config() -> dict:
    """A minimal config that passes validation, for mutation in tests."""
    return {
        "canvas": {
            "width_mm": 210.0,
            "height_mm": 297.0,
            "margin_mm": 10.0,
            "origin": "top-left",
        },
        "source_image": {
            "path": "Image_Process/assets/sample.jpg",
            "blur_kernel_size": 5,
            "blur_sigma": 0,
            "downscale_max_dimension_px": 400,
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
            "min_region_area_mm2": 25.0,
            "morph_open_kernel_px": 3,
            "skip_white_regions": True,
        },
        "path_generation": {
            "tool_width_mm": 3.0,
            "stroke_overlap_ratio": 0.15,
            "mask_erosion_mm": 1.5,
            "home_position_mm": [10.0, 10.0],
        },
        "border_generation": {
            "draw_borders": True,
            "simplify_epsilon_ratio": 0.002,
        },
        "output": {
            "directory": "output",
            "painting_paths_file": "image_to_mondrian_painting_paths.json",
            "preview_svg_file": "image_to_mondrian_path_preview.svg",
        },
    }


class TestValidateConfig(unittest.TestCase):
    def test_valid_config_has_no_errors(self):
        self.assertEqual(validate_config(make_valid_config()), [])

    def test_repo_config_is_valid(self):
        config = load_config(CONFIGS_DIR / "image_to_mondrian_demo_a4.json")
        self.assertEqual(validate_config(config), [])

    def test_missing_section_is_reported(self):
        config = make_valid_config()
        del config["palette"]
        errors = validate_config(config)
        self.assertTrue(any("palette" in error for error in errors))

    def test_margin_larger_than_half_the_short_side_is_an_error(self):
        config = make_valid_config()
        config["canvas"]["margin_mm"] = 105.0  # 2 * 105 >= 210 (A4 width)
        errors = validate_config(config)
        self.assertTrue(any("margin_mm" in error for error in errors))

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

    def test_invalid_color_space_is_an_error(self):
        config = make_valid_config()
        config["palette"]["color_space"] = "cmyk"
        errors = validate_config(config)
        self.assertTrue(any("color_space" in error for error in errors))

    def test_negative_bilateral_d_is_an_error(self):
        config = make_valid_config()
        config["source_image"]["bilateral_d"] = -1
        errors = validate_config(config)
        self.assertTrue(any("bilateral_d" in error for error in errors))

    def test_bilateral_d_is_optional(self):
        config = make_valid_config()
        self.assertNotIn("bilateral_d", config["source_image"])
        self.assertEqual(validate_config(config), [])

    def test_zero_bilateral_sigma_color_is_an_error(self):
        config = make_valid_config()
        config["source_image"]["bilateral_sigma_color"] = 0
        errors = validate_config(config)
        self.assertTrue(any("bilateral_sigma_color" in error for error in errors))

    def test_neutral_chroma_percentile_out_of_range_is_an_error(self):
        config = make_valid_config()
        config["palette"]["neutral_chroma_percentile"] = 150
        errors = validate_config(config)
        self.assertTrue(any("neutral_chroma_percentile" in error for error in errors))

    def test_neutral_chroma_percentile_is_optional(self):
        config = make_valid_config()
        self.assertNotIn("neutral_chroma_percentile", config["palette"])
        self.assertEqual(validate_config(config), [])

    def test_non_bool_protect_face_features_is_an_error(self):
        config = make_valid_config()
        config["segmentation"]["protect_face_features"] = "yes"
        errors = validate_config(config)
        self.assertTrue(any("protect_face_features" in error for error in errors))

    def test_negative_face_protection_margin_is_an_error(self):
        config = make_valid_config()
        config["segmentation"]["face_protection_margin_px"] = -1
        errors = validate_config(config)
        self.assertTrue(any("face_protection_margin_px" in error for error in errors))

    def test_negative_neutral_chroma_threshold_is_an_error(self):
        config = make_valid_config()
        config["palette"]["neutral_chroma_threshold"] = -5
        errors = validate_config(config)
        self.assertTrue(any("neutral_chroma_threshold" in error for error in errors))

    def test_neutral_chroma_threshold_is_optional(self):
        config = make_valid_config()
        self.assertNotIn("neutral_chroma_threshold", config["palette"])
        self.assertEqual(validate_config(config), [])

    def test_min_region_area_must_be_positive(self):
        config = make_valid_config()
        config["segmentation"]["min_region_area_mm2"] = 0
        errors = validate_config(config)
        self.assertTrue(any("min_region_area_mm2" in error for error in errors))

    def test_negative_morph_close_kernel_is_an_error(self):
        config = make_valid_config()
        config["segmentation"]["morph_close_kernel_px"] = -1
        errors = validate_config(config)
        self.assertTrue(any("morph_close_kernel_px" in error for error in errors))

    def test_morph_close_kernel_is_optional(self):
        config = make_valid_config()
        self.assertNotIn("morph_close_kernel_px", config["segmentation"])
        self.assertEqual(validate_config(config), [])

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


class TestLoadConfig(unittest.TestCase):
    def test_missing_file_raises_config_error(self):
        with self.assertRaises(ConfigError):
            load_config(CONFIGS_DIR / "does_not_exist.json")


if __name__ == "__main__":
    unittest.main()
