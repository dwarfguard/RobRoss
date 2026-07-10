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
        "artwork": {
            "palette_mode": "monochrome",
            "min_split_depth": 2,
            "max_split_depth": 5,
        },
        "path_generation": {
            "tool_width_mm": 1.0,
            "stroke_overlap_ratio": 0.0,
            "edge_inset_mm": 5.0,
        },
        "output": {
            "directory": "output",
            "painting_plan_file": "painting_plan.json",
            "painting_paths_file": "painting_paths.json",
            "preview_svg_file": "mondrian_preview.svg",
            "path_preview_svg_file": "path_preview.svg",
        },
    }


class TestValidateConfig(unittest.TestCase):
    def test_valid_config_has_no_errors(self):
        self.assertEqual(validate_config(make_valid_config()), [])

    def test_repo_configs_are_valid(self):
        for name in ("demo_v1_a4_pen.json", "mondrian_12x12_paint.json"):
            with self.subTest(config=name):
                config = load_config(CONFIGS_DIR / name)
                self.assertEqual(validate_config(config), [])

    def test_missing_section_is_reported(self):
        config = make_valid_config()
        del config["artwork"]
        errors = validate_config(config)
        self.assertTrue(any("artwork" in error for error in errors))

    def test_margin_defaults_to_zero_when_absent(self):
        config = make_valid_config()
        del config["canvas"]["margin_mm"]
        self.assertEqual(validate_config(config), [])

    def test_negative_margin_is_an_error(self):
        config = make_valid_config()
        config["canvas"]["margin_mm"] = -1.0
        errors = validate_config(config)
        self.assertTrue(any("margin_mm" in error for error in errors))

    def test_margin_larger_than_half_the_short_side_is_an_error(self):
        config = make_valid_config()
        config["canvas"]["margin_mm"] = 105.0  # 2 * 105 >= 210 (A4 width)
        errors = validate_config(config)
        self.assertTrue(any("margin_mm" in error for error in errors))

    def test_min_split_depth_must_be_a_non_negative_integer(self):
        for bad_value in (-1, 1.5, "2", True):
            config = make_valid_config()
            config["artwork"]["min_split_depth"] = bad_value
            errors = validate_config(config)
            self.assertTrue(
                any("min_split_depth" in error for error in errors),
                f"expected error for min_split_depth={bad_value!r}",
            )

    def test_overlap_ratio_of_one_is_an_error(self):
        config = make_valid_config()
        config["path_generation"]["stroke_overlap_ratio"] = 1.0
        errors = validate_config(config)
        self.assertTrue(any("stroke_overlap_ratio" in error for error in errors))


class TestLoadConfig(unittest.TestCase):
    def test_missing_file_raises_config_error(self):
        with self.assertRaises(ConfigError):
            load_config(CONFIGS_DIR / "does_not_exist.json")


if __name__ == "__main__":
    unittest.main()
