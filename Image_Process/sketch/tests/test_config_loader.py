import unittest

from context import CONFIGS_DIR

from config_loader import DEFAULT_DETAIL_MAX_PATHS, load_config, validate_config
from generate_sketch_paths import build_painting_paths


class TestConfigLoader(unittest.TestCase):
    def test_demo_config_is_valid(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        self.assertEqual(validate_config(config), [])

    def test_rejects_negative_minimum_stroke_length(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        config["source_image"]["min_stroke_length_mm"] = -1
        self.assertTrue(
            any("min_stroke_length_mm" in error for error in validate_config(config))
        )

    def test_rejects_non_boolean_join_setting(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        config["path_generation"]["join_overlapping_strokes"] = "yes"
        self.assertTrue(
            any("join_overlapping_strokes" in error for error in validate_config(config))
        )

    def test_rejects_out_of_range_join_turn(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        config["path_generation"]["max_join_turn_degrees"] = -1
        self.assertTrue(
            any("max_join_turn_degrees" in error for error in validate_config(config))
        )

    def test_legacy_config_without_two_phase_sections_uses_valid_defaults(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        config.pop("semantic_preprocessing")
        config.pop("outline_extraction")
        config.pop("detail_extraction")
        config["output"].pop("semantic_outline_image_file")

        self.assertEqual(validate_config(config), [])

    def test_missing_detail_budget_derives_400_path_default(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        config["detail_extraction"].pop("max_paths")

        painting_paths = build_painting_paths(
            [], config, CONFIGS_DIR / "legacy_sketch.json", CONFIGS_DIR / "photo.png",
            {"baseline": 0.0, "optimized": 0.0},
        )

        self.assertEqual(DEFAULT_DETAIL_MAX_PATHS, 400)
        self.assertEqual(painting_paths["path_settings"]["detail_max_paths"], 400)

    def test_explicit_detail_budget_overrides_default(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        config["detail_extraction"]["max_paths"] = 17

        painting_paths = build_painting_paths(
            [], config, CONFIGS_DIR / "sketch.json", CONFIGS_DIR / "photo.png",
            {"baseline": 0.0, "optimized": 0.0},
        )

        self.assertEqual(painting_paths["path_settings"]["detail_max_paths"], 17)

    def test_rejects_detail_length_below_hard_floor(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        config["detail_extraction"]["min_path_length_mm"] = 0.49

        self.assertTrue(
            any("at least 0.5 mm" in error for error in validate_config(config))
        )

    def test_rejects_invalid_semantic_timeout_and_detail_budget(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        config["semantic_preprocessing"]["timeout_seconds"] = 0
        config["detail_extraction"]["max_paths"] = 2.5

        errors = validate_config(config)
        self.assertTrue(any("timeout_seconds" in error for error in errors))
        self.assertTrue(any("max_paths" in error for error in errors))


if __name__ == "__main__":
    unittest.main()
