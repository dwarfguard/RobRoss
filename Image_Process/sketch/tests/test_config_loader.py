import unittest

from context import CONFIGS_DIR

from config_loader import (
    DEFAULT_DETAIL_BLUR_KERNEL_SIZE,
    DEFAULT_DETAIL_CANNY_THRESHOLD1,
    DEFAULT_DETAIL_CANNY_THRESHOLD2,
    DEFAULT_DETAIL_DEDUPLICATION_DISTANCE_MM,
    DEFAULT_DETAIL_MAX_PATHS,
    DEFAULT_CONTINUOUS_PATHS,
    DEFAULT_OUTLINE_MIN_PATH_LENGTH_MM,
    DEFAULT_SEMANTIC_PROMPT,
    DEFAULT_SEMANTIC_MODEL,
    DEFAULT_SEMANTIC_PROVIDER,
    DEFAULT_SMOOTHING_RADIUS_MM,
    load_config,
    validate_config,
)
from generate_sketch_paths import (
    build_painting_paths,
    resolve_detail_deduplication_distance,
)


class TestConfigLoader(unittest.TestCase):
    def test_demo_config_is_valid(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        self.assertEqual(validate_config(config), [])
        self.assertEqual(DEFAULT_SEMANTIC_PROVIDER, "qwen")
        self.assertEqual(DEFAULT_SEMANTIC_MODEL, "qwen-image-2.0-pro")
        self.assertEqual(config["semantic_preprocessing"]["model"], DEFAULT_SEMANTIC_MODEL)
        self.assertIsInstance(config["semantic_preprocessing"]["prompt"], str)
        self.assertTrue(config["semantic_preprocessing"]["prompt"].strip())

    def test_4k_landscape_config_is_valid(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_4k_landscape.json")
        self.assertEqual(validate_config(config), [])
        self.assertEqual(config["canvas"]["width_mm"], 525.0)
        self.assertEqual(config["canvas"]["height_mm"], 375.0)

    def test_qwen_model_is_configurable(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        config["semantic_preprocessing"]["model"] = "qwen-image-edit-plus"
        self.assertEqual(validate_config(config), [])

    def test_balanced_detail_and_prompt_defaults(self):
        self.assertEqual(DEFAULT_DETAIL_BLUR_KERNEL_SIZE, 5)
        self.assertEqual(DEFAULT_DETAIL_CANNY_THRESHOLD1, 120)
        self.assertEqual(DEFAULT_DETAIL_CANNY_THRESHOLD2, 240)
        self.assertEqual(DEFAULT_DETAIL_MAX_PATHS, 400)
        self.assertEqual(DEFAULT_OUTLINE_MIN_PATH_LENGTH_MM, 1.0)
        self.assertEqual(
            load_config(CONFIGS_DIR / "sketch_demo_a4.json")["outline_extraction"][
                "min_path_length_mm"
            ],
            1.0,
        )
        for constraint in (
            "one clean glasses frame",
            "without lens reflections or duplicate rims",
            "hair primarily as an outer silhouette",
            "individual flyaway hairs",
            "hatching",
            "architecture and background contours",
        ):
            self.assertIn(constraint, DEFAULT_SEMANTIC_PROMPT)

    def test_deduplication_default_tracks_pen_width_and_is_capped(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        self.assertNotIn("deduplication_distance_mm", config["detail_extraction"])

        config["path_generation"]["tool_width_mm"] = 0.4
        self.assertAlmostEqual(resolve_detail_deduplication_distance(config), 0.3)
        config["path_generation"]["tool_width_mm"] = 2.0
        self.assertEqual(
            resolve_detail_deduplication_distance(config),
            DEFAULT_DETAIL_DEDUPLICATION_DISTANCE_MM,
        )

    def test_explicit_deduplication_distance_overrides_pen_default(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        config["detail_extraction"]["deduplication_distance_mm"] = 0.42
        config["path_generation"]["tool_width_mm"] = 2.0

        self.assertEqual(resolve_detail_deduplication_distance(config), 0.42)

    def test_rejects_non_qwen_provider(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        config["semantic_preprocessing"]["provider"] = "gemini"
        self.assertTrue(any("must be 'qwen'" in error for error in validate_config(config)))

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

    def test_new_profile_opts_in_but_missing_fields_keep_legacy_behavior(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        self.assertEqual(config["path_generation"]["smoothing_radius_mm"], 0.5)
        self.assertTrue(config["path_generation"]["continuous_paths"])

        config["path_generation"].pop("smoothing_radius_mm")
        config["path_generation"].pop("continuous_paths")
        self.assertEqual(validate_config(config), [])
        self.assertEqual(DEFAULT_SMOOTHING_RADIUS_MM, 0.0)
        self.assertFalse(DEFAULT_CONTINUOUS_PATHS)

        painting_paths = build_painting_paths(
            [], config, CONFIGS_DIR / "legacy.json", CONFIGS_DIR / "photo.png",
            {"baseline": 0.0, "optimized": 0.0},
        )
        self.assertEqual(painting_paths["path_settings"]["smoothing_radius_mm"], 0.0)
        self.assertFalse(painting_paths["path_settings"]["continuous_paths"])

    def test_rejects_smoothing_out_of_bounds_and_non_boolean_continuous_mode(self):
        config = load_config(CONFIGS_DIR / "sketch_demo_a4.json")
        for invalid in (-0.1, 2.1, True):
            with self.subTest(invalid=invalid):
                config["path_generation"]["smoothing_radius_mm"] = invalid
                self.assertTrue(any(
                    "smoothing_radius_mm" in error for error in validate_config(config)
                ))
        config["path_generation"]["smoothing_radius_mm"] = 0.5
        config["path_generation"]["continuous_paths"] = "yes"
        self.assertTrue(any(
            "continuous_paths" in error for error in validate_config(config)
        ))

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
