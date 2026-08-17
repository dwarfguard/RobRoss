import unittest
from pathlib import Path

import context  # noqa: F401

import generate_output_gallery as gallery


def _run(debug):
    return {
        "name": "sketch_demo_a4",
        "run_dir": Path("output/sketch_demo_a4"),
        "files": {
            "paths_json": "a_very_long_painting_paths_filename_that_must_wrap.json",
            "path_preview_svg": "a_very_long_path_preview_filename_that_must_wrap.svg",
        },
        "painting_paths": {
            "style": "sketch",
            "canvas": {"width_mm": 210, "height_mm": 297},
            "validation": {"passed": True, "errors": []},
            "debug": debug,
        },
        "created_ts": 1,
    }


class TestSemanticBadge(unittest.TestCase):
    def test_semantic_badge_variants(self):
        self.assertIn(
            "Gemini enhanced",
            gallery.render_semantic_badge(
                {
                    "semantic_preprocessing_status": "used",
                    "semantic_preprocessing_provider": "gemini",
                }
            ),
        )
        self.assertIn(
            "Qwen disabled",
            gallery.render_semantic_badge(
                {
                    "semantic_preprocessing_status": "disabled",
                    "semantic_preprocessing_provider": "qwen",
                }
            ),
        )
        fallback = gallery.render_semantic_badge(
            {
                "semantic_preprocessing_status": "local_fallback",
                "semantic_preprocessing_reason": "missing_dependency",
            }
        )
        self.assertIn("Local fallback: missing dependency", fallback)

        reused = gallery.render_semantic_badge(
            {
                "semantic_preprocessing_status": "reused",
                "semantic_preprocessing_provider": "qwen",
            }
        )
        self.assertIn("Qwen image reused", reused)

    def test_unknown_reason_does_not_expose_raw_exception_or_api_key(self):
        secret = "RuntimeError: request failed with key=SECRET-123"
        markup = gallery.render_card(
            _run(
                {
                    "semantic_preprocessing_status": "local_fallback",
                    "semantic_preprocessing_reason": secret,
                }
            )
        )

        self.assertIn("Local fallback: unknown reason", markup)
        self.assertNotIn(secret, markup)
        self.assertNotIn("SECRET-123", markup)


class TestGalleryContainment(unittest.TestCase):
    def test_long_diagnostic_values_remain_in_wrapping_card_markup(self):
        long_value = "output/" + "deeply_nested_diagnostic_segment_" * 20
        markup = gallery.render_card(_run({"diagnostic_path": long_value}))

        self.assertIn(long_value, markup)
        self.assertIn("<table class='stats'>", markup)
        self.assertIn("min-width: 0", gallery.STYLE)
        self.assertIn("table-layout: fixed", gallery.STYLE)
        self.assertIn("overflow-wrap: anywhere", gallery.STYLE)
        self.assertIn("word-break: break-word", gallery.STYLE)
        self.assertIn("max-width: 100%", gallery.STYLE)
        self.assertIn("minmax(min(340px, 100%), 1fr)", gallery.STYLE)


if __name__ == "__main__":
    unittest.main()
