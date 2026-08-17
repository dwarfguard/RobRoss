import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

import context  # noqa: F401
import cv2
import numpy as np
from PIL import Image

from generate_sketch_paths import (
    build_commands,
    build_two_phase_strokes,
    rank_detail_candidates,
    remove_outline_duplicate_segments,
    render_svg,
)
from semantic_preprocessing import SemanticPreprocessingError


def _pipeline_config(image_path, output_dir):
    return {
        "project": "test",
        "profile_name": "two_phase_test",
        "style": "sketch",
        "canvas": {
            "width_mm": 100.0,
            "height_mm": 100.0,
            "margin_mm": 5.0,
            "origin": "top-left",
        },
        "source_image": {
            "path": str(image_path),
            "canny_threshold1": 30,
            "canny_threshold2": 90,
            "simplify_epsilon_ratio": 0.002,
            "simplify_min_epsilon_px": 0.5,
        },
        "semantic_preprocessing": {
            "enabled": True,
            "provider": "qwen",
            "model": "test-model",
            "prompt": "test prompt",
            "timeout_seconds": 1,
            "fallback_to_local": True,
        },
        "outline_extraction": {
            "canny_threshold1": 30,
            "canny_threshold2": 90,
            "blur_kernel_size": 3,
            "contrast_clip_limit": 2.0,
            "line_art_threshold": 220,
            "min_path_length_mm": 1.0,
            "border_suppression_band_px": 2,
        },
        "detail_extraction": {
            "canny_threshold1": 30,
            "canny_threshold2": 90,
            "blur_kernel_size": 3,
            "min_path_length_mm": 0.5,
            "max_paths": 8,
            "deduplication_distance_mm": 0.5,
            "connection_distance_mm": 2.0,
            "isolation_distance_mm": 8.0,
            "isolated_speckle_max_length_mm": 2.0,
        },
        "path_generation": {
            "tool_width_mm": 1.0,
            "home_position_mm": [5.0, 5.0],
            "join_overlapping_strokes": True,
            "max_join_turn_degrees": 60.0,
        },
        "output": {
            "directory": str(output_dir),
            "semantic_outline_image_file": "outline.png",
            "painting_paths_file": "paths.json",
            "preview_svg_file": "preview.svg",
        },
    }


class TestTwoPhaseCommands(unittest.TestCase):
    def test_outlines_precede_details_with_one_tool_setup(self):
        commands = build_commands(
            [[(0.0, 0.0), (10.0, 0.0)], [(10.0, 0.0), (20.0, 0.0)]],
            [[(1.0, 1.0), (2.0, 2.0)]],
        )

        labels = [
            command["label"]
            for command in commands
            if command["command"] == "lower_tool"
        ]
        self.assertEqual(
            labels,
            ["sketch_outline_1", "sketch_outline_2", "sketch_detail_1"],
        )
        self.assertEqual(
            sum(command["command"] == "select_tool" for command in commands), 1
        )
        self.assertEqual(
            sum(command["command"] == "dip_paint" for command in commands), 1
        )

    def test_outline_and_detail_preview_strokes_are_equally_opaque(self):
        commands = build_commands(
            [[(0.0, 0.0), (10.0, 0.0)]],
            [[(1.0, 1.0), (2.0, 2.0)]],
        )
        svg = render_svg(
            {
                "canvas": {
                    "width_mm": 100.0,
                    "height_mm": 100.0,
                    "width_in": 4.0,
                    "height_in": 4.0,
                },
                "path_settings": {"tool_width_mm": 1.0},
                "commands": commands,
            }
        )

        root = ET.fromstring(svg)
        lines = root.findall("{http://www.w3.org/2000/svg}line")
        appearances = {
            (
                line.attrib["stroke"],
                line.attrib["stroke-width"],
                line.attrib["stroke-linecap"],
                line.attrib["stroke-opacity"],
            )
            for line in lines
        }
        self.assertEqual(len(lines), 2)
        self.assertEqual(appearances, {("black", "1.0", "round", "1.0")})


class TestDetailSelection(unittest.TestCase):
    def test_detail_budget_prefers_strong_connected_longer_candidates(self):
        outline = [([(0.0, 0.0), (20.0, 0.0)], False)]
        details = [
            ([(2.0, 0.5), (10.0, 0.5)], False),
            ([(50.0, 50.0), (50.6, 50.0)], False),
            ([(3.0, 4.0), (6.0, 4.0)], False),
        ]
        selected, stats = rank_detail_candidates(
            details,
            outline,
            {
                "min_path_length_mm": 0.5,
                "max_paths": 1,
                "connection_distance_mm": 2.0,
                "isolation_distance_mm": 8.0,
                "isolated_speckle_max_length_mm": 2.0,
            },
            edge_strengths=[1.0, 0.1, 0.4],
        )

        self.assertEqual(selected, [details[0]])
        self.assertEqual(stats["detail_num_isolated_speckles_filtered"], 1)
        self.assertEqual(stats["detail_num_budget_paths_filtered"], 1)

    def test_dedup_removes_covered_segment_without_bridging_the_gap(self):
        outline = [([(0.0, 0.0), (10.0, 0.0)], False)]
        detail = [
            (
                [
                    (0.0, 0.0),
                    (10.0, 0.0),
                    (12.0, 2.0),
                    (14.0, 2.0),
                    (16.0, 0.0),
                ],
                False,
            )
        ]

        remaining, removed = remove_outline_duplicate_segments(
            detail, outline, distance_mm=0.1
        )

        self.assertEqual(removed, 1)
        self.assertEqual(remaining[0][0][0], (10.0, 0.0))
        self.assertNotIn(((0.0, 0.0), (12.0, 2.0)), [
            (points[0], points[1]) for points, _closed in remaining
        ])


class TestSemanticFallbacks(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)
        self.image_path = self.root / "person.png"
        image = np.full((80, 80), 255, dtype=np.uint8)
        cv2.rectangle(image, (18, 10), (62, 70), 0, 2)
        cv2.circle(image, (40, 30), 10, 0, 2)
        cv2.line(image, (25, 48), (55, 48), 0, 2)
        cv2.imwrite(str(self.image_path), image)
        self.config = _pipeline_config(self.image_path, self.root)

    def tearDown(self):
        self.temp_dir.cleanup()

    def _assert_local_fallback(self, generator, expected_reason):
        outlines, details, _image_path, stats = build_two_phase_strokes(
            self.config,
            output_dir=self.root,
            semantic_generator=generator,
        )
        self.assertTrue(outlines or details)
        self.assertEqual(stats["semantic_preprocessing_status"], "local_fallback")
        self.assertEqual(stats["semantic_preprocessing_reason"], expected_reason)
        self.assertTrue((self.root / "outline.png").is_file())

    def test_qwen_success_uses_generated_outline(self):
        generated = np.full((80, 80), 255, dtype=np.uint8)
        cv2.rectangle(generated, (20, 12), (60, 68), 0, 1)
        ok, encoded = cv2.imencode(".png", generated)
        self.assertTrue(ok)

        outlines, _details, _image_path, stats = build_two_phase_strokes(
            self.config,
            output_dir=self.root,
            semantic_generator=lambda *_args: encoded.tobytes(),
        )

        self.assertTrue(outlines)
        self.assertEqual(stats["semantic_preprocessing_status"], "used")
        self.assertEqual(stats["semantic_preprocessing_reason"], "success")
        self.assertEqual(stats["semantic_line_art_threshold"], 220)

    def test_saved_semantic_mode_bypasses_qwen_and_records_provenance(self):
        generated = np.full((80, 80), 255, dtype=np.uint8)
        cv2.rectangle(generated, (20, 12), (60, 68), 0, 1)
        saved = self.root / "saved_qwen.png"
        cv2.imwrite(str(saved), generated)
        original_bytes = saved.read_bytes()
        self.config["retrace_provenance"] = {"source_run": "upload_1_sketch"}

        outlines, _details, _image_path, stats = build_two_phase_strokes(
            self.config,
            output_dir=self.root,
            semantic_generator=lambda *_args: self.fail("Qwen must not run"),
            saved_semantic_image=saved,
        )

        self.assertTrue(outlines)
        self.assertEqual(stats["semantic_preprocessing_status"], "reused")
        self.assertEqual(stats["semantic_preprocessing_reason"], "saved_semantic_image")
        self.assertEqual(stats["retrace_provenance"]["source_run"], "upload_1_sketch")
        self.assertEqual(saved.read_bytes(), original_bytes)

    def test_saved_semantic_mode_fails_closed_for_missing_or_bad_image(self):
        bad = self.root / "bad.png"
        bad.write_bytes(b"not an image")
        for saved in (self.root / "missing.png", bad):
            with self.subTest(saved=saved), self.assertRaisesRegex(
                RuntimeError, "missing or unreadable"
            ):
                build_two_phase_strokes(
                    self.config,
                    output_dir=self.root,
                    semantic_generator=lambda *_args: self.fail("Qwen must not run"),
                    saved_semantic_image=saved,
                )

    def test_semantic_outline_length_floor_retains_one_mm_and_filters_shorter(self):
        generated = np.full((100, 100), 255, dtype=np.uint8)
        # The 100px image maps to a 90mm drawable box. These disconnected
        # marks are approximately 0.9mm and 1.8mm long on the canvas.
        cv2.line(generated, (10, 10), (11, 10), 0, 1)
        cv2.line(generated, (20, 20), (22, 20), 0, 1)
        ok, encoded = cv2.imencode(".png", generated)
        self.assertTrue(ok)
        config = _pipeline_config(self.image_path, self.root)
        config["source_image"]["simplify_min_epsilon_px"] = 0.0
        config["outline_extraction"]["min_path_length_mm"] = 1.0
        config["path_generation"]["join_overlapping_strokes"] = False

        outlines, _details, _image_path, stats = build_two_phase_strokes(
            config,
            output_dir=self.root,
            semantic_generator=lambda *_args: encoded.tobytes(),
        )

        self.assertEqual(len(outlines), 1)
        self.assertEqual(stats["outline_min_path_length_mm"], 1.0)
        self.assertEqual(stats["outline_num_short_paths_filtered"], 1)
        self.assertEqual(
            stats["outline_joined_length_distribution"],
            {
                "under_1_mm": 1,
                "1_to_under_2_mm": 1,
                "2_to_under_5_mm": 0,
                "5_mm_and_over": 0,
            },
        )

        config["outline_extraction"]["min_path_length_mm"] = 2.0
        outlines, _details, _image_path, stats = build_two_phase_strokes(
            config,
            output_dir=self.root,
            semantic_generator=lambda *_args: encoded.tobytes(),
        )
        self.assertEqual(outlines, [])
        self.assertEqual(stats["outline_min_path_length_mm"], 2.0)

    def test_orientation_6_semantic_outline_and_local_details_share_registration(self):
        raw = np.full((40, 80, 3), 255, dtype=np.uint8)
        cv2.rectangle(raw, (12, 6), (42, 32), (0, 0, 0), 2)
        oriented_path = self.root / "oriented.jpg"
        image = Image.fromarray(cv2.cvtColor(raw, cv2.COLOR_BGR2RGB))
        exif = image.getexif()
        exif[274] = 6
        image.save(oriented_path, exif=exif, quality=100, subsampling=0)

        from canny_edges import read_grayscale

        oriented = read_grayscale(oriented_path)
        ok, generated = cv2.imencode(".png", oriented)
        self.assertTrue(ok)
        config = _pipeline_config(oriented_path, self.root)
        config["detail_extraction"].update(
            {
                "blur_kernel_size": 0,
                "deduplication_distance_mm": 0.0,
                "max_paths": 100,
            }
        )

        outlines, details, _image_path, stats = build_two_phase_strokes(
            config,
            output_dir=self.root,
            semantic_generator=lambda *_args: generated.tobytes(),
        )

        self.assertTrue(outlines)
        self.assertTrue(details)
        self.assertEqual(stats["source_exif_orientation"], 6)
        self.assertTrue(stats["source_orientation_normalized"])
        self.assertEqual(
            stats["detail_working_dimensions_px"], {"width": 40, "height": 80}
        )
        outline_points = [point for points, _closed in outlines for point in points]
        detail_points = [point for points, _closed in details for point in points]
        outline_center = tuple(
            sum(point[axis] for point in outline_points) / len(outline_points)
            for axis in (0, 1)
        )
        detail_center = tuple(
            sum(point[axis] for point in detail_points) / len(detail_points)
            for axis in (0, 1)
        )
        self.assertLess(abs(outline_center[0] - detail_center[0]), 5.0)
        self.assertLess(abs(outline_center[1] - detail_center[1]), 5.0)

    def test_detail_filter_defaults_and_explicit_overrides_reach_opencv(self):
        generated = np.full((80, 80), 255, dtype=np.uint8)
        cv2.rectangle(generated, (20, 12), (60, 68), 0, 1)
        ok, encoded = cv2.imencode(".png", generated)
        self.assertTrue(ok)
        real_blur = cv2.GaussianBlur
        real_canny = cv2.Canny

        cases = (
            ({}, (5, 5), (120.0, 240.0)),
            (
                {"blur_kernel_size": 3, "canny_threshold1": 41, "canny_threshold2": 93},
                (3, 3),
                (41.0, 93.0),
            ),
        )
        for overrides, expected_kernel, expected_thresholds in cases:
            with self.subTest(overrides=overrides):
                config = _pipeline_config(self.image_path, self.root)
                for field in ("blur_kernel_size", "canny_threshold1", "canny_threshold2"):
                    config["detail_extraction"].pop(field, None)
                config["detail_extraction"].update(overrides)
                with mock.patch(
                    "generate_sketch_paths.cv2.GaussianBlur", side_effect=real_blur
                ) as blur, mock.patch(
                    "generate_sketch_paths.cv2.Canny", side_effect=real_canny
                ) as canny:
                    build_two_phase_strokes(
                        config,
                        output_dir=self.root,
                        semantic_generator=lambda *_args: encoded.tobytes(),
                    )

                self.assertEqual(blur.call_args.args[1], expected_kernel)
                self.assertEqual(
                    (
                        canny.call_args.kwargs["threshold1"],
                        canny.call_args.kwargs["threshold2"],
                    ),
                    expected_thresholds,
                )

    def test_all_categorized_failures_fall_back_to_valid_local_paths(self):
        for reason in (
            "missing_api_key",
            "missing_dependency",
            "timeout",
            "safety_rejection",
            "network",
            "quota",
            "api_failure",
        ):
            with self.subTest(reason=reason):
                def fail(*_args, failure_reason=reason):
                    raise SemanticPreprocessingError(failure_reason)

                self._assert_local_fallback(fail, reason)

    def test_malformed_image_response_falls_back(self):
        self._assert_local_fallback(lambda *_args: b"not an image", "malformed_response")

    def test_disabled_semantic_preprocessing_is_recorded_and_stays_local(self):
        self.config["semantic_preprocessing"]["enabled"] = False
        outlines, details, _image_path, stats = build_two_phase_strokes(
            self.config,
            output_dir=self.root,
            semantic_generator=lambda *_args: self.fail("generator must not run"),
        )

        self.assertTrue(outlines or details)
        self.assertEqual(stats["semantic_preprocessing_status"], "disabled")
        self.assertEqual(
            stats["semantic_preprocessing_reason"], "disabled_by_config"
        )


if __name__ == "__main__":
    unittest.main()
