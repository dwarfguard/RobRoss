import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import context  # noqa: F401

import app as webapp
import route_adapters


class _Upload:
    filename = "portrait.png"

    def save(self, path):
        Path(path).write_bytes(b"image")


class TestSketchSemanticControl(unittest.TestCase):
    def test_form_exposes_checked_sketch_only_control(self):
        markup = webapp.render_upload_form()

        self.assertIn("Enhance overlapping forms with Qwen", markup)
        self.assertIn('id="semantic-input" checked', markup)
        self.assertIn('id="qwen-model-input"', markup)
        self.assertIn('value="qwen-image-2.0-pro"', markup)
        self.assertIn("Maximum detail paths", markup)
        self.assertIn('id="detail-max-paths-input"', markup)
        self.assertIn('value="400"', markup)
        self.assertIn('min="0" step="1"', markup)
        self.assertIn("Line-art threshold", markup)
        self.assertIn('id="line-art-threshold-input"', markup)
        self.assertIn('value="210"', markup)
        self.assertIn("Semantic outline floor", markup)
        self.assertIn('id="outline-floor-input"', markup)
        self.assertIn('value="1.0"', markup)
        self.assertIn("Smoothing radius", markup)
        self.assertIn('id="smoothing-radius-input"', markup)
        self.assertIn('value="0.5"', markup)
        self.assertIn('id="continuous-paths-input" checked', markup)
        self.assertIn("routeSelect.value === 'sketch'", markup)

    def test_process_passes_checked_and_unchecked_values(self):
        client = webapp.app.test_client()
        for checkbox_value, expected in (("on", True), (None, False)):
            data = {
                "route": "sketch",
                "image": (io.BytesIO(b"image"), "portrait.png"),
            }
            if checkbox_value is not None:
                data["enhance_overlapping_forms"] = checkbox_value
            with self.subTest(expected=expected), mock.patch.object(
                route_adapters, "run_route", return_value="run"
            ) as run_route:
                response = client.post(
                    "/process", data=data, content_type="multipart/form-data"
                )
                self.assertEqual(response.status_code, 303)
                self.assertEqual(
                    run_route.call_args.kwargs["semantic_preprocessing_enabled"],
                    expected,
                )
                self.assertEqual(
                    run_route.call_args.kwargs["qwen_model"],
                    "qwen-image-2.0-pro",
                )
                self.assertEqual(run_route.call_args.kwargs["detail_max_paths"], 400)
                self.assertEqual(run_route.call_args.kwargs["line_art_threshold"], 210)
                self.assertEqual(
                    run_route.call_args.kwargs["outline_min_path_length_mm"], 1.0
                )
                self.assertEqual(run_route.call_args.kwargs["smoothing_radius_mm"], 0.5)
                self.assertFalse(run_route.call_args.kwargs["continuous_paths"])

    def test_process_accepts_explicit_qwen_model(self):
        client = webapp.app.test_client()
        with mock.patch.object(
            route_adapters, "run_route", return_value="run"
        ) as run_route:
            response = client.post(
                "/process",
                data={
                    "route": "sketch",
                    "qwen_model": "qwen-image-edit-plus",
                    "image": (io.BytesIO(b"image"), "portrait.png"),
                },
                content_type="multipart/form-data",
            )

        self.assertEqual(response.status_code, 303)
        self.assertEqual(
            run_route.call_args.kwargs["qwen_model"], "qwen-image-edit-plus"
        )

    def test_process_accepts_explicit_detail_budget(self):
        client = webapp.app.test_client()
        with mock.patch.object(
            route_adapters, "run_route", return_value="run"
        ) as run_route:
            response = client.post(
                "/process",
                data={
                    "route": "sketch",
                    "maximum_detail_paths": "27",
                    "image": (io.BytesIO(b"image"), "portrait.png"),
                },
                content_type="multipart/form-data",
            )

        self.assertEqual(response.status_code, 303)
        self.assertEqual(run_route.call_args.kwargs["detail_max_paths"], 27)

    def test_process_rejects_invalid_detail_budgets(self):
        client = webapp.app.test_client()
        for invalid in ("", "-1", "2.5", "four"):
            with self.subTest(invalid=invalid), mock.patch.object(
                route_adapters, "run_route", return_value="run"
            ) as run_route:
                response = client.post(
                    "/process",
                    data={
                        "route": "sketch",
                        "maximum_detail_paths": invalid,
                        "image": (io.BytesIO(b"image"), "portrait.png"),
                    },
                    content_type="multipart/form-data",
                )

                self.assertEqual(response.status_code, 400)
                self.assertIn(b"must be a non-negative integer", response.data)
                run_route.assert_not_called()

    def test_run_route_records_choice_in_per_upload_config(self):
        output_parent = Path(context.REPO_ROOT) / "output"
        with tempfile.TemporaryDirectory(
            prefix="webapp_semantic_test_", dir=output_parent
        ) as temporary_output:
            temporary_output = Path(temporary_output)
            with mock.patch.object(route_adapters, "OUTPUT_DIR", temporary_output), mock.patch.object(
                route_adapters.time, "time", return_value=12345
            ), mock.patch.object(route_adapters.subprocess, "run") as run_process:
                run_process.return_value.returncode = 0
                route_adapters.run_route(
                    "sketch",
                    _Upload(),
                    semantic_preprocessing_enabled=False,
                    qwen_model="qwen-image-edit-plus",
                    line_art_threshold=202,
                    outline_min_path_length_mm=0.7,
                    smoothing_radius_mm=0.8,
                    continuous_paths=False,
                    detail_max_paths=27,
                )

            config_path = temporary_output / "upload_12345_sketch" / "_config.json"
            config = json.loads(config_path.read_text(encoding="utf-8"))
            self.assertFalse(config["semantic_preprocessing"]["enabled"])
            self.assertEqual(
                config["semantic_preprocessing"]["model"], "qwen-image-edit-plus"
            )
            self.assertEqual(config["detail_extraction"]["max_paths"], 27)
            self.assertEqual(config["outline_extraction"]["line_art_threshold"], 202)
            self.assertEqual(config["outline_extraction"]["min_path_length_mm"], 0.7)
            self.assertEqual(config["path_generation"]["smoothing_radius_mm"], 0.8)
            self.assertFalse(config["path_generation"]["continuous_paths"])

    def test_process_passes_explicit_tracing_settings(self):
        client = webapp.app.test_client()
        with mock.patch.object(
            route_adapters, "run_route", return_value="run"
        ) as run_route:
            response = client.post(
                "/process",
                data={
                    "route": "sketch",
                    "line_art_threshold": "201",
                    "semantic_outline_floor_mm": "0.6",
                    "smoothing_radius_mm": "1.2",
                    "continuous_paths": "on",
                    "maximum_detail_paths": "31",
                    "image": (io.BytesIO(b"image"), "portrait.png"),
                },
                content_type="multipart/form-data",
            )
        self.assertEqual(response.status_code, 303)
        self.assertEqual(run_route.call_args.kwargs["line_art_threshold"], 201)
        self.assertEqual(run_route.call_args.kwargs["outline_min_path_length_mm"], 0.6)
        self.assertEqual(run_route.call_args.kwargs["smoothing_radius_mm"], 1.2)
        self.assertTrue(run_route.call_args.kwargs["continuous_paths"])

    def test_run_route_rejects_invalid_detail_budget_before_creating_run(self):
        with self.assertRaisesRegex(ValueError, "non-negative integer"):
            route_adapters.run_route(
                "sketch", _Upload(), detail_max_paths=True
            )

    def test_rejects_blank_qwen_model_before_creating_run(self):
        with self.assertRaisesRegex(ValueError, "Qwen model"):
            route_adapters.run_route("sketch", _Upload(), qwen_model="  ")


if __name__ == "__main__":
    unittest.main()
