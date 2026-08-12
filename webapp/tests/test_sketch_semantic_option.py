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

        self.assertIn("Enhance overlapping forms with Gemini", markup)
        self.assertIn('id="semantic-input" checked', markup)
        self.assertIn("Maximum detail paths", markup)
        self.assertIn('id="detail-max-paths-input"', markup)
        self.assertIn('value="400"', markup)
        self.assertIn('min="0" step="1"', markup)
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
                self.assertEqual(run_route.call_args.kwargs["detail_max_paths"], 400)

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
                    detail_max_paths=27,
                )

            config_path = temporary_output / "upload_12345_sketch" / "_config.json"
            config = json.loads(config_path.read_text(encoding="utf-8"))
            self.assertFalse(config["semantic_preprocessing"]["enabled"])
            self.assertEqual(config["detail_extraction"]["max_paths"], 27)

    def test_run_route_rejects_invalid_detail_budget_before_creating_run(self):
        with self.assertRaisesRegex(ValueError, "non-negative integer"):
            route_adapters.run_route(
                "sketch", _Upload(), detail_max_paths=True
            )


if __name__ == "__main__":
    unittest.main()
