import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import context

import app as webapp
import route_adapters


class _Upload:
    filename = "drawing.png"

    def save(self, path):
        Path(path).write_bytes(b"image")


class TestPaperSizeSelection(unittest.TestCase):
    def test_form_exposes_4k_landscape_for_supported_routes(self):
        markup = webapp.render_upload_form()

        self.assertIn("Paper size", markup)
        self.assertIn('value="4k_landscape"', markup)
        self.assertIn("4K landscape (525 x 375 mm)", markup)
        self.assertIn(
            "routeSelect.value === 'sketch' || routeSelect.value === 'line_art'",
            markup,
        )

    def test_process_passes_selected_paper_size(self):
        client = webapp.app.test_client()
        with mock.patch.object(
            route_adapters, "run_route", return_value="run"
        ) as run_route:
            response = client.post(
                "/process",
                data={
                    "route": "line_art",
                    "paper_size": "4k_landscape",
                    "image": (io.BytesIO(b"image"), "drawing.png"),
                },
                content_type="multipart/form-data",
            )

        self.assertEqual(response.status_code, 303)
        self.assertEqual(run_route.call_args.kwargs["paper_size"], "4k_landscape")

    def test_4k_template_is_recorded_for_sketch_and_line_art(self):
        output_parent = Path(context.REPO_ROOT) / "output"
        for route_key in ("sketch", "line_art"):
            with self.subTest(route=route_key), tempfile.TemporaryDirectory(
                prefix="webapp_paper_test_", dir=output_parent
            ) as temporary_output:
                temporary_output = Path(temporary_output)
                with mock.patch.object(
                    route_adapters, "OUTPUT_DIR", temporary_output
                ), mock.patch.object(
                    route_adapters.time, "time", return_value=67890
                ), mock.patch.object(
                    route_adapters.subprocess, "run"
                ) as run_process:
                    run_process.return_value.returncode = 0
                    route_adapters.run_route(
                        route_key, _Upload(), paper_size="4k_landscape"
                    )

                config_path = (
                    temporary_output
                    / f"upload_67890_{route_key}"
                    / "_config.json"
                )
                config = json.loads(config_path.read_text(encoding="utf-8"))
                self.assertEqual(config["canvas"]["name"], "4K landscape")
                self.assertEqual(config["canvas"]["width_mm"], 525.0)
                self.assertEqual(config["canvas"]["height_mm"], 375.0)

    def test_unknown_paper_size_is_rejected_before_creating_a_run(self):
        output_parent = Path(context.REPO_ROOT) / "output"
        with tempfile.TemporaryDirectory(
            prefix="webapp_paper_test_", dir=output_parent
        ) as temporary_output, mock.patch.object(
            route_adapters, "OUTPUT_DIR", Path(temporary_output)
        ):
            with self.assertRaisesRegex(ValueError, "Unsupported paper size"):
                route_adapters.run_route(
                    "sketch", _Upload(), paper_size="custom"
                )
            self.assertEqual(list(Path(temporary_output).iterdir()), [])


if __name__ == "__main__":
    unittest.main()
