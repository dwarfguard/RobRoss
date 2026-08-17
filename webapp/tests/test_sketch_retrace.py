import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import context  # noqa: F401

import app as webapp
import generate_output_gallery as gallery
import route_adapters


class TestSketchRetraceAdapter(unittest.TestCase):
    def setUp(self):
        output_parent = Path(context.REPO_ROOT) / "output"
        self.temporary = tempfile.TemporaryDirectory(
            prefix="webapp_retrace_test_", dir=output_parent
        )
        self.output_dir = Path(self.temporary.name)
        self.source_name = "upload_10_sketch"
        self.source_dir = self.output_dir / self.source_name
        self.source_dir.mkdir()
        self.photo = self.source_dir / "upload_portrait.png"
        self.semantic = self.source_dir / "sketch_outline_intermediate.png"
        self.photo.write_bytes(b"photo bytes")
        self.semantic.write_bytes(b"saved qwen bytes")
        relative_photo = self.photo.relative_to(context.REPO_ROOT)
        self.config = {
            "profile_name": self.source_name,
            "style": "sketch",
            "source_image": {"path": str(relative_photo)},
            "semantic_preprocessing": {"enabled": True, "provider": "qwen"},
            "outline_extraction": {
                "line_art_threshold": 203,
                "min_path_length_mm": 1.2,
            },
            "detail_extraction": {"max_paths": 123},
            "path_generation": {
                "smoothing_radius_mm": 0.4,
                "continuous_paths": True,
            },
            "output": {
                "directory": str(self.source_dir.relative_to(context.REPO_ROOT)),
                "semantic_outline_image_file": self.semantic.name,
                "painting_paths_file": "paths.json",
                "preview_svg_file": "preview.svg",
            },
        }
        (self.source_dir / "_config.json").write_text(
            json.dumps(self.config), encoding="utf-8"
        )
        (self.source_dir / "paths.json").write_text(
            json.dumps({
                "style": "sketch",
                "debug": {"semantic_preprocessing_status": "used"},
            }),
            encoding="utf-8",
        )

    def tearDown(self):
        self.temporary.cleanup()

    def test_retrace_creates_distinct_run_copies_bytes_and_uses_saved_mode(self):
        with mock.patch.object(route_adapters, "OUTPUT_DIR", self.output_dir), mock.patch.object(
            route_adapters.time, "time", return_value=222
        ), mock.patch.object(route_adapters.subprocess, "run") as process:
            process.return_value.returncode = 0
            new_name = route_adapters.retrace_sketch_run(
                self.source_name,
                line_art_threshold=211,
                outline_min_path_length_mm=0.8,
                smoothing_radius_mm=0.6,
                continuous_paths=False,
                detail_max_paths=77,
            )

        self.assertEqual(new_name, "retrace_222_sketch")
        self.assertTrue(self.source_dir.is_dir())
        new_dir = self.output_dir / new_name
        self.assertEqual((new_dir / "source_photo.png").read_bytes(), b"photo bytes")
        self.assertEqual((new_dir / "saved_semantic.png").read_bytes(), b"saved qwen bytes")
        new_config = json.loads((new_dir / "_config.json").read_text(encoding="utf-8"))
        self.assertEqual(new_config["outline_extraction"]["line_art_threshold"], 211)
        self.assertEqual(new_config["outline_extraction"]["min_path_length_mm"], 0.8)
        self.assertEqual(new_config["path_generation"]["smoothing_radius_mm"], 0.6)
        self.assertFalse(new_config["path_generation"]["continuous_paths"])
        self.assertEqual(new_config["detail_extraction"]["max_paths"], 77)
        self.assertEqual(
            new_config["retrace_provenance"]["source_run"], self.source_name
        )
        command = process.call_args.args[0]
        self.assertIn("--saved-semantic-image", command)
        self.assertNotEqual(new_dir, self.source_dir)

    def test_retrace_rejects_invalid_non_sketch_and_missing_inputs(self):
        cases = (
            ("invalid/run", "Invalid run name"),
            ("missing", "No such run"),
        )
        for run_name, message in cases:
            with self.subTest(run_name=run_name), mock.patch.object(
                route_adapters, "OUTPUT_DIR", self.output_dir
            ), self.assertRaisesRegex(ValueError, message):
                route_adapters.retrace_sketch_run(
                    run_name,
                    line_art_threshold=210,
                    outline_min_path_length_mm=1.0,
                    smoothing_radius_mm=0.5,
                    continuous_paths=True,
                    detail_max_paths=400,
                )

        for mutation, message in (
            (lambda config: config.update(style="mondrian"), "Only sketch"),
            (lambda config: config["source_image"].update(path="outside.png"), "inside"),
            (
                lambda config: config["output"].update(
                    semantic_outline_image_file="missing.png"
                ),
                "missing",
            ),
        ):
            with self.subTest(message=message):
                changed = json.loads(json.dumps(self.config))
                mutation(changed)
                (self.source_dir / "_config.json").write_text(
                    json.dumps(changed), encoding="utf-8"
                )
                with mock.patch.object(route_adapters, "OUTPUT_DIR", self.output_dir), self.assertRaisesRegex(
                    ValueError, message
                ):
                    route_adapters.retrace_sketch_run(
                        self.source_name,
                        line_art_threshold=210,
                        outline_min_path_length_mm=1.0,
                        smoothing_radius_mm=0.5,
                        continuous_paths=True,
                        detail_max_paths=400,
                    )

    def test_subprocess_failure_removes_only_new_run(self):
        with mock.patch.object(route_adapters, "OUTPUT_DIR", self.output_dir), mock.patch.object(
            route_adapters.time, "time", return_value=333
        ), mock.patch.object(route_adapters.subprocess, "run") as process:
            process.return_value.returncode = 9
            process.return_value.stderr = "safe failure"
            with self.assertRaises(route_adapters.RouteRunError):
                route_adapters.retrace_sketch_run(
                    self.source_name,
                    line_art_threshold=210,
                    outline_min_path_length_mm=1.0,
                    smoothing_radius_mm=0.5,
                    continuous_paths=True,
                    detail_max_paths=400,
                )
        self.assertTrue(self.source_dir.is_dir())
        self.assertFalse((self.output_dir / "retrace_333_sketch").exists())

    def test_local_fallback_run_is_not_eligible_for_qwen_retrace(self):
        (self.source_dir / "paths.json").write_text(
            json.dumps({
                "style": "sketch",
                "debug": {"semantic_preprocessing_status": "local_fallback"},
            }),
            encoding="utf-8",
        )
        with mock.patch.object(
            route_adapters, "OUTPUT_DIR", self.output_dir
        ), self.assertRaisesRegex(ValueError, "requires a successfully generated"):
            route_adapters.retrace_sketch_run(
                self.source_name,
                line_art_threshold=210,
                outline_min_path_length_mm=1.0,
                smoothing_radius_mm=0.5,
                continuous_paths=True,
                detail_max_paths=400,
            )


class TestSketchRetraceWeb(unittest.TestCase):
    def test_gallery_form_prefills_saved_settings_for_eligible_run(self):
        output_parent = Path(context.REPO_ROOT) / "output"
        with tempfile.TemporaryDirectory(
            prefix="gallery_retrace_test_", dir=output_parent
        ) as temporary:
            run_dir = Path(temporary) / "run"
            run_dir.mkdir()
            photo = run_dir / "photo.png"
            semantic = run_dir / "semantic.png"
            photo.write_bytes(b"photo")
            semantic.write_bytes(b"semantic")
            config = {
                "style": "sketch",
                "source_image": {"path": str(photo.relative_to(context.REPO_ROOT))},
                "outline_extraction": {
                    "line_art_threshold": 199,
                    "min_path_length_mm": 1.4,
                },
                "detail_extraction": {"max_paths": 88},
                "path_generation": {
                    "smoothing_radius_mm": 0.7,
                    "continuous_paths": True,
                },
                "output": {"semantic_outline_image_file": semantic.name},
            }
            (run_dir / "_config.json").write_text(json.dumps(config), encoding="utf-8")
            run = {
                "name": "run",
                "run_dir": run_dir,
                "config": config,
                "files": {"paths_json": "paths.json"},
                "painting_paths": {
                    "style": "sketch",
                    "canvas": {},
                    "validation": {"passed": True},
                    "debug": {"semantic_preprocessing_status": "used"},
                },
                "created_ts": 1,
            }
            with mock.patch.object(gallery, "OUTPUT_DIR", Path(temporary)):
                markup = gallery.render_card(run, "/output/", show_delete=True)
        self.assertIn("Retrace saved Qwen image", markup)
        self.assertIn('value="199"', markup)
        self.assertIn('value="1.4"', markup)
        self.assertIn('value="0.7"', markup)
        self.assertIn('value="88"', markup)
        self.assertIn('name="continuous_paths" checked', markup)

    def test_retrace_endpoint_validates_and_passes_all_settings(self):
        client = webapp.app.test_client()
        with mock.patch.object(
            route_adapters, "retrace_sketch_run", return_value="retrace_1_sketch"
        ) as retrace, mock.patch.object(gallery, "main"):
            response = client.post(
                "/output/upload_1_sketch/retrace",
                data={
                    "line_art_threshold": "205",
                    "semantic_outline_floor_mm": "0.9",
                    "smoothing_radius_mm": "0.4",
                    "continuous_paths": "on",
                    "maximum_detail_paths": "66",
                },
            )
        self.assertEqual(response.status_code, 303)
        retrace.assert_called_once_with(
            "upload_1_sketch",
            line_art_threshold=205,
            outline_min_path_length_mm=0.9,
            smoothing_radius_mm=0.4,
            continuous_paths=True,
            detail_max_paths=66,
        )

    def test_retrace_endpoint_rejects_invalid_settings(self):
        client = webapp.app.test_client()
        valid = {
            "line_art_threshold": "210",
            "semantic_outline_floor_mm": "1.0",
            "smoothing_radius_mm": "0.5",
            "maximum_detail_paths": "400",
        }
        for field, invalid in (
            ("line_art_threshold", "256"),
            ("semantic_outline_floor_mm", "-1"),
            ("smoothing_radius_mm", "2.1"),
            ("maximum_detail_paths", "2.5"),
        ):
            data = dict(valid)
            data[field] = invalid
            with self.subTest(field=field), mock.patch.object(
                route_adapters, "retrace_sketch_run"
            ) as retrace:
                response = client.post("/output/run/retrace", data=data)
                self.assertEqual(response.status_code, 400)
                retrace.assert_not_called()


if __name__ == "__main__":
    unittest.main()
