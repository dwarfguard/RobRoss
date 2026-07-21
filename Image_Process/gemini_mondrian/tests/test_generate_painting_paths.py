import json
import tempfile
import unittest
from pathlib import Path

import context  # noqa: F401

from generate_painting_paths import build_downstream_config


class TestBuildDownstreamConfig(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmpdir.cleanup)
        self.repo_root_relative_template = context.REPO_ROOT / "configs" / "image_to_mondrian_demo_a4.json"

    def _config(self):
        return {
            "downstream_template_config": "configs/image_to_mondrian_demo_a4.json",
        }

    def test_clones_template_and_overrides_source_image_and_output_dir(self):
        output_dir = context.REPO_ROOT / "output" / "gemini_mondrian_unittest"
        gemini_image_path = output_dir / "gemini_styled.png"

        downstream_config = build_downstream_config(self._config(), gemini_image_path, output_dir)

        self.assertEqual(
            downstream_config["source_image"]["path"],
            "output/gemini_mondrian_unittest/gemini_styled.png",
        )
        self.assertEqual(
            downstream_config["output"]["directory"],
            "output/gemini_mondrian_unittest",
        )

    def test_leaves_other_template_fields_untouched(self):
        output_dir = context.REPO_ROOT / "output" / "gemini_mondrian_unittest"
        gemini_image_path = output_dir / "gemini_styled.png"
        template = json.loads(self.repo_root_relative_template.read_text(encoding="utf-8"))

        downstream_config = build_downstream_config(self._config(), gemini_image_path, output_dir)

        self.assertEqual(downstream_config["palette"], template["palette"])
        self.assertEqual(downstream_config["segmentation"], template["segmentation"])
        self.assertEqual(downstream_config["canvas"], template["canvas"])

    def test_marks_profile_name_as_generated_via_gemini(self):
        output_dir = context.REPO_ROOT / "output" / "gemini_mondrian_unittest"
        gemini_image_path = output_dir / "gemini_styled.png"
        template = json.loads(self.repo_root_relative_template.read_text(encoding="utf-8"))

        downstream_config = build_downstream_config(self._config(), gemini_image_path, output_dir)

        self.assertEqual(
            downstream_config["profile_name"],
            f"{template['profile_name']}_via_gemini",
        )

    def test_does_not_mutate_the_template_file_on_disk(self):
        output_dir = context.REPO_ROOT / "output" / "gemini_mondrian_unittest"
        gemini_image_path = output_dir / "gemini_styled.png"
        before = self.repo_root_relative_template.read_text(encoding="utf-8")

        build_downstream_config(self._config(), gemini_image_path, output_dir)

        after = self.repo_root_relative_template.read_text(encoding="utf-8")
        self.assertEqual(before, after)


if __name__ == "__main__":
    unittest.main()
