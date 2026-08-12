import builtins
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import context  # noqa: F401

from semantic_preprocessing import (
    SemanticPreprocessingError,
    _classify_exception,
    generate_outline_image,
)


class TestSemanticPreprocessingClient(unittest.TestCase):
    def test_missing_key_is_categorized_before_import_or_file_access(self):
        with mock.patch.dict(os.environ, {}, clear=True), self.assertRaises(
            SemanticPreprocessingError
        ) as caught:
            generate_outline_image(Path("missing.png"), "prompt", "model", 60)

        self.assertEqual(caught.exception.reason, "missing_api_key")

    def test_missing_dependency_is_categorized(self):
        real_import = builtins.__import__

        def import_without_google(name, *args, **kwargs):
            if name == "google" or name.startswith("google."):
                raise ModuleNotFoundError(name)
            return real_import(name, *args, **kwargs)

        with tempfile.TemporaryDirectory() as temp_dir:
            image_path = Path(temp_dir) / "source.png"
            image_path.write_bytes(b"source")
            with mock.patch.dict(os.environ, {"GEMINI_API_KEY": "test"}), mock.patch(
                "builtins.__import__", side_effect=import_without_google
            ), self.assertRaises(SemanticPreprocessingError) as caught:
                generate_outline_image(image_path, "prompt", "model", 60)

        self.assertEqual(caught.exception.reason, "missing_dependency")

    def test_sdk_errors_are_reduced_to_safe_reason_codes(self):
        cases = (
            (TimeoutError("request details"), "timeout"),
            (RuntimeError("resource exhausted: quota"), "quota"),
            (ConnectionError("private endpoint"), "network"),
            (RuntimeError("blocked by safety policy"), "safety_rejection"),
            (RuntimeError("internal request details"), "api_failure"),
        )
        for error, expected in cases:
            with self.subTest(expected=expected):
                self.assertEqual(_classify_exception(error), expected)


if __name__ == "__main__":
    unittest.main()
