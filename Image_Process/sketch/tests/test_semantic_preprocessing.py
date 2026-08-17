import builtins
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
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

        def import_without_requests(name, *args, **kwargs):
            if name == "requests":
                raise ModuleNotFoundError(name)
            return real_import(name, *args, **kwargs)

        with tempfile.TemporaryDirectory() as temp_dir:
            image_path = Path(temp_dir) / "source.png"
            image_path.write_bytes(b"source")
            with mock.patch.dict(os.environ, {"DASHSCOPE_API_KEY": "test"}), mock.patch(
                "builtins.__import__", side_effect=import_without_requests
            ), self.assertRaises(SemanticPreprocessingError) as caught:
                generate_outline_image(image_path, "prompt", "model", 60)

        self.assertEqual(caught.exception.reason, "missing_dependency")

    def test_sdk_errors_are_reduced_to_safe_reason_codes(self):
        cases = (
            (TimeoutError("request details"), "timeout"),
            (RuntimeError("resource exhausted: quota"), "quota"),
            (ConnectionError("private endpoint"), "network"),
            (RuntimeError("proxy connection failed"), "network"),
            (RuntimeError("blocked by safety policy"), "safety_rejection"),
            (RuntimeError("internal request details"), "api_failure"),
        )
        for error, expected in cases:
            with self.subTest(expected=expected):
                self.assertEqual(_classify_exception(error), expected)

    def test_qwen_request_uses_configured_model_and_downloads_first_image(self):
        response = {
            "status_code": 200,
            "output": {
                "choices": [
                    {"message": {"content": [{"image": "https://example.test/out.png"}]}}
                ]
            },
        }
        post = mock.Mock(
            return_value=SimpleNamespace(status_code=200, json=lambda: response)
        )
        get = mock.Mock(
            return_value=SimpleNamespace(
                content=b"generated png", raise_for_status=mock.Mock()
            )
        )
        fake_requests = SimpleNamespace(post=post, get=get)

        with tempfile.TemporaryDirectory() as temp_dir:
            image_path = Path(temp_dir) / "source.png"
            image_path.write_bytes(b"source image")
            with mock.patch.dict(
                os.environ, {"DASHSCOPE_API_KEY": "test"}, clear=True
            ), mock.patch.dict(
                sys.modules,
                {"requests": fake_requests},
            ):
                result = generate_outline_image(
                    image_path, "line art prompt", "qwen-image-edit-plus", 12
                )

        self.assertEqual(result, b"generated png")
        args, kwargs = post.call_args
        self.assertEqual(
            args[0],
            "https://ws-2xu46zkb1q2h7j9k.cn-beijing.maas.aliyuncs.com/api/v1/services/"
            "aigc/multimodal-generation/generation",
        )
        self.assertEqual(kwargs["json"]["model"], "qwen-image-edit-plus")
        self.assertFalse(kwargs["json"]["parameters"]["prompt_extend"])
        self.assertFalse(kwargs["json"]["parameters"]["watermark"])
        self.assertTrue(
            kwargs["json"]["input"]["messages"][0]["content"][0]["image"].startswith(
                "data:image/png;base64,"
            )
        )
        self.assertEqual(
            kwargs["json"]["input"]["messages"][0]["content"][1],
            {"text": "line art prompt"},
        )
        get.assert_called_once_with("https://example.test/out.png", timeout=12.0)


if __name__ == "__main__":
    unittest.main()
