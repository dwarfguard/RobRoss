import os
import unittest
from unittest import mock

import context  # noqa: F401

from gemini_client import GeminiConfigError, generate_styled_image


class TestGenerateStyledImage(unittest.TestCase):
    def test_missing_api_key_raises_clear_error_not_a_network_call(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            os.environ.pop("GEMINI_API_KEY", None)
            with self.assertRaises(GeminiConfigError) as cm:
                generate_styled_image(
                    context.REPO_ROOT / "Image_Process/assets/sample.jpg",
                    "prompt",
                    "gemini-2.5-flash-image",
                )
            self.assertIn("GEMINI_API_KEY", str(cm.exception))

    def test_missing_source_image_raises_clear_error(self):
        with mock.patch.dict(os.environ, {"GEMINI_API_KEY": "test-key"}):
            with self.assertRaises(GeminiConfigError) as cm:
                generate_styled_image(
                    context.REPO_ROOT / "Image_Process/gemini_mondrian/assets/does_not_exist.jpg",
                    "prompt",
                    "gemini-2.5-flash-image",
                )
            self.assertIn("source image not found", str(cm.exception))


if __name__ == "__main__":
    unittest.main()
