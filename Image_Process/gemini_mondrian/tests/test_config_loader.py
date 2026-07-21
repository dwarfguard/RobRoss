import unittest

from context import CONFIGS_DIR

from config_loader import ConfigError, load_config, validate_config


def make_valid_config() -> dict:
    """A minimal config that passes validation, for mutation in tests."""
    return {
        "profile_name": "gemini_mondrian_test",
        "project": "R.O.B Ross",
        "style": "gemini_mondrian",
        "source_image": {
            "path": "Image_Process/image_to_mondrian/assets/sample.jpg",
        },
        "gemini": {
            "model": "gemini-2.5-flash-image",
            "prompt": "Redraw this photo in the style of a Piet Mondrian painting.",
        },
        "downstream_template_config": str(CONFIGS_DIR / "image_to_mondrian_demo_a4.json"),
        "output": {
            "directory": "output/gemini_mondrian_test",
            "gemini_output_image_file": "gemini_styled.png",
        },
    }


class TestValidateConfig(unittest.TestCase):
    def test_valid_config_has_no_errors(self):
        self.assertEqual(validate_config(make_valid_config()), [])

    def test_repo_config_is_valid(self):
        config = load_config(CONFIGS_DIR / "gemini_mondrian_demo_a4.json")
        self.assertEqual(validate_config(config), [])

    def test_missing_section_is_reported(self):
        config = make_valid_config()
        del config["gemini"]
        errors = validate_config(config)
        self.assertTrue(any("gemini" in error for error in errors))

    def test_empty_source_image_path_is_an_error(self):
        config = make_valid_config()
        config["source_image"]["path"] = ""
        errors = validate_config(config)
        self.assertTrue(any("source_image.path" in error for error in errors))

    def test_missing_gemini_model_is_an_error(self):
        config = make_valid_config()
        del config["gemini"]["model"]
        errors = validate_config(config)
        self.assertTrue(any("gemini.model" in error for error in errors))

    def test_missing_gemini_prompt_is_an_error(self):
        config = make_valid_config()
        del config["gemini"]["prompt"]
        errors = validate_config(config)
        self.assertTrue(any("gemini.prompt" in error for error in errors))

    def test_missing_downstream_template_config_is_an_error(self):
        config = make_valid_config()
        del config["downstream_template_config"]
        errors = validate_config(config)
        self.assertTrue(any("downstream_template_config" in error for error in errors))

    def test_nonexistent_downstream_template_config_is_an_error(self):
        config = make_valid_config()
        config["downstream_template_config"] = "configs/does_not_exist.json"
        errors = validate_config(config)
        self.assertTrue(any("downstream_template_config" in error for error in errors))

    def test_missing_output_directory_is_an_error(self):
        config = make_valid_config()
        del config["output"]["directory"]
        errors = validate_config(config)
        self.assertTrue(any("output.directory" in error for error in errors))

    def test_missing_gemini_output_image_file_is_an_error(self):
        config = make_valid_config()
        del config["output"]["gemini_output_image_file"]
        errors = validate_config(config)
        self.assertTrue(any("output.gemini_output_image_file" in error for error in errors))


class TestLoadConfig(unittest.TestCase):
    def test_missing_file_raises_config_error(self):
        with self.assertRaises(ConfigError):
            load_config(CONFIGS_DIR / "does_not_exist.json")


if __name__ == "__main__":
    unittest.main()
