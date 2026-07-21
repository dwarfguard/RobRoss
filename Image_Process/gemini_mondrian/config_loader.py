"""Shared config loading + validation for the gemini_mondrian pipeline.

This route's own config is deliberately thin - it only describes the Gemini
preprocessing step (source photo, model, prompt) and which existing
image_to_mondrian config to clone as the downstream vectorization template.
Canvas size, palette, segmentation, path-generation, and border-generation
settings all come from that downstream template config instead of being
duplicated into a second schema here.

Mirrors Image_Process/sketch/config_loader.py's shape (load_config /
validate_config / ConfigError), not image_to_mondrian's - the schemas are
different enough that copying image_to_mondrian's validator wouldn't fit.
"""

import json
from pathlib import Path

REQUIRED_SECTIONS = ["source_image", "gemini", "output"]


class ConfigError(ValueError):
    """Raised when a config file is missing or fails validation."""


def validate_config(config: dict) -> list:
    """Return a list of human-readable error strings (empty if config is valid)."""
    errors = []

    for section in REQUIRED_SECTIONS:
        if section not in config:
            errors.append(f"Missing required top-level section: '{section}'.")

    source_image = config.get("source_image", {})
    if not source_image.get("path"):
        errors.append("source_image.path is missing or empty.")

    gemini = config.get("gemini", {})
    if not gemini.get("model"):
        errors.append("gemini.model is missing or empty.")
    if not gemini.get("prompt"):
        errors.append("gemini.prompt is missing or empty.")

    downstream_template_config = config.get("downstream_template_config")
    if not downstream_template_config:
        errors.append("downstream_template_config is missing or empty.")
    elif not Path(downstream_template_config).is_file():
        errors.append(
            f"downstream_template_config points to a file that doesn't exist: "
            f"{downstream_template_config!r}."
        )

    output = config.get("output", {})
    for field in ("directory", "gemini_output_image_file"):
        if not output.get(field):
            errors.append(f"output.{field} is missing or empty.")

    return errors


def load_config(path) -> dict:
    """Load and validate a gemini_mondrian pipeline config JSON file.

    Raises ConfigError with all problems listed if the file is missing,
    isn't valid JSON, or fails validate_config().
    """
    config_path = Path(path)

    if not config_path.exists():
        raise ConfigError(f"Config file not found: {config_path}")

    with open(config_path, "r", encoding="utf-8") as file:
        try:
            config = json.load(file)
        except json.JSONDecodeError as exc:
            raise ConfigError(f"Config file {config_path} is not valid JSON: {exc}") from exc

    errors = validate_config(config)
    if errors:
        details = "\n".join(f"  - {error}" for error in errors)
        raise ConfigError(f"Invalid config {config_path}:\n{details}")

    return config
