"""Shared config loading + validation for the line_art (clean line-art
tracing) pipeline.

Mirrors Image_Process/sketch/config_loader.py's shape (load_config /
validate_config / ConfigError) and its minimal 4-section schema (canvas +
source_image + path_generation + output, no palette/segmentation/border
sections - line_art traces single strokes, it doesn't fill regions), but
validates this pipeline's own source_image fields: a fixed binarization
threshold instead of Canny thresholds, plus a skeleton spur-pruning length
and a minimum physical stroke length that sketch/ doesn't need.
"""

import json
from pathlib import Path

REQUIRED_SECTIONS = ["canvas", "source_image", "path_generation", "output"]

# Defaults for optional fields, applied by load_config() rather than
# required in every config file.
DEFAULT_BINARY_THRESHOLD = 128
DEFAULT_MIN_SPUR_LENGTH_PX = 5.0
DEFAULT_MIN_STROKE_LENGTH_MM = 1.0
DEFAULT_SIMPLIFY_EPSILON_RATIO = 0.002


class ConfigError(ValueError):
    """Raised when a config file is missing or fails validation."""


def _is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def validate_config(config: dict) -> list:
    """Return a list of human-readable error strings (empty if config is valid)."""
    errors = []

    for section in REQUIRED_SECTIONS:
        if section not in config:
            errors.append(f"Missing required top-level section: '{section}'.")

    canvas = config.get("canvas", {})
    width = canvas.get("width_mm")
    if not _is_number(width) or width <= 0:
        errors.append(f"canvas.width_mm must be a positive number, got {width!r}.")

    height = canvas.get("height_mm")
    if not _is_number(height) or height <= 0:
        errors.append(f"canvas.height_mm must be a positive number, got {height!r}.")

    origin = canvas.get("origin")
    if origin != "top-left":
        errors.append(f"canvas.origin must be 'top-left', got {origin!r}.")

    # margin_mm is optional (defaults to 0): how far the traced drawing
    # stays inside the physical canvas/paper edge.
    margin = canvas.get("margin_mm", 0.0)
    if not _is_number(margin) or margin < 0:
        errors.append(f"canvas.margin_mm must be a number >= 0, got {margin!r}.")
    elif _is_number(width) and _is_number(height) and width > 0 and height > 0:
        if 2 * margin >= min(width, height):
            errors.append(
                f"canvas.margin_mm ({margin!r}) is too large: twice the margin "
                f"must be smaller than the canvas' short side ({min(width, height)!r} mm)."
            )

    source_image = config.get("source_image", {})
    if not source_image.get("path"):
        errors.append("source_image.path is missing or empty.")

    binary_threshold = source_image.get("binary_threshold", DEFAULT_BINARY_THRESHOLD)
    if not _is_number(binary_threshold) or not (0 <= binary_threshold <= 255):
        errors.append(
            f"source_image.binary_threshold must be a number in [0, 255], got {binary_threshold!r}."
        )

    min_spur_length_px = source_image.get("min_spur_length_px", DEFAULT_MIN_SPUR_LENGTH_PX)
    if not _is_number(min_spur_length_px) or min_spur_length_px < 0:
        errors.append(
            f"source_image.min_spur_length_px must be a number >= 0, got {min_spur_length_px!r}."
        )

    min_stroke_length_mm = source_image.get("min_stroke_length_mm", DEFAULT_MIN_STROKE_LENGTH_MM)
    if not _is_number(min_stroke_length_mm) or min_stroke_length_mm < 0:
        errors.append(
            f"source_image.min_stroke_length_mm must be a number >= 0, got {min_stroke_length_mm!r}."
        )

    epsilon_ratio = source_image.get("simplify_epsilon_ratio", DEFAULT_SIMPLIFY_EPSILON_RATIO)
    if not _is_number(epsilon_ratio) or epsilon_ratio <= 0:
        errors.append(f"source_image.simplify_epsilon_ratio must be > 0, got {epsilon_ratio!r}.")

    path_generation = config.get("path_generation", {})
    tool_width = path_generation.get("tool_width_mm")
    if not _is_number(tool_width) or tool_width <= 0:
        errors.append(f"path_generation.tool_width_mm must be > 0, got {tool_width!r}.")

    home_position = path_generation.get("home_position_mm")
    if home_position is not None:
        if (
            not isinstance(home_position, (list, tuple))
            or len(home_position) != 2
            or not all(_is_number(v) for v in home_position)
        ):
            errors.append(f"path_generation.home_position_mm must be a two-number list, got {home_position!r}.")

    output = config.get("output", {})
    for field in ("painting_paths_file", "preview_svg_file"):
        if not output.get(field):
            errors.append(f"output.{field} is missing or empty.")

    return errors


def load_config(path) -> dict:
    """Load and validate a line_art pipeline config JSON file.

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
