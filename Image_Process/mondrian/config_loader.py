"""Shared config loading + validation for the Mondrian pipeline.

Both mondrian_generator.py and generate_painting_paths.py load their
settings (canvas size, artwork rules, path/tool settings, output file
names) from a single JSON config file (see configs/*.json) instead of
hardcoding them. This module is the one place that knows what a valid
config looks like, so both scripts agree on the same rules.
"""

import json
from pathlib import Path

REQUIRED_SECTIONS = ["canvas", "artwork", "path_generation", "output"]
REQUIRED_OUTPUT_FILES = [
    "painting_plan_file",
    "painting_paths_file",
    "preview_svg_file",
    "path_preview_svg_file",
]


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

    # margin_mm is optional (defaults to 0): how far the entire artwork,
    # border included, stays inside the physical canvas/paper edge.
    margin = canvas.get("margin_mm", 0.0)
    if not _is_number(margin) or margin < 0:
        errors.append(f"canvas.margin_mm must be a number >= 0, got {margin!r}.")
    elif _is_number(width) and _is_number(height) and width > 0 and height > 0:
        if 2 * margin >= min(width, height):
            errors.append(
                f"canvas.margin_mm ({margin!r}) is too large: twice the margin "
                f"must be smaller than the canvas' short side ({min(width, height)!r} mm)."
            )

    artwork = config.get("artwork", {})
    # min_split_depth is optional (defaults to 1): depths below it always
    # subdivide, so a shallow random stop can't produce an empty artwork.
    min_split_depth = artwork.get("min_split_depth", 1)
    if not isinstance(min_split_depth, int) or isinstance(min_split_depth, bool) or min_split_depth < 0:
        errors.append(f"artwork.min_split_depth must be an integer >= 0, got {min_split_depth!r}.")

    path_generation = config.get("path_generation", {})
    tool_width = path_generation.get("tool_width_mm")
    if not _is_number(tool_width) or tool_width <= 0:
        errors.append(f"path_generation.tool_width_mm must be > 0, got {tool_width!r}.")

    edge_inset = path_generation.get("edge_inset_mm")
    if not _is_number(edge_inset) or edge_inset < 0:
        errors.append(f"path_generation.edge_inset_mm must be >= 0, got {edge_inset!r}.")

    overlap_ratio = path_generation.get("stroke_overlap_ratio")
    if not _is_number(overlap_ratio) or not (0 <= overlap_ratio < 1):
        errors.append(
            f"path_generation.stroke_overlap_ratio must satisfy 0 <= ratio < 1, got {overlap_ratio!r}."
        )

    output = config.get("output", {})
    for field in REQUIRED_OUTPUT_FILES:
        if not output.get(field):
            errors.append(f"output.{field} is missing or empty.")

    return errors


def load_config(path) -> dict:
    """Load and validate a pipeline config JSON file.

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
