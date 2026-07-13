"""Shared config loading + validation for the image_to_mondrian pipeline.

Same REQUIRED_SECTIONS/ConfigError/_is_number/validate_config/load_config
pattern as Image_Process/mondrian/config_loader.py and
Image_Process/sketch/config_loader.py, adapted to this module's own schema.
Not imported cross-folder, per repo convention (each Image_Process/<algo>/
folder is self-contained).
"""

import json
import re
from pathlib import Path

REQUIRED_SECTIONS = [
    "canvas",
    "source_image",
    "palette",
    "segmentation",
    "path_generation",
    "border_generation",
    "output",
]
REQUIRED_OUTPUT_FILES = ["painting_paths_file", "preview_svg_file"]

_HEX_COLOR_RE = re.compile(r"^#[0-9a-fA-F]{6}$")


class ConfigError(ValueError):
    """Raised when a config file is missing or fails validation."""


def _is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _validate_canvas(canvas: dict) -> list:
    errors = []

    width = canvas.get("width_mm")
    if not _is_number(width) or width <= 0:
        errors.append(f"canvas.width_mm must be a positive number, got {width!r}.")

    height = canvas.get("height_mm")
    if not _is_number(height) or height <= 0:
        errors.append(f"canvas.height_mm must be a positive number, got {height!r}.")

    origin = canvas.get("origin")
    if origin != "top-left":
        errors.append(f"canvas.origin must be 'top-left', got {origin!r}.")

    margin = canvas.get("margin_mm", 0.0)
    if not _is_number(margin) or margin < 0:
        errors.append(f"canvas.margin_mm must be a number >= 0, got {margin!r}.")
    elif _is_number(width) and _is_number(height) and width > 0 and height > 0:
        if 2 * margin >= min(width, height):
            errors.append(
                f"canvas.margin_mm ({margin!r}) is too large: twice the margin "
                f"must be smaller than the canvas' short side ({min(width, height)!r} mm)."
            )

    return errors


def _validate_source_image(source_image: dict) -> list:
    errors = []

    if not source_image.get("path"):
        errors.append("source_image.path is missing or empty.")

    blur_kernel_size = source_image.get("blur_kernel_size")
    if blur_kernel_size is not None:
        if (
            not isinstance(blur_kernel_size, int)
            or isinstance(blur_kernel_size, bool)
            or blur_kernel_size <= 0
            or blur_kernel_size % 2 == 0
        ):
            errors.append(
                f"source_image.blur_kernel_size must be a positive odd integer, got {blur_kernel_size!r}."
            )

    blur_sigma = source_image.get("blur_sigma")
    if blur_sigma is not None and (not _is_number(blur_sigma) or blur_sigma < 0):
        errors.append(f"source_image.blur_sigma must be a number >= 0, got {blur_sigma!r}.")

    downscale = source_image.get("downscale_max_dimension_px")
    if downscale is not None:
        if not isinstance(downscale, int) or isinstance(downscale, bool) or downscale <= 0:
            errors.append(
                f"source_image.downscale_max_dimension_px must be a positive integer, got {downscale!r}."
            )

    return errors


def _validate_palette(palette: dict) -> list:
    errors = []

    colors = palette.get("colors")
    if not isinstance(colors, list) or len(colors) == 0:
        errors.append("palette.colors must be a non-empty list.")
        colors = []

    seen_names = set()
    for index, entry in enumerate(colors):
        if not isinstance(entry, dict):
            errors.append(f"palette.colors[{index}] must be an object with 'name' and 'hex'.")
            continue

        name = entry.get("name")
        if not name or not isinstance(name, str):
            errors.append(f"palette.colors[{index}].name must be a non-empty string.")
        elif name in seen_names:
            errors.append(f"palette.colors[{index}].name '{name}' is duplicated.")
        else:
            seen_names.add(name)

        hex_value = entry.get("hex")
        if not isinstance(hex_value, str) or not _HEX_COLOR_RE.match(hex_value):
            errors.append(
                f"palette.colors[{index}].hex must look like '#RRGGBB', got {hex_value!r}."
            )

    color_space = palette.get("color_space", "lab")
    if color_space not in ("rgb", "lab"):
        errors.append(f"palette.color_space must be 'rgb' or 'lab', got {color_space!r}.")

    return errors


def _validate_segmentation(segmentation: dict) -> list:
    errors = []

    min_area = segmentation.get("min_region_area_mm2")
    if not _is_number(min_area) or min_area <= 0:
        errors.append(
            f"segmentation.min_region_area_mm2 must be > 0, got {min_area!r}."
        )

    morph_kernel = segmentation.get("morph_open_kernel_px", 0)
    if not isinstance(morph_kernel, int) or isinstance(morph_kernel, bool) or morph_kernel < 0:
        errors.append(
            f"segmentation.morph_open_kernel_px must be an integer >= 0, got {morph_kernel!r}."
        )

    skip_white = segmentation.get("skip_white_regions")
    if skip_white is not None and not isinstance(skip_white, bool):
        errors.append(
            f"segmentation.skip_white_regions must be a bool, got {skip_white!r}."
        )

    return errors


def _validate_path_generation(path_generation: dict) -> list:
    errors = []

    tool_width = path_generation.get("tool_width_mm")
    if not _is_number(tool_width) or tool_width <= 0:
        errors.append(f"path_generation.tool_width_mm must be > 0, got {tool_width!r}.")

    overlap_ratio = path_generation.get("stroke_overlap_ratio")
    if not _is_number(overlap_ratio) or not (0 <= overlap_ratio < 1):
        errors.append(
            f"path_generation.stroke_overlap_ratio must satisfy 0 <= ratio < 1, got {overlap_ratio!r}."
        )

    mask_erosion = path_generation.get("mask_erosion_mm")
    if not _is_number(mask_erosion) or mask_erosion < 0:
        errors.append(f"path_generation.mask_erosion_mm must be >= 0, got {mask_erosion!r}.")

    home_position = path_generation.get("home_position_mm")
    if home_position is not None:
        if (
            not isinstance(home_position, (list, tuple))
            or len(home_position) != 2
            or not all(_is_number(v) for v in home_position)
        ):
            errors.append(
                f"path_generation.home_position_mm must be a two-number list, got {home_position!r}."
            )

    return errors


def _validate_border_generation(border_generation: dict) -> list:
    errors = []

    draw_borders = border_generation.get("draw_borders")
    if draw_borders is not None and not isinstance(draw_borders, bool):
        errors.append(f"border_generation.draw_borders must be a bool, got {draw_borders!r}.")

    epsilon_ratio = border_generation.get("simplify_epsilon_ratio")
    if not _is_number(epsilon_ratio) or epsilon_ratio <= 0:
        errors.append(
            f"border_generation.simplify_epsilon_ratio must be > 0, got {epsilon_ratio!r}."
        )

    return errors


def validate_config(config: dict) -> list:
    """Return a list of human-readable error strings (empty if config is valid)."""
    errors = []

    for section in REQUIRED_SECTIONS:
        if section not in config:
            errors.append(f"Missing required top-level section: '{section}'.")

    errors.extend(_validate_canvas(config.get("canvas", {})))
    errors.extend(_validate_source_image(config.get("source_image", {})))
    errors.extend(_validate_palette(config.get("palette", {})))
    errors.extend(_validate_segmentation(config.get("segmentation", {})))
    errors.extend(_validate_path_generation(config.get("path_generation", {})))
    errors.extend(_validate_border_generation(config.get("border_generation", {})))

    output = config.get("output", {})
    for field in REQUIRED_OUTPUT_FILES:
        if not output.get(field):
            errors.append(f"output.{field} is missing or empty.")
    if not output.get("directory"):
        errors.append("output.directory is missing or empty.")

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
