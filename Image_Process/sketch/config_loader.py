"""Shared config loading + validation for the sketch (outline-tracing) pipeline.

Mirrors Image_Process/mondrian/config_loader.py's shape (load_config /
validate_config / ConfigError) but validates this pipeline's own config
schema: canvas + source_image + path_generation + output, instead of
mondrian's canvas + artwork + path_generation + output.
"""

import json
import math
from pathlib import Path

REQUIRED_SECTIONS = ["canvas", "source_image", "path_generation", "output"]

# Defaults for optional fields, applied by load_config() rather than
# required in every config file.
DEFAULT_CANNY_THRESHOLD1 = 100
DEFAULT_CANNY_THRESHOLD2 = 200
DEFAULT_MIN_STROKE_LENGTH_MM = 2.0
DEFAULT_SIMPLIFY_EPSILON_RATIO = 0.002
DEFAULT_SIMPLIFY_MIN_EPSILON_PX = 1.0
DEFAULT_JOIN_OVERLAPPING_STROKES = True
DEFAULT_MAX_JOIN_TURN_DEGREES = 60.0
# These are compatibility defaults for configs created before smoothing and
# continuous paths were introduced.  New profiles opt in explicitly.
DEFAULT_SMOOTHING_RADIUS_MM = 0.0
DEFAULT_CONTINUOUS_PATHS = False
DEFAULT_SEMANTIC_ENABLED = True
DEFAULT_SEMANTIC_PROVIDER = "qwen"
DEFAULT_SEMANTIC_MODEL = "qwen-image-2.0-pro"
DEFAULT_SEMANTIC_PROMPT = (
    "Convert this photo into clean monochrome black line art on a white "
    "background for a drawing robot. Preserve the subject's pose, proportions, "
    "facial likeness, framing, and enough major architecture and background "
    "contours to keep the scene recognizable. Draw long structural outlines "
    "clearly. Where limbs or similarly dark garments overlap, infer and draw "
    "distinct visible boundaries so the forms remain readable. If glasses are "
    "present, draw one clean glasses frame without lens reflections or duplicate "
    "rims. Treat hair primarily as an outer silhouette with only a few grouped "
    "internal lines. Do not draw individual flyaway hairs, shading, hatching, "
    "texture scribbles, reflections, text, borders, or a frame. Use sparse, "
    "smooth, single-width lines and do not invent accessories or other objects."
)
DEFAULT_SEMANTIC_TIMEOUT_SECONDS = 60.0
DEFAULT_SEMANTIC_FALLBACK_TO_LOCAL = True
DEFAULT_OUTLINE_CANNY_THRESHOLD1 = 60
DEFAULT_OUTLINE_CANNY_THRESHOLD2 = 160
DEFAULT_OUTLINE_BLUR_KERNEL_SIZE = 5
DEFAULT_OUTLINE_CONTRAST_CLIP_LIMIT = 2.0
DEFAULT_OUTLINE_LINE_ART_THRESHOLD = 210
DEFAULT_OUTLINE_MIN_PATH_LENGTH_MM = 1.0
DEFAULT_BORDER_SUPPRESSION_BAND_PX = 6
DEFAULT_DETAIL_CANNY_THRESHOLD1 = 120
DEFAULT_DETAIL_CANNY_THRESHOLD2 = 240
DEFAULT_DETAIL_BLUR_KERNEL_SIZE = 5
DEFAULT_DETAIL_MIN_PATH_LENGTH_MM = 0.5
DETAIL_MIN_PATH_HARD_FLOOR_MM = 0.5
DEFAULT_DETAIL_MAX_PATHS = 400
DEFAULT_DETAIL_DEDUPLICATION_DISTANCE_MM = 0.75
DEFAULT_DETAIL_DEDUPLICATION_PEN_WIDTH_RATIO = 0.75
DEFAULT_DETAIL_CONNECTION_DISTANCE_MM = 2.0
DEFAULT_DETAIL_ISOLATION_DISTANCE_MM = 8.0
DEFAULT_DETAIL_ISOLATED_SPECKLE_MAX_LENGTH_MM = 2.0
DEFAULT_SEMANTIC_OUTLINE_IMAGE_FILE = "sketch_outline_intermediate.png"


class ConfigError(ValueError):
    """Raised when a config file is missing or fails validation."""


def _is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _validate_odd_kernel(value, field: str, errors: list) -> None:
    if (
        not isinstance(value, int)
        or isinstance(value, bool)
        or value < 0
        or (value > 0 and value % 2 == 0)
    ):
        errors.append(f"{field} must be 0 or a positive odd integer, got {value!r}.")


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

    threshold1 = source_image.get("canny_threshold1", DEFAULT_CANNY_THRESHOLD1)
    if not _is_number(threshold1) or threshold1 < 0:
        errors.append(f"source_image.canny_threshold1 must be a number >= 0, got {threshold1!r}.")

    threshold2 = source_image.get("canny_threshold2", DEFAULT_CANNY_THRESHOLD2)
    if not _is_number(threshold2) or threshold2 < 0:
        errors.append(f"source_image.canny_threshold2 must be a number >= 0, got {threshold2!r}.")

    min_stroke_length_mm = source_image.get(
        "min_stroke_length_mm", DEFAULT_MIN_STROKE_LENGTH_MM
    )
    if not _is_number(min_stroke_length_mm) or min_stroke_length_mm < 0:
        errors.append(
            f"source_image.min_stroke_length_mm must be a number >= 0, got {min_stroke_length_mm!r}."
        )

    epsilon_ratio = source_image.get("simplify_epsilon_ratio", DEFAULT_SIMPLIFY_EPSILON_RATIO)
    if not _is_number(epsilon_ratio) or epsilon_ratio <= 0:
        errors.append(f"source_image.simplify_epsilon_ratio must be > 0, got {epsilon_ratio!r}.")

    min_epsilon_px = source_image.get("simplify_min_epsilon_px", DEFAULT_SIMPLIFY_MIN_EPSILON_PX)
    if not _is_number(min_epsilon_px) or min_epsilon_px < 0:
        errors.append(
            f"source_image.simplify_min_epsilon_px must be a number >= 0, got {min_epsilon_px!r}."
        )

    path_generation = config.get("path_generation", {})
    tool_width = path_generation.get("tool_width_mm")
    if not _is_number(tool_width) or tool_width <= 0:
        errors.append(f"path_generation.tool_width_mm must be > 0, got {tool_width!r}.")

    join_strokes = path_generation.get(
        "join_overlapping_strokes", DEFAULT_JOIN_OVERLAPPING_STROKES
    )
    if not isinstance(join_strokes, bool):
        errors.append(
            "path_generation.join_overlapping_strokes must be a boolean, "
            f"got {join_strokes!r}."
        )

    max_join_turn = path_generation.get(
        "max_join_turn_degrees", DEFAULT_MAX_JOIN_TURN_DEGREES
    )
    if not _is_number(max_join_turn) or not (0 <= max_join_turn <= 180):
        errors.append(
            "path_generation.max_join_turn_degrees must be a number in [0, 180], "
            f"got {max_join_turn!r}."
        )

    smoothing_radius = path_generation.get(
        "smoothing_radius_mm", DEFAULT_SMOOTHING_RADIUS_MM
    )
    if not _is_number(smoothing_radius) or not (0 <= smoothing_radius <= 2):
        errors.append(
            "path_generation.smoothing_radius_mm must be a number in [0, 2], "
            f"got {smoothing_radius!r}."
        )

    continuous_paths = path_generation.get(
        "continuous_paths", DEFAULT_CONTINUOUS_PATHS
    )
    if not isinstance(continuous_paths, bool):
        errors.append(
            "path_generation.continuous_paths must be a boolean, "
            f"got {continuous_paths!r}."
        )

    semantic = config.get("semantic_preprocessing", {})
    semantic_enabled = semantic.get("enabled", DEFAULT_SEMANTIC_ENABLED)
    if not isinstance(semantic_enabled, bool):
        errors.append(
            "semantic_preprocessing.enabled must be a boolean, "
            f"got {semantic_enabled!r}."
        )
    provider = semantic.get("provider", DEFAULT_SEMANTIC_PROVIDER)
    if provider != "qwen":
        errors.append(
            "semantic_preprocessing.provider must be 'qwen', "
            f"got {provider!r}."
        )
    model = semantic.get("model", DEFAULT_SEMANTIC_MODEL)
    if not isinstance(model, str) or not model.strip():
        errors.append("semantic_preprocessing.model must be a non-empty string.")
    prompt = semantic.get("prompt", DEFAULT_SEMANTIC_PROMPT)
    if not isinstance(prompt, str) or not prompt.strip():
        errors.append("semantic_preprocessing.prompt must be a non-empty string.")
    timeout = semantic.get("timeout_seconds", DEFAULT_SEMANTIC_TIMEOUT_SECONDS)
    if not _is_number(timeout) or timeout <= 0:
        errors.append(
            "semantic_preprocessing.timeout_seconds must be > 0, "
            f"got {timeout!r}."
        )
    fallback = semantic.get(
        "fallback_to_local", DEFAULT_SEMANTIC_FALLBACK_TO_LOCAL
    )
    if not isinstance(fallback, bool):
        errors.append(
            "semantic_preprocessing.fallback_to_local must be a boolean, "
            f"got {fallback!r}."
        )

    outline = config.get("outline_extraction", {})
    for field, default in (
        ("canny_threshold1", DEFAULT_OUTLINE_CANNY_THRESHOLD1),
        ("canny_threshold2", DEFAULT_OUTLINE_CANNY_THRESHOLD2),
    ):
        value = outline.get(field, default)
        if not _is_number(value) or value < 0:
            errors.append(
                f"outline_extraction.{field} must be a number >= 0, got {value!r}."
            )
    _validate_odd_kernel(
        outline.get("blur_kernel_size", DEFAULT_OUTLINE_BLUR_KERNEL_SIZE),
        "outline_extraction.blur_kernel_size",
        errors,
    )
    contrast = outline.get(
        "contrast_clip_limit", DEFAULT_OUTLINE_CONTRAST_CLIP_LIMIT
    )
    if not _is_number(contrast) or contrast < 0:
        errors.append(
            "outline_extraction.contrast_clip_limit must be a number >= 0, "
            f"got {contrast!r}."
        )
    line_art_threshold = outline.get(
        "line_art_threshold", DEFAULT_OUTLINE_LINE_ART_THRESHOLD
    )
    if (
        not isinstance(line_art_threshold, int)
        or isinstance(line_art_threshold, bool)
        or not 0 <= line_art_threshold <= 255
    ):
        errors.append(
            "outline_extraction.line_art_threshold must be an integer in [0, 255], "
            f"got {line_art_threshold!r}."
        )
    outline_minimum = outline.get(
        "min_path_length_mm", DEFAULT_OUTLINE_MIN_PATH_LENGTH_MM
    )
    if (
        not _is_number(outline_minimum)
        or not math.isfinite(outline_minimum)
        or outline_minimum < 0
    ):
        errors.append(
            "outline_extraction.min_path_length_mm must be a number >= 0, "
            f"got {outline_minimum!r}."
        )
    border_band = outline.get(
        "border_suppression_band_px", DEFAULT_BORDER_SUPPRESSION_BAND_PX
    )
    if (
        not isinstance(border_band, int)
        or isinstance(border_band, bool)
        or border_band < 0
    ):
        errors.append(
            "outline_extraction.border_suppression_band_px must be an integer >= 0, "
            f"got {border_band!r}."
        )

    detail = config.get("detail_extraction", {})
    for field, default in (
        ("canny_threshold1", DEFAULT_DETAIL_CANNY_THRESHOLD1),
        ("canny_threshold2", DEFAULT_DETAIL_CANNY_THRESHOLD2),
    ):
        value = detail.get(field, default)
        if not _is_number(value) or value < 0:
            errors.append(
                f"detail_extraction.{field} must be a number >= 0, got {value!r}."
            )
    _validate_odd_kernel(
        detail.get("blur_kernel_size", DEFAULT_DETAIL_BLUR_KERNEL_SIZE),
        "detail_extraction.blur_kernel_size",
        errors,
    )
    detail_minimum = detail.get(
        "min_path_length_mm", DEFAULT_DETAIL_MIN_PATH_LENGTH_MM
    )
    if (
        not _is_number(detail_minimum)
        or detail_minimum < DETAIL_MIN_PATH_HARD_FLOOR_MM
    ):
        errors.append(
            "detail_extraction.min_path_length_mm must be at least "
            f"{DETAIL_MIN_PATH_HARD_FLOOR_MM} mm, got {detail_minimum!r}."
        )
    max_paths = detail.get("max_paths", DEFAULT_DETAIL_MAX_PATHS)
    if not isinstance(max_paths, int) or isinstance(max_paths, bool) or max_paths < 0:
        errors.append(
            f"detail_extraction.max_paths must be an integer >= 0, got {max_paths!r}."
        )
    for field, default in (
        ("deduplication_distance_mm", DEFAULT_DETAIL_DEDUPLICATION_DISTANCE_MM),
        ("connection_distance_mm", DEFAULT_DETAIL_CONNECTION_DISTANCE_MM),
        ("isolation_distance_mm", DEFAULT_DETAIL_ISOLATION_DISTANCE_MM),
        (
            "isolated_speckle_max_length_mm",
            DEFAULT_DETAIL_ISOLATED_SPECKLE_MAX_LENGTH_MM,
        ),
    ):
        value = detail.get(field, default)
        if not _is_number(value) or value < 0:
            errors.append(
                f"detail_extraction.{field} must be a number >= 0, got {value!r}."
            )

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
    semantic_image_file = output.get(
        "semantic_outline_image_file", DEFAULT_SEMANTIC_OUTLINE_IMAGE_FILE
    )
    if not isinstance(semantic_image_file, str) or not semantic_image_file.strip():
        errors.append("output.semantic_outline_image_file must be a non-empty string.")

    return errors


def load_config(path) -> dict:
    """Load and validate a sketch pipeline config JSON file.

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
