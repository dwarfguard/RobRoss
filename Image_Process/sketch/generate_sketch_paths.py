"""Sketch (outline-tracing) route: turn an arbitrary source image into a
painting_paths.json-format file, directly - no intermediate painting_plan.json,
unlike the mondrian route, since there's no vector design to describe first.

    Config profile (configs/*.json)
      -> optional Qwen line art or deterministic local structural outline
      -> original-photo likeness detail extraction + ranking + deduplication
      -> exact-overlap joins and independent per-phase travel ordering
      -> one ordered outline-then-detail robot command list (+ previews)

The output uses the shared painting-path command vocabulary and is consumable
by the same ros2/robross_painter executor. New profiles emit one continuous
paint_path per traced line; older configs that omit continuous_paths retain
the segmented paint_stroke representation.
"""

import argparse
import json
import math
import sys
from html import escape
from pathlib import Path

import cv2
import numpy as np

from canny_edges import (
    edge_strength_map,
    line_art_mask,
    local_outline_mask,
    read_grayscale_with_metadata,
    simplify,
    suppress_border_artifacts,
    trace_mask,
)
from config_loader import (
    DEFAULT_BORDER_SUPPRESSION_BAND_PX,
    DEFAULT_DETAIL_BLUR_KERNEL_SIZE,
    DEFAULT_DETAIL_CANNY_THRESHOLD1,
    DEFAULT_DETAIL_CANNY_THRESHOLD2,
    DEFAULT_DETAIL_CONNECTION_DISTANCE_MM,
    DEFAULT_DETAIL_DEDUPLICATION_DISTANCE_MM,
    DEFAULT_DETAIL_DEDUPLICATION_PEN_WIDTH_RATIO,
    DEFAULT_DETAIL_ISOLATED_SPECKLE_MAX_LENGTH_MM,
    DEFAULT_DETAIL_ISOLATION_DISTANCE_MM,
    DEFAULT_DETAIL_MAX_PATHS,
    DEFAULT_DETAIL_MIN_PATH_LENGTH_MM,
    DEFAULT_JOIN_OVERLAPPING_STROKES,
    DEFAULT_MAX_JOIN_TURN_DEGREES,
    DEFAULT_CONTINUOUS_PATHS,
    DEFAULT_OUTLINE_BLUR_KERNEL_SIZE,
    DEFAULT_OUTLINE_CANNY_THRESHOLD1,
    DEFAULT_OUTLINE_CANNY_THRESHOLD2,
    DEFAULT_OUTLINE_CONTRAST_CLIP_LIMIT,
    DEFAULT_OUTLINE_LINE_ART_THRESHOLD,
    DEFAULT_OUTLINE_MIN_PATH_LENGTH_MM,
    DEFAULT_SEMANTIC_ENABLED,
    DEFAULT_SEMANTIC_FALLBACK_TO_LOCAL,
    DEFAULT_SEMANTIC_MODEL,
    DEFAULT_SEMANTIC_OUTLINE_IMAGE_FILE,
    DEFAULT_SEMANTIC_PROMPT,
    DEFAULT_SEMANTIC_PROVIDER,
    DEFAULT_SEMANTIC_TIMEOUT_SECONDS,
    DEFAULT_SIMPLIFY_EPSILON_RATIO,
    DEFAULT_SIMPLIFY_MIN_EPSILON_PX,
    DEFAULT_SMOOTHING_RADIUS_MM,
    load_config,
)
from path_ordering import join_overlapping_strokes, order_strokes, total_travel_distance
from path_validation import validate_painting_paths
from semantic_preprocessing import SemanticPreprocessingError, generate_outline_image

DEFAULT_CONFIG_FILE = "configs/sketch_demo_a4.json"
LINE_COLOR = "black"

# Preview-only setting (does not affect painting_paths.json). Both phases use
# the same physical black pen, so the preview must render them identically.
STROKE_PREVIEW_OPACITY = 1.0


# --- Command builders (same vocabulary as the mondrian route) ----------

def select_tool(color: str, label: str) -> dict:
    return {"command": "select_tool", "label": label, "color": color}


def dip_paint(color: str, label: str) -> dict:
    return {"command": "dip_paint", "label": label, "color": color}


def move_to(x: float, y: float, label: str) -> dict:
    return {"command": "move_to", "label": label, "x_mm": round(x, 2), "y_mm": round(y, 2)}


def lower_tool(label: str) -> dict:
    return {"command": "lower_tool", "label": label}


def paint_stroke(from_point, to_point, color: str, label: str) -> dict:
    return {
        "command": "paint_stroke",
        "label": label,
        "color": color,
        "from_mm": [round(from_point[0], 2), round(from_point[1], 2)],
        "to_mm": [round(to_point[0], 2), round(to_point[1], 2)],
    }


def paint_path(points, color: str, label: str) -> dict:
    return {
        "command": "paint_path",
        "label": label,
        "color": color,
        "points_mm": [[round(x, 2), round(y, 2)] for x, y in points],
    }


def lift_tool(label: str) -> dict:
    return {"command": "lift_tool", "label": label}


# --- Pixel-space strokes -> canvas millimeters --------------------------

def map_points_to_canvas(points_xy, image_size, drawable_origin, drawable_size):
    """Scale pixel-space (x, y) points onto the drawable canvas box,
    preserving aspect ratio (uses the smaller of the width/height scale
    factors) and centering the result within the box. Canvas and image
    both use a top-left, y-down origin, so no axis flip is needed."""
    img_w, img_h = image_size
    box_x, box_y = drawable_origin
    box_w, box_h = drawable_size

    scale = min(box_w / img_w, box_h / img_h)
    offset_x = box_x + (box_w - img_w * scale) / 2
    offset_y = box_y + (box_h - img_h * scale) / 2

    return [(offset_x + x * scale, offset_y + y * scale) for x, y in points_xy]


def _path_length(points) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def _drawable_box(config: dict):
    canvas = config["canvas"]
    margin = canvas.get("margin_mm", 0.0)
    return (
        (margin, margin),
        (canvas["width_mm"] - 2 * margin, canvas["height_mm"] - 2 * margin),
    )


def resolve_detail_deduplication_distance(config: dict) -> float:
    """Resolve explicit coverage or a pen-width-aware default capped at 0.75mm."""
    detail = config.get("detail_extraction", {})
    if "deduplication_distance_mm" in detail:
        return float(detail["deduplication_distance_mm"])
    tool_width = float(config["path_generation"]["tool_width_mm"])
    return min(
        tool_width * DEFAULT_DETAIL_DEDUPLICATION_PEN_WIDTH_RATIO,
        DEFAULT_DETAIL_DEDUPLICATION_DISTANCE_MM,
    )


def _canvas_transform(image_size, drawable_origin, drawable_size):
    image_width, image_height = image_size
    box_x, box_y = drawable_origin
    box_width, box_height = drawable_size
    scale = min(box_width / image_width, box_height / image_height)
    return (
        scale,
        box_x + (box_width - image_width * scale) / 2,
        box_y + (box_height - image_height * scale) / 2,
    )


def _dedupe_consecutive(points):
    deduped = []
    for point in points:
        point = (float(point[0]), float(point[1]))
        if not deduped or point != deduped[-1]:
            deduped.append(point)
    return deduped


def smooth_canvas_path(points, closed: bool, radius_mm: float):
    """Gaussian-smooth a dense path using arc length as the sample axis.

    Open endpoints are restored exactly, which also fixes every shared
    skeleton-junction anchor. Closed paths use cyclic padding. The sigma is
    capped at one sixth of path length so small facial marks cannot be blurred
    away by a radius intended for long outlines.

    Returns ``(smoothed_points, effective_radius_mm)``.
    """
    requested = max(0.0, float(radius_mm))
    dense = _dedupe_consecutive(points)
    if closed and len(dense) > 1 and dense[0] == dense[-1]:
        dense.pop()
    minimum_points = 3 if closed else 2
    if requested == 0 or len(dense) < minimum_points:
        effective = (
            0.0
            if requested == 0
            else min(requested, _path_length(dense) / 6.0)
        )
        return dense, effective

    array = np.asarray(dense, dtype=np.float64)
    if closed:
        segment_lengths = np.linalg.norm(
            np.roll(array, -1, axis=0) - array, axis=1
        )
    else:
        segment_lengths = np.linalg.norm(np.diff(array, axis=0), axis=1)
    total_length = float(segment_lengths.sum())
    if total_length <= 0:
        return dense, 0.0

    effective = min(requested, total_length / 6.0)
    if effective <= 0:
        return dense, 0.0

    # Resample at uniform arc-length intervals before convolution. Raster
    # centerlines contain both axial and diagonal pixel steps, so indexing the
    # raw points directly would give those directions unequal smoothing weight.
    if closed:
        extended = np.vstack((array, array[0]))
        cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        sample_distances = np.linspace(0.0, total_length, len(array), endpoint=False)
    else:
        extended = array
        cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        sample_distances = np.linspace(0.0, total_length, len(array))
    uniform = np.column_stack(
        [np.interp(sample_distances, cumulative, extended[:, axis]) for axis in (0, 1)]
    )

    spacing = total_length / (len(uniform) if closed else max(1, len(uniform) - 1))
    sigma_samples = effective / max(spacing, 1e-9)
    half_width = max(1, int(math.ceil(3.0 * sigma_samples)))
    offsets = np.arange(-half_width, half_width + 1, dtype=np.float64)
    kernel = np.exp(-0.5 * (offsets / max(sigma_samples, 1e-9)) ** 2)
    kernel /= kernel.sum()
    pad_mode = "wrap" if closed else "edge"
    smoothed = np.column_stack(
        [
            np.convolve(
                np.pad(uniform[:, axis], half_width, mode=pad_mode),
                kernel,
                mode="valid",
            )
            for axis in (0, 1)
        ]
    )
    if not closed:
        smoothed[0] = array[0]
        smoothed[-1] = array[-1]
    return [tuple(point) for point in smoothed], effective


def _simplify_canvas_points(points, closed, epsilon_ratio, min_epsilon_mm):
    array = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
    if len(array) < 3:
        return [tuple(float(v) for v in point) for point in array.reshape(-1, 2)]
    epsilon = max(
        float(min_epsilon_mm),
        float(epsilon_ratio) * cv2.arcLength(array, closed),
    )
    approximation = cv2.approxPolyDP(array, epsilon, closed)
    return [
        tuple(float(v) for v in point)
        for point in approximation.reshape(-1, 2)
    ]


def _map_traced_strokes(
    raw_strokes,
    image_size,
    drawable_origin,
    drawable_size,
    epsilon_ratio,
    min_epsilon_px,
    smoothing_radius_mm=DEFAULT_SMOOTHING_RADIUS_MM,
):
    traced = []
    points_before = 0
    points_after_smoothing = 0
    points_after_simplification = 0
    effective_radii = []
    scale, _offset_x, _offset_y = _canvas_transform(
        image_size, drawable_origin, drawable_size
    )
    for points_xy, closed in raw_strokes:
        points_before += len(points_xy)
        if smoothing_radius_mm <= 0:
            # Preserve the pre-feature integer-pixel simplification path exactly
            # when smoothing is disabled or absent from an older config.
            simplified = simplify(points_xy, closed, epsilon_ratio, min_epsilon_px)
            smoothed_count = len(points_xy)
            effective_radius = 0.0
            canvas_points = map_points_to_canvas(
                simplified, image_size, drawable_origin, drawable_size
            )
        else:
            dense_canvas_points = map_points_to_canvas(
                points_xy, image_size, drawable_origin, drawable_size
            )
            smoothed, effective_radius = smooth_canvas_path(
                dense_canvas_points, closed, smoothing_radius_mm
            )
            smoothed_count = len(smoothed)
            canvas_points = _simplify_canvas_points(
                smoothed,
                closed,
                epsilon_ratio,
                float(min_epsilon_px) * scale,
            )
        points_after_smoothing += smoothed_count
        effective_radii.append(effective_radius)
        simplified = canvas_points
        if len(simplified) < 2:
            continue
        if closed and simplified[0] != simplified[-1]:
            simplified = simplified + [simplified[0]]
        points_after_simplification += len(simplified)
        traced.append((simplified, closed))
    return traced, {
        "points_before_smoothing": points_before,
        "points_after_smoothing": points_after_smoothing,
        "points_after_simplification": points_after_simplification,
        "effective_smoothing_radius_mm": round(max(effective_radii, default=0.0), 4),
        "minimum_effective_smoothing_radius_mm": round(
            min(effective_radii, default=0.0), 4
        ),
    }


def _decode_generated_grayscale(image_bytes: bytes):
    encoded = np.frombuffer(image_bytes, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise SemanticPreprocessingError("malformed_response")
    if image.ndim == 2:
        return image
    if image.shape[2] == 4:
        color = image[:, :, :3].astype(np.float32)
        alpha = image[:, :, 3:4].astype(np.float32) / 255.0
        image = np.clip(color * alpha + 255.0 * (1.0 - alpha), 0, 255).astype(
            np.uint8
        )
    return cv2.cvtColor(image[:, :, :3], cv2.COLOR_BGR2GRAY)


def _safe_semantic_reason(error: Exception) -> str:
    if isinstance(error, SemanticPreprocessingError):
        return error.reason
    if isinstance(error, (ImportError, ModuleNotFoundError)):
        return "missing_dependency"
    if isinstance(error, TimeoutError):
        return "timeout"
    return "api_failure"


def _structural_mask(
    config: dict,
    image_path: Path,
    original_gray,
    output_dir: Path = None,
    semantic_generator=None,
    saved_semantic_image: Path = None,
):
    """Return structural mask, source size, and sanitized semantic metrics."""
    semantic = config.get("semantic_preprocessing", {})
    outline = config.get("outline_extraction", {})
    enabled = semantic.get("enabled", DEFAULT_SEMANTIC_ENABLED)
    provider = semantic.get("provider", DEFAULT_SEMANTIC_PROVIDER)
    model = semantic.get("model", DEFAULT_SEMANTIC_MODEL)
    fallback = semantic.get("fallback_to_local", DEFAULT_SEMANTIC_FALLBACK_TO_LOCAL)
    output_filename = config.get("output", {}).get(
        "semantic_outline_image_file", DEFAULT_SEMANTIC_OUTLINE_IMAGE_FILE
    )
    output_path = output_dir / output_filename if output_dir is not None else None

    status = "disabled"
    reason = "disabled_by_config"
    generated_gray = None
    if saved_semantic_image is not None:
        saved_semantic_image = Path(saved_semantic_image)
        try:
            generated_bytes = saved_semantic_image.read_bytes()
            if not generated_bytes:
                raise OSError("empty file")
            generated_gray = _decode_generated_grayscale(generated_bytes)
        except (OSError, SemanticPreprocessingError):
            raise RuntimeError(
                f"Saved semantic image is missing or unreadable: {saved_semantic_image}"
            ) from None
        status = "reused"
        reason = "saved_semantic_image"
    elif enabled:
        generator = semantic_generator or generate_outline_image
        try:
            generated_bytes = generator(
                image_path,
                semantic.get("prompt", DEFAULT_SEMANTIC_PROMPT),
                model,
                semantic.get("timeout_seconds", DEFAULT_SEMANTIC_TIMEOUT_SECONDS),
            )
            generated_gray = _decode_generated_grayscale(generated_bytes)
            status = "used"
            reason = "success"
        except Exception as error:
            reason = _safe_semantic_reason(error)
            if not fallback:
                raise RuntimeError(
                    f"Semantic preprocessing failed ({reason}) and local fallback is disabled."
                ) from None
            status = "local_fallback"

    if generated_gray is not None:
        semantic_threshold = outline.get(
            "line_art_threshold", DEFAULT_OUTLINE_LINE_ART_THRESHOLD
        )
        mask = line_art_mask(
            generated_gray,
            semantic_threshold,
        )
        image_size = (generated_gray.shape[1], generated_gray.shape[0])
        if output_path is not None:
            if status == "reused":
                if output_path.resolve() != saved_semantic_image.resolve():
                    output_path.write_bytes(generated_bytes)
            else:
                cv2.imwrite(str(output_path), generated_gray)
    else:
        source = config["source_image"]
        threshold1 = outline.get(
            "canny_threshold1",
            source.get("canny_threshold1", DEFAULT_OUTLINE_CANNY_THRESHOLD1),
        )
        threshold2 = outline.get(
            "canny_threshold2",
            source.get("canny_threshold2", DEFAULT_OUTLINE_CANNY_THRESHOLD2),
        )
        mask = local_outline_mask(
            original_gray,
            threshold1,
            threshold2,
            outline.get("blur_kernel_size", DEFAULT_OUTLINE_BLUR_KERNEL_SIZE),
            outline.get(
                "contrast_clip_limit", DEFAULT_OUTLINE_CONTRAST_CLIP_LIMIT
            ),
        )
        image_size = (original_gray.shape[1], original_gray.shape[0])

    cleaned_mask, border_metrics = suppress_border_artifacts(
        mask,
        outline.get(
            "border_suppression_band_px", DEFAULT_BORDER_SUPPRESSION_BAND_PX
        ),
    )
    if output_path is not None and generated_gray is None:
        cv2.imwrite(
            str(output_path), np.where(cleaned_mask, 0, 255).astype(np.uint8)
        )
    metrics = {
        "semantic_preprocessing_status": status,
        "semantic_preprocessing_reason": reason,
        "semantic_preprocessing_provider": provider,
        "semantic_preprocessing_model": model,
        "semantic_outline_image_file": str(output_path) if output_path else output_filename,
        "semantic_reused_image_file": (
            str(saved_semantic_image) if saved_semantic_image is not None else None
        ),
        "semantic_line_art_threshold": (
            semantic_threshold if generated_gray is not None else None
        ),
        "outline_border_components_filtered": border_metrics[
            "border_components_filtered"
        ],
        "outline_border_pixels_filtered": border_metrics["border_pixels_filtered"],
    }
    return cleaned_mask, image_size, metrics


class _SegmentDistanceIndex:
    """Small uniform-grid index for point-to-outline distance queries."""

    def __init__(self, strokes, cell_size=2.0):
        self.cell_size = max(float(cell_size), 0.1)
        self.grid = {}
        self.segments = []
        for points, _closed in strokes:
            for start, end in zip(points, points[1:]):
                segment_index = len(self.segments)
                self.segments.append((start, end))
                min_col = math.floor(min(start[0], end[0]) / self.cell_size)
                max_col = math.floor(max(start[0], end[0]) / self.cell_size)
                min_row = math.floor(min(start[1], end[1]) / self.cell_size)
                max_row = math.floor(max(start[1], end[1]) / self.cell_size)
                for col in range(min_col, max_col + 1):
                    for row in range(min_row, max_row + 1):
                        self.grid.setdefault((col, row), set()).add(segment_index)

    @staticmethod
    def _point_segment_distance(point, segment):
        start, end = segment
        delta_x = end[0] - start[0]
        delta_y = end[1] - start[1]
        denominator = delta_x * delta_x + delta_y * delta_y
        if denominator == 0:
            return math.dist(point, start)
        position = (
            (point[0] - start[0]) * delta_x
            + (point[1] - start[1]) * delta_y
        ) / denominator
        position = max(0.0, min(1.0, position))
        projection = (start[0] + position * delta_x, start[1] + position * delta_y)
        return math.dist(point, projection)

    def distance(self, point, search_distance):
        if not self.segments:
            return math.inf
        center_col = math.floor(point[0] / self.cell_size)
        center_row = math.floor(point[1] / self.cell_size)
        radius = max(1, math.ceil(float(search_distance) / self.cell_size))
        candidates = set()
        for col in range(center_col - radius, center_col + radius + 1):
            for row in range(center_row - radius, center_row + radius + 1):
                candidates.update(self.grid.get((col, row), ()))
        if not candidates:
            return math.inf
        return min(
            self._point_segment_distance(point, self.segments[index])
            for index in candidates
        )


def remove_outline_duplicate_segments(
    detail_strokes, outline_strokes, distance_mm: float
):
    """Remove only detail segments already covered by structural ink.

    Remaining neighboring segments stay contiguous. No gap is bridged, so this
    operation cannot create a pen-down connector through blank space.
    Returns ``(remaining_strokes, removed_segment_count)``.
    """
    if distance_mm <= 0 or not outline_strokes:
        return list(detail_strokes), 0
    index = _SegmentDistanceIndex(outline_strokes, cell_size=distance_mm)
    remaining = []
    removed = 0
    for points, closed in detail_strokes:
        chunks = []
        current = []
        for start, end in zip(points, points[1:]):
            midpoint = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
            covered = all(
                index.distance(sample, distance_mm) <= distance_mm
                for sample in (start, midpoint, end)
            )
            if covered:
                removed += 1
                if len(current) >= 2:
                    chunks.append(current)
                current = []
                continue
            if not current:
                current = [start, end]
            elif current[-1] == start:
                current.append(end)
            else:
                if len(current) >= 2:
                    chunks.append(current)
                current = [start, end]
        if len(current) >= 2:
            chunks.append(current)
        if not chunks:
            continue
        unchanged = len(chunks) == 1 and len(chunks[0]) == len(points)
        remaining.extend((chunk, closed and unchanged) for chunk in chunks)
    return remaining, removed


def _sample_edge_strength(
    points,
    strength_map,
    image_size,
    drawable_origin,
    drawable_size,
):
    scale, offset_x, offset_y = _canvas_transform(
        image_size, drawable_origin, drawable_size
    )
    image_width, image_height = image_size
    samples = []
    samples_mm = list(points)
    samples_mm.extend(
        ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        for a, b in zip(points, points[1:])
    )
    for x_mm, y_mm in samples_mm:
        x = min(image_width - 1, max(0, round((x_mm - offset_x) / scale)))
        y = min(image_height - 1, max(0, round((y_mm - offset_y) / scale)))
        samples.append(float(strength_map[y, x]))
    return sum(samples) / len(samples) if samples else 0.0


def rank_detail_candidates(
    detail_strokes,
    outline_strokes,
    detail_settings: dict,
    edge_strengths=None,
):
    """Filter, score, and budget likeness-detail candidates deterministically."""
    minimum = detail_settings.get(
        "min_path_length_mm", DEFAULT_DETAIL_MIN_PATH_LENGTH_MM
    )
    max_paths = detail_settings.get("max_paths", DEFAULT_DETAIL_MAX_PATHS)
    connection_distance = detail_settings.get(
        "connection_distance_mm", DEFAULT_DETAIL_CONNECTION_DISTANCE_MM
    )
    isolation_distance = detail_settings.get(
        "isolation_distance_mm", DEFAULT_DETAIL_ISOLATION_DISTANCE_MM
    )
    speckle_maximum = detail_settings.get(
        "isolated_speckle_max_length_mm",
        DEFAULT_DETAIL_ISOLATED_SPECKLE_MAX_LENGTH_MM,
    )
    search_distance = max(connection_distance, isolation_distance, 1.0)
    outline_index = _SegmentDistanceIndex(
        outline_strokes, cell_size=max(connection_distance, 1.0)
    )

    strengths = edge_strengths or [1.0] * len(detail_strokes)
    ranked = []
    short_filtered = 0
    isolated_filtered = 0
    for candidate_index, ((points, closed), strength) in enumerate(
        zip(detail_strokes, strengths)
    ):
        length = _path_length(points)
        if length < minimum:
            short_filtered += 1
            continue
        samples = list(points)
        samples.extend(
            ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            for a, b in zip(points, points[1:])
        )
        proximity = min(
            (outline_index.distance(point, search_distance) for point in samples),
            default=math.inf,
        )
        endpoint_connected = any(
            outline_index.distance(point, connection_distance) <= connection_distance
            for point in (points[0], points[-1])
        )
        if length <= speckle_maximum and proximity > isolation_distance:
            isolated_filtered += 1
            continue

        length_score = min(length / 20.0, 1.0)
        proximity_score = 0.0 if math.isinf(proximity) else 1.0 / (1.0 + proximity / 4.0)
        isolation_penalty = (
            1.0 if math.isinf(proximity) else min(proximity / max(isolation_distance, 0.1), 1.0)
        )
        score = (
            0.35 * length_score
            + 0.30 * max(0.0, min(float(strength), 1.0))
            + 0.25 * proximity_score
            + 0.15 * float(endpoint_connected)
            - 0.10 * isolation_penalty
        )
        ranked.append((score, length, candidate_index, (points, closed)))

    ranked.sort(key=lambda item: (-item[0], -item[1], item[2]))
    selected = [item[3] for item in ranked[:max_paths]]
    return selected, {
        "detail_num_short_paths_filtered": short_filtered,
        "detail_num_isolated_speckles_filtered": isolated_filtered,
        "detail_num_budget_paths_filtered": max(0, len(ranked) - max_paths),
        "detail_num_ranked_candidates": len(ranked),
    }


def build_two_phase_strokes(
    config: dict,
    output_dir: Path = None,
    semantic_generator=None,
    saved_semantic_image: Path = None,
):
    """Extract ordered-phase inputs: structural outlines, then photo details."""
    source = config["source_image"]
    path_generation = config["path_generation"]
    outline_settings = config.get("outline_extraction", {})
    detail_settings = config.get("detail_extraction", {})
    image_path = Path(source["path"])
    original_gray, source_metadata = read_grayscale_with_metadata(image_path)
    original_size = (original_gray.shape[1], original_gray.shape[0])
    drawable_origin, drawable_size = _drawable_box(config)
    epsilon_ratio = source.get(
        "simplify_epsilon_ratio", DEFAULT_SIMPLIFY_EPSILON_RATIO
    )
    min_epsilon_px = source.get(
        "simplify_min_epsilon_px", DEFAULT_SIMPLIFY_MIN_EPSILON_PX
    )
    if output_dir is not None:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

    outline_mask, outline_size, stats = _structural_mask(
        config,
        image_path,
        original_gray,
        output_dir=output_dir,
        semantic_generator=semantic_generator,
        saved_semantic_image=saved_semantic_image,
    )
    raw_outline = trace_mask(outline_mask)
    smoothing_radius = path_generation.get(
        "smoothing_radius_mm", DEFAULT_SMOOTHING_RADIUS_MM
    )
    traced_outline, outline_smoothing_stats = _map_traced_strokes(
        raw_outline,
        outline_size,
        drawable_origin,
        drawable_size,
        epsilon_ratio,
        min_epsilon_px,
        smoothing_radius,
    )
    should_join = path_generation.get(
        "join_overlapping_strokes", DEFAULT_JOIN_OVERLAPPING_STROKES
    )
    max_join_turn = path_generation.get(
        "max_join_turn_degrees", DEFAULT_MAX_JOIN_TURN_DEGREES
    )
    if should_join:
        joined_outline, outline_joins = join_overlapping_strokes(
            traced_outline, max_turn_degrees=max_join_turn
        )
    else:
        joined_outline, outline_joins = traced_outline, 0
    outline_minimum = outline_settings.get(
        "min_path_length_mm",
        source.get("min_stroke_length_mm", DEFAULT_OUTLINE_MIN_PATH_LENGTH_MM),
    )
    joined_outline_lengths = [_path_length(points) for points, _ in joined_outline]
    outline_strokes = [
        (points, closed)
        for (points, closed), length in zip(joined_outline, joined_outline_lengths)
        if length >= outline_minimum
    ]

    def length_distribution(lengths):
        return {
            "under_1_mm": sum(length < 1.0 for length in lengths),
            "1_to_under_2_mm": sum(1.0 <= length < 2.0 for length in lengths),
            "2_to_under_5_mm": sum(2.0 <= length < 5.0 for length in lengths),
            "5_mm_and_over": sum(length >= 5.0 for length in lengths),
        }

    detail_working = original_gray
    detail_blur = detail_settings.get(
        "blur_kernel_size", DEFAULT_DETAIL_BLUR_KERNEL_SIZE
    )
    if detail_blur > 1:
        detail_working = cv2.GaussianBlur(
            detail_working, (detail_blur, detail_blur), 0
        )
    detail_threshold1 = detail_settings.get(
        "canny_threshold1",
        DEFAULT_DETAIL_CANNY_THRESHOLD1,
    )
    detail_threshold2 = detail_settings.get(
        "canny_threshold2",
        DEFAULT_DETAIL_CANNY_THRESHOLD2,
    )
    detail_mask = cv2.Canny(
        detail_working,
        threshold1=float(detail_threshold1),
        threshold2=float(detail_threshold2),
    ) > 0
    detail_mask, detail_border_metrics = suppress_border_artifacts(
        detail_mask,
        outline_settings.get(
            "border_suppression_band_px", DEFAULT_BORDER_SUPPRESSION_BAND_PX
        ),
    )
    raw_detail = trace_mask(detail_mask)
    traced_detail, detail_smoothing_stats = _map_traced_strokes(
        raw_detail,
        original_size,
        drawable_origin,
        drawable_size,
        epsilon_ratio,
        min_epsilon_px,
        smoothing_radius,
    )
    deduplication_distance = resolve_detail_deduplication_distance(config)
    deduplicated_detail, duplicate_segments = remove_outline_duplicate_segments(
        traced_detail,
        outline_strokes,
        deduplication_distance,
    )
    strengths_map = edge_strength_map(original_gray)
    strengths = [
        _sample_edge_strength(
            points,
            strengths_map,
            original_size,
            drawable_origin,
            drawable_size,
        )
        for points, _closed in deduplicated_detail
    ]
    selected_detail, detail_stats = rank_detail_candidates(
        deduplicated_detail,
        outline_strokes,
        detail_settings,
        strengths,
    )
    if should_join:
        detail_strokes, detail_joins = join_overlapping_strokes(
            selected_detail, max_turn_degrees=max_join_turn
        )
    else:
        detail_strokes, detail_joins = selected_detail, 0

    stats.update(
        {
            **source_metadata,
            "detail_working_dimensions_px": {
                "width": original_size[0],
                "height": original_size[1],
            },
            "detail_deduplication_distance_mm": deduplication_distance,
            "smoothing_radius_requested_mm": smoothing_radius,
            "smoothing_radius_effective_mm": max(
                outline_smoothing_stats["effective_smoothing_radius_mm"],
                detail_smoothing_stats["effective_smoothing_radius_mm"],
            ),
            "outline_points_before_smoothing": outline_smoothing_stats[
                "points_before_smoothing"
            ],
            "outline_points_after_smoothing": outline_smoothing_stats[
                "points_after_smoothing"
            ],
            "outline_points_after_simplification": outline_smoothing_stats[
                "points_after_simplification"
            ],
            "outline_effective_smoothing_radius_mm": outline_smoothing_stats[
                "effective_smoothing_radius_mm"
            ],
            "detail_points_before_smoothing": detail_smoothing_stats[
                "points_before_smoothing"
            ],
            "detail_points_after_smoothing": detail_smoothing_stats[
                "points_after_smoothing"
            ],
            "detail_points_after_simplification": detail_smoothing_stats[
                "points_after_simplification"
            ],
            "detail_effective_smoothing_radius_mm": detail_smoothing_stats[
                "effective_smoothing_radius_mm"
            ],
            "points_before_smoothing": (
                outline_smoothing_stats["points_before_smoothing"]
                + detail_smoothing_stats["points_before_smoothing"]
            ),
            "points_after_smoothing": (
                outline_smoothing_stats["points_after_smoothing"]
                + detail_smoothing_stats["points_after_smoothing"]
            ),
            "points_after_simplification": (
                outline_smoothing_stats["points_after_simplification"]
                + detail_smoothing_stats["points_after_simplification"]
            ),
            "outline_num_raw_paths": len(raw_outline),
            "outline_num_overlap_joins": outline_joins,
            "outline_min_path_length_mm": outline_minimum,
            "outline_joined_length_distribution": length_distribution(
                joined_outline_lengths
            ),
            "outline_num_short_paths_filtered": len(joined_outline)
            - len(outline_strokes),
            "outline_short_paths_filtered_distance_mm": round(
                sum(
                    length
                    for length in joined_outline_lengths
                    if length < outline_minimum
                ),
                2,
            ),
            "outline_retained_distance_mm": round(
                sum(
                    length
                    for length in joined_outline_lengths
                    if length >= outline_minimum
                ),
                2,
            ),
            "outline_num_paths": len(outline_strokes),
            "detail_num_raw_paths": len(raw_detail),
            "detail_border_components_filtered": detail_border_metrics[
                "border_components_filtered"
            ],
            "detail_border_pixels_filtered": detail_border_metrics[
                "border_pixels_filtered"
            ],
            "detail_num_duplicate_segments_filtered": duplicate_segments,
            "detail_num_overlap_joins": detail_joins,
            "detail_num_paths": len(detail_strokes),
        }
    )
    stats.update(detail_stats)
    provenance = config.get("retrace_provenance")
    if isinstance(provenance, dict):
        stats["retrace_provenance"] = provenance
    return outline_strokes, detail_strokes, image_path, stats


def build_canvas_strokes(config: dict, include_stats: bool = False) -> tuple:
    """Compatibility wrapper returning the two phases as one ordered list."""
    output_dir = Path(config["output"]["directory"])
    outlines, details, image_path, stats = build_two_phase_strokes(
        config, output_dir=output_dir
    )
    combined = outlines + details
    if include_stats:
        return combined, image_path, stats
    return combined, image_path


# --- Ordered strokes -> path commands -----------------------------------

def stroke_to_commands(
    points,
    stroke_index: int,
    phase: str = "outline",
    continuous_paths: bool = DEFAULT_CONTINUOUS_PATHS,
) -> list:
    """Turn one traced line into a pen-down command bracket.

    Consecutive duplicate points are collapsed so neither command mode emits
    zero-length geometry. Older callers remain segmented by default.
    """
    if phase not in ("outline", "detail"):
        raise ValueError(f"Unknown sketch phase: {phase!r}")
    label = f"sketch_{phase}_{stroke_index}"
    deduped = []
    for point in points:
        rounded = (round(point[0], 2), round(point[1], 2))
        if not deduped or rounded != deduped[-1]:
            deduped.append(rounded)
    if len(deduped) < 2:
        return []

    commands = [move_to(deduped[0][0], deduped[0][1], label), lower_tool(label)]
    if continuous_paths:
        commands.append(paint_path(deduped, LINE_COLOR, label))
    else:
        for from_point, to_point in zip(deduped, deduped[1:]):
            commands.append(paint_stroke(from_point, to_point, LINE_COLOR, label))
    commands.append(lift_tool(label))
    return commands


def build_commands(
    ordered_outline_strokes: list,
    ordered_detail_strokes=None,
    continuous_paths: bool = DEFAULT_CONTINUOUS_PATHS,
) -> list:
    """Emit one tool setup followed by every outline and then every detail."""
    ordered_detail_strokes = ordered_detail_strokes or []
    commands = [select_tool(LINE_COLOR, "sketch"), dip_paint(LINE_COLOR, "sketch")]
    for index, points in enumerate(ordered_outline_strokes, start=1):
        commands.extend(
            stroke_to_commands(points, index, "outline", continuous_paths)
        )
    for index, points in enumerate(ordered_detail_strokes, start=1):
        commands.extend(stroke_to_commands(points, index, "detail", continuous_paths))
    return commands


def build_painting_paths(
    commands: list,
    config: dict,
    config_path: Path,
    image_path: Path,
    pen_up_travel_mm: dict,
    optimization_stats: dict = None,
) -> dict:
    stroke_commands = [cmd for cmd in commands if cmd["command"] == "paint_stroke"]
    path_commands = [cmd for cmd in commands if cmd["command"] == "paint_path"]
    total_distance = sum(
        math.dist(cmd["from_mm"], cmd["to_mm"]) for cmd in stroke_commands
    ) + sum(
        _path_length(cmd["points_mm"]) for cmd in path_commands
    )

    def count_commands(command_name: str) -> int:
        return sum(1 for cmd in commands if cmd["command"] == command_name)

    canvas = config["canvas"]
    path_generation = config["path_generation"]

    debug = {
        "num_commands": len(commands),
        "num_paint_stroke_commands": len(stroke_commands),
        "num_paint_path_commands": len(path_commands),
        "continuous_paths": path_generation.get(
            "continuous_paths", DEFAULT_CONTINUOUS_PATHS
        ),
        "estimated_total_paint_distance_mm": round(total_distance, 2),
        "num_traced_lines": count_commands("lower_tool"),
        "num_select_tool_commands": count_commands("select_tool"),
        "num_lift_tool_commands": count_commands("lift_tool"),
        "num_lower_tool_commands": count_commands("lower_tool"),
        "num_dip_paint_commands": count_commands("dip_paint"),
        "baseline_pen_up_travel_mm": round(pen_up_travel_mm["baseline"], 2),
        "optimized_pen_up_travel_mm": round(pen_up_travel_mm["optimized"], 2),
    }
    if optimization_stats:
        debug.update(optimization_stats)

    return {
        "project": config.get("project"),
        "style": config.get("style", "sketch"),
        "version": "0.2",
        "config": {
            "profile_name": config.get("profile_name"),
            "source_file": str(config_path),
        },
        "source_file": str(image_path),
        "units": "mm",
        "canvas": {
            "width_mm": canvas["width_mm"],
            "height_mm": canvas["height_mm"],
            "width_in": canvas["width_mm"] / 25.4,
            "height_in": canvas["height_mm"] / 25.4,
            "origin": canvas["origin"],
        },
        "path_settings": {
            "tool_width_mm": path_generation["tool_width_mm"],
            "join_overlapping_strokes": path_generation.get(
                "join_overlapping_strokes", DEFAULT_JOIN_OVERLAPPING_STROKES
            ),
            "max_join_turn_degrees": path_generation.get(
                "max_join_turn_degrees", DEFAULT_MAX_JOIN_TURN_DEGREES
            ),
            "smoothing_radius_mm": path_generation.get(
                "smoothing_radius_mm", DEFAULT_SMOOTHING_RADIUS_MM
            ),
            "continuous_paths": path_generation.get(
                "continuous_paths", DEFAULT_CONTINUOUS_PATHS
            ),
            "detail_min_path_length_mm": config.get("detail_extraction", {}).get(
                "min_path_length_mm", DEFAULT_DETAIL_MIN_PATH_LENGTH_MM
            ),
            "detail_max_paths": config.get("detail_extraction", {}).get(
                "max_paths", DEFAULT_DETAIL_MAX_PATHS
            ),
            "outline_detail_deduplication_distance_mm": (
                resolve_detail_deduplication_distance(config)
            ),
        },
        "commands": commands,
        "debug": debug,
    }


# --- SVG preview ---------------------------------------------------------

def render_svg(painting_paths: dict) -> str:
    """Draw command geometry exactly as represented in painting_paths JSON."""
    canvas = painting_paths["canvas"]
    width_mm = canvas["width_mm"]
    height_mm = canvas["height_mm"]
    tool_width = painting_paths["path_settings"]["tool_width_mm"]

    elements = [f'<rect x="0" y="0" width="{width_mm}" height="{height_mm}" fill="white" />']

    for cmd in painting_paths["commands"]:
        if cmd["command"] == "paint_stroke":
            x1, y1 = cmd["from_mm"]
            x2, y2 = cmd["to_mm"]
            elements.append(
                f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
                f'stroke="{escape(cmd["color"])}" stroke-width="{tool_width}" '
                f'stroke-linecap="round" stroke-opacity="{STROKE_PREVIEW_OPACITY}" />'
            )
        elif cmd["command"] == "paint_path":
            points = " ".join(f"{x},{y}" for x, y in cmd["points_mm"])
            elements.append(
                f'<polyline points="{points}" fill="none" '
                f'stroke="{escape(cmd["color"])}" stroke-width="{tool_width}" '
                f'stroke-linecap="round" stroke-linejoin="round" '
                f'stroke-opacity="{STROKE_PREVIEW_OPACITY}" />'
            )

    svg_body = "\n  ".join(elements)
    width_in = canvas["width_in"]
    height_in = canvas["height_in"]

    return f'''<svg
  xmlns="http://www.w3.org/2000/svg"
  width="{width_in}in"
  height="{height_in}in"
  viewBox="0 0 {width_mm} {height_mm}"
>
  {svg_body}
</svg>
'''


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert a source image into a painting_paths.json-format file via Canny edge tracing."
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_FILE,
        help=f"Path to a sketch pipeline config JSON file (default: {DEFAULT_CONFIG_FILE}).",
    )
    parser.add_argument(
        "--saved-semantic-image",
        help=(
            "Reuse this saved semantic line-art image and bypass Qwen entirely. "
            "Missing, empty, or undecodable files are fatal."
        ),
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_config(config_path)

    output = config["output"]
    output_dir = Path(output["directory"])
    paths_output_file = output_dir / output["painting_paths_file"]
    svg_output_file = output_dir / output["preview_svg_file"]
    output_dir.mkdir(parents=True, exist_ok=True)

    outline_strokes, detail_strokes, image_path, optimization_stats = (
        build_two_phase_strokes(
            config,
            output_dir=output_dir,
            saved_semantic_image=(
                Path(args.saved_semantic_image)
                if args.saved_semantic_image is not None
                else None
            ),
        )
    )
    if not outline_strokes and not detail_strokes:
        print(f"No traced lines found in {image_path} - nothing to write.")
        sys.exit(1)

    path_generation = config["path_generation"]
    canvas = config["canvas"]
    margin = canvas.get("margin_mm", 0.0)
    home_position = tuple(path_generation.get("home_position_mm", (margin, margin)))

    outline_baseline_travel = total_travel_distance(
        [points for points, _ in outline_strokes], home_position
    )
    ordered_outline = order_strokes(outline_strokes, home_position)
    outline_optimized_travel = total_travel_distance(
        ordered_outline, home_position
    )
    detail_home = ordered_outline[-1][-1] if ordered_outline else home_position
    detail_baseline_travel = total_travel_distance(
        [points for points, _ in detail_strokes], detail_home
    )
    ordered_detail = order_strokes(detail_strokes, detail_home)
    detail_optimized_travel = total_travel_distance(ordered_detail, detail_home)
    baseline_travel = outline_baseline_travel + detail_baseline_travel
    optimized_travel = outline_optimized_travel + detail_optimized_travel

    optimization_stats.update(
        {
            "outline_paint_distance_mm": round(
                sum(_path_length(points) for points in ordered_outline), 2
            ),
            "outline_num_lifts": len(ordered_outline),
            "outline_baseline_pen_up_travel_mm": round(outline_baseline_travel, 2),
            "outline_optimized_pen_up_travel_mm": round(
                outline_optimized_travel, 2
            ),
            "detail_paint_distance_mm": round(
                sum(_path_length(points) for points in ordered_detail), 2
            ),
            "detail_num_lifts": len(ordered_detail),
            "detail_baseline_pen_up_travel_mm": round(detail_baseline_travel, 2),
            "detail_optimized_pen_up_travel_mm": round(
                detail_optimized_travel, 2
            ),
        }
    )

    commands = build_commands(
        ordered_outline,
        ordered_detail,
        continuous_paths=path_generation.get(
            "continuous_paths", DEFAULT_CONTINUOUS_PATHS
        ),
    )
    painting_paths = build_painting_paths(
        commands, config, config_path, image_path,
        {"baseline": baseline_travel, "optimized": optimized_travel},
        optimization_stats,
    )

    validation = validate_painting_paths(painting_paths)
    painting_paths["validation"] = validation

    with open(paths_output_file, "w", encoding="utf-8") as file:
        json.dump(painting_paths, file, indent=2)
    print(
        f"Generated {paths_output_file} "
        f"({len(ordered_outline)} outlines + {len(ordered_detail)} details)"
    )
    print(
        "Semantic outline: "
        f"{optimization_stats['semantic_preprocessing_status']} "
        f"({optimization_stats['semantic_preprocessing_reason']})"
    )
    print(
        "Two-phase paths: "
        f"{optimization_stats['outline_num_raw_paths']} raw outline components -> "
        f"{len(ordered_outline)} outlines; "
        f"{optimization_stats['detail_num_raw_paths']} raw detail components -> "
        f"{len(ordered_detail)} selected details"
    )

    svg_content = render_svg(painting_paths)
    with open(svg_output_file, "w", encoding="utf-8") as file:
        file.write(svg_content)
    print(f"Generated {svg_output_file}")

    reduction = (
        (1 - optimized_travel / baseline_travel) * 100
        if baseline_travel > 0
        else 0.0
    )
    print(
        f"Pen-up travel: {baseline_travel:.1f}mm baseline -> {optimized_travel:.1f}mm ordered "
        f"({reduction:.1f}% reduction)"
    )

    for warning in validation["warnings"]:
        print(f"Validation warning: {warning}")
    if validation["passed"]:
        print(f"Validation passed ({len(validation['warnings'])} warnings).")
    else:
        for error in validation["errors"]:
            print(f"Validation error: {error}")
        print(
            f"Validation FAILED with {len(validation['errors'])} error(s) - "
            f"do not send {paths_output_file} to the robot."
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
