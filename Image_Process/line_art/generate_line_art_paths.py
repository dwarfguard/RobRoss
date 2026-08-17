"""line_art (clean line-art tracing) route: turn an already-clean line-art
image (technical illustration, product diagram - not a photo) into a
painting_paths.json-format file, directly - no intermediate
painting_plan.json, like the sketch route.

    Config profile (configs/*.json)
      -> line_tracing.binarize() + close_mask() + skeletonize_mask() + extract_strokes()
         (threshold -> bridge antialiasing gaps -> skeleton -> traced centerlines, no Canny)
      -> line_tracing.prune_spurs() + simplify()
      -> path_ordering.join_overlapping_strokes()  (compatible shared junctions only)
      -> path_ordering.order_strokes()  (greedy nearest-neighbor travel order)
      -> generate_line_art_paths.py     -> output/<painting_paths_file> (+ preview SVG)

Unlike the sketch route (which emits one paint_stroke per point pair),
each traced line here becomes a single paint_path command - a continuous
polyline, same as Image_Process/mondrian/generate_painting_paths.py uses
for curved/long lines (see docs/painting-paths-format.md). This suits
line_art's typical input better: a diagram's outline or a hatching mark is
one continuous pen-down motion, not a series of independent short strokes.
"""

import argparse
import json
import math
import sys
from html import escape
from pathlib import Path

import cv2
import numpy as np

from config_loader import (
    DEFAULT_BINARY_THRESHOLD,
    DEFAULT_JOIN_OVERLAPPING_STROKES,
    DEFAULT_MAX_JOIN_TURN_DEGREES,
    DEFAULT_MIN_SPUR_LENGTH_PX,
    DEFAULT_MIN_STROKE_LENGTH_MM,
    DEFAULT_MORPH_CLOSE_KERNEL_PX,
    DEFAULT_SIMPLIFY_EPSILON_RATIO,
    DEFAULT_SIMPLIFY_MIN_EPSILON_PX,
    DEFAULT_SMOOTHING_RADIUS_MM,
    load_config,
)
from line_tracing import (
    binarize,
    close_mask,
    extract_strokes,
    prune_skeleton_spurs,
    prune_spurs,
    simplify,
    skeletonize_mask,
)
from path_ordering import join_overlapping_strokes, order_strokes, total_travel_distance
from path_validation import validate_painting_paths

DEFAULT_CONFIG_FILE = "configs/line_art_demo_a4.json"
LINE_COLOR = "black"

# Preview-only setting (does not affect painting_paths.json).
STROKE_PREVIEW_OPACITY = 0.85


# --- Command builders (same vocabulary as the mondrian route) ----------

def select_tool(color: str, label: str) -> dict:
    return {"command": "select_tool", "label": label, "color": color}


def dip_paint(color: str, label: str) -> dict:
    return {"command": "dip_paint", "label": label, "color": color}


def move_to(x: float, y: float, label: str) -> dict:
    return {"command": "move_to", "label": label, "x_mm": round(x, 2), "y_mm": round(y, 2)}


def lower_tool(label: str) -> dict:
    return {"command": "lower_tool", "label": label}


def paint_path(points, color: str, label: str) -> dict:
    """Continuous pen-down polyline through all points."""
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


def _canvas_scale(image_size, drawable_size) -> float:
    image_width, image_height = image_size
    box_width, box_height = drawable_size
    return min(box_width / image_width, box_height / image_height)


def _dedupe_consecutive(points):
    deduped = []
    for point in points:
        point = (float(point[0]), float(point[1]))
        if not deduped or point != deduped[-1]:
            deduped.append(point)
    return deduped


def smooth_canvas_path(points, closed: bool, radius_mm: float):
    """Gaussian-smooth a dense centerline on a uniform arc-length axis.

    Open endpoints remain exact so separate paths still meet at shared
    skeleton junctions. Closed paths use cyclic padding. The effective radius
    is capped at one sixth of path length so short intentional marks survive.
    """
    requested = max(0.0, float(radius_mm))
    dense = _dedupe_consecutive(points)
    if closed and len(dense) > 1 and dense[0] == dense[-1]:
        dense.pop()
    minimum_points = 3 if closed else 2
    if requested == 0 or len(dense) < minimum_points:
        effective = 0.0 if requested == 0 else min(requested, _path_length(dense) / 6.0)
        return dense, effective

    array = np.asarray(dense, dtype=np.float64)
    if closed:
        segment_lengths = np.linalg.norm(np.roll(array, -1, axis=0) - array, axis=1)
    else:
        segment_lengths = np.linalg.norm(np.diff(array, axis=0), axis=1)
    total_length = float(segment_lengths.sum())
    if total_length <= 0:
        return dense, 0.0

    effective = min(requested, total_length / 6.0)
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
        return [tuple(float(value) for value in point) for point in array.reshape(-1, 2)]
    epsilon = max(
        float(min_epsilon_mm),
        float(epsilon_ratio) * cv2.arcLength(array, closed),
    )
    approximation = cv2.approxPolyDP(array, epsilon, closed)
    return [tuple(float(value) for value in point) for point in approximation.reshape(-1, 2)]


def _map_traced_strokes(
    strokes,
    image_size,
    drawable_origin,
    drawable_size,
    epsilon_ratio,
    min_epsilon_px,
    smoothing_radius_mm,
):
    traced = []
    effective_radii = []
    points_before = 0
    points_after = 0
    scale = _canvas_scale(image_size, drawable_size)
    for points_xy, closed in strokes:
        points_before += len(points_xy)
        if smoothing_radius_mm <= 0:
            simplified = simplify(points_xy, closed, epsilon_ratio, min_epsilon_px)
            canvas_points = map_points_to_canvas(
                simplified, image_size, drawable_origin, drawable_size
            )
            effective_radius = 0.0
        else:
            dense_canvas_points = map_points_to_canvas(
                points_xy, image_size, drawable_origin, drawable_size
            )
            smoothed, effective_radius = smooth_canvas_path(
                dense_canvas_points, closed, smoothing_radius_mm
            )
            canvas_points = _simplify_canvas_points(
                smoothed, closed, epsilon_ratio, float(min_epsilon_px) * scale
            )
        if len(canvas_points) < 2:
            continue
        if closed and canvas_points[0] != canvas_points[-1]:
            canvas_points = canvas_points + [canvas_points[0]]
        points_after += len(canvas_points)
        effective_radii.append(effective_radius)
        traced.append((canvas_points, closed))
    return traced, {
        "smoothing_radius_mm": float(smoothing_radius_mm),
        "effective_smoothing_radius_mm": round(max(effective_radii, default=0.0), 4),
        "points_before_smoothing": points_before,
        "points_after_smoothing_and_simplification": points_after,
    }


def build_canvas_strokes(config: dict, include_stats: bool = False) -> tuple:
    """Run binarize + skeletonize + trace + prune + simplify + canvas
    mapping. Returns (strokes_data, image_path) where strokes_data is a
    list of (points_mm, closed) not yet distance-ordered."""
    canvas = config["canvas"]
    source_image = config["source_image"]

    image_path = Path(source_image["path"])
    threshold = source_image.get("binary_threshold", DEFAULT_BINARY_THRESHOLD)
    morph_close_kernel_px = source_image.get("morph_close_kernel_px", DEFAULT_MORPH_CLOSE_KERNEL_PX)
    min_spur_length_px = source_image.get("min_spur_length_px", DEFAULT_MIN_SPUR_LENGTH_PX)
    min_stroke_length_mm = source_image.get("min_stroke_length_mm", DEFAULT_MIN_STROKE_LENGTH_MM)
    epsilon_ratio = source_image.get("simplify_epsilon_ratio", DEFAULT_SIMPLIFY_EPSILON_RATIO)
    min_epsilon_px = source_image.get(
        "simplify_min_epsilon_px", DEFAULT_SIMPLIFY_MIN_EPSILON_PX
    )

    mask, image_size = binarize(image_path, threshold)
    mask = close_mask(mask, morph_close_kernel_px)
    skeleton = skeletonize_mask(mask)
    skeleton = prune_skeleton_spurs(skeleton, min_spur_length_px)
    raw_strokes = extract_strokes(skeleton)
    pruned_strokes = prune_spurs(raw_strokes, min_spur_length_px)

    margin = canvas.get("margin_mm", 0.0)
    drawable_origin = (margin, margin)
    drawable_size = (canvas["width_mm"] - 2 * margin, canvas["height_mm"] - 2 * margin)

    path_generation = config["path_generation"]
    smoothing_radius_mm = path_generation.get(
        "smoothing_radius_mm", DEFAULT_SMOOTHING_RADIUS_MM
    )
    traced_strokes, smoothing_stats = _map_traced_strokes(
        pruned_strokes,
        image_size,
        drawable_origin,
        drawable_size,
        epsilon_ratio,
        min_epsilon_px,
        smoothing_radius_mm,
    )
    should_join = path_generation.get(
        "join_overlapping_strokes", DEFAULT_JOIN_OVERLAPPING_STROKES
    )
    max_join_turn = path_generation.get(
        "max_join_turn_degrees", DEFAULT_MAX_JOIN_TURN_DEGREES
    )
    if should_join:
        joined_strokes, num_joins = join_overlapping_strokes(
            traced_strokes, max_turn_degrees=max_join_turn
        )
    else:
        joined_strokes, num_joins = traced_strokes, 0

    strokes_data = []
    for points, closed in joined_strokes:
        stroke_length_mm = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        if stroke_length_mm >= min_stroke_length_mm:
            strokes_data.append((points, closed))

    stats = {
        "num_traced_lines_before_optimization": len(traced_strokes),
        "num_overlap_joins": num_joins,
        "num_short_lines_filtered": len(joined_strokes) - len(strokes_data),
        "num_traced_lines_after_optimization": len(strokes_data),
        **smoothing_stats,
    }
    if include_stats:
        return strokes_data, image_path, stats
    return strokes_data, image_path


# --- Ordered strokes -> path commands -----------------------------------

def stroke_to_commands(points, stroke_index: int) -> list:
    """One traced line -> move_to + lower_tool + one paint_path (through
    every point) + lift_tool. Consecutive duplicate points (possible after
    simplification of very short/noisy loops) are collapsed first so the
    paint_path has no zero-length segments."""
    label = f"line_art_line_{stroke_index}"
    deduped = [points[0]]
    for point in points[1:]:
        if point != deduped[-1]:
            deduped.append(point)
    if len(deduped) < 2:
        return []

    return [
        move_to(deduped[0][0], deduped[0][1], label),
        lower_tool(label),
        paint_path(deduped, LINE_COLOR, label),
        lift_tool(label),
    ]


def build_commands(ordered_strokes: list) -> list:
    commands = [select_tool(LINE_COLOR, "line_art"), dip_paint(LINE_COLOR, "line_art")]
    for index, points in enumerate(ordered_strokes, start=1):
        commands.extend(stroke_to_commands(points, index))
    return commands


def build_painting_paths(
    commands: list,
    config: dict,
    config_path: Path,
    image_path: Path,
    pen_up_travel_mm: dict,
    optimization_stats: dict = None,
) -> dict:
    path_commands = [cmd for cmd in commands if cmd["command"] == "paint_path"]
    total_distance = sum(
        math.dist(a, b)
        for cmd in path_commands
        for a, b in zip(cmd["points_mm"], cmd["points_mm"][1:])
    )

    def count_commands(command_name: str) -> int:
        return sum(1 for cmd in commands if cmd["command"] == command_name)

    canvas = config["canvas"]
    path_generation = config["path_generation"]

    debug = {
        "num_commands": len(commands),
        "num_paint_path_commands": len(path_commands),
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
        "style": config.get("style", "line_art"),
        "version": "0.1",
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
            "smoothing_radius_mm": path_generation.get(
                "smoothing_radius_mm", DEFAULT_SMOOTHING_RADIUS_MM
            ),
            "join_overlapping_strokes": path_generation.get(
                "join_overlapping_strokes", DEFAULT_JOIN_OVERLAPPING_STROKES
            ),
            "max_join_turn_degrees": path_generation.get(
                "max_join_turn_degrees", DEFAULT_MAX_JOIN_TURN_DEGREES
            ),
        },
        "commands": commands,
        "debug": debug,
    }


# --- SVG preview ---------------------------------------------------------

def render_svg(painting_paths: dict) -> str:
    """Draw every paint_path command as a polyline, for a quick visual check."""
    canvas = painting_paths["canvas"]
    width_mm = canvas["width_mm"]
    height_mm = canvas["height_mm"]
    tool_width = painting_paths["path_settings"]["tool_width_mm"]

    elements = [f'<rect x="0" y="0" width="{width_mm}" height="{height_mm}" fill="white" />']

    for cmd in painting_paths["commands"]:
        if cmd["command"] != "paint_path":
            continue
        points_attr = " ".join(f"{x},{y}" for x, y in cmd["points_mm"])
        elements.append(
            f'<polyline points="{points_attr}" fill="none" '
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
        description="Convert a clean line-art image into a painting_paths.json-format file via threshold + skeletonize tracing."
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_FILE,
        help=f"Path to a line_art pipeline config JSON file (default: {DEFAULT_CONFIG_FILE}).",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_config(config_path)

    output = config["output"]
    output_dir = Path(output["directory"])
    paths_output_file = output_dir / output["painting_paths_file"]
    svg_output_file = output_dir / output["preview_svg_file"]
    output_dir.mkdir(exist_ok=True)

    strokes_data, image_path, optimization_stats = build_canvas_strokes(
        config, include_stats=True
    )
    if not strokes_data:
        print(f"No traced lines found in {image_path} - nothing to write.")
        sys.exit(1)

    path_generation = config["path_generation"]
    canvas = config["canvas"]
    margin = canvas.get("margin_mm", 0.0)
    home_position = tuple(path_generation.get("home_position_mm", (margin, margin)))

    baseline_travel = total_travel_distance([points for points, _ in strokes_data], home_position)
    ordered_strokes = order_strokes(strokes_data, home_position)
    optimized_travel = total_travel_distance(ordered_strokes, home_position)

    commands = build_commands(ordered_strokes)
    painting_paths = build_painting_paths(
        commands, config, config_path, image_path,
        {"baseline": baseline_travel, "optimized": optimized_travel},
        optimization_stats,
    )

    validation = validate_painting_paths(painting_paths)
    painting_paths["validation"] = validation

    with open(paths_output_file, "w", encoding="utf-8") as file:
        json.dump(painting_paths, file, indent=2)
    print(f"Generated {paths_output_file} ({len(ordered_strokes)} traced lines)")
    print(
        "Stroke optimization: "
        f"{optimization_stats['num_traced_lines_before_optimization']} traced -> "
        f"{optimization_stats['num_traced_lines_after_optimization']} pen-down paths "
        f"({optimization_stats['num_overlap_joins']} overlap joins, "
        f"{optimization_stats['num_short_lines_filtered']} short lines filtered)"
    )

    svg_content = render_svg(painting_paths)
    with open(svg_output_file, "w", encoding="utf-8") as file:
        file.write(svg_content)
    print(f"Generated {svg_output_file}")

    print(
        f"Pen-up travel: {baseline_travel:.1f}mm baseline -> {optimized_travel:.1f}mm ordered "
        f"({(1 - optimized_travel / baseline_travel) * 100:.1f}% reduction)"
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
