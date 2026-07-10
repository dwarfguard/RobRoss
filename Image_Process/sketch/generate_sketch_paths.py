"""Sketch (outline-tracing) route: turn an arbitrary source image into a
painting_paths.json-format file, directly - no intermediate painting_plan.json,
unlike the mondrian route, since there's no vector design to describe first.

    Config profile (configs/*.json)
      -> canny_edges.extract_strokes()  (Canny edges -> skeleton -> traced strokes)
      -> path_ordering.order_strokes()  (greedy nearest-neighbor travel order)
      -> generate_sketch_paths.py       -> output/<painting_paths_file> (+ preview SVG)

The output uses the exact same command vocabulary as
Image_Process/mondrian/generate_painting_paths.py
(select_tool/dip_paint/move_to/lower_tool/paint_stroke/lift_tool), so it is
consumable by the same ros2/robross_painter executor: a curved traced line
becomes one lower_tool/lift_tool bracket containing a straight paint_stroke
per consecutive point pair (see docs/painting-paths-format.md: "A
paint_stroke is one straight line; curves are multiple strokes.").
"""

import argparse
import json
import math
import sys
from html import escape
from pathlib import Path

from canny_edges import extract_strokes, simplify
from config_loader import DEFAULT_CANNY_THRESHOLD1, DEFAULT_CANNY_THRESHOLD2, DEFAULT_SIMPLIFY_EPSILON_RATIO, load_config
from path_ordering import order_strokes, total_travel_distance
from path_validation import validate_painting_paths

DEFAULT_CONFIG_FILE = "configs/sketch_demo_a4.json"
LINE_COLOR = "black"

# Preview-only setting (does not affect painting_paths.json).
STROKE_PREVIEW_OPACITY = 0.75


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


def build_canvas_strokes(config: dict) -> tuple:
    """Run edge detection + simplification + canvas mapping. Returns
    (strokes_data, image_path) where strokes_data is a list of
    (points_mm, closed) not yet distance-ordered."""
    canvas = config["canvas"]
    source_image = config["source_image"]

    image_path = Path(source_image["path"])
    threshold1 = source_image.get("canny_threshold1", DEFAULT_CANNY_THRESHOLD1)
    threshold2 = source_image.get("canny_threshold2", DEFAULT_CANNY_THRESHOLD2)
    epsilon_ratio = source_image.get("simplify_epsilon_ratio", DEFAULT_SIMPLIFY_EPSILON_RATIO)

    raw_strokes, image_size = extract_strokes(image_path, threshold1, threshold2)

    margin = canvas.get("margin_mm", 0.0)
    drawable_origin = (margin, margin)
    drawable_size = (canvas["width_mm"] - 2 * margin, canvas["height_mm"] - 2 * margin)

    strokes_data = []
    for points_xy, closed in raw_strokes:
        simplified = simplify(points_xy, closed, epsilon_ratio)
        if len(simplified) < 2:
            continue
        canvas_points = map_points_to_canvas(simplified, image_size, drawable_origin, drawable_size)
        strokes_data.append((canvas_points, closed))

    return strokes_data, image_path


# --- Ordered strokes -> path commands -----------------------------------

def stroke_to_commands(points, stroke_index: int) -> list:
    """One traced line -> move_to + lower_tool + one paint_stroke per
    consecutive point pair + lift_tool. Consecutive duplicate points
    (possible after simplification of very short/noisy loops) are
    collapsed so no zero-length paint_stroke is emitted."""
    label = f"sketch_line_{stroke_index}"
    deduped = [points[0]]
    for point in points[1:]:
        if point != deduped[-1]:
            deduped.append(point)
    if len(deduped) < 2:
        return []

    commands = [move_to(deduped[0][0], deduped[0][1], label), lower_tool(label)]
    for from_point, to_point in zip(deduped, deduped[1:]):
        commands.append(paint_stroke(from_point, to_point, LINE_COLOR, label))
    commands.append(lift_tool(label))
    return commands


def build_commands(ordered_strokes: list) -> list:
    commands = [select_tool(LINE_COLOR, "sketch"), dip_paint(LINE_COLOR, "sketch")]
    for index, points in enumerate(ordered_strokes, start=1):
        commands.extend(stroke_to_commands(points, index))
    return commands


def build_painting_paths(commands: list, config: dict, config_path: Path, image_path: Path, pen_up_travel_mm: dict) -> dict:
    stroke_commands = [cmd for cmd in commands if cmd["command"] == "paint_stroke"]
    total_distance = sum(math.dist(cmd["from_mm"], cmd["to_mm"]) for cmd in stroke_commands)

    def count_commands(command_name: str) -> int:
        return sum(1 for cmd in commands if cmd["command"] == command_name)

    canvas = config["canvas"]
    path_generation = config["path_generation"]

    return {
        "project": config.get("project"),
        "style": config.get("style", "sketch"),
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
        },
        "commands": commands,
        "debug": {
            "num_commands": len(commands),
            "num_paint_stroke_commands": len(stroke_commands),
            "estimated_total_paint_distance_mm": round(total_distance, 2),
            "num_traced_lines": count_commands("lower_tool"),
            "num_select_tool_commands": count_commands("select_tool"),
            "num_lift_tool_commands": count_commands("lift_tool"),
            "num_lower_tool_commands": count_commands("lower_tool"),
            "num_dip_paint_commands": count_commands("dip_paint"),
            "baseline_pen_up_travel_mm": round(pen_up_travel_mm["baseline"], 2),
            "optimized_pen_up_travel_mm": round(pen_up_travel_mm["optimized"], 2),
        },
    }


# --- SVG preview ---------------------------------------------------------

def render_svg(painting_paths: dict) -> str:
    """Draw every paint_stroke command as a line, for a quick visual check."""
    canvas = painting_paths["canvas"]
    width_mm = canvas["width_mm"]
    height_mm = canvas["height_mm"]
    tool_width = painting_paths["path_settings"]["tool_width_mm"]

    elements = [f'<rect x="0" y="0" width="{width_mm}" height="{height_mm}" fill="white" />']

    for cmd in painting_paths["commands"]:
        if cmd["command"] != "paint_stroke":
            continue
        x1, y1 = cmd["from_mm"]
        x2, y2 = cmd["to_mm"]
        elements.append(
            f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
            f'stroke="{escape(cmd["color"])}" stroke-width="{tool_width}" '
            f'stroke-linecap="round" stroke-opacity="{STROKE_PREVIEW_OPACITY}" />'
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
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_config(config_path)

    output = config["output"]
    output_dir = Path(output["directory"])
    paths_output_file = output_dir / output["painting_paths_file"]
    svg_output_file = output_dir / output["preview_svg_file"]
    output_dir.mkdir(exist_ok=True)

    strokes_data, image_path = build_canvas_strokes(config)
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
    )

    validation = validate_painting_paths(painting_paths)
    painting_paths["validation"] = validation

    with open(paths_output_file, "w", encoding="utf-8") as file:
        json.dump(painting_paths, file, indent=2)
    print(f"Generated {paths_output_file} ({len(ordered_strokes)} traced lines)")

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
