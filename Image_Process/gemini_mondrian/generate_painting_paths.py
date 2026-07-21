"""gemini_mondrian route: Gemini image-to-image restyle -> standalone
vectorization -> painting_paths.json.

    Config profile (configs/gemini_mondrian_*.json)
      -> gemini_client.generate_styled_image()   photo -> Gemini-generated Mondrian-style image
      -> color_quantize.quantize_to_palette()    Gemini image -> 5-color label image
      -> segmentation.segment_image()            per-color connected regions, speckle filtered
      -> region_fill.region_to_pixel_strokes()   erode + scanline fill, per region
      -> border_tracing.trace_region_contours()  black grid lines between color blocks
      -> path_ordering.order_strokes()           greedy nearest-neighbor travel order, per color group
      -> generate_painting_paths.py              -> output/<painting_paths_file> (+ preview SVG + quantized preview PNG)

The vectorization stages (quantize/segment/fill/border-trace/order) used to
run by subprocess-calling Image_Process/image_to_mondrian/'s pipeline
unmodified. That was replaced with this route's own, deliberately simpler
color_quantize.py/segmentation.py after testing showed image_to_mondrian's
real-photo-tuned parameters (bilateral filtering, large morphological
closing, MediaPipe face protection) badly distort Gemini's already-clean,
already-flat-colored output - see those two modules' docstrings for the
full explanation. region_fill.py/border_tracing.py/path_ordering.py/
path_validation.py are copied byte-for-byte from image_to_mondrian, since
they're pure computational geometry with no real-photo assumptions baked in.

Output uses the exact same command vocabulary as the other three routes
(select_tool/dip_paint/move_to/lower_tool/paint_stroke/lift_tool), so it is
consumable by the same ros2/robross_painter executor.
"""

import argparse
import json
import math
import sys
from html import escape
from pathlib import Path

import cv2
import numpy as np

import border_tracing
import color_quantize
import region_fill
import segmentation
from config_loader import load_config
from gemini_client import generate_styled_image
from path_ordering import order_strokes
from path_validation import validate_painting_paths

DEFAULT_CONFIG_FILE = "configs/gemini_mondrian_demo_a4.json"

# Preview-only setting (does not affect painting_paths.json).
STROKE_PREVIEW_OPACITY = 0.85
TRAVEL_LINE_COLOR = "#999999"
TRAVEL_LINE_WIDTH_MM = 1.0


# --- Command builders (same vocabulary as the other routes) -------------

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


# --- Pixel space <-> canvas millimeters ---------------------------------

def image_to_canvas_transform(image_size: tuple, canvas: dict) -> tuple:
    """Aspect-fit-and-center an image inside the canvas' margin box.
    Returns (scale_mm_per_px, offset_mm)."""
    img_w, img_h = image_size
    margin = canvas.get("margin_mm", 0.0)
    box_w = canvas["width_mm"] - 2 * margin
    box_h = canvas["height_mm"] - 2 * margin

    scale = min(box_w / img_w, box_h / img_h)
    offset_x = margin + (box_w - img_w * scale) / 2
    offset_y = margin + (box_h - img_h * scale) / 2
    return scale, (offset_x, offset_y)


def px_to_mm(point_px, scale_mm_per_px: float, offset_mm: tuple) -> tuple:
    x_px, y_px = point_px
    return (offset_mm[0] + x_px * scale_mm_per_px, offset_mm[1] + y_px * scale_mm_per_px)


# --- Gemini image -> quantized/segmented regions -------------------------

def build_regions(config: dict, image_path: Path) -> tuple:
    """Load + quantize + segment the Gemini-generated image (not the
    original source photo - image_path is the styled output).

    Returns (kept_regions, dropped_small_count, cleaned_label_image,
    image_size, scale_mm_per_px, offset_mm). cleaned_label_image has had
    single-pixel speckle removed (morphological open) but is not filtered
    by area - it's also the right input for border tracing, so noise
    specks that are opened away don't leave jagged black scribbles even
    though they're too small to fill with color.
    """
    canvas = config["canvas"]
    source_image = config["source_image"]
    palette_colors = config["palette"]["colors"]
    color_space = config["palette"].get("color_space", "lab")
    segmentation_cfg = config["segmentation"]

    image = color_quantize.load_image(image_path)
    image = color_quantize.preprocess(
        image,
        downscale_max_dimension_px=source_image.get("downscale_max_dimension_px"),
        blur_kernel_size=source_image.get("blur_kernel_size", 0),
    )

    label_image = color_quantize.quantize_to_palette(image, palette_colors, color_space)

    height, width = label_image.shape
    scale_mm_per_px, offset_mm = image_to_canvas_transform((width, height), canvas)

    min_area_px = segmentation_cfg["min_region_area_mm2"] / (scale_mm_per_px ** 2)
    kept_regions, dropped_count, cleaned_label_image = segmentation.segment_image(
        label_image,
        palette_colors,
        segmentation_cfg.get("morph_open_kernel_px", 0),
        min_area_px,
    )

    return kept_regions, dropped_count, cleaned_label_image, (width, height), scale_mm_per_px, offset_mm


def build_region_strokes(regions: list, path_settings: dict, scale_mm_per_px: float, offset_mm: tuple, skip_white: bool) -> dict:
    """Fill every kept region, bucketed by color name. Returns
    {color_name: [(from_mm, to_mm), ...]}."""
    tool_width_px = path_settings["tool_width_mm"] / scale_mm_per_px
    mask_erosion_px = round(path_settings["mask_erosion_mm"] / scale_mm_per_px)
    stroke_overlap_ratio = path_settings["stroke_overlap_ratio"]

    strokes_by_color = {}
    for region in regions:
        if skip_white and region["color_name"] == "white":
            continue

        pixel_strokes = region_fill.region_to_pixel_strokes(
            region["mask"], tool_width_px, stroke_overlap_ratio, mask_erosion_px
        )
        if not pixel_strokes:
            continue

        mm_strokes = [
            (px_to_mm(p0, scale_mm_per_px, offset_mm), px_to_mm(p1, scale_mm_per_px, offset_mm))
            for p0, p1 in pixel_strokes
        ]
        strokes_by_color.setdefault(region["color_name"], []).extend(mm_strokes)

    return strokes_by_color


def build_border_strokes(regions: list, border_cfg: dict, scale_mm_per_px: float, offset_mm: tuple) -> list:
    """Trace the black grid lines between color blocks: one (or a few)
    closed contour(s) per kept region. Returns a list of (points_mm,
    closed) - same shape path_ordering.order_strokes() expects. Traces
    every kept region including ones skip_white excludes from fill, so a
    skipped white shape still gets an outline instead of disappearing
    entirely."""
    if not border_cfg.get("draw_borders", True):
        return []

    epsilon_ratio = border_cfg["simplify_epsilon_ratio"]
    border_strokes = []
    for region in regions:
        for points_xy, closed in border_tracing.trace_region_contours(region["mask"]):
            simplified = border_tracing.simplify(points_xy, closed, epsilon_ratio)
            if len(simplified) < 3:
                continue
            # order_strokes()/the polyline command builder don't carry the
            # closed flag through to drawing, so bake the closing edge into
            # the point list itself here.
            if closed and simplified[0] != simplified[-1]:
                simplified = simplified + [simplified[0]]
            points_mm = [px_to_mm(p, scale_mm_per_px, offset_mm) for p in simplified]
            border_strokes.append((points_mm, closed))

    return border_strokes


# --- Ordered strokes -> path commands -----------------------------------

def _fill_strokes_to_commands(ordered_points: list, color_hex: str, label_prefix: str) -> list:
    commands = []
    for index, (p0, p1) in enumerate(ordered_points, start=1):
        label = f"{label_prefix}_{index}"
        commands.append(move_to(p0[0], p0[1], label))
        commands.append(lower_tool(label))
        commands.append(paint_stroke(p0, p1, color_hex, label))
        commands.append(lift_tool(label))
    return commands


def _polyline_strokes_to_commands(ordered_polylines: list, color_hex: str, label_prefix: str) -> list:
    commands = []
    for index, points in enumerate(ordered_polylines, start=1):
        label = f"{label_prefix}_{index}"
        deduped = [points[0]]
        for point in points[1:]:
            if point != deduped[-1]:
                deduped.append(point)
        if len(deduped) < 2:
            continue
        commands.append(move_to(deduped[0][0], deduped[0][1], label))
        commands.append(lower_tool(label))
        for from_point, to_point in zip(deduped, deduped[1:]):
            commands.append(paint_stroke(from_point, to_point, color_hex, label))
        commands.append(lift_tool(label))
    return commands


def order_and_build_commands(
    strokes_by_color: dict,
    border_strokes: list,
    palette_colors: list,
    home_position_mm: tuple,
) -> list:
    """Group by color (fewest physical pen changes), order each group with
    greedy nearest-neighbor to cut travel, then draw the black grid lines
    last."""
    commands = []
    current_position = home_position_mm

    for color in palette_colors:
        color_name = color["name"]
        mm_strokes = strokes_by_color.get(color_name)
        if not mm_strokes:
            continue

        strokes_data = [([p0, p1], False) for p0, p1 in mm_strokes]
        ordered = order_strokes(strokes_data, home_position=current_position)

        commands.append(select_tool(color["hex"], color_name))
        commands.append(dip_paint(color["hex"], color_name))
        commands.extend(_fill_strokes_to_commands(ordered, color["hex"], f"{color_name}_fill"))
        current_position = ordered[-1][-1]

    if border_strokes:
        border_hex = next((c["hex"] for c in palette_colors if c["name"] == "black"), "#000000")
        strokes_data = [(points, closed) for points, closed in border_strokes]
        ordered = order_strokes(strokes_data, home_position=current_position)

        commands.append(select_tool(border_hex, "border"))
        commands.append(dip_paint(border_hex, "border"))
        commands.extend(_polyline_strokes_to_commands(ordered, border_hex, "border"))

    return commands


# --- Assembling painting_paths.json -------------------------------------

def build_painting_paths(
    commands: list,
    config: dict,
    config_path: Path,
    source_image_path: Path,
    gemini_image_path: Path,
    region_debug: dict,
) -> dict:
    stroke_commands = [cmd for cmd in commands if cmd["command"] == "paint_stroke"]
    total_distance = sum(math.dist(cmd["from_mm"], cmd["to_mm"]) for cmd in stroke_commands)

    def count_commands(command_name: str) -> int:
        return sum(1 for cmd in commands if cmd["command"] == command_name)

    canvas = config["canvas"]
    path_generation = config["path_generation"]
    border_generation = config["border_generation"]

    return {
        "project": config.get("project"),
        "style": config.get("style", "gemini_mondrian"),
        "version": "0.1",
        "config": {
            "profile_name": config.get("profile_name"),
            "source_file": str(config_path),
        },
        "source_file": str(source_image_path),
        "gemini_image_file": str(gemini_image_path),
        "gemini_model": config["gemini"]["model"],
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
            "stroke_overlap_ratio": path_generation["stroke_overlap_ratio"],
            "mask_erosion_mm": path_generation["mask_erosion_mm"],
        },
        "border_settings": {
            "draw_borders": border_generation.get("draw_borders", True),
            "simplify_epsilon_ratio": border_generation["simplify_epsilon_ratio"],
        },
        "commands": commands,
        "debug": {
            "num_commands": len(commands),
            "num_paint_stroke_commands": len(stroke_commands),
            "estimated_total_paint_distance_mm": round(total_distance, 2),
            "num_select_tool_commands": count_commands("select_tool"),
            "num_lift_tool_commands": count_commands("lift_tool"),
            "num_lower_tool_commands": count_commands("lower_tool"),
            "num_dip_paint_commands": count_commands("dip_paint"),
            **region_debug,
        },
    }


# --- SVG / debug preview -------------------------------------------------

def render_svg(painting_paths: dict) -> str:
    """Draw every paint_stroke command as a colored line, for a quick visual check."""
    canvas = painting_paths["canvas"]
    width_mm = canvas["width_mm"]
    height_mm = canvas["height_mm"]
    width_in = canvas["width_in"]
    height_in = canvas["height_in"]

    elements = [f'<rect x="0" y="0" width="{width_mm}" height="{height_mm}" fill="white" />']
    last_point = None

    for cmd in painting_paths["commands"]:
        if cmd["command"] == "move_to":
            point = (cmd["x_mm"], cmd["y_mm"])
            if last_point is not None:
                elements.append(
                    f'<line x1="{last_point[0]}" y1="{last_point[1]}" '
                    f'x2="{point[0]}" y2="{point[1]}" '
                    f'stroke="{TRAVEL_LINE_COLOR}" stroke-width="{TRAVEL_LINE_WIDTH_MM}" '
                    f'stroke-dasharray="4,3" />'
                )
            last_point = point

        elif cmd["command"] == "paint_stroke":
            x1, y1 = cmd["from_mm"]
            x2, y2 = cmd["to_mm"]
            elements.append(
                f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
                f'stroke="{escape(cmd["color"])}" stroke-width="{painting_paths["path_settings"]["tool_width_mm"]}" '
                f'stroke-linecap="round" stroke-opacity="{STROKE_PREVIEW_OPACITY}" />'
            )
            last_point = (x2, y2)

    svg_body = "\n  ".join(elements)

    return f'''<svg
  xmlns="http://www.w3.org/2000/svg"
  width="{width_in}in"
  height="{height_in}in"
  viewBox="0 0 {width_mm} {height_mm}"
>
  {svg_body}
</svg>
'''


def render_quantized_preview(label_image: np.ndarray, palette_colors: list) -> np.ndarray:
    """A BGR debug image: every pixel painted its quantized palette color,
    useful for tuning min_region_area_mm2/palette without a full run.
    Pixels with no color (index -1, opened away as speckle) stay white,
    so they read as "unclaimed" rather than being confused with a real
    black region."""
    height, width = label_image.shape
    preview = np.full((height, width, 3), 255, dtype=np.uint8)
    for color_index, color in enumerate(palette_colors):
        bgr = color_quantize.hex_to_bgr(color["hex"])
        preview[label_image == color_index] = bgr
    return preview


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Photo -> Gemini Mondrian-style transfer -> painting_paths.json."
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_FILE,
        help=f"Path to a gemini_mondrian pipeline config JSON file (default: {DEFAULT_CONFIG_FILE}).",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_config(config_path)

    output = config["output"]
    output_dir = Path(output["directory"])
    output_dir.mkdir(parents=True, exist_ok=True)
    paths_output_file = output_dir / output["painting_paths_file"]
    svg_output_file = output_dir / output["preview_svg_file"]
    quantized_preview_file = output.get("quantized_preview_png_file")

    source_image_path = Path(config["source_image"]["path"])
    gemini_image_path = output_dir / output["gemini_output_image_file"]

    image_bytes = generate_styled_image(
        source_image_path, config["gemini"]["prompt"], config["gemini"]["model"]
    )
    gemini_image_path.write_bytes(image_bytes)
    print(f"Generated {gemini_image_path}")

    regions, dropped_count, label_image, image_size, scale_mm_per_px, offset_mm = build_regions(
        config, gemini_image_path
    )

    path_generation = config["path_generation"]
    canvas = config["canvas"]
    margin = canvas.get("margin_mm", 0.0)
    home_position = tuple(path_generation.get("home_position_mm", (margin, margin)))
    skip_white = config["segmentation"].get("skip_white_regions", True)

    strokes_by_color = build_region_strokes(regions, path_generation, scale_mm_per_px, offset_mm, skip_white)
    border_strokes = build_border_strokes(regions, config["border_generation"], scale_mm_per_px, offset_mm)

    palette_colors = config["palette"]["colors"]
    commands = order_and_build_commands(strokes_by_color, border_strokes, palette_colors, home_position)

    if not any(cmd["command"] == "paint_stroke" for cmd in commands):
        print(f"No paintable regions found in {gemini_image_path} - nothing to write.")
        sys.exit(1)

    num_regions_by_color = {}
    for region in regions:
        num_regions_by_color[region["color_name"]] = num_regions_by_color.get(region["color_name"], 0) + 1

    region_debug = {
        "num_regions_total": len(regions) + dropped_count,
        "num_regions_after_filter": len(regions),
        "num_regions_dropped_small": dropped_count,
        "num_regions_by_color": num_regions_by_color,
        "num_border_strokes": len(border_strokes),
    }

    painting_paths = build_painting_paths(
        commands, config, config_path, source_image_path, gemini_image_path, region_debug
    )

    validation = validate_painting_paths(painting_paths)
    painting_paths["validation"] = validation

    with open(paths_output_file, "w", encoding="utf-8") as file:
        json.dump(painting_paths, file, indent=2)
    print(f"Generated {paths_output_file}")

    svg_content = render_svg(painting_paths)
    with open(svg_output_file, "w", encoding="utf-8") as file:
        file.write(svg_content)
    print(f"Generated {svg_output_file}")

    if quantized_preview_file:
        preview_path = output_dir / quantized_preview_file
        cv2.imwrite(str(preview_path), render_quantized_preview(label_image, palette_colors))
        print(f"Generated {preview_path}")

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
