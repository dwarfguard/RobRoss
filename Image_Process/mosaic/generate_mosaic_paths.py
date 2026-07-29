"""generate_mosaic_paths.py — grid-based 4-color mosaic from a source photo.

Divides the image into a regular grid of cells. Each cell is filled with the
dominant palette colour (red, yellow, blue, white — the majority pixel count
wins; ties go to the first colour in palette order). No black grid lines.

Config profile (configs/*.json):
  -> color_quantize.quantize_to_palette()   (photo -> 4-color label image)
  -> mosaic.grid_majority_vote()            (grid cell -> dominant colour)
  -> mosaic.fill_cell_strokes()             (one rectangle fill per cell)
  -> path_ordering.order_strokes()          (greedy nearest-neighbor travel)
  -> generate_mosaic_paths.py               -> output/<painting_paths_file>

Output uses the same command vocabulary as the other routes
(select_tool/dip_paint/move_to/lower_tool/paint_stroke/lift_tool).
"""

import argparse
import json
import math
import sys
from html import escape
from pathlib import Path

import cv2
import numpy as np

import color_quantize
from config_loader import load_config, ConfigError
from path_ordering import order_strokes
from path_validation import validate_painting_paths

DEFAULT_CONFIG_FILE = "configs/mosaic_demo_a4.json"

# Preview-only settings (do not affect painting_paths.json).
STROKE_PREVIEW_OPACITY = 0.85
TRAVEL_LINE_COLOR = "#999999"
TRAVEL_LINE_WIDTH_MM = 1.0


# --- Command builders (same vocabulary as the other routes) ------------------

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


# --- Pixel space <-> canvas millimeters ------------------------------------

def image_to_canvas_transform(image_size: tuple, canvas: dict) -> tuple:
    """Aspect-fit-and-center an image inside the canvas' margin box."""
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


# --- Grid / majority vote --------------------------------------------------

def compute_cell_size_px(cell_size_mm: float, scale_mm_per_px: float) -> float:
    """Convert the config cell size (mm) to pixels at the processed resolution.

    Returns the floating-point pixel side length — grid_majority_vote will snap
    to a whole number of rows/cols from it so cells fill exactly."""
    return max(1.0, cell_size_mm / scale_mm_per_px)


def grid_majority_vote(
    label_image: np.ndarray,
    cell_size_px: float,
    palette_colors: list,
    active_color_indices: set,
) -> list:
    """Split the image into grid cells, assign each cell its dominant colour.

    Parameters
    ----------
    label_image : (H, W) int array of palette indices.
    cell_size_px : side length of a square cell in pixels.
    palette_colors : list of {"name", "hex"} dicts.
    active_color_indices : set of palette indices that count (e.g. {red, yellow, blue}).

    Returns
    -------
    list of dicts:
        {"row": int, "col": int,
         "x0_px": int, "y0_px": int, "x1_px": int, "y1_px": int,
         "color_index": int, "color_name": str, "color_hex": str}
    Cells whose dominant colour is white (or not in active set) are included
    but can be skipped at fill time via `skip_white_cells`.
    """
    height, width = label_image.shape

    # Snap to an integer number of rows/cols so the grid fills the image exactly
    # — no partial/half cells at the right or bottom edge.
    cols = max(1, round(width / cell_size_px))
    rows = max(1, round(height / cell_size_px))
    # Exact cell width/height in px (can be fractional; fences below snap to int
    # so the last fence lands exactly at width/height).
    exact_cell_w = width / cols
    exact_cell_h = height / rows

    cells = []
    for ri in range(rows):
        row_start = round(ri * exact_cell_h)
        row_end = round((ri + 1) * exact_cell_h) if ri < rows - 1 else height
        for ci in range(cols):
            col_start = round(ci * exact_cell_w)
            col_end = round((ci + 1) * exact_cell_w) if ci < cols - 1 else width

            block = label_image[row_start:row_end, col_start:col_end]

            # Count pixels for every palette index, then pick the majority.
            # White is included in the vote so it competes fairly with red/yellow/blue.
            best_index = 0
            best_count = -1
            for aci in active_color_indices:
                cnt = int((block == aci).sum())
                if cnt > best_count:
                    best_count = cnt
                    best_index = aci

            color = palette_colors[best_index]
            cells.append({
                "row": ri,
                "col": ci,
                "x0_px": col_start,
                "y0_px": row_start,
                "x1_px": col_end,
                "y1_px": row_end,
                "color_index": best_index,
                "color_name": color["name"],
                "color_hex": color["hex"],
            })

    return cells


def fill_cell_strokes(
    cells: list,
    scale_mm_per_px: float,
    offset_mm: tuple,
    skip_white: bool,
) -> dict:
    """Convert each cell into a rectangle-fill stroke list, by colour.

    Each cell becomes four paint_strokes tracing its boundary rectangle with
    parallel horizontal scanlines (same as region_fill but for a perfect
    rectangle). Actually, for simplicity and to produce compact output, each
    cell produces horizontal scanline strokes like mondrian's stripe fill.

    Returns {color_name: [(from_mm, to_mm), ...]}.
    """
    strokes_by_color = {}

    for cell in cells:
        if skip_white and cell["color_name"] == "white":
            continue

        color_name = cell["color_name"]
        x0, y0 = px_to_mm((cell["x0_px"], cell["y0_px"]), scale_mm_per_px, offset_mm)
        x1, y1 = px_to_mm((cell["x1_px"], cell["y1_px"]), scale_mm_per_px, offset_mm)

        # For each cell, fill the entire rectangle with horizontal scanlines.
        # Each scanline is one paint_stroke from left edge to right edge.
        # Stroke spacing = tool_width_mm * (1 - stroke_overlap_ratio),
        # but we need those values — they're applied per-call in
        # build_cell_strokes below.

        # Store as pixel-coordinate rectangle; the caller handles scanline spacing.
        strokes_by_color.setdefault(color_name, []).append((x0, y0, x1, y1))

    return strokes_by_color


def build_cell_strokes(
    cells: list,
    path_settings: dict,
    scale_mm_per_px: float,
    offset_mm: tuple,
    skip_white: bool,
) -> dict:
    """Fill every cell with horizontal scanline strokes, bucketed by colour.

    Returns {color_name: [(from_mm, to_mm), ...]}.
    """
    tool_width_mm = path_settings["tool_width_mm"]
    stroke_overlap_ratio = path_settings["stroke_overlap_ratio"]
    stripe_step_mm = tool_width_mm * (1 - stroke_overlap_ratio)

    strokes_by_color = {}

    for cell in cells:
        if skip_white and cell["color_name"] == "white":
            continue

        color_name = cell["color_name"]
        x0, y0 = px_to_mm((cell["x0_px"], cell["y0_px"]), scale_mm_per_px, offset_mm)
        x1, y1 = px_to_mm((cell["x1_px"], cell["y1_px"]), scale_mm_per_px, offset_mm)

        # Horizontal scanline fill from top to bottom.
        y = y0 + tool_width_mm / 2
        while y < y1 - 1e-9:
            strokes_by_color.setdefault(color_name, []).append(
                ((x0, y), (x1, y))
            )
            y += stripe_step_mm

        # Always include the bottom edge.
        strokes_by_color.setdefault(color_name, []).append(
            ((x0, y1), (x1, y1))
        )

    return strokes_by_color


# --- Ordered strokes -> path commands --------------------------------------


def _fill_strokes_to_commands(
    ordered_points: list, color_hex: str, label_prefix: str
) -> list:
    commands = []
    for index, (p0, p1) in enumerate(ordered_points, start=1):
        label = f"{label_prefix}_{index}"
        commands.append(move_to(p0[0], p0[1], label))
        commands.append(lower_tool(label))
        commands.append(paint_stroke(p0, p1, color_hex, label))
        commands.append(lift_tool(label))
    return commands


def order_and_build_commands(
    strokes_by_color: dict,
    palette_colors: list,
    home_position_mm: tuple,
) -> list:
    """Group by colour, greedy-order each group, no border strokes."""
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
        commands.extend(
            _fill_strokes_to_commands(ordered, color["hex"], f"{color_name}_fill")
        )
        current_position = ordered[-1][-1]

    return commands


# --- Assembling painting_paths.json ----------------------------------------

def build_painting_paths(
    commands: list,
    config: dict,
    config_path: Path,
    image_path: Path,
    grid_debug: dict,
) -> dict:
    stroke_commands = [cmd for cmd in commands if cmd["command"] == "paint_stroke"]
    total_distance = sum(
        math.dist(cmd["from_mm"], cmd["to_mm"]) for cmd in stroke_commands
    )

    def count_commands(command_name: str) -> int:
        return sum(1 for cmd in commands if cmd["command"] == command_name)

    canvas = config["canvas"]
    path_generation = config["path_generation"]

    return {
        "project": config.get("project"),
        "style": config.get("style", "mosaic"),
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
            "stroke_overlap_ratio": path_generation["stroke_overlap_ratio"],
            "mask_erosion_mm": path_generation.get("mask_erosion_mm", 0),
        },
        "border_settings": {
            "draw_borders": False,
            "simplify_epsilon_ratio": config.get("border_generation", {}).get(
                "simplify_epsilon_ratio", 0.002
            ),
        },
        "mosaic_settings": {
            "cell_size_mm": config["mosaic"]["cell_size_mm"],
            "color_threshold_ratio": config["mosaic"].get("color_threshold_ratio", 0.5),
            "skip_white_cells": config["mosaic"].get("skip_white_cells", True),
            "active_colors": config["mosaic"].get("active_colors", []),
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
            **grid_debug,
        },
    }


# --- SVG / debug preview ---------------------------------------------------

def render_svg(
    painting_paths: dict,
    *,
    mosaic_cells: list = None,
    scale_mm_per_px: float = None,
    offset_mm: tuple = None,
    skip_white: bool = True,
) -> str:
    """Draw a preview SVG. When mosaic_cells is provided (mosaic route), each
    cell is rendered as a filled coloured rectangle matched to what grid_majority_vote
    assigned — much cleaner than thousands of overlaid paint_stroke lines.
    Otherwise (image_to_mondrian route), every paint_stroke command is drawn
    as an individual coloured line."""
    canvas = painting_paths["canvas"]
    width_mm = canvas["width_mm"]
    height_mm = canvas["height_mm"]
    width_in = canvas["width_in"]
    height_in = canvas["height_in"]

    elements = [
        f'<rect x="0" y="0" width="{width_mm}" height="{height_mm}" fill="white" />'
    ]

    if mosaic_cells is not None and scale_mm_per_px is not None and offset_mm is not None:
        # --- Mosaic route: one filled <rect> per non-white cell ---
        for cell in mosaic_cells:
            if skip_white and cell["color_name"] == "white":
                continue
            x0, y0 = px_to_mm((cell["x0_px"], cell["y0_px"]), scale_mm_per_px, offset_mm)
            x1, y1 = px_to_mm((cell["x1_px"], cell["y1_px"]), scale_mm_per_px, offset_mm)
            w = round(x1 - x0, 3)
            h = round(y1 - y0, 3)
            elements.append(
                f'<rect x="{round(x0, 2)}" y="{round(y0, 2)}" '
                f'width="{w}" height="{h}" '
                f'fill="{escape(cell["color_hex"])}" '
                f'stroke="#cccccc" stroke-width="0.3" />'
            )
    else:
        # --- Generic route: paint_stroke lines + travel dashes ---
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
                    f'stroke="{escape(cmd["color"])}" '
                    f'stroke-width="{painting_paths["path_settings"]["tool_width_mm"]}" '
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


def render_quantized_preview(
    label_image: np.ndarray, palette_colors: list
) -> np.ndarray:
    """BGR debug image: every pixel painted its quantized palette colour."""
    height, width = label_image.shape
    preview = np.full((height, width, 3), 255, dtype=np.uint8)
    for color_index, color in enumerate(palette_colors):
        bgr = color_quantize.hex_to_bgr(color["hex"])
        preview[label_image == color_index] = bgr
    return preview


def render_mosaic_preview(
    cells: list,
    image_size: tuple,
    palette_colors: list,
    skip_white: bool,
) -> np.ndarray:
    """BGR debug image: each cell filled with its dominant colour."""
    height, width = image_size
    preview = np.full((height, width, 3), 255, dtype=np.uint8)

    for cell in cells:
        if skip_white and cell["color_name"] == "white":
            continue
        bgr = color_quantize.hex_to_bgr(cell["color_hex"])
        preview[cell["y0_px"]:cell["y1_px"], cell["x0_px"]:cell["x1_px"]] = bgr

    return preview


# --- Main pipeline ---------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert a source photo into a 4-colour mosaic painting_paths.json."
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_FILE,
        help=f"Path to a pipeline config JSON file (default: {DEFAULT_CONFIG_FILE}).",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_config(config_path)

    # --- Validate mosaic section exists ---
    if "mosaic" not in config:
        raise ConfigError(
            f"Config {config_path} is missing the 'mosaic' section required "
            f"by the mosaic route."
        )

    mosaic_cfg = config["mosaic"]
    output = config["output"]
    output_dir = Path(output["directory"])
    paths_output_file = output_dir / output["painting_paths_file"]
    svg_output_file = output_dir / output["preview_svg_file"]
    quantized_preview_file = output.get("quantized_preview_png_file")
    mosaic_preview_file = output.get("mosaic_preview_png_file")
    output_dir.mkdir(exist_ok=True)

    # --- Load & preprocess image ---
    source_image = config["source_image"]
    palette_colors = config["palette"]["colors"]
    color_space = config["palette"].get("color_space", "lab")
    neutral_chroma_threshold = config["palette"].get("neutral_chroma_threshold", 0.0)

    image = color_quantize.load_image(source_image["path"])
    image = color_quantize.preprocess(
        image,
        blur_kernel_size=source_image.get("blur_kernel_size", 5),
        blur_sigma=source_image.get("blur_sigma", 0),
        downscale_max_dimension_px=source_image.get("downscale_max_dimension_px"),
        bilateral_d=source_image.get("bilateral_d", 0),
        bilateral_sigma_color=source_image.get("bilateral_sigma_color", 75),
        bilateral_sigma_space=source_image.get("bilateral_sigma_space", 75),
    )

    # --- Quantize to 4-colour palette ---
    if not neutral_chroma_threshold and config["palette"].get(
        "neutral_chroma_percentile"
    ) is not None:
        neutral_chroma_threshold = color_quantize.compute_adaptive_chroma_threshold(
            image, config["palette"]["neutral_chroma_percentile"]
        )
    label_image = color_quantize.quantize_to_palette(
        image, palette_colors, color_space, neutral_chroma_threshold
    )

    height, width = label_image.shape
    image_path = Path(source_image["path"])
    scale_mm_per_px, offset_mm = image_to_canvas_transform(
        (width, height), config["canvas"]
    )

    # --- Grid division & majority vote ---
    cell_size_mm = mosaic_cfg["cell_size_mm"]
    cell_size_px = compute_cell_size_px(cell_size_mm, scale_mm_per_px)

    # Build set of active colour indices from active_colors list
    active_color_names = set(mosaic_cfg.get("active_colors", []))
    active_color_indices = set()
    for ci, c in enumerate(palette_colors):
        if c["name"] in active_color_names:
            active_color_indices.add(ci)

    cells = grid_majority_vote(
        label_image, cell_size_px, palette_colors, active_color_indices
    )

    # --- Build fill strokes from cells ---
    path_generation = config["path_generation"]
    canvas = config["canvas"]
    margin = canvas.get("margin_mm", 0.0)
    home_position = tuple(
        path_generation.get("home_position_mm", (margin, margin))
    )

    skip_white = mosaic_cfg.get("skip_white_cells", True)

    strokes_by_color = build_cell_strokes(
        cells, path_generation, scale_mm_per_px, offset_mm, skip_white
    )

    # --- Order & build commands (no borders) ---
    commands = order_and_build_commands(
        strokes_by_color, palette_colors, home_position
    )

    if not any(cmd["command"] == "paint_stroke" for cmd in commands):
        print(f"No paintable cells found in {image_path} - nothing to write.")
        sys.exit(1)

    # --- Debug info ---
    num_cells_by_color = {}
    filled_cells = [c for c in cells if not (skip_white and c["color_name"] == "white")]
    for cell in filled_cells:
        num_cells_by_color[cell["color_name"]] = (
            num_cells_by_color.get(cell["color_name"], 0) + 1
        )

    # Derive actual grid dimensions from the cell list (guaranteed to fill exactly).
    num_cols = max(c["col"] for c in cells) + 1 if cells else 0
    num_rows = max(c["row"] for c in cells) + 1 if cells else 0

    grid_debug = {
        "num_cells_total": len(cells),
        "num_cells_filled": len(filled_cells),
        "num_cells_skipped_white": len(cells) - len(filled_cells),
        "grid_cols": num_cols,
        "grid_rows": num_rows,
        "cell_size_mm": cell_size_mm,
        "num_cells_by_color": num_cells_by_color,
        "num_border_strokes": 0,
    }

    painting_paths = build_painting_paths(
        commands, config, config_path, image_path, grid_debug
    )

    validation = validate_painting_paths(painting_paths)
    painting_paths["validation"] = validation

    with open(paths_output_file, "w", encoding="utf-8") as file:
        json.dump(painting_paths, file, indent=2)
    print(f"Generated {paths_output_file}")

    svg_content = render_svg(
        painting_paths,
        mosaic_cells=cells,
        scale_mm_per_px=scale_mm_per_px,
        offset_mm=offset_mm,
        skip_white=skip_white,
    )
    with open(svg_output_file, "w", encoding="utf-8") as file:
        file.write(svg_content)
    print(f"Generated {svg_output_file}")

    if quantized_preview_file:
        preview_path = output_dir / quantized_preview_file
        cv2.imwrite(
            str(preview_path), render_quantized_preview(label_image, palette_colors)
        )
        print(f"Generated {preview_path}")

    if mosaic_preview_file:
        mosaic_path = output_dir / mosaic_preview_file
        cv2.imwrite(
            str(mosaic_path),
            render_mosaic_preview(cells, (height, width), palette_colors, skip_white),
        )
        print(f"Generated {mosaic_path}")

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