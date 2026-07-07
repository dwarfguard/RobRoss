import argparse
import json
import random
from pathlib import Path
from dataclasses import dataclass
from html import escape

from config_loader import load_config

DEFAULT_CONFIG_FILE = "configs/demo_v1_a4_pen.json"

# Fallback color name, used when an accent color isn't in COLOR_NAMES below
# (e.g. a custom palette supplied by a config).
DEFAULT_COLOR_NAME = "block"

# Human-readable names for the classic De Stijl palette, used to build
# labels like "red_block_1". Colors outside this map still work, they just
# get a generic "block_N" label instead of "red_block_N".
COLOR_NAMES = {
    "#d62828": "red",
    "#f7c600": "yellow",
    "#1d4ed8": "blue",
    "#e5e5e5": "gray",
}


@dataclass
class Rect:
    x: float
    y: float
    width: float
    height: float
    fill: str
    label: str | None = None


@dataclass
class Line:
    x1: float
    y1: float
    x2: float
    y2: float
    stroke: str = "black"
    stroke_width: float = 6.0
    label: str | None = None


def svg_rect(rect: Rect) -> str:
    return (
        f'<rect x="{rect.x}" y="{rect.y}" '
        f'width="{rect.width}" height="{rect.height}" '
        f'fill="{escape(rect.fill)}" />'
    )


def svg_line(line: Line) -> str:
    return (
        f'<line x1="{line.x1}" y1="{line.y1}" '
        f'x2="{line.x2}" y2="{line.y2}" '
        f'stroke="{escape(line.stroke)}" '
        f'stroke-width="{line.stroke_width}" '
        f'stroke-linecap="square" />'
    )


def subdivide(x, y, w, h, depth, min_w, min_h, max_depth, min_split_depth, rng):
    """
    Recursively split a rectangle into smaller cells, Mondrian-style.

    min_w/min_h are the smallest allowed cell width/height on each axis, so
    a non-square canvas doesn't produce slivers along its shorter side.

    Depths below min_split_depth always split (when the cell is big enough
    to split at all), so a shallow random stop can never produce an empty,
    border-only artwork.

    Returns a tuple (leaf_cells, split_lines):
    - leaf_cells is a list of (x, y, w, h) tuples for undivided regions.
    - split_lines is a list of Line objects marking every cut made.
    """
    can_split_v = w >= min_w * 2
    can_split_h = h >= min_h * 2

    if depth >= max_depth or not (can_split_v or can_split_h):
        return [(x, y, w, h)], []

    # Stop probability grows with depth so cells get progressively coarser.
    stop_chance = 0.12 + 0.15 * depth
    if depth >= min_split_depth and rng.random() < stop_chance:
        return [(x, y, w, h)], []

    if can_split_v and can_split_h:
        vertical = rng.random() < (w / (w + h))
    else:
        vertical = can_split_v

    leaves = []
    lines = []

    if vertical:
        min_frac = min_w / w
        frac = rng.uniform(min_frac, 1 - min_frac)
        split_x = x + w * frac
        left_leaves, left_lines = subdivide(x, y, split_x - x, h, depth + 1, min_w, min_h, max_depth, min_split_depth, rng)
        right_leaves, right_lines = subdivide(split_x, y, x + w - split_x, h, depth + 1, min_w, min_h, max_depth, min_split_depth, rng)
        leaves = left_leaves + right_leaves
        lines = left_lines + right_lines + [Line(split_x, y, split_x, y + h)]
    else:
        min_frac = min_h / h
        frac = rng.uniform(min_frac, 1 - min_frac)
        split_y = y + h * frac
        top_leaves, top_lines = subdivide(x, y, w, split_y - y, depth + 1, min_w, min_h, max_depth, min_split_depth, rng)
        bottom_leaves, bottom_lines = subdivide(x, split_y, w, y + h - split_y, depth + 1, min_w, min_h, max_depth, min_split_depth, rng)
        leaves = top_leaves + bottom_leaves
        lines = top_lines + bottom_lines + [Line(x, split_y, x + w, split_y)]

    return leaves, lines


def generate_mondrian_layout(config: dict, seed: int | None = None) -> tuple[list[Rect], list[Line]]:
    """
    Generate a randomized Mondrian-style layout: the colored rectangles and
    the black grid lines that go with them, sized and styled according to
    `config` (canvas size, palette, stroke widths, etc.).

    This is the single source of truth for "what the robot should paint".
    Both the SVG preview and the JSON painting plan are built from the
    rectangles and lines returned here, so they always describe the same
    artwork.

    Coordinate system:
    - Units are millimeters.
    - (0, 0) is the top-left corner.
    - x increases to the right.
    - y increases downward.
    """
    rng = random.Random(seed)

    canvas = config["canvas"]
    width_mm = canvas["width_mm"]
    height_mm = canvas["height_mm"]
    margin_mm = canvas.get("margin_mm", 0.0)

    artwork = config["artwork"]
    min_cell_fraction = artwork.get("min_cell_fraction", 0.14)
    max_split_depth = artwork.get("max_split_depth", 5)
    min_split_depth = artwork.get("min_split_depth", 1)
    background_color = artwork.get("background_color", "white")
    line_color = artwork.get("line_color", "black")
    palette_mode = artwork.get("palette_mode", "color")
    stroke_width_min = artwork.get("stroke_width_min_mm", 5.0)
    stroke_width_max = artwork.get("stroke_width_max_mm", 8.0)

    # Drawable region: the whole artwork, border included, is kept
    # margin_mm inside the physical canvas/paper edge, so the tool never
    # draws right at the edge (where calibration error or paper lift is
    # most dangerous).
    draw_x = margin_mm
    draw_y = margin_mm
    draw_w = width_mm - 2 * margin_mm
    draw_h = height_mm - 2 * margin_mm

    min_w = draw_w * min_cell_fraction
    min_h = draw_h * min_cell_fraction
    leaves, lines = subdivide(
        draw_x, draw_y, draw_w, draw_h, 0, min_w, min_h, max_split_depth, min_split_depth, rng
    )

    # Background and color regions. The background rect is only needed for
    # the SVG preview (real canvases already start in background_color), so
    # it is not labeled and is filtered out of the painting plan later.
    rectangles = [Rect(0, 0, width_mm, height_mm, background_color)]

    # Monochrome profiles (e.g. the pen-only Demo v1 target) skip colored
    # fills entirely: no accent cells are picked, so only grid lines and
    # the border remain, matching a pen/line-only artwork.
    if palette_mode == "monochrome":
        accent_cells = []
        palette = []
    else:
        accent_colors = artwork.get("accent_colors", [])
        neutral_accent_color = artwork.get("neutral_accent_color")
        neutral_accent_probability = artwork.get("neutral_accent_probability", 0.0)

        accent_count = min(len(leaves), rng.randint(2, 4)) if accent_colors else 0
        accent_cells = rng.sample(leaves, k=accent_count) if accent_count else []
        palette = accent_colors.copy()
        rng.shuffle(palette)
        if neutral_accent_color and rng.random() < neutral_accent_probability:
            palette.append(neutral_accent_color)

    color_counts: dict[str, int] = {}
    for i, (cx, cy, cw, ch) in enumerate(accent_cells):
        color = palette[i % len(palette)]
        color_counts[color] = color_counts.get(color, 0) + 1
        color_name = COLOR_NAMES.get(color, DEFAULT_COLOR_NAME)
        label = f"{color_name}_block_{color_counts[color]}"
        rectangles.append(Rect(cx, cy, cw, ch, color, label=label))

    stroke_width = rng.uniform(stroke_width_min, stroke_width_max)
    for i, line in enumerate(lines, start=1):
        line.stroke = line_color
        line.stroke_width = stroke_width
        line.label = f"grid_line_{i}"

    # Outer border, added and labeled last so it paints after the interior
    # grid lines. It sits on the drawable region's edge (margin_mm inside
    # the paper), and width/height are handled separately so this works for
    # non-square canvases like A4 portrait.
    lines.extend([
        Line(draw_x, draw_y, draw_x + draw_w, draw_y, stroke=line_color, stroke_width=stroke_width, label="border_top"),
        Line(draw_x, draw_y + draw_h, draw_x + draw_w, draw_y + draw_h, stroke=line_color, stroke_width=stroke_width, label="border_bottom"),
        Line(draw_x, draw_y, draw_x, draw_y + draw_h, stroke=line_color, stroke_width=stroke_width, label="border_left"),
        Line(draw_x + draw_w, draw_y, draw_x + draw_w, draw_y + draw_h, stroke=line_color, stroke_width=stroke_width, label="border_right"),
    ])

    return rectangles, lines


def render_svg(rectangles: list[Rect], lines: list[Line], canvas: dict) -> str:
    """Render the given rectangles and lines as an SVG document sized to canvas."""
    width_mm = canvas["width_mm"]
    height_mm = canvas["height_mm"]
    width_in = width_mm / 25.4
    height_in = height_mm / 25.4

    svg_elements = []

    for rect in rectangles:
        svg_elements.append(svg_rect(rect))

    for line in lines:
        svg_elements.append(svg_line(line))

    svg_body = "\n  ".join(svg_elements)

    return f'''<svg
  xmlns="http://www.w3.org/2000/svg"
  width="{width_in}in"
  height="{height_in}in"
  viewBox="0 0 {width_mm} {height_mm}"
>
  {svg_body}
</svg>
'''


def build_painting_plan(rectangles: list[Rect], lines: list[Line], config: dict, config_path: Path, seed: int | None) -> dict:
    """
    Build a robot-friendly painting plan from the same rectangles and lines
    used to render the SVG.

    Colored rectangles come first (painted first), and black grid lines
    come last, since they visually clean up imperfect rectangle edges.
    """
    canvas = config["canvas"]
    background_color = config["artwork"].get("background_color", "white")
    operations = []

    # Only non-background rectangles are real paint operations: the canvas
    # already starts in background_color, so there is nothing to paint there.
    colored_rects = [rect for rect in rectangles if rect.fill.lower() != background_color.lower()]
    for rect in colored_rects:
        operations.append({
            "operation": "paint_rectangle",
            "label": rect.label,
            "color": rect.fill,
            "x_mm": round(rect.x, 2),
            "y_mm": round(rect.y, 2),
            "width_mm": round(rect.width, 2),
            "height_mm": round(rect.height, 2),
            "fill_strategy": "solid_fill",
        })

    for line in lines:
        operations.append({
            "operation": "paint_line",
            "label": line.label,
            "color": line.stroke,
            "from_mm": [round(line.x1, 2), round(line.y1, 2)],
            "to_mm": [round(line.x2, 2), round(line.y2, 2)],
            "stroke_width_mm": round(line.stroke_width, 2),
        })

    colors_used = sorted({rect.fill for rect in colored_rects})

    return {
        "project": config.get("project", "R.O.B Ross"),
        "style": config.get("style", "mondrian"),
        "version": "0.1",
        "config": {
            "profile_name": config.get("profile_name"),
            "source_file": str(config_path),
        },
        "canvas": {
            "width_mm": canvas["width_mm"],
            "height_mm": canvas["height_mm"],
            "width_in": round(canvas["width_mm"] / 25.4, 4),
            "height_in": round(canvas["height_mm"] / 25.4, 4),
            "margin_mm": canvas.get("margin_mm", 0.0),
            "origin": canvas["origin"],
        },
        "units": "mm",
        "coordinate_system": {
            "x_direction": canvas.get("x_direction", "right"),
            "y_direction": canvas.get("y_direction", "down"),
        },
        "assumptions": [
            f"Canvas starts {background_color}.",
            f"All strokes stay at least {canvas.get('margin_mm', 0.0)} mm inside the canvas edge.",
            "Colored rectangles are painted before black grid lines.",
            "Black grid lines are painted last to clean up rectangle edges.",
            "This is an intermediate painting plan, not final robot motor code.",
        ],
        "operations": operations,
        "debug": {
            "seed": seed,
            "num_rectangles": len(colored_rects),
            "num_lines": len(lines),
            "num_operations": len(operations),
            "colors_used": colors_used,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate a randomized Mondrian-style SVG and a matching painting plan JSON."
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_FILE,
        help=f"Path to a pipeline config JSON file (default: {DEFAULT_CONFIG_FILE}).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducible output (default: random each run).",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_config(config_path)

    output = config["output"]
    output_dir = Path(output["directory"])
    svg_output_file = output_dir / output["preview_svg_file"]
    json_output_file = output_dir / output["painting_plan_file"]

    output_dir.mkdir(exist_ok=True)

    # Resolve the seed up front so we can report it, even when it was
    # picked randomly, so this exact graphic can be reproduced later.
    seed = args.seed if args.seed is not None else random.SystemRandom().randrange(2**32)

    # Generate the layout once. Both the SVG and the JSON painting plan are
    # built from these same rectangles and lines, so they always match.
    rectangles, lines = generate_mondrian_layout(config, seed=seed)

    svg_content = render_svg(rectangles, lines, config["canvas"])
    with open(svg_output_file, "w", encoding="utf-8") as file:
        file.write(svg_content)
    print(f"Generated {svg_output_file} (seed={seed})")

    painting_plan = build_painting_plan(rectangles, lines, config, config_path, seed=seed)
    with open(json_output_file, "w", encoding="utf-8") as file:
        json.dump(painting_plan, file, indent=2)
    print(f"Generated {json_output_file} (seed={seed})")


if __name__ == "__main__":
    main()
