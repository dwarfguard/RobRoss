import argparse
import json
import random
from pathlib import Path
from dataclasses import dataclass
from html import escape


# 12 inches = 304.8 mm
CANVAS_SIZE_MM = 304.8
CANVAS_SIZE_IN = 12

OUTPUT_DIR = Path("output")
SVG_OUTPUT_FILE = OUTPUT_DIR / "mondrian_preview.svg"
JSON_OUTPUT_FILE = OUTPUT_DIR / "painting_plan.json"

# Classic De Stijl palette.
ACCENT_COLORS = ["#d62828", "#f7c600", "#1d4ed8"]  # red, yellow, blue
NEUTRAL_ACCENT = "#e5e5e5"  # occasional gray block, seen in some Mondrian works

# Human-readable names for each color, used to build labels like "red_block_1".
COLOR_NAMES = {
    "#d62828": "red",
    "#f7c600": "yellow",
    "#1d4ed8": "blue",
    NEUTRAL_ACCENT: "gray",
}

MIN_CELL_FRACTION = 0.14  # smallest cell edge, as a fraction of canvas size
MAX_SPLIT_DEPTH = 5


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


def subdivide(x, y, w, h, depth, min_size, rng):
    """
    Recursively split a rectangle into smaller cells, Mondrian-style.

    Returns a tuple (leaf_cells, split_lines):
    - leaf_cells is a list of (x, y, w, h) tuples for undivided regions.
    - split_lines is a list of Line objects marking every cut made.
    """
    can_split_v = w >= min_size * 2
    can_split_h = h >= min_size * 2

    # Stop probability grows with depth so cells get progressively coarser.
    stop_chance = 0.12 + 0.15 * depth
    if depth >= MAX_SPLIT_DEPTH or not (can_split_v or can_split_h) or rng.random() < stop_chance:
        return [(x, y, w, h)], []

    if can_split_v and can_split_h:
        vertical = rng.random() < (w / (w + h))
    else:
        vertical = can_split_v

    leaves = []
    lines = []

    if vertical:
        min_frac = min_size / w
        frac = rng.uniform(min_frac, 1 - min_frac)
        split_x = x + w * frac
        left_leaves, left_lines = subdivide(x, y, split_x - x, h, depth + 1, min_size, rng)
        right_leaves, right_lines = subdivide(split_x, y, x + w - split_x, h, depth + 1, min_size, rng)
        leaves = left_leaves + right_leaves
        lines = left_lines + right_lines + [Line(split_x, y, split_x, y + h)]
    else:
        min_frac = min_size / h
        frac = rng.uniform(min_frac, 1 - min_frac)
        split_y = y + h * frac
        top_leaves, top_lines = subdivide(x, y, w, split_y - y, depth + 1, min_size, rng)
        bottom_leaves, bottom_lines = subdivide(x, split_y, w, y + h - split_y, depth + 1, min_size, rng)
        leaves = top_leaves + bottom_leaves
        lines = top_lines + bottom_lines + [Line(x, split_y, x + w, split_y)]

    return leaves, lines


def generate_mondrian_layout(seed: int | None = None) -> tuple[list[Rect], list[Line]]:
    """
    Generate a randomized Mondrian-style layout: the colored rectangles and
    the black grid lines that go with them.

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

    min_size = CANVAS_SIZE_MM * MIN_CELL_FRACTION
    leaves, lines = subdivide(0, 0, CANVAS_SIZE_MM, CANVAS_SIZE_MM, 0, min_size, rng)

    # Background and color regions. The white background is only needed for
    # the SVG preview (real canvases already start white), so it is not
    # labeled and is filtered out of the painting plan later.
    rectangles = [Rect(0, 0, CANVAS_SIZE_MM, CANVAS_SIZE_MM, "white")]

    # Pick a handful of leaf cells to fill with accent colors; the rest
    # stay white (the background already covers them).
    accent_count = min(len(leaves), rng.randint(2, 4))
    accent_cells = rng.sample(leaves, k=accent_count)
    palette = ACCENT_COLORS.copy()
    rng.shuffle(palette)
    if rng.random() < 0.25:
        palette.append(NEUTRAL_ACCENT)

    color_counts: dict[str, int] = {}
    for i, (cx, cy, cw, ch) in enumerate(accent_cells):
        color = palette[i % len(palette)]
        color_counts[color] = color_counts.get(color, 0) + 1
        color_name = COLOR_NAMES.get(color, "block")
        label = f"{color_name}_block_{color_counts[color]}"
        rectangles.append(Rect(cx, cy, cw, ch, color, label=label))

    stroke_width = rng.uniform(5.0, 8.0)
    for i, line in enumerate(lines, start=1):
        line.stroke_width = stroke_width
        line.label = f"grid_line_{i}"

    # Outer border, added and labeled last so it paints after the interior
    # grid lines.
    lines.extend([
        Line(0, 0, CANVAS_SIZE_MM, 0, stroke_width=stroke_width, label="border_top"),
        Line(0, CANVAS_SIZE_MM, CANVAS_SIZE_MM, CANVAS_SIZE_MM, stroke_width=stroke_width, label="border_bottom"),
        Line(0, 0, 0, CANVAS_SIZE_MM, stroke_width=stroke_width, label="border_left"),
        Line(CANVAS_SIZE_MM, 0, CANVAS_SIZE_MM, CANVAS_SIZE_MM, stroke_width=stroke_width, label="border_right"),
    ])

    return rectangles, lines


def render_svg(rectangles: list[Rect], lines: list[Line]) -> str:
    """Render the given rectangles and lines as an SVG document."""
    svg_elements = []

    for rect in rectangles:
        svg_elements.append(svg_rect(rect))

    for line in lines:
        svg_elements.append(svg_line(line))

    svg_body = "\n  ".join(svg_elements)

    return f'''<svg
  xmlns="http://www.w3.org/2000/svg"
  width="12in"
  height="12in"
  viewBox="0 0 {CANVAS_SIZE_MM} {CANVAS_SIZE_MM}"
>
  {svg_body}
</svg>
'''


def build_painting_plan(rectangles: list[Rect], lines: list[Line], seed: int | None) -> dict:
    """
    Build a robot-friendly painting plan from the same rectangles and lines
    used to render the SVG.

    Colored rectangles come first (painted first), and black grid lines
    come last, since they visually clean up imperfect rectangle edges.
    """
    operations = []

    # Only non-white rectangles are real paint operations: the canvas
    # already starts white, so there is nothing to paint there.
    colored_rects = [rect for rect in rectangles if rect.fill.lower() != "white"]
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
        "project": "R.O.B Ross",
        "style": "mondrian",
        "version": "0.1",
        "canvas": {
            "width_mm": CANVAS_SIZE_MM,
            "height_mm": CANVAS_SIZE_MM,
            "width_in": CANVAS_SIZE_IN,
            "height_in": CANVAS_SIZE_IN,
            "origin": "top-left",
        },
        "units": "mm",
        "coordinate_system": {
            "x_direction": "right",
            "y_direction": "down",
        },
        "assumptions": [
            "Canvas starts white.",
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
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducible output (default: random each run).",
    )
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(exist_ok=True)

    # Resolve the seed up front so we can report it, even when it was
    # picked randomly, so this exact graphic can be reproduced later.
    seed = args.seed if args.seed is not None else random.SystemRandom().randrange(2**32)

    # Generate the layout once. Both the SVG and the JSON painting plan are
    # built from these same rectangles and lines, so they always match.
    rectangles, lines = generate_mondrian_layout(seed=seed)

    svg_content = render_svg(rectangles, lines)
    with open(SVG_OUTPUT_FILE, "w", encoding="utf-8") as file:
        file.write(svg_content)
    print(f"Generated {SVG_OUTPUT_FILE} (seed={seed})")

    painting_plan = build_painting_plan(rectangles, lines, seed=seed)
    with open(JSON_OUTPUT_FILE, "w", encoding="utf-8") as file:
        json.dump(painting_plan, file, indent=2)
    print(f"Generated {JSON_OUTPUT_FILE} (seed={seed})")


if __name__ == "__main__":
    main()