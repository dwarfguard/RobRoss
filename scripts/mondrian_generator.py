import argparse
import random
from pathlib import Path
from dataclasses import dataclass
from html import escape


# 12 inches = 304.8 mm
CANVAS_SIZE_MM = 304.8

OUTPUT_DIR = Path("output")
OUTPUT_FILE = OUTPUT_DIR / "mondrian_preview.svg"

# Classic De Stijl palette.
ACCENT_COLORS = ["#d62828", "#f7c600", "#1d4ed8"]  # red, yellow, blue
NEUTRAL_ACCENT = "#e5e5e5"  # occasional gray block, seen in some Mondrian works

MIN_CELL_FRACTION = 0.14  # smallest cell edge, as a fraction of canvas size
MAX_SPLIT_DEPTH = 5


@dataclass
class Rect:
    x: float
    y: float
    width: float
    height: float
    fill: str


@dataclass
class Line:
    x1: float
    y1: float
    x2: float
    y2: float
    stroke: str = "black"
    stroke_width: float = 6.0


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


def generate_mondrian_svg(seed: int | None = None) -> str:
    """
    Generate a randomized Mondrian-style SVG.

    Coordinate system:
    - Units are millimeters.
    - (0, 0) is the top-left corner.
    - x increases to the right.
    - y increases downward.
    """
    rng = random.Random(seed)

    min_size = CANVAS_SIZE_MM * MIN_CELL_FRACTION
    leaves, lines = subdivide(0, 0, CANVAS_SIZE_MM, CANVAS_SIZE_MM, 0, min_size, rng)

    # Background and color regions.
    rectangles = [Rect(0, 0, CANVAS_SIZE_MM, CANVAS_SIZE_MM, "white")]

    # Pick a handful of leaf cells to fill with accent colors; the rest
    # stay white (the background already covers them).
    accent_count = min(len(leaves), rng.randint(2, 4))
    accent_cells = rng.sample(leaves, k=accent_count)
    palette = ACCENT_COLORS.copy()
    rng.shuffle(palette)
    if rng.random() < 0.25:
        palette.append(NEUTRAL_ACCENT)

    for i, (cx, cy, cw, ch) in enumerate(accent_cells):
        color = palette[i % len(palette)]
        rectangles.append(Rect(cx, cy, cw, ch, color))

    stroke_width = rng.uniform(5.0, 8.0)
    for line in lines:
        line.stroke_width = stroke_width

    # Outer border.
    lines.extend([
        Line(0, 0, CANVAS_SIZE_MM, 0, stroke_width=stroke_width),
        Line(0, CANVAS_SIZE_MM, CANVAS_SIZE_MM, CANVAS_SIZE_MM, stroke_width=stroke_width),
        Line(0, 0, 0, CANVAS_SIZE_MM, stroke_width=stroke_width),
        Line(CANVAS_SIZE_MM, 0, CANVAS_SIZE_MM, CANVAS_SIZE_MM, stroke_width=stroke_width),
    ])

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


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a randomized Mondrian-style SVG.")
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

    svg_content = generate_mondrian_svg(seed=seed)

    with open(OUTPUT_FILE, "w", encoding="utf-8") as file:
        file.write(svg_content)

    print(f"Generated {OUTPUT_FILE} (seed={seed})")


if __name__ == "__main__":
    main()