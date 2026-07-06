from pathlib import Path
from dataclasses import dataclass
from html import escape


# 12 inches = 304.8 mm
CANVAS_SIZE_MM = 304.8

OUTPUT_DIR = Path("output")
OUTPUT_FILE = OUTPUT_DIR / "mondrian_preview.svg"


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


def generate_mondrian_svg() -> str:
    """
    Generate a hardcoded Mondrian-style SVG.

    Coordinate system:
    - Units are millimeters.
    - (0, 0) is the top-left corner.
    - x increases to the right.
    - y increases downward.
    """

    # Background and color regions.
    rectangles = [
        # Full white canvas background
        Rect(0, 0, CANVAS_SIZE_MM, CANVAS_SIZE_MM, "white"),

        # Color blocks
        Rect(0, 0, 95, 120, "#d62828"),          # red block
        Rect(101, 0, 203.8, 55, "white"),
        Rect(101, 61, 85, 59, "#f7c600"),        # yellow block
        Rect(192, 61, 112.8, 120, "white"),
        Rect(0, 126, 95, 178.8, "white"),
        Rect(101, 126, 85, 90, "white"),
        Rect(101, 222, 203.8, 82.8, "#1d4ed8"), # blue block
        Rect(192, 187, 112.8, 29, "#e5e5e5"),   # gray block
    ]

    # Black grid lines.
    lines = [
        # vertical lines
        Line(98, 0, 98, CANVAS_SIZE_MM),
        Line(189, 61, 189, CANVAS_SIZE_MM),

        # horizontal lines
        Line(0, 123, 189, 123),
        Line(98, 58, CANVAS_SIZE_MM, 58),
        Line(189, 184, CANVAS_SIZE_MM, 184),
        Line(98, 219, CANVAS_SIZE_MM, 219),

        # outer border
        Line(0, 0, CANVAS_SIZE_MM, 0),
        Line(0, CANVAS_SIZE_MM, CANVAS_SIZE_MM, CANVAS_SIZE_MM),
        Line(0, 0, 0, CANVAS_SIZE_MM),
        Line(CANVAS_SIZE_MM, 0, CANVAS_SIZE_MM, CANVAS_SIZE_MM),
    ]

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
    OUTPUT_DIR.mkdir(exist_ok=True)

    svg_content = generate_mondrian_svg()

    with open(OUTPUT_FILE, "w", encoding="utf-8") as file:
        file.write(svg_content)

    print(f"Generated {OUTPUT_FILE}")


if __name__ == "__main__":
    main()