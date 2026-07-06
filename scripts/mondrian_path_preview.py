import argparse
import os
import random
import sys
from html import escape

sys.path.insert(0, os.path.dirname(__file__))
import mondrian_robot_path as mrp  # noqa: E402  (reuses build_mondrian_robot_path, no path logic here)

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'output')
OUTPUT_FILE = os.path.join(OUTPUT_DIR, 'mondrian_path_preview.svg')

LEGEND_ROW_HEIGHT_MM = 14.0
LEGEND_SWATCH_SIZE_MM = 10.0
LEGEND_TOP_MARGIN_MM = 10.0


def tool_label(tool):
    return "outline / border" if tool["kind"] == "line" else "fill"


def render_strokes(tool, brush_width_mm):
    elements = []
    for stroke in tool["strokes"]:
        points = " ".join(f"{x:.1f},{y:.1f}" for x, y in stroke)
        elements.append(
            f'<polyline points="{points}" fill="none" stroke="{escape(tool["color"])}" '
            f'stroke-width="{brush_width_mm}" stroke-linecap="round" stroke-linejoin="round" '
            f'opacity="0.85" />'
        )
    return elements


def render_legend(tools, canvas_size_mm):
    elements = []
    y = canvas_size_mm + LEGEND_TOP_MARGIN_MM
    for tool in tools:
        swatch_y = y
        text_y = y + LEGEND_SWATCH_SIZE_MM * 0.8
        elements.append(
            f'<rect x="0" y="{swatch_y:.1f}" width="{LEGEND_SWATCH_SIZE_MM}" '
            f'height="{LEGEND_SWATCH_SIZE_MM}" fill="{escape(tool["color"])}" '
            f'stroke="black" stroke-width="0.5" />'
        )
        label = f'{tool["color"]} — {tool_label(tool)} ({len(tool["strokes"])} strokes)'
        elements.append(
            f'<text x="{LEGEND_SWATCH_SIZE_MM + 4}" y="{text_y:.1f}" '
            f'font-family="sans-serif" font-size="8">{escape(label)}</text>'
        )
        y += LEGEND_ROW_HEIGHT_MM
    return elements, y


def render_preview_svg(result, line_brush_width_mm, fill_brush_width_mm):
    canvas_size_mm = result["canvas_size_mm"]
    tools = result["tools"]

    svg_elements = [f'<rect x="0" y="0" width="{canvas_size_mm}" height="{canvas_size_mm}" fill="white" />']

    # Draw fills first and the black outline/border last, same layering as
    # mondrian_generator.py's own SVG output, so the grid stays visible on
    # top of the color instead of being partially hidden underneath it.
    for tool in sorted(tools, key=lambda t: t["kind"] == "line"):
        brush_width_mm = line_brush_width_mm if tool["kind"] == "line" else fill_brush_width_mm
        svg_elements.extend(render_strokes(tool, brush_width_mm))

    legend_elements, total_height = render_legend(tools, canvas_size_mm)
    svg_elements.extend(legend_elements)

    svg_body = "\n  ".join(svg_elements)

    return f'''<svg
  xmlns="http://www.w3.org/2000/svg"
  width="{canvas_size_mm}mm"
  height="{total_height}mm"
  viewBox="0 0 {canvas_size_mm} {total_height}"
>
  {svg_body}
</svg>
'''


def main():
    parser = argparse.ArgumentParser(
        description="Render a rough-coloring SVG preview of a Mondrian robot path plan, "
                    "with a legend marking which color goes to which brush/paint.")
    parser.add_argument("--seed", type=int, default=None,
                        help="Random seed for reproducible output (default: random each run).")
    parser.add_argument("--line-brush-width-mm", type=float, default=mrp.DEFAULT_LINE_BRUSH_WIDTH_MM,
                         help="Brush width used to draw the outline/border strokes.")
    parser.add_argument("--fill-brush-width-mm", type=float, default=mrp.DEFAULT_FILL_BRUSH_WIDTH_MM,
                         help="Brush width used to draw the fill strokes.")
    args = parser.parse_args()

    seed = args.seed if args.seed is not None else random.SystemRandom().randrange(2**32)

    result = mrp.build_mondrian_robot_path(
        seed=seed,
        line_brush_width_mm=args.line_brush_width_mm,
        fill_brush_width_mm=args.fill_brush_width_mm,
    )
    svg_content = render_preview_svg(result, args.line_brush_width_mm, args.fill_brush_width_mm)

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        f.write(svg_content)

    print(f"Generated {OUTPUT_FILE} (seed={seed})")


if __name__ == "__main__":
    main()
