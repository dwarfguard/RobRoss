import json
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(__file__))
import mondrian_generator  # noqa: E402  (reuses Rect/Line design data, no SVG re-parsing)
from path_ordering import order_strokes, total_travel_distance  # noqa: E402

# Robot brush widths in mm - separate knobs for the black grid-line brush and
# the color-fill brush in case hardware ends up using different tools for
# each. Both default to ~6mm, matching mondrian_generator.py's Line.stroke_width
# default and its randomized 5-8mm range.
DEFAULT_LINE_BRUSH_WIDTH_MM = 6.0
DEFAULT_FILL_BRUSH_WIDTH_MM = 6.0
DEFAULT_HOME_POSITION = (0.0, 0.0)

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'output')
DEBUG_OUTPUT_PATH = os.path.join(OUTPUT_DIR, 'mondrian_robot_path.json')


def trace_line(line):
    """A grid/border line is drawn in a single centerline pass, since the
    brush width and the design's line width are roughly the same."""
    return [(line.x1, line.y1), (line.x2, line.y2)]


def fill_rect(rect, brush_width_mm):
    """Boustrophedon (zigzag) scan path covering a rectangle's interior,
    inset by half the brush width on each side so the fill doesn't paint
    over the grid lines already drawn along the cell borders."""
    margin = brush_width_mm / 2
    x0, y0 = rect.x + margin, rect.y + margin
    x1, y1 = rect.x + rect.width - margin, rect.y + rect.height - margin

    if x1 <= x0 or y1 <= y0:
        # Cell too small for this brush to fill cleanly - skip rather than
        # produce a degenerate/out-of-bounds path.
        return None

    points = []
    y = y0
    left_to_right = True
    while y <= y1:
        row = [(x0, y), (x1, y)] if left_to_right else [(x1, y), (x0, y)]
        points.extend(row)
        left_to_right = not left_to_right
        y += brush_width_mm

    return points


def build_mondrian_robot_path(seed=None,
                               line_brush_width_mm=DEFAULT_LINE_BRUSH_WIDTH_MM,
                               fill_brush_width_mm=DEFAULT_FILL_BRUSH_WIDTH_MM,
                               home_position=DEFAULT_HOME_POSITION):
    """Returns a dict describing everything the robot needs to draw one
    generated Mondrian design, grouped by tool (the black line brush, and
    one group per accent color) so dedicated brushes/paint aren't swapped
    more than necessary:

    {
        "canvas_size_mm": float,
        "tools": [
            {"kind": "line", "color": "black", "strokes": list[list[(x, y)]]},
            {"kind": "fill", "color": "#d62828", "strokes": list[list[(x, y)]]},
            ...
        ],
    }
    """
    design = mondrian_generator.generate_mondrian_design(seed)

    tools = []

    line_strokes = [(trace_line(line), False) for line in design.lines]
    ordered_lines = order_strokes(line_strokes, home_position)
    tools.append({"kind": "line", "color": "black", "strokes": ordered_lines})

    fills_by_color = defaultdict(list)
    for rect in design.rectangles:
        if rect.fill == "white":
            continue  # background - nothing to paint
        path = fill_rect(rect, fill_brush_width_mm)
        if path is not None:
            fills_by_color[rect.fill].append((path, False))

    for color, strokes_data in fills_by_color.items():
        ordered_fill = order_strokes(strokes_data, home_position)
        tools.append({"kind": "fill", "color": color, "strokes": ordered_fill})

    return {"canvas_size_mm": design.canvas_size_mm, "tools": tools}


if __name__ == "__main__":
    result = build_mondrian_robot_path(seed=None)

    for tool in result["tools"]:
        strokes = tool["strokes"]
        total_points = sum(len(points) for points in strokes)
        travel = total_travel_distance(strokes, DEFAULT_HOME_POSITION)
        print(f"[{tool['kind']}] color={tool['color']} "
              f"strokes={len(strokes)} points={total_points} travel={travel:.1f}mm")

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    with open(DEBUG_OUTPUT_PATH, 'w') as f:
        json.dump(result, f)
    print(f"wrote {DEBUG_OUTPUT_PATH}")
