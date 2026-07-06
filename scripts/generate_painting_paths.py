import json
import math
from html import escape
from pathlib import Path

# This script does NOT generate a new random layout. It reads the layout
# already decided by mondrian_generator.py and converts it into concrete
# robot-style stroke paths.

OUTPUT_DIR = Path("output")
PLAN_INPUT_FILE = OUTPUT_DIR / "painting_plan.json"
PATHS_OUTPUT_FILE = OUTPUT_DIR / "painting_paths.json"
SVG_OUTPUT_FILE = OUTPUT_DIR / "path_preview.svg"

# Path settings: tune these to match the real paintbrush/tool later.
TOOL_WIDTH_MM = 10.0       # width of a single paint stroke
STROKE_OVERLAP_RATIO = 0.25  # how much each stripe overlaps the previous one
EDGE_INSET_MM = 3.0        # keep strokes this far inside a rectangle's edge

# Preview-only settings (do not affect painting_paths.json, just the SVG).
# Strokes are drawn semi-transparent so overlapping stripes show up as a
# visibly darker band, and travel moves (tool lifted, moving between
# strokes) are drawn as thin dashed gray lines, so the boustrophedon
# pattern is actually visible instead of looking like one solid block.
STROKE_PREVIEW_OPACITY = 0.55
TRAVEL_LINE_COLOR = "#999999"
TRAVEL_LINE_WIDTH_MM = 1.0


def load_painting_plan(path: Path) -> dict:
    """Load the painting plan JSON produced by mondrian_generator.py."""
    if not path.exists():
        raise FileNotFoundError(
            f"Could not find {path}. Run scripts/mondrian_generator.py first."
        )
    with open(path, "r", encoding="utf-8") as file:
        return json.load(file)


# --- Command builders --------------------------------------------------
# Each command is a small dict with a "command" name, a "label" for
# debugging, and whatever fields that command needs. Keeping them as
# plain dicts (instead of classes) keeps this script simple to read.

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


# --- Converting painting_plan.json operations into stroke commands -----

def compute_stripe_row_centers(y: float, height: float) -> list[float]:
    """
    Return the y-position of each horizontal stripe center needed to fill
    `height`, spaced so each stripe overlaps the previous one by
    STROKE_OVERLAP_RATIO.
    """
    if height <= TOOL_WIDTH_MM:
        # Rectangle is too short for more than one stripe; paint one
        # stroke straight down the middle.
        return [y + height / 2]

    stripe_step = TOOL_WIDTH_MM * (1 - STROKE_OVERLAP_RATIO)
    last_center = y + height - TOOL_WIDTH_MM / 2

    centers = []
    center = y + TOOL_WIDTH_MM / 2
    while center < last_center - 1e-9:
        centers.append(center)
        center += stripe_step

    # Always include the final row so the bottom edge gets covered too.
    centers.append(last_center)
    return centers


def rectangle_to_commands(op: dict) -> list[dict]:
    """Convert one paint_rectangle operation into boustrophedon stripe strokes."""
    label = op["label"]
    color = op["color"]

    x = op["x_mm"] + EDGE_INSET_MM
    y = op["y_mm"] + EDGE_INSET_MM
    width = op["width_mm"] - 2 * EDGE_INSET_MM
    height = op["height_mm"] - 2 * EDGE_INSET_MM

    if width <= 0 or height <= 0:
        print(f"Skipping {label}: too small to paint after edge inset")
        return []

    row_centers = compute_stripe_row_centers(y, height)
    left_x = x
    right_x = x + width

    commands = [select_tool(color, label), dip_paint(color, label)]

    for row_index, row_y in enumerate(row_centers):
        # Boustrophedon pathing: alternate direction each row.
        if row_index % 2 == 0:
            start_x, end_x = left_x, right_x
        else:
            start_x, end_x = right_x, left_x

        row_label = f"{label}_row{row_index + 1}"
        commands.append(move_to(start_x, row_y, row_label))
        commands.append(lower_tool(row_label))
        commands.append(paint_stroke((start_x, row_y), (end_x, row_y), color, row_label))
        commands.append(lift_tool(row_label))

    return commands


def line_to_commands(op: dict) -> list[dict]:
    """Convert one paint_line operation into a single stroke."""
    label = op["label"]
    color = op["color"]
    from_point = op["from_mm"]
    to_point = op["to_mm"]

    return [
        select_tool(color, label),
        dip_paint(color, label),
        move_to(from_point[0], from_point[1], label),
        lower_tool(label),
        paint_stroke(from_point, to_point, color, label),
        lift_tool(label),
    ]


def build_commands(plan: dict) -> list[dict]:
    """
    Walk the painting plan's operations in order and convert each one into
    stroke commands. Operation order is preserved, so if the plan already
    has black grid lines last, the resulting commands do too.
    """
    commands = []
    for op in plan["operations"]:
        if op["operation"] == "paint_rectangle":
            commands.extend(rectangle_to_commands(op))
        elif op["operation"] == "paint_line":
            commands.extend(line_to_commands(op))
    return commands


def build_painting_paths(plan: dict, commands: list[dict]) -> dict:
    """Assemble the full painting_paths.json structure."""
    stroke_commands = [cmd for cmd in commands if cmd["command"] == "paint_stroke"]
    total_distance = sum(
        math.dist(cmd["from_mm"], cmd["to_mm"]) for cmd in stroke_commands
    )

    return {
        "project": plan["project"],
        "style": plan["style"],
        "version": "0.1",
        "source_file": str(PLAN_INPUT_FILE),
        "units": "mm",
        "canvas": plan["canvas"],
        "path_settings": {
            "tool_width_mm": TOOL_WIDTH_MM,
            "stroke_overlap_ratio": STROKE_OVERLAP_RATIO,
            "edge_inset_mm": EDGE_INSET_MM,
        },
        "commands": commands,
        "debug": {
            "num_commands": len(commands),
            "num_paint_stroke_commands": len(stroke_commands),
            "estimated_total_paint_distance_mm": round(total_distance, 2),
        },
    }


# --- SVG preview ---------------------------------------------------------

def render_svg(painting_paths: dict) -> str:
    """Draw every paint_stroke command as a colored line, for a quick visual check."""
    canvas = painting_paths["canvas"]
    width_mm = canvas["width_mm"]
    height_mm = canvas["height_mm"]
    width_in = canvas.get("width_in", width_mm / 25.4)
    height_in = canvas.get("height_in", height_mm / 25.4)

    elements = [f'<rect x="0" y="0" width="{width_mm}" height="{height_mm}" fill="white" />']

    # Track where the tool last was so we can draw a line for every travel
    # move (tool lifted, moving to the start of the next stroke). Without
    # this, back-to-back overlapping strokes just look like one solid
    # block and the actual path/order is invisible.
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
                f'stroke="{escape(cmd["color"])}" stroke-width="{TOOL_WIDTH_MM}" '
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


def main() -> None:
    OUTPUT_DIR.mkdir(exist_ok=True)

    plan = load_painting_plan(PLAN_INPUT_FILE)
    commands = build_commands(plan)
    painting_paths = build_painting_paths(plan, commands)

    with open(PATHS_OUTPUT_FILE, "w", encoding="utf-8") as file:
        json.dump(painting_paths, file, indent=2)
    print(f"Generated {PATHS_OUTPUT_FILE}")

    svg_content = render_svg(painting_paths)
    with open(SVG_OUTPUT_FILE, "w", encoding="utf-8") as file:
        file.write(svg_content)
    print(f"Generated {SVG_OUTPUT_FILE}")


if __name__ == "__main__":
    main()
