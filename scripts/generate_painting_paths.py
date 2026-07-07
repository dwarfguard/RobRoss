import argparse
import json
import math
from html import escape
from pathlib import Path

from config_loader import load_config
from path_validation import validate_painting_paths

# This script does NOT generate a new random layout. It reads the layout
# already decided by mondrian_generator.py and converts it into concrete
# robot-style stroke paths.

DEFAULT_CONFIG_FILE = "configs/demo_v1_a4_pen.json"

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
            f"Could not find {path}. Run scripts/mondrian_generator.py first (with the same --config)."
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

def compute_stripe_row_centers(y: float, height: float, tool_width_mm: float, stroke_overlap_ratio: float) -> list[float]:
    """
    Return the y-position of each horizontal stripe center needed to fill
    `height`, spaced so each stripe overlaps the previous one by
    stroke_overlap_ratio.
    """
    if height <= tool_width_mm:
        # Rectangle is too short for more than one stripe; paint one
        # stroke straight down the middle.
        return [y + height / 2]

    stripe_step = tool_width_mm * (1 - stroke_overlap_ratio)
    last_center = y + height - tool_width_mm / 2

    centers = []
    center = y + tool_width_mm / 2
    while center < last_center - 1e-9:
        centers.append(center)
        center += stripe_step

    # Always include the final row so the bottom edge gets covered too.
    centers.append(last_center)
    return centers


def rectangle_to_commands(op: dict, path_settings: dict) -> list[dict]:
    """Convert one paint_rectangle operation into boustrophedon stripe strokes."""
    label = op["label"]
    color = op["color"]

    tool_width_mm = path_settings["tool_width_mm"]
    stroke_overlap_ratio = path_settings["stroke_overlap_ratio"]
    edge_inset_mm = path_settings["edge_inset_mm"]

    x = op["x_mm"] + edge_inset_mm
    y = op["y_mm"] + edge_inset_mm
    width = op["width_mm"] - 2 * edge_inset_mm
    height = op["height_mm"] - 2 * edge_inset_mm

    if width <= 0 or height <= 0:
        print(f"Skipping {label}: too small to paint after edge inset")
        return []

    row_centers = compute_stripe_row_centers(y, height, tool_width_mm, stroke_overlap_ratio)
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


def build_commands(plan: dict, path_settings: dict) -> list[dict]:
    """
    Walk the painting plan's operations in order and convert each one into
    stroke commands. Operation order is preserved, so if the plan already
    has black grid lines last, the resulting commands do too.
    """
    commands = []
    for op in plan["operations"]:
        if op["operation"] == "paint_rectangle":
            commands.extend(rectangle_to_commands(op, path_settings))
        elif op["operation"] == "paint_line":
            commands.extend(line_to_commands(op))
    return commands


def build_painting_paths(plan: dict, commands: list[dict], config: dict, config_path: Path, plan_path: Path) -> dict:
    """Assemble the full painting_paths.json structure."""
    stroke_commands = [cmd for cmd in commands if cmd["command"] == "paint_stroke"]
    total_distance = sum(
        math.dist(cmd["from_mm"], cmd["to_mm"]) for cmd in stroke_commands
    )

    def count_commands(command_name: str) -> int:
        return sum(1 for cmd in commands if cmd["command"] == command_name)

    def count_travel_only_move_to() -> int:
        # A move_to is "just travelling" if it is not immediately followed by
        # lower_tool (i.e. it isn't positioning the tool to start a stroke).
        count = 0
        for index, cmd in enumerate(commands):
            if cmd["command"] != "move_to":
                continue
            next_cmd = commands[index + 1] if index + 1 < len(commands) else None
            if next_cmd is None or next_cmd["command"] != "lower_tool":
                count += 1
        return count

    num_fill_regions = sum(1 for op in plan["operations"] if op["operation"] == "paint_rectangle")
    num_grid_lines = sum(1 for op in plan["operations"] if op["operation"] == "paint_line")

    path_generation = config["path_generation"]

    return {
        "project": plan["project"],
        "style": plan["style"],
        "version": "0.1",
        "config": {
            "profile_name": config.get("profile_name"),
            "source_file": str(config_path),
        },
        "source_file": str(plan_path),
        "units": "mm",
        "canvas": plan["canvas"],
        "path_settings": {
            "tool_width_mm": path_generation["tool_width_mm"],
            "stroke_overlap_ratio": path_generation["stroke_overlap_ratio"],
            "edge_inset_mm": path_generation["edge_inset_mm"],
        },
        "commands": commands,
        "debug": {
            "num_commands": len(commands),
            "num_paint_stroke_commands": len(stroke_commands),
            "estimated_total_paint_distance_mm": round(total_distance, 2),
            "num_fill_regions": num_fill_regions,
            "num_grid_lines": num_grid_lines,
            "num_select_tool_commands": count_commands("select_tool"),
            "num_lift_tool_commands": count_commands("lift_tool"),
            "num_lower_tool_commands": count_commands("lower_tool"),
            "num_dip_paint_commands": count_commands("dip_paint"),
            "num_move_to_commands": count_travel_only_move_to(),
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


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert a painting_plan.json into robot-style stroke path commands."
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_FILE,
        help=f"Path to a pipeline config JSON file (default: {DEFAULT_CONFIG_FILE}).",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_config(config_path)

    output = config["output"]
    output_dir = Path(output["directory"])
    plan_input_file = output_dir / output["painting_plan_file"]
    paths_output_file = output_dir / output["painting_paths_file"]
    svg_output_file = output_dir / output["path_preview_svg_file"]

    output_dir.mkdir(exist_ok=True)

    plan = load_painting_plan(plan_input_file)
    commands = build_commands(plan, config["path_generation"])
    painting_paths = build_painting_paths(plan, commands, config, config_path, plan_input_file)

    validation = validate_painting_paths(painting_paths)
    painting_paths["validation"] = validation

    with open(paths_output_file, "w", encoding="utf-8") as file:
        json.dump(painting_paths, file, indent=2)
    print(f"Generated {paths_output_file}")

    svg_content = render_svg(painting_paths)
    with open(svg_output_file, "w", encoding="utf-8") as file:
        file.write(svg_content)
    print(f"Generated {svg_output_file}")


if __name__ == "__main__":
    main()
