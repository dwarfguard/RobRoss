import argparse
import json
import math
import sys
from html import escape
from pathlib import Path

from config_loader import load_config
from path_optimizer import (
    optimize_polylines,
    polyline_length,
    serpentine_points,
    travel_distance,
)
from path_validation import validate_painting_paths

# This script does NOT generate a new random layout. It reads the layout
# already decided by mondrian_generator.py and converts it into concrete
# robot-style stroke paths.
#
# Stroke ordering is NOT the plan's creation order: operations are turned
# into continuous polylines (one serpentine per rectangle fill, chained
# touching lines) and reordered nearest-neighbor by path_optimizer.py, so
# the pen draws continuously and travels as little as possible. Only the
# fills-before-lines phase split is preserved (grid goes over fills).

DEFAULT_CONFIG_FILE = "configs/demo_v1_a4_pen.json"

# Preview-only settings (do not affect painting_paths.json, just the SVG).
# Strokes are drawn semi-transparent so overlapping stripes show up as a
# visibly darker band, and travel moves (tool lifted, moving between
# strokes) are drawn as thin dashed gray lines, so the boustrophedon
# pattern is actually visible instead of looking like one solid block.
STROKE_PREVIEW_OPACITY = 0.55
TRAVEL_LINE_COLOR = "#999999"
TRAVEL_LINE_WIDTH_MM = 1.0

# Animated-preview-only settings (do not affect painting_paths.json).
# The animation plays strokes in command order at these constant speeds,
# with a short pause for tool actions, so the pacing is watchable — these
# are visual pacing knobs, not robot motion parameters.
ANIMATION_PAINT_SPEED_MM_S = 50.0
ANIMATION_TRAVEL_SPEED_MM_S = 200.0
ANIMATION_TOOL_PAUSE_S = 0.15
ANIMATION_UNDERLAY_OPACITY = 0.12
ANIMATION_MARKER_COLOR = "#e63946"

# Used when a config doesn't define output.path_animation_svg_file (the
# field is optional so older/third-party configs keep working).
DEFAULT_PATH_ANIMATION_SVG_FILE = "path_animation.svg"


def load_painting_plan(path: Path) -> dict:
    """Load the painting plan JSON produced by mondrian_generator.py."""
    if not path.exists():
        raise FileNotFoundError(
            f"Could not find {path}. Run Image_Process/mondrian/mondrian_generator.py first (with the same --config)."
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


def paint_path(points, color: str, label: str) -> dict:
    """Continuous pen-down polyline through all points (enables curved
    lines later: sample the curve densely into points)."""
    return {
        "command": "paint_path",
        "label": label,
        "color": color,
        "points_mm": [[round(x, 2), round(y, 2)] for x, y in points],
    }


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


def rectangle_to_polyline(op: dict, path_settings: dict) -> dict | None:
    """
    Convert one paint_rectangle operation into a single continuous
    serpentine polyline: boustrophedon rows connected inside the fill, so
    the whole region paints without lifting the tool. Returns None when
    the rectangle is too small to paint after the edge inset.
    """
    label = op["label"]

    tool_width_mm = path_settings["tool_width_mm"]
    stroke_overlap_ratio = path_settings["stroke_overlap_ratio"]
    edge_inset_mm = path_settings["edge_inset_mm"]

    x = op["x_mm"] + edge_inset_mm
    y = op["y_mm"] + edge_inset_mm
    width = op["width_mm"] - 2 * edge_inset_mm
    height = op["height_mm"] - 2 * edge_inset_mm

    if width <= 0 or height <= 0:
        print(f"Skipping {label}: too small to paint after edge inset")
        return None

    row_centers = compute_stripe_row_centers(y, height, tool_width_mm, stroke_overlap_ratio)
    points = serpentine_points(row_centers, x, x + width)
    return {"points": points, "color": op["color"], "label": label}


def line_to_polyline(op: dict) -> dict:
    """Convert one paint_line operation into a two-point polyline."""
    return {
        "points": [tuple(op["from_mm"]), tuple(op["to_mm"])],
        "color": op["color"],
        "label": op["label"],
    }


def polyline_to_commands(poly: dict) -> list[dict]:
    """Emit the motion commands for one continuous polyline."""
    points = poly["points"]
    label = poly["label"]
    return [
        move_to(points[0][0], points[0][1], label),
        lower_tool(label),
        paint_path(points, poly["color"], label),
        lift_tool(label),
    ]


def rectangle_to_commands(op: dict, path_settings: dict) -> list[dict]:
    """Convert one paint_rectangle operation into boustrophedon stripe strokes.

    Legacy per-operation converter (one lift per row, plan order); the
    pipeline now uses rectangle_to_polyline + path_optimizer instead.
    """
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
    """Convert one paint_line operation into a single stroke.

    Legacy per-operation converter; the pipeline now uses
    line_to_polyline + path_optimizer instead.
    """
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
    Convert the painting plan's operations into optimized stroke commands.

    Two phases keep the painting semantics of the plan: all rectangle
    fills first, then all lines (the black grid paints over the fills).
    Within each phase, operations are grouped by color (one select_tool /
    dip_paint per group), turned into continuous polylines, chained where
    endpoints touch, and reordered nearest-neighbor — so the command
    order comes from travel optimization, not the subdivide creation
    order.
    """
    fill_polys = []
    line_polys = []
    for op in plan["operations"]:
        if op["operation"] == "paint_rectangle":
            poly = rectangle_to_polyline(op, path_settings)
            if poly is not None:
                fill_polys.append(poly)
        elif op["operation"] == "paint_line":
            line_polys.append(line_to_polyline(op))

    commands = []
    position = (0.0, 0.0)  # tool starts near the canvas origin

    for phase_polys in (fill_polys, line_polys):
        # Group by color, in order of first appearance in the plan.
        color_order = []
        by_color = {}
        for poly in phase_polys:
            if poly["color"] not in by_color:
                color_order.append(poly["color"])
                by_color[poly["color"]] = []
            by_color[poly["color"]].append(poly)

        for color in color_order:
            ordered, position = optimize_polylines(by_color[color], position)
            group_label = ordered[0]["label"]
            commands.append(select_tool(color, group_label))
            commands.append(dip_paint(color, group_label))
            for poly in ordered:
                commands.extend(polyline_to_commands(poly))

    return commands


def build_painting_paths(plan: dict, commands: list[dict], config: dict, config_path: Path, plan_path: Path) -> dict:
    """Assemble the full painting_paths.json structure."""
    stroke_commands = [cmd for cmd in commands if cmd["command"] == "paint_stroke"]
    path_commands = [cmd for cmd in commands if cmd["command"] == "paint_path"]
    total_distance = sum(
        math.dist(cmd["from_mm"], cmd["to_mm"]) for cmd in stroke_commands
    ) + sum(polyline_length(cmd["points_mm"]) for cmd in path_commands)

    # Lifted-travel distance actually incurred by the emitted command
    # order, and what the plan's raw creation order would have cost, so
    # the optimizer's effect is visible in the debug block.
    painted = [
        {"points": [tuple(cmd["from_mm"]), tuple(cmd["to_mm"])]}
        if cmd["command"] == "paint_stroke"
        else {"points": [tuple(p) for p in cmd["points_mm"]]}
        for cmd in commands
        if cmd["command"] in ("paint_stroke", "paint_path")
    ]
    total_travel = travel_distance(painted, (0.0, 0.0))

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
            "num_paint_path_commands": len(path_commands),
            "estimated_total_paint_distance_mm": round(total_distance, 2),
            "estimated_total_travel_distance_mm": round(total_travel, 2),
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

        elif cmd["command"] == "paint_path":
            points_attr = " ".join(f"{x},{y}" for x, y in cmd["points_mm"])
            elements.append(
                f'<polyline points="{points_attr}" fill="none" '
                f'stroke="{escape(cmd["color"])}" stroke-width="{painting_paths["path_settings"]["tool_width_mm"]}" '
                f'stroke-linecap="round" stroke-linejoin="round" '
                f'stroke-opacity="{STROKE_PREVIEW_OPACITY}" />'
            )
            last_point = tuple(cmd["points_mm"][-1])

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


# --- Animated SVG preview -------------------------------------------------

def build_animation_timeline(commands: list) -> tuple[list, float]:
    """
    Walk the commands in order and turn them into timed animation events.

    Returns (events, total_duration_s). Each event is a dict with a "type"
    ("place", "travel", "stroke", "lower_tool", "lift_tool", "dip_paint"),
    a "start" time in seconds, and — for motion events — "from"/"to" points,
    a "dur", and (strokes only) a "color".

    Only the command list and geometry matter here, so this works for any
    config profile: monochrome line-only paths and colored boustrophedon
    fills produce the same kinds of events.
    """
    events = []
    time_s = 0.0
    position = None

    for cmd in commands:
        name = cmd["command"]

        if name == "move_to":
            target = (cmd["x_mm"], cmd["y_mm"])
            if position is None:
                events.append({"type": "place", "at": target, "start": time_s})
            elif target != position:
                dur = max(math.dist(position, target) / ANIMATION_TRAVEL_SPEED_MM_S, 0.02)
                events.append({"type": "travel", "from": position, "to": target, "start": time_s, "dur": dur})
                time_s += dur
            position = target

        elif name in ("paint_stroke", "paint_path"):
            if name == "paint_stroke":
                points = [tuple(cmd["from_mm"]), tuple(cmd["to_mm"])]
            else:
                points = [tuple(p) for p in cmd["points_mm"]]
            start_pt = points[0]
            if position is None:
                events.append({"type": "place", "at": start_pt, "start": time_s})
            elif position != start_pt:
                # Current generators always move_to the stroke start first,
                # but stay robust for hand-written or future command lists.
                dur = max(math.dist(position, start_pt) / ANIMATION_TRAVEL_SPEED_MM_S, 0.02)
                events.append({"type": "travel", "from": position, "to": start_pt, "start": time_s, "dur": dur})
                time_s += dur
            # A paint_path animates as back-to-back per-segment strokes
            # with no pause in between: visually one continuous line.
            for seg_from, seg_to in zip(points, points[1:]):
                dur = max(math.dist(seg_from, seg_to) / ANIMATION_PAINT_SPEED_MM_S, 0.02)
                events.append({
                    "type": "stroke",
                    "from": seg_from,
                    "to": seg_to,
                    "color": cmd["color"],
                    "start": time_s,
                    "dur": dur,
                })
                time_s += dur
            position = points[-1]

        elif name in ("lower_tool", "lift_tool", "dip_paint"):
            events.append({"type": name, "start": time_s})
            time_s += ANIMATION_TOOL_PAUSE_S

        # select_tool costs no animation time.

    return events, time_s


def render_animated_svg(painting_paths: dict) -> str:
    """
    Render the commands as a self-contained animated SVG (SMIL, no
    JavaScript): strokes draw themselves in command order, travel moves
    appear as dashed gray lines, and a round marker follows the tool
    (solid while lowered, faded while lifted). A faint underlay of the
    finished artwork shows the target while it draws. Open in a web
    browser; reload the page to replay.
    """
    canvas = painting_paths["canvas"]
    width_mm = canvas["width_mm"]
    height_mm = canvas["height_mm"]
    width_in = canvas.get("width_in", width_mm / 25.4)
    height_in = canvas.get("height_in", height_mm / 25.4)
    tool_width = painting_paths["path_settings"]["tool_width_mm"]

    events, total_s = build_animation_timeline(painting_paths["commands"])
    strokes = [ev for ev in events if ev["type"] == "stroke"]

    elements = [f'<rect x="0" y="0" width="{width_mm}" height="{height_mm}" fill="white" />']

    # Faint underlay of the finished artwork, so the viewer can see the
    # target composition while the animated strokes draw over it.
    for ev in strokes:
        elements.append(
            f'<line x1="{ev["from"][0]}" y1="{ev["from"][1]}" '
            f'x2="{ev["to"][0]}" y2="{ev["to"][1]}" '
            f'stroke="{escape(ev["color"])}" stroke-width="{tool_width}" '
            f'stroke-linecap="round" stroke-opacity="{ANIMATION_UNDERLAY_OPACITY}" />'
        )

    for ev in events:
        start = round(ev["start"], 3)

        if ev["type"] == "stroke":
            length = round(max(math.dist(ev["from"], ev["to"]), 0.01), 2)
            elements.append(
                f'<line x1="{ev["from"][0]}" y1="{ev["from"][1]}" '
                f'x2="{ev["to"][0]}" y2="{ev["to"][1]}" '
                f'stroke="{escape(ev["color"])}" stroke-width="{tool_width}" '
                f'stroke-linecap="round" stroke-opacity="{STROKE_PREVIEW_OPACITY}" '
                f'stroke-dasharray="{length}" stroke-dashoffset="{length}">'
                f'<animate attributeName="stroke-dashoffset" from="{length}" to="0" '
                f'begin="{start}s" dur="{round(ev["dur"], 3)}s" fill="freeze" />'
                f'</line>'
            )

        elif ev["type"] == "travel":
            elements.append(
                f'<line x1="{ev["from"][0]}" y1="{ev["from"][1]}" '
                f'x2="{ev["to"][0]}" y2="{ev["to"][1]}" '
                f'stroke="{TRAVEL_LINE_COLOR}" stroke-width="{TRAVEL_LINE_WIDTH_MM}" '
                f'stroke-dasharray="4,3" opacity="0">'
                f'<set attributeName="opacity" to="1" begin="{start}s" fill="freeze" />'
                f'</line>'
            )

    # Tool marker on top: follows every motion event, solid while the tool
    # is lowered and faded while it travels lifted.
    motions = [ev for ev in events if ev["type"] in ("travel", "stroke")]
    if motions:
        first = next(ev for ev in events if ev["type"] in ("place", "travel", "stroke"))
        marker_x, marker_y = first["at"] if first["type"] == "place" else first["from"]
        marker_animations = []
        for ev in events:
            start = round(ev["start"], 3)
            if ev["type"] in ("travel", "stroke"):
                dur = round(ev["dur"], 3)
                for attr, axis in (("cx", 0), ("cy", 1)):
                    marker_animations.append(
                        f'<animate attributeName="{attr}" from="{ev["from"][axis]}" '
                        f'to="{ev["to"][axis]}" begin="{start}s" dur="{dur}s" fill="freeze" />'
                    )
            elif ev["type"] == "lower_tool":
                marker_animations.append(
                    f'<set attributeName="fill-opacity" to="0.9" begin="{start}s" fill="freeze" />'
                )
            elif ev["type"] == "lift_tool":
                marker_animations.append(
                    f'<set attributeName="fill-opacity" to="0.35" begin="{start}s" fill="freeze" />'
                )
        elements.append(
            f'<circle cx="{marker_x}" cy="{marker_y}" r="{max(tool_width, 2.0)}" '
            f'fill="{ANIMATION_MARKER_COLOR}" fill-opacity="0.35" '
            f'stroke="{ANIMATION_MARKER_COLOR}" stroke-width="0.6">'
            + "".join(marker_animations)
            + '</circle>'
        )

    svg_body = "\n  ".join(elements)

    return f'''<svg
  xmlns="http://www.w3.org/2000/svg"
  width="{width_in}in"
  height="{height_in}in"
  viewBox="0 0 {width_mm} {height_mm}"
>
  <title>Painting path animation (~{total_s:.0f} s) — open in a browser, reload to replay</title>
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
    animation_output_file = output_dir / (
        output.get("path_animation_svg_file") or DEFAULT_PATH_ANIMATION_SVG_FILE
    )

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

    animated_svg_content = render_animated_svg(painting_paths)
    with open(animation_output_file, "w", encoding="utf-8") as file:
        file.write(animated_svg_content)
    _, animation_duration_s = build_animation_timeline(commands)
    print(
        f"Generated {animation_output_file} "
        f"(~{animation_duration_s:.0f}s animation — open in a web browser, reload to replay)"
    )

    # Surface the validation result on the console and in the exit code,
    # so a failing file can't be mistaken for a good one. Output files are
    # still written above (even on failure) so they can be inspected.
    for warning in validation["warnings"]:
        print(f"Validation warning: {warning}")
    if validation["passed"]:
        print(f"Validation passed ({len(validation['warnings'])} warnings).")
    else:
        for error in validation["errors"]:
            print(f"Validation error: {error}")
        print(
            f"Validation FAILED with {len(validation['errors'])} error(s) — "
            f"do not send {paths_output_file} to the robot."
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
