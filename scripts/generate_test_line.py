"""Generate a single straight test-line path file for the first hardware contact test.

The hardware checklist's first contact test is one 50 mm horizontal line.
This script emits that line as a painting_paths.json-format file, so the
very first pen stroke exercises the exact same file format and (future)
robot adapter path as the real generated artwork — instead of a
hand-entered line on the robot side.

Usage:
    python3 scripts/generate_test_line.py --config configs/demo_v1_a4_pen.json
    python3 scripts/generate_test_line.py --start 80 140 --end 130 140
"""

import argparse
import json
import math
import sys
from pathlib import Path

from config_loader import load_config
from generate_painting_paths import (
    dip_paint,
    lift_tool,
    lower_tool,
    move_to,
    paint_stroke,
    render_svg,
    select_tool,
)
from path_validation import validate_painting_paths

DEFAULT_CONFIG_FILE = "configs/demo_v1_a4_pen.json"

# Default line from docs/hardware-test-checklist.md section 9:
# a 50 mm horizontal line near the middle of A4 paper.
DEFAULT_START = (80.0, 140.0)
DEFAULT_END = (130.0, 140.0)

# Output names are fixed (not config fields) so the test-line files can
# never overwrite the real artwork outputs from the same config.
TEST_LINE_PATHS_FILE = "test_line_paths.json"
TEST_LINE_PREVIEW_SVG_FILE = "test_line_preview.svg"


def build_test_line_paths(config: dict, config_path: Path, start, end) -> dict:
    """Build a painting_paths.json-style dict containing one test stroke."""
    canvas = config["canvas"]
    path_generation = config["path_generation"]
    color = config["artwork"].get("line_color", "black")
    label = "test_line"

    commands = [
        select_tool(color, label),
        dip_paint(color, label),
        move_to(start[0], start[1], label),
        lower_tool(label),
        paint_stroke(start, end, color, label),
        lift_tool(label),
    ]

    return {
        "project": config.get("project", "R.O.B Ross"),
        "style": "test_line",
        "version": "0.1",
        "config": {
            "profile_name": config.get("profile_name"),
            "source_file": str(config_path),
        },
        "units": "mm",
        "canvas": {
            "width_mm": canvas["width_mm"],
            "height_mm": canvas["height_mm"],
            "width_in": round(canvas["width_mm"] / 25.4, 4),
            "height_in": round(canvas["height_mm"] / 25.4, 4),
            "margin_mm": canvas.get("margin_mm", 0.0),
            "origin": canvas["origin"],
        },
        "path_settings": {
            "tool_width_mm": path_generation["tool_width_mm"],
            "stroke_overlap_ratio": path_generation["stroke_overlap_ratio"],
            "edge_inset_mm": path_generation["edge_inset_mm"],
        },
        "commands": commands,
        "debug": {
            "num_commands": len(commands),
            "num_paint_stroke_commands": 1,
            "estimated_total_paint_distance_mm": round(math.dist(start, end), 2),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate a single straight test-line painting_paths file for the first hardware contact test."
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_FILE,
        help=f"Path to a pipeline config JSON file (default: {DEFAULT_CONFIG_FILE}).",
    )
    parser.add_argument(
        "--start",
        type=float,
        nargs=2,
        metavar=("X_MM", "Y_MM"),
        default=list(DEFAULT_START),
        help=f"Line start point in mm (default: {DEFAULT_START[0]} {DEFAULT_START[1]}).",
    )
    parser.add_argument(
        "--end",
        type=float,
        nargs=2,
        metavar=("X_MM", "Y_MM"),
        default=list(DEFAULT_END),
        help=f"Line end point in mm (default: {DEFAULT_END[0]} {DEFAULT_END[1]}).",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_config(config_path)

    output_dir = Path(config["output"]["directory"])
    output_dir.mkdir(exist_ok=True)
    paths_output_file = output_dir / TEST_LINE_PATHS_FILE
    svg_output_file = output_dir / TEST_LINE_PREVIEW_SVG_FILE

    start = tuple(args.start)
    end = tuple(args.end)
    painting_paths = build_test_line_paths(config, config_path, start, end)

    validation = validate_painting_paths(painting_paths)
    painting_paths["validation"] = validation

    with open(paths_output_file, "w", encoding="utf-8") as file:
        json.dump(painting_paths, file, indent=2)
    print(f"Generated {paths_output_file}")

    svg_content = render_svg(painting_paths)
    with open(svg_output_file, "w", encoding="utf-8") as file:
        file.write(svg_content)
    print(f"Generated {svg_output_file}")

    length = math.dist(start, end)
    print(f"Test line: {start} -> {end}, length {length:.1f} mm.")

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
