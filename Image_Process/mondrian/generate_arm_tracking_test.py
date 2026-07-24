"""Generate staged paths for diagnosing canvas-normal tracking errors.

The fixtures isolate same-segment direction changes before exercising a
compact curve. They are separate files so each stage can be reviewed and run
independently on hardware.

Usage:
    python3 Image_Process/mondrian/generate_arm_tracking_test.py
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
    paint_path,
    render_svg,
    select_tool,
)
from path_validation import validate_painting_paths

DEFAULT_CONFIG_FILE = "configs/demo_v1_a4_pen.json"

DIRECTION_SHAPES = [
    ("plus_y_retract_same_segment", [(105.0, 85.0), (105.0, 115.0)]),
    ("minus_y_extend_same_segment", [(105.0, 115.0), (105.0, 85.0)]),
    ("plus_x_control_same_segment", [(90.0, 140.0), (120.0, 140.0)]),
    ("minus_x_control_same_segment", [(120.0, 140.0), (90.0, 140.0)]),
]

REVERSAL_SHAPES = [
    ("plus_y_then_minus_y", [(100.0, 170.0), (100.0, 200.0), (100.0, 170.0)]),
    ("minus_y_then_plus_y", [(110.0, 200.0), (110.0, 170.0), (110.0, 200.0)]),
]


def build_alternating_curve() -> list[tuple[float, float]]:
    """Return a compact +X curve that repeatedly alternates Y direction."""
    return [
        (
            80.0 + 50.0 * index / 40,
            245.0 + 10.0 * math.sin(4.0 * math.pi * index / 40),
        )
        for index in range(41)
    ]


def build_fixture(config: dict, config_path: Path, style: str, shapes) -> dict:
    """Build one independently executable painting-path fixture."""
    canvas = config["canvas"]
    path_generation = config["path_generation"]
    color = config["artwork"].get("line_color", "black")

    commands = [select_tool(color, style), dip_paint(color, style)]
    for label, points in shapes:
        commands.extend(
            [
                move_to(points[0][0], points[0][1], label),
                lower_tool(label),
                paint_path(points, color, label),
                lift_tool(label),
            ]
        )

    total_distance = sum(
        math.dist(start, end)
        for _, points in shapes
        for start, end in zip(points, points[1:])
    )

    return {
        "project": config.get("project", "R.O.B Ross"),
        "style": style,
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
            "num_paint_path_commands": len(shapes),
            "num_path_points": sum(len(points) for _, points in shapes),
            "estimated_total_paint_distance_mm": round(total_distance, 2),
        },
    }


def build_arm_tracking_tests(config: dict, config_path: Path) -> dict[str, dict]:
    """Return the three fixtures keyed by their output filename stem."""
    return {
        "arm_tracking_direction_test": build_fixture(
            config, config_path, "arm_tracking_direction_test", DIRECTION_SHAPES
        ),
        "arm_tracking_reversal_test": build_fixture(
            config, config_path, "arm_tracking_reversal_test", REVERSAL_SHAPES
        ),
        "arm_tracking_curve_test": build_fixture(
            config,
            config_path,
            "arm_tracking_curve_test",
            [("plus_x_alternating_y_curve", build_alternating_curve())],
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate staged paths for diagnosing arm tracking on paper."
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_FILE,
        help=f"Path to a pipeline config JSON file (default: {DEFAULT_CONFIG_FILE}).",
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    config = load_config(config_path)
    output_dir = Path(config["output"]["directory"])
    output_dir.mkdir(exist_ok=True)

    failed = False
    for stem, painting_paths in build_arm_tracking_tests(config, config_path).items():
        validation = validate_painting_paths(painting_paths)
        painting_paths["validation"] = validation
        paths_output_file = output_dir / f"{stem}_paths.json"
        svg_output_file = output_dir / f"{stem}_preview.svg"

        with open(paths_output_file, "w", encoding="utf-8") as file:
            json.dump(painting_paths, file, indent=2)
        with open(svg_output_file, "w", encoding="utf-8") as file:
            file.write(render_svg(painting_paths))

        debug = painting_paths["debug"]
        print(
            f"Generated {paths_output_file} and {svg_output_file}: "
            f"{debug['num_paint_path_commands']} paths, "
            f"{debug['estimated_total_paint_distance_mm']:.1f} mm painted."
        )
        for warning in validation["warnings"]:
            print(f"Validation warning: {warning}")
        if not validation["passed"]:
            failed = True
            for error in validation["errors"]:
                print(f"Validation error: {error}")

    if failed:
        print("Validation FAILED; do not send the generated paths to the robot.")
        sys.exit(1)
    print("All arm-tracking fixtures passed validation.")


if __name__ == "__main__":
    main()
