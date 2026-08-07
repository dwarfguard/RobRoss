"""Generate deterministic curved-line paths for staged hardware testing.

The test card exercises one smooth S-curve, a closed circle, a sinusoidal
squiggle, and a polyline with sharp corners. Each shape is a separate
continuous ``paint_path`` so failures can be associated with one geometry.

Usage:
    python3 Image_Process/mondrian/generate_curve_test.py
    python3 Image_Process/mondrian/generate_curve_test.py --config configs/demo_v1_a4_pen.json
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
CURVE_TEST_PATHS_FILE = "curve_test_paths.json"
CURVE_TEST_PREVIEW_SVG_FILE = "curve_test_preview.svg"

CIRCLE_CENTER = (55.0, 120.0)
CIRCLE_RADIUS_MM = 28.0
SHARP_CORNER_POINTS = [
    (20.0, 180.0),
    (55.0, 180.0),
    (55.0, 215.0),
    (85.0, 185.0),
    (115.0, 215.0),
    (145.0, 165.0),
    (160.0, 215.0),
    (190.0, 190.0),
]


def sample_cubic_bezier(start, control_1, control_2, end, segments: int) -> list[tuple[float, float]]:
    """Sample a cubic Bezier curve, including both endpoints."""
    points = []
    for index in range(segments + 1):
        t = index / segments
        one_minus_t = 1.0 - t
        x = (
            one_minus_t**3 * start[0]
            + 3.0 * one_minus_t**2 * t * control_1[0]
            + 3.0 * one_minus_t * t**2 * control_2[0]
            + t**3 * end[0]
        )
        y = (
            one_minus_t**3 * start[1]
            + 3.0 * one_minus_t**2 * t * control_1[1]
            + 3.0 * one_minus_t * t**2 * control_2[1]
            + t**3 * end[1]
        )
        points.append((x, y))
    return points


def build_curve_test_shapes() -> list[tuple[str, list[tuple[float, float]]]]:
    """Return the fixed A4 test-card geometry in execution order."""
    smooth_s_curve = sample_cubic_bezier(
        (20.0, 45.0),
        (65.0, 10.0),
        (145.0, 80.0),
        (190.0, 45.0),
        segments=60,
    )

    circle = [
        (
            CIRCLE_CENTER[0] + CIRCLE_RADIUS_MM * math.cos(2.0 * math.pi * index / 48),
            CIRCLE_CENTER[1] + CIRCLE_RADIUS_MM * math.sin(2.0 * math.pi * index / 48),
        )
        for index in range(49)
    ]

    squiggle = [
        (
            100.0 + 90.0 * index / 96,
            120.0 + 24.0 * math.sin(5.0 * math.pi * index / 96),
        )
        for index in range(97)
    ]

    return [
        ("smooth_s_curve", smooth_s_curve),
        ("closed_circle", circle),
        ("sine_squiggle", squiggle),
        ("sharp_corners", SHARP_CORNER_POINTS),
    ]


def build_curve_test_paths(config: dict, config_path: Path) -> dict:
    """Build a painting_paths-style dict containing the curve test card."""
    canvas = config["canvas"]
    path_generation = config["path_generation"]
    color = config["artwork"].get("line_color", "black")
    shapes = build_curve_test_shapes()

    commands = [
        select_tool(color, "curve_test"),
        dip_paint(color, "curve_test"),
    ]
    for label, points in shapes:
        commands.extend(
            [
                move_to(points[0][0], points[0][1], label),
                lower_tool(label),
                paint_path(points, color, label),
                lift_tool(label),
            ]
        )

    path_commands = [command for command in commands if command["command"] == "paint_path"]
    total_distance = sum(
        math.dist(start, end)
        for command in path_commands
        for start, end in zip(command["points_mm"], command["points_mm"][1:])
    )

    return {
        "project": config.get("project", "R.O.B Ross"),
        "style": "curve_test",
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
            "num_paint_path_commands": len(path_commands),
            "num_path_points": sum(len(command["points_mm"]) for command in path_commands),
            "estimated_total_paint_distance_mm": round(total_distance, 2),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate deterministic curved-line painting paths for staged hardware testing."
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
    paths_output_file = output_dir / CURVE_TEST_PATHS_FILE
    svg_output_file = output_dir / CURVE_TEST_PREVIEW_SVG_FILE

    painting_paths = build_curve_test_paths(config, config_path)
    validation = validate_painting_paths(painting_paths)
    painting_paths["validation"] = validation

    with open(paths_output_file, "w", encoding="utf-8") as file:
        json.dump(painting_paths, file, indent=2)
    print(f"Generated {paths_output_file}")

    with open(svg_output_file, "w", encoding="utf-8") as file:
        file.write(render_svg(painting_paths))
    print(f"Generated {svg_output_file}")

    debug = painting_paths["debug"]
    print(
        f"Curve test: {debug['num_paint_path_commands']} paths, "
        f"{debug['num_path_points']} points, "
        f"{debug['estimated_total_paint_distance_mm']:.1f} mm painted."
    )

    for warning in validation["warnings"]:
        print(f"Validation warning: {warning}")
    if validation["passed"]:
        print(f"Validation passed ({len(validation['warnings'])} warnings).")
    else:
        for error in validation["errors"]:
            print(f"Validation error: {error}")
        print(
            f"Validation FAILED with {len(validation['errors'])} error(s); "
            f"do not send {paths_output_file} to the robot."
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
