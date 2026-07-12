"""Validation helpers for painting_paths.json-style data.

This module is meant to be imported (e.g. by generate_painting_paths.py),
not run as a standalone CLI. It checks that generated path commands are
structurally correct, within canvas bounds, and safe enough for early
software/hardware review.
"""

import math

# Command types that validate_command knows how to check. Anything else
# is treated as unknown and produces a warning rather than an error.
KNOWN_COMMANDS = {
    "select_tool",
    "dip_paint",
    "move_to",
    "lower_tool",
    "paint_stroke",
    "lift_tool",
}


def is_number(value) -> bool:
    """Return True for int or float, but False for bool."""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def stroke_distance(from_point, to_point) -> float:
    """Return the Euclidean distance between two [x, y] points."""
    return math.dist(from_point, to_point)


def point_inside_canvas(point: list, canvas: dict) -> bool:
    """Return True if point is within [0, width] x [0, height] of canvas."""
    if not isinstance(point, (list, tuple)) or len(point) != 2:
        return False
    x, y = point
    if not is_number(x) or not is_number(y):
        return False

    width = canvas.get("width_mm")
    height = canvas.get("height_mm")
    if not is_number(width) or not is_number(height):
        return False

    return 0 <= x <= width and 0 <= y <= height


def validate_canvas(canvas: dict) -> tuple:
    """Validate canvas metadata. Returns (errors, warnings)."""
    errors = []
    warnings = []

    if not canvas:
        errors.append("Canvas is missing.")
        return errors, warnings

    width = canvas.get("width_mm")
    if not is_number(width) or width <= 0:
        errors.append(f"Canvas width_mm must be a positive number, got {width!r}.")

    height = canvas.get("height_mm")
    if not is_number(height) or height <= 0:
        errors.append(f"Canvas height_mm must be a positive number, got {height!r}.")

    origin = canvas.get("origin")
    if origin is None:
        warnings.append("Canvas origin is not specified.")
    elif origin != "top-left":
        warnings.append(f"Canvas origin is '{origin}', expected 'top-left'.")

    return errors, warnings


def _describe(command: dict, index: int) -> str:
    label = command.get("label")
    if label:
        return f"command #{index} ({label})"
    return f"command #{index}"


def validate_command(command: dict, index: int, canvas: dict) -> tuple:
    """Validate a single command's structure. Returns (errors, warnings)."""
    errors = []
    warnings = []

    if not isinstance(command, dict):
        errors.append(f"command #{index} is not a valid command object.")
        return errors, warnings

    command_type = command.get("command")
    if not command_type:
        errors.append(f"{_describe(command, index)} is missing a 'command' field.")
        return errors, warnings

    if command_type not in KNOWN_COMMANDS:
        warnings.append(f"{_describe(command, index)} has unknown command type '{command_type}'.")
        return errors, warnings

    desc = _describe(command, index)

    if command_type in ("select_tool", "dip_paint"):
        if not command.get("color"):
            warnings.append(f"{desc} ({command_type}) is missing a color field.")

    elif command_type == "move_to":
        x = command.get("x_mm")
        y = command.get("y_mm")
        if not is_number(x) or not is_number(y):
            errors.append(f"{desc} (move_to) must have numeric x_mm and y_mm.")
        elif not point_inside_canvas([x, y], canvas):
            # The robot physically travels to move_to targets, so an
            # out-of-bounds travel is as dangerous as an out-of-bounds stroke.
            errors.append(f"{desc} (move_to) [{x}, {y}] is outside canvas bounds.")

    elif command_type == "paint_stroke":
        if not command.get("color"):
            warnings.append(f"{desc} (paint_stroke) is missing a color field.")

        from_point = command.get("from_mm")
        to_point = command.get("to_mm")

        if from_point is None or to_point is None:
            errors.append(f"{desc} (paint_stroke) must have from_mm and to_mm.")
        else:
            valid_points = True
            for name, point in (("from_mm", from_point), ("to_mm", to_point)):
                if (
                    not isinstance(point, (list, tuple))
                    or len(point) != 2
                    or not all(is_number(v) for v in point)
                ):
                    errors.append(f"{desc} (paint_stroke) {name} must be a two-number list.")
                    valid_points = False

            if valid_points:
                if not point_inside_canvas(from_point, canvas):
                    errors.append(f"{desc} (paint_stroke) from_mm {from_point} is outside canvas bounds.")
                if not point_inside_canvas(to_point, canvas):
                    errors.append(f"{desc} (paint_stroke) to_mm {to_point} is outside canvas bounds.")

                if stroke_distance(from_point, to_point) == 0:
                    errors.append(f"{desc} (paint_stroke) has zero distance (from_mm equals to_mm).")

    # lower_tool and lift_tool need no coordinates or extra fields.

    return errors, warnings


def validate_painting_paths(painting_paths: dict) -> dict:
    """Validate a full painting_paths.json-style structure.

    Returns a dict of the form:
        {"passed": bool, "errors": [...], "warnings": [...]}
    """
    errors = []
    warnings = []

    canvas = painting_paths.get("canvas")
    canvas_errors, canvas_warnings = validate_canvas(canvas)
    errors.extend(canvas_errors)
    warnings.extend(canvas_warnings)

    commands = painting_paths.get("commands")
    if commands is None:
        errors.append("Painting paths are missing a 'commands' list.")
        commands = []
    elif not isinstance(commands, list):
        errors.append("'commands' must be a list.")
        commands = []

    for index, command in enumerate(commands):
        command_errors, command_warnings = validate_command(command, index, canvas or {})
        errors.extend(command_errors)
        warnings.extend(command_warnings)

    return {
        "passed": len(errors) == 0,
        "errors": errors,
        "warnings": warnings,
    }
