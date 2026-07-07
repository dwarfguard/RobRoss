"""Validation helpers for this repo's robot path data - the
{"canvas_size"/"canvas_size_mm": ..., "tools": [{"kind", "color", "strokes"}]}
shape returned by both sketch_robot_path.build_sketch_robot_path() and
mondrian_robot_path.build_mondrian_robot_path().

This module is meant to be imported, not run as a standalone CLI. It
checks that a path result is structurally correct, within canvas bounds,
and free of degenerate (zero-length) strokes - a basic sanity check before
handing the path off to anything downstream.
"""

import math

KNOWN_KINDS = {"line", "fill"}


def is_number(value) -> bool:
    """Return True for int or float, but False for bool."""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def get_canvas_size(result: dict):
    """Return (width, height) from either canvas_size (w, h) or the square
    canvas_size_mm, or None if neither is present/valid."""
    if "canvas_size_mm" in result:
        size = result["canvas_size_mm"]
        return (size, size) if is_number(size) else None

    if "canvas_size" in result:
        size = result["canvas_size"]
        if isinstance(size, (list, tuple)) and len(size) == 2 and all(is_number(v) for v in size):
            return tuple(size)
        return None

    return None


def point_inside_canvas(point, width, height) -> bool:
    if not isinstance(point, (list, tuple)) or len(point) != 2:
        return False
    x, y = point
    if not is_number(x) or not is_number(y):
        return False
    return 0 <= x <= width and 0 <= y <= height


def validate_canvas(result: dict) -> tuple:
    """Validate canvas size metadata. Returns (errors, warnings)."""
    errors = []
    warnings = []

    if "canvas_size_mm" not in result and "canvas_size" not in result:
        errors.append("Result is missing 'canvas_size' or 'canvas_size_mm'.")
        return errors, warnings

    size = get_canvas_size(result)
    if size is None:
        errors.append("Canvas size is present but not a valid number/2-number pair.")
        return errors, warnings

    width, height = size
    if width <= 0 or height <= 0:
        errors.append(f"Canvas size must be positive, got ({width!r}, {height!r}).")

    return errors, warnings


def _describe(tool_index: int, tool: dict, stroke_index: int = None) -> str:
    kind = tool.get("kind", "?") if isinstance(tool, dict) else "?"
    color = tool.get("color", "?") if isinstance(tool, dict) else "?"
    base = f"tool #{tool_index} (kind={kind!r}, color={color!r})"
    if stroke_index is not None:
        return f"{base} stroke #{stroke_index}"
    return base


def validate_stroke(stroke, tool_index: int, tool: dict, stroke_index: int, width, height) -> tuple:
    """Validate a single stroke (list of (x, y) points). Returns (errors, warnings)."""
    errors = []
    warnings = []
    desc = _describe(tool_index, tool, stroke_index)

    if not isinstance(stroke, (list, tuple)):
        errors.append(f"{desc} is not a list of points.")
        return errors, warnings

    if len(stroke) < 2:
        errors.append(f"{desc} has fewer than 2 points ({len(stroke)}) - can't draw a real stroke.")
        return errors, warnings

    total_length = 0.0
    for point_index, point in enumerate(stroke):
        if not point_inside_canvas(point, width, height):
            errors.append(f"{desc} point #{point_index} {point!r} is outside canvas bounds "
                           f"([0, {width}] x [0, {height}]).")
        if point_index > 0:
            total_length += math.dist(stroke[point_index - 1], point)

    if total_length == 0:
        errors.append(f"{desc} is degenerate (zero total path length - all points coincide).")

    return errors, warnings


def validate_tool(tool: dict, tool_index: int, width, height) -> tuple:
    """Validate a single tool group's structure and its strokes. Returns (errors, warnings)."""
    errors = []
    warnings = []

    if not isinstance(tool, dict):
        errors.append(f"tool #{tool_index} is not a valid tool object.")
        return errors, warnings

    kind = tool.get("kind")
    if kind not in KNOWN_KINDS:
        warnings.append(f"tool #{tool_index} has unknown kind {kind!r} (expected one of {sorted(KNOWN_KINDS)}).")

    if not tool.get("color"):
        warnings.append(f"tool #{tool_index} is missing a color field.")

    strokes = tool.get("strokes")
    if not isinstance(strokes, list):
        errors.append(f"tool #{tool_index} ({kind}) 'strokes' must be a list.")
        return errors, warnings

    for stroke_index, stroke in enumerate(strokes):
        stroke_errors, stroke_warnings = validate_stroke(stroke, tool_index, tool, stroke_index, width, height)
        errors.extend(stroke_errors)
        warnings.extend(stroke_warnings)

    return errors, warnings


def validate_robot_path(result: dict) -> dict:
    """Validate a full robot path result (as returned by build_sketch_robot_path()
    or build_mondrian_robot_path()).

    Returns a dict of the form:
        {"passed": bool, "errors": [...], "warnings": [...]}
    """
    errors = []
    warnings = []

    canvas_errors, canvas_warnings = validate_canvas(result)
    errors.extend(canvas_errors)
    warnings.extend(canvas_warnings)

    size = get_canvas_size(result)
    width, height = size if size is not None else (math.inf, math.inf)

    tools = result.get("tools")
    if tools is None:
        errors.append("Result is missing a 'tools' list.")
        tools = []
    elif not isinstance(tools, list):
        errors.append("'tools' must be a list.")
        tools = []

    for tool_index, tool in enumerate(tools):
        tool_errors, tool_warnings = validate_tool(tool, tool_index, width, height)
        errors.extend(tool_errors)
        warnings.extend(tool_warnings)

    return {
        "passed": len(errors) == 0,
        "errors": errors,
        "warnings": warnings,
    }
