import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import canny  # noqa: E402  (reruns edge detection/skeletonization, gives us `strokes` + `simplify`)
from path_ordering import order_strokes, total_travel_distance  # noqa: E402
from path_validation import validate_robot_path  # noqa: E402

# Canvas physical size/origin are still TBD from the hardware side - keep them
# as parameters so nothing here is hard-coded to a guessed number.
DEFAULT_CANVAS_SIZE = (300.0, 300.0)  # placeholder (width, height), unit TBD
DEFAULT_HOME_POSITION = (0.0, 0.0)

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'output')
DEBUG_OUTPUT_PATH = os.path.join(OUTPUT_DIR, 'sketch_robot_path.json')


def _canvas_strokes_data(canvas_size, origin):
    """Simplified, canvas-scaled (points, closed) pairs for every stroke,
    still in canny.strokes order (i.e. not yet distance-ordered)."""
    img_h, img_w = canny.skeleton.shape
    image_size = (img_w, img_h)
    strokes_data = []
    for points_yx, closed in canny.strokes:
        simplified_xy = canny.simplify(points_yx, closed)
        canvas_points = map_to_canvas(simplified_xy, image_size, canvas_size, origin)
        strokes_data.append((canvas_points, closed))
    return strokes_data


def map_to_canvas(points_xy, image_size, canvas_size, origin="top-left"):
    """Scale pixel-space (x, y) points onto a physical canvas, preserving
    aspect ratio. `origin` controls whether the canvas Y axis points down
    (image convention, "top-left") or up ("bottom-left")."""
    img_w, img_h = image_size
    canvas_w, canvas_h = canvas_size
    scale = min(canvas_w / img_w, canvas_h / img_h)
    mapped = []
    for x, y in points_xy:
        cx = x * scale
        cy = y * scale
        if origin == "bottom-left":
            cy = canvas_h - cy
        mapped.append((cx, cy))
    return mapped


def build_sketch_robot_path(canvas_size=DEFAULT_CANVAS_SIZE, origin="top-left",
                             home_position=DEFAULT_HOME_POSITION):
    """Returns {"canvas_size": (w, h), "tools": [{"kind": "line", "color": "black",
    "strokes": list[list[(x, y)]]}]} - single monochrome pen tool, ordered to
    minimize pen-up travel, scaled onto canvas_size. Same top-level shape as
    mondrian_robot_path.build_mondrian_robot_path() so both routes' output is
    easy to consume the same way, even though this route only has one tool."""
    strokes_data = _canvas_strokes_data(canvas_size, origin)
    ordered = order_strokes(strokes_data, home_position)
    return {"canvas_size": canvas_size, "tools": [{"kind": "line", "color": "black", "strokes": ordered}]}


if __name__ == "__main__":
    strokes_data = _canvas_strokes_data(DEFAULT_CANVAS_SIZE, "top-left")

    baseline_distance = total_travel_distance(
        [points for points, _ in strokes_data], DEFAULT_HOME_POSITION
    )
    ordered = order_strokes(strokes_data, DEFAULT_HOME_POSITION)
    ordered_distance = total_travel_distance(ordered, DEFAULT_HOME_POSITION)

    total_points = sum(len(points) for points in ordered)
    print(f"strokes: {len(ordered)}")
    print(f"total points: {total_points}")
    print(f"baseline pen-up travel: {baseline_distance:.1f}")
    print(f"optimized pen-up travel: {ordered_distance:.1f}")
    print(f"reduction: {(1 - ordered_distance / baseline_distance) * 100:.1f}%")

    result = {"canvas_size": DEFAULT_CANVAS_SIZE, "tools": [{"kind": "line", "color": "black", "strokes": ordered}]}

    validation = validate_robot_path(result)
    result["validation"] = validation
    print(f"validation: passed={validation['passed']} "
          f"errors={len(validation['errors'])} warnings={len(validation['warnings'])}")
    for message in validation["errors"]:
        print(f"  ERROR: {message}")
    for message in validation["warnings"]:
        print(f"  WARNING: {message}")

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    with open(DEBUG_OUTPUT_PATH, 'w') as f:
        json.dump(result, f)
    print(f"wrote {DEBUG_OUTPUT_PATH}")
