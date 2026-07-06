import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import canny  # noqa: E402  (reruns edge detection/skeletonization, gives us `strokes` + `simplify`)
from path_ordering import order_strokes, total_travel_distance  # noqa: E402

# Canvas physical size/origin are still TBD from the hardware side - keep them
# as parameters so nothing here is hard-coded to a guessed number.
DEFAULT_CANVAS_SIZE = (300.0, 300.0)  # placeholder (width, height), unit TBD
DEFAULT_HOME_POSITION = (0.0, 0.0)

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'output')
DEBUG_OUTPUT_PATH = os.path.join(OUTPUT_DIR, 'robot_path_preview.json')


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


def build_robot_path(canvas_size=DEFAULT_CANVAS_SIZE, origin="top-left",
                      home_position=DEFAULT_HOME_POSITION):
    """Returns list[list[(x, y)]]: one continuous pen-down waypoint sequence
    per stroke, ordered to minimize pen-up travel, scaled onto canvas_size."""
    img_h, img_w = canny.skeleton.shape
    image_size = (img_w, img_h)

    strokes_data = []
    for points_yx, closed in canny.strokes:
        simplified_xy = canny.simplify(points_yx, closed)
        canvas_points = map_to_canvas(simplified_xy, image_size, canvas_size, origin)
        strokes_data.append((canvas_points, closed))

    return order_strokes(strokes_data, home_position)


if __name__ == "__main__":
    img_h, img_w = canny.skeleton.shape
    image_size = (img_w, img_h)

    strokes_data = []
    for points_yx, closed in canny.strokes:
        simplified_xy = canny.simplify(points_yx, closed)
        canvas_points = map_to_canvas(simplified_xy, image_size, DEFAULT_CANVAS_SIZE, "top-left")
        strokes_data.append((canvas_points, closed))

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

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    with open(DEBUG_OUTPUT_PATH, 'w') as f:
        json.dump({"canvas_size": DEFAULT_CANVAS_SIZE, "strokes": ordered}, f)
    print(f"wrote {DEBUG_OUTPUT_PATH}")
