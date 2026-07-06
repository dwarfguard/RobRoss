import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import canny  # noqa: E402  (reruns edge detection/skeletonization, gives us `strokes` + `simplify`)

# Canvas physical size/origin are still TBD from the hardware side - keep them
# as parameters so nothing here is hard-coded to a guessed number.
DEFAULT_CANVAS_SIZE = (300.0, 300.0)  # placeholder (width, height), unit TBD
DEFAULT_HOME_POSITION = (0.0, 0.0)


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


def order_strokes(strokes_data, home_position=DEFAULT_HOME_POSITION):
    """Greedy nearest-neighbor ordering to cut down pen-up travel. Open
    strokes may be walked in reverse if that end is closer to the pen's
    current position; closed strokes (loops) keep their existing start
    point since either direction covers the same ground."""
    remaining = list(range(len(strokes_data)))
    ordered = []
    current = home_position
    while remaining:
        best_idx = best_reverse = None
        best_dist = math.inf
        for idx in remaining:
            points, closed = strokes_data[idx]
            d_start = math.dist(current, points[0])
            if d_start < best_dist:
                best_dist, best_idx, best_reverse = d_start, idx, False
            if not closed:
                d_end = math.dist(current, points[-1])
                if d_end < best_dist:
                    best_dist, best_idx, best_reverse = d_end, idx, True
        points, _ = strokes_data[best_idx]
        if best_reverse:
            points = list(reversed(points))
        ordered.append(points)
        current = points[-1]
        remaining.remove(best_idx)
    return ordered


def total_travel_distance(strokes_points, home_position=DEFAULT_HOME_POSITION):
    """Sum of pen-up jumps between the end of one stroke and the start of
    the next, in stroke-list order."""
    total = 0.0
    current = home_position
    for points in strokes_points:
        total += math.dist(current, points[0])
        current = points[-1]
    return total


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
