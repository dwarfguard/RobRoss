"""Greedy nearest-neighbor stroke ordering, shared by any script in this
folder that needs to sequence a set of independent strokes into a travel
order. Ported unchanged from the `raymond` branch's `scripts/path_ordering.py`.
"""

import math

DEFAULT_HOME_POSITION = (0.0, 0.0)


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
