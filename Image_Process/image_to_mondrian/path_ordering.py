# CANONICAL SOURCE for path_ordering — see file-top comment in the copies at
# Image_Process/gemini_mondrian/path_ordering.py, Image_Process/sketch/path_ordering.py,
# and Image_Process/line_art/path_ordering.py.
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
    point since either direction covers the same ground.

    Backed by a uniform-grid spatial hash so each "nearest remaining
    endpoint" query only touches nearby cells instead of rescanning every
    remaining stroke - roughly O(n) overall on evenly spread input rather
    than O(n^2), which matters once a busy image produces thousands of
    border contours. The output is identical to the original linear scan:
    the pick minimizes the tuple (distance, stroke_index, endpoint_rank),
    with endpoint_rank 0 = start (forward) and 1 = end (reversed), which
    reproduces the old scan's exact tie-breaking (ascending index, start
    before end)."""
    n = len(strokes_data)
    if n == 0:
        return []

    # Every stroke offers its start point (rank 0); an open stroke also
    # offers its end point (rank 1) - the option of drawing it reversed.
    endpoints = []  # (x, y, idx, rank)
    for idx, (points, closed) in enumerate(strokes_data):
        sx, sy = points[0]
        endpoints.append((sx, sy, idx, 0))
        if not closed:
            ex, ey = points[-1]
            endpoints.append((ex, ey, idx, 1))

    min_x = min(p[0] for p in endpoints)
    min_y = min(p[1] for p in endpoints)
    max_x = max(p[0] for p in endpoints)
    max_y = max(p[1] for p in endpoints)

    # Aim for ~1 endpoint per cell on evenly spread input; guard the
    # degenerate all-coincident / collinear cases where a span collapses.
    span = max(max_x - min_x, max_y - min_y)
    cell_size = span / math.sqrt(len(endpoints)) if span > 0 else 1.0

    def cell_of(x, y):
        return (int((x - min_x) // cell_size), int((y - min_y) // cell_size))

    grid = {}
    for x, y, idx, rank in endpoints:
        grid.setdefault(cell_of(x, y), {})[(idx, rank)] = (x, y)

    def drop_endpoint(point, idx, rank):
        cell = cell_of(point[0], point[1])
        bucket = grid[cell]
        del bucket[(idx, rank)]
        if not bucket:  # keep grid iteration proportional to live candidates
            del grid[cell]

    # Populated-cell bounds. Because cell_size = span / sqrt(#endpoints), this
    # box is only ever ~sqrt(#endpoints) cells across, so a ring search that
    # starts *inside* it always terminates within O(sqrt n) rings. The one way
    # to get unbounded empty-ring expansion is a start point far *outside* the
    # box - and only the initial home_position can be outside, since every
    # later pen position sits on a former endpoint (hence inside the box). That
    # outside case scans the remaining endpoints directly instead of walking
    # empty rings all the way out to a distant, tightly-clustered group.
    cols = [c for c, _ in grid]
    rows = [r for _, r in grid]
    gmin_col, gmax_col = min(cols), max(cols)
    gmin_row, gmax_row = min(rows), max(rows)

    def ring_cells(ccol, crow, radius):
        """Cells at Chebyshev distance exactly `radius` from (ccol, crow)."""
        if radius == 0:
            yield (ccol, crow)
            return
        for col in range(ccol - radius, ccol + radius + 1):
            yield (col, crow - radius)
            yield (col, crow + radius)
        for row in range(crow - radius + 1, crow + radius):
            yield (ccol - radius, row)
            yield (ccol + radius, row)

    def nearest_remaining(current):
        """(idx, rank) of the remaining endpoint minimizing
        (distance, idx, rank) from `current` - the exact selection the
        original linear scan makes (ascending index, start before end)."""
        ccol, crow = cell_of(current[0], current[1])
        best_key = best = None
        inside = gmin_col <= ccol <= gmax_col and gmin_row <= crow <= gmax_row
        if not inside:
            # Nothing nearby to exploit: scan every live endpoint once
            # (bounded by the number remaining) rather than expanding empty
            # rings from a distant start.
            for bucket in grid.values():
                for (idx, rank), (px, py) in bucket.items():
                    key = (math.dist(current, (px, py)), idx, rank)
                    if best_key is None or key < best_key:
                        best_key, best = key, (idx, rank)
            return best
        radius = 0
        while True:
            for cell in ring_cells(ccol, crow, radius):
                bucket = grid.get(cell)
                if not bucket:
                    continue
                for (idx, rank), (px, py) in bucket.items():
                    key = (math.dist(current, (px, py)), idx, rank)
                    if best_key is None or key < best_key:
                        best_key, best = key, (idx, rank)
            # After searching every cell within Chebyshev radius `radius`, any
            # unsearched candidate is at Euclidean distance >= radius *
            # cell_size; once that lower bound passes the best distance found,
            # no closer-or-tied candidate can remain. covers_all is the
            # belt-and-braces stop once the square spans the whole box.
            covers_all = (
                ccol - radius <= gmin_col and ccol + radius >= gmax_col and
                crow - radius <= gmin_row and crow + radius >= gmax_row
            )
            if best_key is not None and (radius * cell_size > best_key[0] or covers_all):
                return best
            if covers_all:  # whole grid searched; nothing left to find
                return best
            radius += 1

    ordered = []
    current = home_position
    for _ in range(n):
        best_idx, best_rank = nearest_remaining(current)
        points, closed = strokes_data[best_idx]
        # Drop both of the chosen stroke's endpoints from the grid.
        drop_endpoint(points[0], best_idx, 0)
        if not closed:
            drop_endpoint(points[-1], best_idx, 1)

        chosen = points if best_rank == 0 else list(reversed(points))
        ordered.append(chosen)
        current = chosen[-1]

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
