# This file is maintained byte-for-byte identical to the canonical source:
#   Image_Process/image_to_mondrian/path_ordering.py
"""Exact-overlap stroke joining and greedy nearest-neighbor travel ordering.

Shared by image routes that turn independent traced lines into an efficient,
continuous drawing sequence.
"""

import math

DEFAULT_HOME_POSITION = (0.0, 0.0)


def _endpoint_direction(points, endpoint_rank):
    """Vector pointing from one endpoint into the stroke."""
    endpoint = points[0] if endpoint_rank == 0 else points[-1]
    candidates = points[1:] if endpoint_rank == 0 else reversed(points[:-1])
    for point in candidates:
        vector = (point[0] - endpoint[0], point[1] - endpoint[1])
        if vector != (0, 0):
            return vector
    return None


def _join_turn_degrees(points_a, endpoint_a, points_b, endpoint_b):
    """Heading change needed to continue from stroke A into stroke B.

    Endpoint vectors point away from their shared junction, so a visually
    straight continuation has 180 degrees between vectors and zero turn.
    """
    vector_a = _endpoint_direction(points_a, endpoint_a)
    vector_b = _endpoint_direction(points_b, endpoint_b)
    if vector_a is None or vector_b is None:
        return math.inf
    magnitude = math.hypot(*vector_a) * math.hypot(*vector_b)
    cosine = (vector_a[0] * vector_b[0] + vector_a[1] * vector_b[1]) / magnitude
    angle = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
    return abs(180.0 - angle)


def join_overlapping_strokes(strokes_data, max_turn_degrees=60.0):
    """Join compatible open strokes whose endpoints exactly overlap.

    At each shared endpoint, the straightest available continuation is paired
    first. Each endpoint participates in at most one join, which carries the
    pen through ordinary junctions while leaving side branches for a separate
    deliberate stroke. No proximity threshold is used and no connector is
    inserted: endpoints at different coordinates are never joined.

    Returns (joined_strokes, num_joins), where num_joins is the resulting
    reduction in pen-down path count. The stroke representation remains
    (points, closed) so the result can be passed directly to order_strokes().
    """
    if not 0 <= max_turn_degrees <= 180:
        raise ValueError("max_turn_degrees must be between 0 and 180")

    endpoint_groups = {}
    for stroke_index, (points, closed) in enumerate(strokes_data):
        if closed or len(points) < 2:
            continue
        endpoint_groups.setdefault(tuple(points[0]), []).append((stroke_index, 0))
        endpoint_groups.setdefault(tuple(points[-1]), []).append((stroke_index, 1))

    links = {}
    for endpoints in endpoint_groups.values():
        candidates = []
        for left_index in range(len(endpoints)):
            stroke_a, endpoint_a = endpoints[left_index]
            for right_index in range(left_index + 1, len(endpoints)):
                stroke_b, endpoint_b = endpoints[right_index]
                if stroke_a == stroke_b:
                    continue
                turn = _join_turn_degrees(
                    strokes_data[stroke_a][0], endpoint_a,
                    strokes_data[stroke_b][0], endpoint_b,
                )
                if turn <= max_turn_degrees:
                    candidates.append(
                        (turn, stroke_a, endpoint_a, stroke_b, endpoint_b)
                    )

        for _, stroke_a, endpoint_a, stroke_b, endpoint_b in sorted(candidates):
            end_a = (stroke_a, endpoint_a)
            end_b = (stroke_b, endpoint_b)
            if end_a in links or end_b in links:
                continue
            links[end_a] = end_b
            links[end_b] = end_a

    remaining = set(range(len(strokes_data)))
    joined_strokes = []
    while remaining:
        seed = min(remaining)
        points, closed = strokes_data[seed]
        if closed or not any((seed, rank) in links for rank in (0, 1)):
            joined_strokes.append((list(points), closed))
            remaining.remove(seed)
            continue

        component = set()
        stack = [seed]
        while stack:
            stroke_index = stack.pop()
            if stroke_index in component:
                continue
            component.add(stroke_index)
            for endpoint_rank in (0, 1):
                linked = links.get((stroke_index, endpoint_rank))
                if linked is not None and linked[0] not in component:
                    stack.append(linked[0])

        free_endpoints = sorted(
            (stroke_index, endpoint_rank)
            for stroke_index in component
            for endpoint_rank in (0, 1)
            if (stroke_index, endpoint_rank) not in links
        )
        is_cycle = not free_endpoints
        current_index, entry_rank = (
            free_endpoints[0] if free_endpoints else (min(component), 0)
        )

        combined = []
        visited = set()
        while current_index not in visited:
            visited.add(current_index)
            current_points = list(strokes_data[current_index][0])
            if entry_rank == 1:
                current_points.reverse()

            if combined:
                # Links are created only for exactly equal endpoints.
                combined.extend(current_points[1:])
            else:
                combined.extend(current_points)

            exit_rank = 1 - entry_rank
            linked = links.get((current_index, exit_rank))
            if linked is None:
                break
            current_index, entry_rank = linked

        joined_strokes.append((combined, is_cycle))
        remaining -= component

    return joined_strokes, len(strokes_data) - len(joined_strokes)


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
