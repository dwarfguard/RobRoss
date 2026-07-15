"""Stroke ordering / chaining optimizer for painting path generation.

Turns the painting plan's operations into continuous pen-down polylines and
reorders them to minimize lifted travel, instead of painting in the order
(and one-stroke-per-lift granularity) the subdivide creation algorithm
happened to produce:

- A rectangle fill becomes ONE serpentine polyline (boustrophedon rows
  connected by short edge segments inside the fill), so the whole region
  is painted without lifting the tool.
- Lines that share an endpoint (within a tolerance) are chained into one
  polyline, so e.g. the four border lines become a single closed loop.
- Polylines are then ordered greedy nearest-neighbor, allowing each
  polyline to be drawn from either end, so the tool always continues from
  wherever it finished rather than jumping in creation order.

A polyline is a dict: {"points": [(x, y), ...], "color": str, "label": str}.
Curved lines can use the same representation later by sampling the curve
densely into polyline points.
"""

import math

# Two endpoints are considered "the same point" (chainable without lifting)
# if they are closer than this. Half a typical pen width: a join this small
# is covered by the stroke itself.
DEFAULT_CHAIN_TOLERANCE_MM = 0.5


def serpentine_points(row_centers: list, left_x: float, right_x: float) -> list:
    """
    Return the corner points of a continuous boustrophedon zigzag covering
    the given stripe rows: left-to-right on the first row, back on the
    next, connected down the shared edge in between.
    """
    points = []
    for index, row_y in enumerate(row_centers):
        if index % 2 == 0:
            points.extend([(left_x, row_y), (right_x, row_y)])
        else:
            points.extend([(right_x, row_y), (left_x, row_y)])
    return points


def polyline_length(points: list) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def _close(a, b, tol: float) -> bool:
    return math.dist(a, b) <= tol


def chain_polylines(polylines: list, tol: float = DEFAULT_CHAIN_TOLERANCE_MM) -> list:
    """
    Greedily merge same-color polylines whose endpoints coincide (within
    tol) into single continuous polylines, reversing candidates as needed.
    Only endpoint-to-endpoint contact chains; a line ending on another
    line's middle (T junction) still needs its own lift.
    """
    unused = [dict(p, points=list(p["points"])) for p in polylines]
    chains = []

    while unused:
        chain = unused.pop(0)
        points = chain["points"]
        labels = [chain["label"]]

        extended = True
        while extended:
            extended = False
            for index, candidate in enumerate(unused):
                if candidate["color"] != chain["color"]:
                    continue
                cpts = candidate["points"]
                if _close(points[-1], cpts[0], tol):
                    points = points + cpts[1:]
                elif _close(points[-1], cpts[-1], tol):
                    points = points + list(reversed(cpts[:-1]))
                elif _close(points[0], cpts[-1], tol):
                    points = cpts[:-1] + points
                elif _close(points[0], cpts[0], tol):
                    points = list(reversed(cpts))[:-1] + points
                else:
                    continue
                labels.append(candidate["label"])
                unused.pop(index)
                extended = True
                break

        chains.append({
            "points": points,
            "color": chain["color"],
            "label": "+".join(labels),
        })

    return chains


def order_polylines(polylines: list, start_point) -> tuple:
    """
    Greedy nearest-neighbor ordering: repeatedly pick the unpainted
    polyline whose nearest end is closest to the current tool position,
    reversing it when its far end is the closer one.

    Returns (ordered_polylines, end_point) so a caller can carry the tool
    position across color groups.
    """
    remaining = list(polylines)
    ordered = []
    position = tuple(start_point)

    while remaining:
        best_index = None
        best_reversed = False
        best_dist = math.inf
        for index, poly in enumerate(remaining):
            d_start = math.dist(position, poly["points"][0])
            d_end = math.dist(position, poly["points"][-1])
            if d_start < best_dist:
                best_index, best_reversed, best_dist = index, False, d_start
            if d_end < best_dist:
                best_index, best_reversed, best_dist = index, True, d_end

        poly = remaining.pop(best_index)
        if best_reversed:
            poly = dict(poly, points=list(reversed(poly["points"])))
        ordered.append(poly)
        position = tuple(poly["points"][-1])

    return ordered, position


def optimize_polylines(polylines: list, start_point,
                       tol: float = DEFAULT_CHAIN_TOLERANCE_MM) -> tuple:
    """Chain endpoint-touching polylines, then order them nearest-neighbor."""
    return order_polylines(chain_polylines(polylines, tol), start_point)


def travel_distance(polylines: list, start_point) -> float:
    """Total lifted-travel distance for painting polylines in list order."""
    total = 0.0
    position = tuple(start_point)
    for poly in polylines:
        total += math.dist(position, poly["points"][0])
        position = tuple(poly["points"][-1])
    return total
