"""Image -> centerline strokes via Canny edge detection + skeletonization.

Ported from the `raymond` branch's `scripts/canny.py`. The original script
ran this logic at import time against a hardcoded image path; here it's
wrapped in `extract_strokes()` so the caller supplies the image path and
Canny thresholds (from a config file) instead.

`cv2.findContours` traces the boundary of each stroke as a thin blob,
walking down one side and back up the other - that mirrored return trip
is a duplicate line. Skeletonizing first collapses every stroke to a
single pixel-wide centerline, then the pixel graph is walked directly so
each stroke is emitted once.
"""

import cv2
import numpy as np
from skimage.morphology import skeletonize

NEIGHBORS_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def extract_strokes(image_path, threshold1: float, threshold2: float):
    """Run Canny edge detection + skeletonization on `image_path`.

    Returns (strokes, image_size) where `strokes` is a list of
    (points_xy, closed) - one entry per independent traced line, points as
    (x, y) pixel coordinates - and `image_size` is (width, height).
    """
    img = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")

    edges = cv2.Canny(img, threshold1=threshold1, threshold2=threshold2)
    skeleton = skeletonize(edges > 0)

    all_pixels = set(map(tuple, np.argwhere(skeleton)))

    def neighbors(pixel):
        y, x = pixel
        return [(y + dy, x + dx) for dy, dx in NEIGHBORS_8 if (y + dy, x + dx) in all_pixels]

    degree = {p: len(neighbors(p)) for p in all_pixels}
    nodes = {p for p, d in degree.items() if d != 2}  # endpoints (deg 1) and junctions (deg >= 3)

    visited_edges = set()
    strokes_yx = []  # list of (points_yx, is_closed)

    def walk(start, first_step):
        path = [start, first_step]
        visited_edges.add(frozenset((start, first_step)))
        prev, current = start, first_step
        while degree.get(current) == 2 and current not in nodes:
            nxt = next((p for p in neighbors(current) if p != prev), None)
            if nxt is None or frozenset((current, nxt)) in visited_edges:
                break
            visited_edges.add(frozenset((current, nxt)))
            path.append(nxt)
            prev, current = current, nxt
        return path

    # Open branches: walk from every junction/endpoint out to the next node.
    for node in nodes:
        for nbr in neighbors(node):
            if frozenset((node, nbr)) not in visited_edges:
                strokes_yx.append((walk(node, nbr), False))

    # Closed loops made entirely of degree-2 pixels (no junctions/endpoints at all).
    for pixel in all_pixels - nodes:
        unvisited = [p for p in neighbors(pixel) if frozenset((pixel, p)) not in visited_edges]
        if unvisited:
            strokes_yx.append((walk(pixel, unvisited[0]), True))

    strokes_xy = [([(x, y) for y, x in points_yx], closed) for points_yx, closed in strokes_yx]
    height, width = skeleton.shape
    return strokes_xy, (width, height)


def simplify(points_xy, closed: bool, epsilon_ratio: float):
    """Reduce point count with Douglas-Peucker while keeping the stroke shape."""
    pts = np.array(points_xy, dtype=np.int32).reshape(-1, 1, 2)
    if len(pts) < 3:
        return [tuple(p) for p in pts.reshape(-1, 2)]
    epsilon = epsilon_ratio * cv2.arcLength(pts, closed)
    approx = cv2.approxPolyDP(pts, epsilon, closed)
    return [tuple(int(v) for v in p) for p in approx.reshape(-1, 2)]
