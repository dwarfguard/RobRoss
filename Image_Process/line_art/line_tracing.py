"""Image -> centerline strokes for already-clean line art, via threshold +
skeletonization (no Canny).

Canny finds gradient transitions, so a stroke with real width (not just a
1px antialiased hairline) has two of them - one where it starts (white to
black) and one where it ends (black to white). Skeletonizing a Canny edge
map therefore collapses each side of the stroke to its own centerline,
producing two parallel traced lines offset by roughly half the stroke
width, instead of one line down the middle. Thresholding the image
directly into a filled stroke mask and skeletonizing *that* converges to
one true centerline per stroke, because there's only one filled region to
collapse, not two edge curves.

The pixel-graph walk in extract_strokes() is ported from
Image_Process/sketch/canny_edges.py, which already treats its skeleton as
an opaque boolean array - it never assumed the skeleton came from Canny -
so the same walk works unchanged on a threshold-built skeleton.
"""

import math

import cv2
import numpy as np
from skimage.morphology import skeletonize

NEIGHBORS_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def load_grayscale(image_path):
    """Read `image_path` as grayscale, compositing RGBA onto white first.

    Handles both opaque line art (alpha uniformly 255, e.g. a plain PNG
    export) and line art on a transparent background (alpha varies) the
    same way - either becomes lines on a white background before
    thresholding.
    """
    img = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")

    if img.ndim == 3 and img.shape[2] == 4:
        bgr = img[:, :, :3].astype(np.float32)
        alpha = img[:, :, 3:4].astype(np.float32) / 255.0
        white = np.full_like(bgr, 255.0)
        composited = bgr * alpha + white * (1 - alpha)
        return cv2.cvtColor(composited.astype(np.uint8), cv2.COLOR_BGR2GRAY)
    if img.ndim == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


def binarize(image_path, threshold: float):
    """Return (mask, image_size) where mask is a boolean array, True for
    pixels darker than `threshold` (i.e. part of a drawn line), and
    image_size is (width, height)."""
    gray = load_grayscale(image_path)
    mask = gray < threshold
    height, width = mask.shape
    return mask, (width, height)


def skeletonize_mask(mask):
    """Thin a boolean stroke mask to single-pixel-wide centerlines."""
    return skeletonize(mask)


def extract_strokes(skeleton):
    """Walk a boolean skeleton's pixel graph into centerline chains.

    Returns a list of (points_xy, closed) - one entry per independent
    stroke, points as (x, y) pixel coordinates. Endpoints (degree 1) and
    junctions (degree >= 3) are graph nodes; runs of degree-2 pixels
    between them are edges, walked once each via `visited_edges`.
    """
    all_pixels = set(map(tuple, np.argwhere(skeleton)))

    def neighbors(pixel):
        y, x = pixel
        return [(y + dy, x + dx) for dy, dx in NEIGHBORS_8 if (y + dy, x + dx) in all_pixels]

    degree = {p: len(neighbors(p)) for p in all_pixels}
    nodes = {p for p, d in degree.items() if d != 2}

    visited_edges = set()
    strokes_yx = []

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

    return [([(x, y) for y, x in points_yx], closed) for points_yx, closed in strokes_yx]


def _polyline_length(points_xy) -> float:
    return sum(math.dist(a, b) for a, b in zip(points_xy, points_xy[1:]))


def prune_spurs(strokes, min_length_px: float):
    """Drop short open branches - skeletonization spurs at junctions and
    line-cap corners, which threshold+skeletonize is more prone to than
    skeletonizing a thin Canny edge map. Closed loops are kept regardless
    of length; only open (non-closed) short branches are spurs."""
    return [
        (points, closed)
        for points, closed in strokes
        if closed or _polyline_length(points) >= min_length_px
    ]


def simplify(points_xy, closed: bool, epsilon_ratio: float):
    """Reduce point count with Douglas-Peucker while keeping the stroke
    shape. Epsilon scales with each stroke's own arc length, so long
    strokes tolerate proportionally more simplification than short ones."""
    pts = np.array(points_xy, dtype=np.int32).reshape(-1, 1, 2)
    if len(pts) < 3:
        return [tuple(p) for p in pts.reshape(-1, 2)]
    epsilon = epsilon_ratio * cv2.arcLength(pts, closed)
    approx = cv2.approxPolyDP(pts, epsilon, closed)
    return [tuple(int(v) for v in p) for p in approx.reshape(-1, 2)]
