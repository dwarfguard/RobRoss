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


def close_mask(mask, kernel_px: int):
    """Morphological closing (dilate then erode) to bridge small gaps in
    an antialiased stroke mask before skeletonizing.

    Thresholding an antialiased line at a fixed cutoff can flicker above
    and below the cutoff along a thin/diagonal stroke, breaking one
    visual line into several disconnected mask blobs; skeletonizing that
    directly produces a fork/junction at every break, which then needs
    aggressive spur pruning to clean up (see prune_spurs). Closing first
    reconnects those breaks so fewer skeleton junctions are noise in the
    first place. `kernel_px <= 0` returns the mask unchanged (opt-out)."""
    if kernel_px <= 0:
        return mask
    kernel = np.ones((kernel_px, kernel_px), np.uint8)
    closed = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
    return closed.astype(bool)


def skeletonize_mask(mask):
    """Thin a boolean stroke mask to single-pixel-wide centerlines."""
    return skeletonize(mask)


def _cluster_nodes(degree, neighbors):
    """Group 8-adjacent junction pixels (degree >= 3) into single logical
    vertices. A sharp or rounded corner often rasterizes into 2-3 junction
    pixels sitting right next to each other rather than one - without
    clustering, each of those pixels independently spawns its own walk,
    producing near-duplicate paths and near-zero-length "edges" between
    them that fragment what should be one continuous stroke (see
    extract_strokes' docstring).

    Deliberately excludes degree-1 endpoints: two unrelated dead-end tips
    that happen to sit next to each other are still two separate open
    branches, not one vertex, so only true junctions (degree >= 3) get
    merged. Returns {pixel: cluster_id}, one entry per junction pixel only
    - callers should treat any pixel missing from this dict as its own
    singleton cluster (identified by the pixel itself)."""
    junctions = {p for p, d in degree.items() if d >= 3}
    cluster_of = {}
    next_id = 0
    for start in junctions:
        if start in cluster_of:
            continue
        stack = [start]
        cluster_of[start] = next_id
        while stack:
            current = stack.pop()
            for nbr in neighbors(current):
                if nbr in junctions and nbr not in cluster_of:
                    cluster_of[nbr] = next_id
                    stack.append(nbr)
        next_id += 1
    return cluster_of


def extract_strokes(skeleton):
    """Walk a boolean skeleton's pixel graph into centerline chains.

    Returns a list of (points_xy, closed) - one entry per independent
    stroke, points as (x, y) pixel coordinates. Endpoints (degree 1) and
    junctions (degree >= 3) are graph nodes; runs of degree-2 pixels
    between them are edges, walked once each via `visited_edges`. Node
    pixels that are 8-adjacent to each other are clustered into a single
    logical vertex (see _cluster_nodes) - intra-cluster edges are skipped,
    and a stroke whose two ends land in the same cluster counts as closed,
    not just a stroke that returns to its exact starting pixel.
    """
    all_pixels = set(map(tuple, np.argwhere(skeleton)))

    def neighbors(pixel):
        y, x = pixel
        return [(y + dy, x + dx) for dy, dx in NEIGHBORS_8 if (y + dy, x + dx) in all_pixels]

    degree = {p: len(neighbors(p)) for p in all_pixels}
    nodes = {p for p, d in degree.items() if d != 2}
    cluster_of = _cluster_nodes(degree, neighbors)

    def logical_id(pixel):
        """The pixel's cluster id if it's part of a merged junction
        cluster, otherwise the pixel itself (a singleton identity)."""
        return cluster_of.get(pixel, pixel)

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
            if frozenset((node, nbr)) in visited_edges:
                continue
            if nbr in nodes and logical_id(nbr) == logical_id(node):
                # Trivial hop within the same logical vertex - not a real edge.
                visited_edges.add(frozenset((node, nbr)))
                continue
            path = walk(node, nbr)
            end = path[-1]
            closed = end in nodes and logical_id(end) == logical_id(node)
            strokes_yx.append((path, closed))

    # Closed loops made entirely of degree-2 pixels (no junctions/endpoints at all).
    for pixel in all_pixels - nodes:
        unvisited = [p for p in neighbors(pixel) if frozenset((pixel, p)) not in visited_edges]
        if unvisited:
            strokes_yx.append((walk(pixel, unvisited[0]), True))

    return [([(x, y) for y, x in points_yx], closed) for points_yx, closed in strokes_yx]


def prune_skeleton_spurs(skeleton, min_length_px: float):
    """Remove short dead-end branches directly from the skeleton's pixel
    set, before extract_strokes() builds the pixel graph - not after.

    A sharp corner (e.g. two strokes meeting at a point) makes
    skeletonize() grow a short "whisker" off that point. If that whisker
    is only cleaned up afterwards, by filtering the strokes extract_strokes()
    already produced (see prune_spurs below), the damage is done: the
    corner pixel was already counted as a degree>=3 junction, so a closed
    loop that should pass straight through that corner gets cut into
    separate open fragments there instead. Removing the whisker pixels
    first lets that corner pixel's degree fall back to 2, so the walk
    treats it as an ordinary pass-through pixel and the loop stays whole.

    Runs to a fixed point: removing one spur can drop a neighboring
    junction's degree to 1, exposing a new spur to remove next round.
    """
    pixels = set(map(tuple, np.argwhere(skeleton)))

    def neighbors(pixel, pixel_set):
        y, x = pixel
        return [(y + dy, x + dx) for dy, dx in NEIGHBORS_8 if (y + dy, x + dx) in pixel_set]

    changed = True
    while changed:
        changed = False
        degree = {p: len(neighbors(p, pixels)) for p in pixels}
        endpoints = [p for p, d in degree.items() if d == 1]
        to_remove = set()
        for endpoint in endpoints:
            if endpoint in to_remove:
                continue
            branch = [endpoint]
            prev, current = None, endpoint
            while True:
                candidates = [p for p in neighbors(current, pixels) if p != prev]
                if current != endpoint and degree[current] != 2:
                    break
                if not candidates:
                    break
                prev, current = current, candidates[0]
                branch.append(current)
                if degree.get(current, 0) != 2:
                    break
            if _polyline_length(branch) < min_length_px:
                to_remove.update(branch[:-1])  # keep the junction pixel itself
        if to_remove:
            pixels -= to_remove
            changed = True

    out = np.zeros_like(skeleton)
    for y, x in pixels:
        out[y, x] = True
    return out


def _polyline_length(points_xy) -> float:
    return sum(math.dist(a, b) for a, b in zip(points_xy, points_xy[1:]))


def prune_spurs(strokes, min_length_px: float):
    """Drop short branches - skeletonization spurs at junctions and
    line-cap corners, which threshold+skeletonize is more prone to than
    skeletonizing a thin Canny edge map. Applies to both open dead-ends and
    closed loops: a closed loop this short is degenerate pixel-level noise
    left over at a clustered-node boundary (see extract_strokes), not a
    real small feature - real closed shapes (e.g. a dot or a letter's
    counter) are comfortably longer than any reasonable spur threshold."""
    return [
        (points, closed)
        for points, closed in strokes
        if _polyline_length(points) >= min_length_px
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
