"""Classic Mondrian black grid lines: trace the boundary between
differently-labeled adjacent pixels in the quantized label image, once
globally, so a shared edge between two color blocks is only drawn once
(not once per side).

The skeleton graph-walk core is copied from Image_Process/sketch/canny_edges.py
(extract_strokes()), generalized to accept any boolean boundary mask instead
of always computing one via cv2.Canny - per this repo's copy-for-self-
containment convention (see path_validation.py).
"""

import cv2
import numpy as np
from skimage.morphology import skeletonize

NEIGHBORS_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def compute_boundary_mask(label_image: np.ndarray) -> np.ndarray:
    """Mark every pixel that differs from its up or left neighbor - a
    single-shot detector of every color-block boundary in the image,
    including boundaries with skipped/dropped (background) regions."""
    boundary = np.zeros(label_image.shape, dtype=bool)
    boundary[1:, :] |= label_image[1:, :] != label_image[:-1, :]
    boundary[:, 1:] |= label_image[:, 1:] != label_image[:, :-1]
    return boundary


def trace_boundary_strokes(boundary_mask: np.ndarray):
    """Skeletonize the boundary mask and walk its pixel graph to trace each
    independent line once. Returns (strokes_xy, (width, height)) - same
    shape as canny_edges.extract_strokes()'s return value."""
    skeleton = skeletonize(boundary_mask)

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

    for node in nodes:
        for nbr in neighbors(node):
            if frozenset((node, nbr)) not in visited_edges:
                strokes_yx.append((walk(node, nbr), False))

    for pixel in all_pixels - nodes:
        unvisited = [p for p in neighbors(pixel) if frozenset((pixel, p)) not in visited_edges]
        if unvisited:
            strokes_yx.append((walk(pixel, unvisited[0]), True))

    strokes_xy = [([(x, y) for y, x in points_yx], closed) for points_yx, closed in strokes_yx]
    height, width = skeleton.shape
    return strokes_xy, (width, height)


def simplify(points_xy, closed: bool, epsilon_ratio: float):
    """Reduce point count with Douglas-Peucker while keeping the line shape."""
    pts = np.array(points_xy, dtype=np.int32).reshape(-1, 1, 2)
    if len(pts) < 3:
        return [tuple(p) for p in pts.reshape(-1, 2)]
    epsilon = epsilon_ratio * cv2.arcLength(pts, closed)
    approx = cv2.approxPolyDP(pts, epsilon, closed)
    return [tuple(int(v) for v in p) for p in approx.reshape(-1, 2)]
