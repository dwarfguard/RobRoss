import os

import cv2
import numpy as np
from skimage.morphology import skeletonize

APPLE_PATH = os.path.join(os.path.dirname(__file__), '..', 'apple.png')

img = cv2.imread(APPLE_PATH, cv2.IMREAD_GRAYSCALE)
edges = cv2.Canny(img, threshold1=100, threshold2=200)  # Canny算子一行搞定 pip install opencv-python

OUTPUT_PATH = os.path.join(os.path.dirname(__file__), 'apple_edges.png')
cv2.imwrite(OUTPUT_PATH, edges)

# cv2.findContours traces the boundary of each stroke as a thin blob, walking
# down one side and back up the other - that mirrored return trip is the
# duplicate line. Skeletonizing first collapses every stroke to a single
# pixel-wide centerline, then we walk that pixel graph directly so each
# stroke is emitted once.
skeleton = skeletonize(edges > 0)

NEIGHBORS_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
all_pixels = set(map(tuple, np.argwhere(skeleton)))


def neighbors(pixel):
    y, x = pixel
    return [(y + dy, x + dx) for dy, dx in NEIGHBORS_8 if (y + dy, x + dx) in all_pixels]


degree = {p: len(neighbors(p)) for p in all_pixels}
nodes = {p for p, d in degree.items() if d != 2}  # endpoints (deg 1) and junctions (deg >= 3)

visited_edges = set()
strokes = []  # list of (points_yx, is_closed)


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
            strokes.append((walk(node, nbr), False))

# Closed loops made entirely of degree-2 pixels (no junctions/endpoints at all).
for pixel in all_pixels - nodes:
    unvisited = [p for p in neighbors(pixel) if frozenset((pixel, p)) not in visited_edges]
    if unvisited:
        strokes.append((walk(pixel, unvisited[0]), True))


def simplify(points_yx, closed):
    """Reduce point count with Douglas-Peucker while keeping the stroke shape."""
    pts_xy = np.array([[x, y] for y, x in points_yx], dtype=np.int32).reshape(-1, 1, 2)
    if len(pts_xy) < 3:
        return [tuple(p) for p in pts_xy.reshape(-1, 2)]
    epsilon = 0.002 * cv2.arcLength(pts_xy, closed)
    approx = cv2.approxPolyDP(pts_xy, epsilon, closed)
    return [tuple(p) for p in approx.reshape(-1, 2)]


def catmull_rom_path(points, closed):
    """Convert a polyline into a smooth SVG path using cubic Bezier segments
    fitted through the points (Catmull-Rom to Bezier control points)."""
    pts = [np.array(p, dtype=float) for p in points]
    if closed and len(pts) > 2 and np.allclose(pts[0], pts[-1]):
        pts.pop()
    n = len(pts)
    if n < 2:
        return None

    def get(i):
        if closed:
            return pts[i % n]
        return pts[max(0, min(n - 1, i))]

    d = [f"M {pts[0][0]:.1f} {pts[0][1]:.1f}"]
    segment_count = n if closed else n - 1
    for i in range(segment_count):
        p0, p1, p2, p3 = get(i - 1), get(i), get(i + 1), get(i + 2)
        c1 = p1 + (p2 - p0) / 6
        c2 = p2 - (p3 - p1) / 6
        d.append(f"C {c1[0]:.1f} {c1[1]:.1f} {c2[0]:.1f} {c2[1]:.1f} {p2[0]:.1f} {p2[1]:.1f}")
    if closed:
        d.append("Z")
    return " ".join(d)


height, width = skeleton.shape
svg_paths = []
for points_yx, closed in strokes:
    simplified = simplify(points_yx, closed)
    path_d = catmull_rom_path(simplified, closed)
    if path_d:
        svg_paths.append(f'<path d="{path_d}" stroke="black" fill="none" stroke-width="1"/>')

svg_content = (
    f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
    f'viewBox="0 0 {width} {height}">\n'
    + "\n".join(svg_paths)
    + "\n</svg>"
)

SVG_PATH = os.path.join(os.path.dirname(__file__), 'apple_edges.svg')
with open(SVG_PATH, 'w') as f:
    f.write(svg_content)
