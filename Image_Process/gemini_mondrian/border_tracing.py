# This file is maintained byte-for-byte identical to the canonical source:
#   Image_Process/image_to_mondrian/border_tracing.py
"""Classic Mondrian black grid lines: trace each kept region's own outline
via cv2.findContours, one closed loop per outer boundary and per hole.

Tracing per-region (rather than walking a global boundary-pixel network
across the whole image) sidesteps a fragmentation problem: a busy real
photo's color-boundary network has huge numbers of junctions (T-junctions
where 3+ quantized colors meet), and any graph walk has to break a new
stroke at every junction - producing thousands of tiny disconnected
fragments for textured photos. A single region's own boundary is always
just one or a few closed loops regardless of how jagged the pixel boundary
is, so cv2.findContours' pixel-level Moore boundary tracing never hits that
"which branch do I take" ambiguity in the first place.

Trade-off: a shared edge between two adjacent kept regions gets traced (and
painted) once from each side - not deduplicated. That's a deliberate choice,
not an oversight: de-duplicating would need geometric edge-matching with
tolerance, and the win from fixing the fragmentation problem is already
large enough (see Image_Process/image_to_mondrian/README.md) that doubling
a thin pen line on shared edges isn't worth the added complexity.
"""

import cv2
import numpy as np


def trace_region_contours(mask: np.ndarray) -> list:
    """One closed loop per outer boundary and per hole in this region's own
    mask. Returns (points_xy, closed=True) tuples - points are already
    (x, y) pixel order (cv2.findContours' native order)."""
    mask_u8 = mask.astype(np.uint8)
    contours, _hierarchy = cv2.findContours(mask_u8, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    return [
        ([tuple(int(v) for v in p) for p in contour.reshape(-1, 2)], True)
        for contour in contours
        if len(contour) >= 3
    ]


def simplify(points_xy, closed: bool, epsilon_ratio: float):
    """Reduce point count with Douglas-Peucker while keeping the line shape."""
    pts = np.array(points_xy, dtype=np.int32).reshape(-1, 1, 2)
    if len(pts) < 3:
        return [tuple(p) for p in pts.reshape(-1, 2)]
    epsilon = epsilon_ratio * cv2.arcLength(pts, closed)
    approx = cv2.approxPolyDP(pts, epsilon, closed)
    return [tuple(int(v) for v in p) for p in approx.reshape(-1, 2)]
