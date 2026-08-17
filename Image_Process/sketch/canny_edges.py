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

try:
    from PIL import Image
except (ImportError, ModuleNotFoundError):  # surfaced when orientation is read
    Image = None

NEIGHBORS_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
EXIF_ORIENTATION_TAG = 274


def _exif_orientation(image_path) -> int:
    """Return the source EXIF orientation, defaulting to normal orientation."""
    if Image is None:
        raise ImportError(
            "Pillow is required to normalize image EXIF orientation."
        )
    try:
        with Image.open(image_path) as image:
            orientation = int(image.getexif().get(EXIF_ORIENTATION_TAG, 1))
    except (OSError, TypeError, ValueError):
        return 1
    return orientation if 1 <= orientation <= 8 else 1


def _apply_exif_orientation(image, orientation: int):
    """Apply all eight EXIF orientation transforms to an OpenCV array."""
    if orientation == 2:
        return cv2.flip(image, 1)
    if orientation == 3:
        return cv2.rotate(image, cv2.ROTATE_180)
    if orientation == 4:
        return cv2.flip(image, 0)
    if orientation == 5:
        return cv2.transpose(image)
    if orientation == 6:
        return cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    if orientation == 7:
        return cv2.flip(cv2.transpose(image), -1)
    if orientation == 8:
        return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return image


def read_grayscale_with_metadata(image_path):
    """Read oriented grayscale plus safe registration metadata.

    Pixel decoding remains in OpenCV so grayscale and alpha behavior stays
    unchanged. EXIF orientation is applied before alpha compositing and before
    any local outline/detail operation.
    """
    orientation = _exif_orientation(image_path)
    image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")
    image = _apply_exif_orientation(image, orientation)
    if image.ndim == 2:
        grayscale = image
    else:
        if image.shape[2] == 4:
            color = image[:, :, :3].astype(np.float32)
            alpha = image[:, :, 3:4].astype(np.float32) / 255.0
            image = np.clip(
                color * alpha + 255.0 * (1.0 - alpha), 0, 255
            ).astype(np.uint8)
        grayscale = cv2.cvtColor(image[:, :, :3], cv2.COLOR_BGR2GRAY)
    return grayscale, {
        "source_exif_orientation": orientation,
        "source_orientation_normalized": orientation != 1,
    }


def read_grayscale(image_path):
    """Read an EXIF-oriented image as grayscale, compositing alpha on white.

    OpenCV's ordinary grayscale loader ignores alpha. That makes a transparent
    PNG background look black and can manufacture a strong crop-edge line, so
    sketch inputs are loaded unchanged and explicitly composited first. The
    unchanged loader also ignores EXIF orientation, so that transform is
    applied explicitly before returning pixels.
    """
    grayscale, _metadata = read_grayscale_with_metadata(image_path)
    return grayscale


def suppress_border_artifacts(mask, band_px: int):
    """Remove components confined to the outer image-edge band.

    A legitimate subject contour may touch or cross the crop boundary. Such a
    component is retained whenever any of its pixels reaches the interior.
    Only components living wholly inside the configurable band are cleared.
    Returns ``(cleaned_mask, metrics)``.
    """
    binary = np.asarray(mask, dtype=bool)
    if band_px <= 0 or not binary.any():
        return binary.copy(), {
            "border_components_filtered": 0,
            "border_pixels_filtered": 0,
        }

    height, width = binary.shape
    band = min(int(band_px), max(0, (min(height, width) - 1) // 2))
    if band == 0:
        return binary.copy(), {
            "border_components_filtered": 0,
            "border_pixels_filtered": 0,
        }

    labels_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        binary.astype(np.uint8), connectivity=8
    )
    cleaned = binary.copy()
    removed_components = 0
    removed_pixels = 0
    for label in range(1, labels_count):
        x, y, component_width, component_height, area = stats[label]
        reaches_interior = (
            x < width - band
            and x + component_width > band
            and y < height - band
            and y + component_height > band
        )
        component = labels == label
        if reaches_interior and np.any(component[band:height - band, band:width - band]):
            continue
        cleaned[component] = False
        removed_components += 1
        removed_pixels += int(area)

    return cleaned, {
        "border_components_filtered": removed_components,
        "border_pixels_filtered": removed_pixels,
    }


def local_outline_mask(
    grayscale,
    threshold1: float,
    threshold2: float,
    blur_kernel_size: int = 5,
    contrast_clip_limit: float = 2.0,
):
    """Build a deterministic structural-edge mask from a source photo."""
    working = np.asarray(grayscale, dtype=np.uint8)
    if contrast_clip_limit > 0:
        clahe = cv2.createCLAHE(clipLimit=float(contrast_clip_limit), tileGridSize=(8, 8))
        working = clahe.apply(working)
    if blur_kernel_size > 1:
        working = cv2.GaussianBlur(
            working, (int(blur_kernel_size), int(blur_kernel_size)), 0
        )
    return cv2.Canny(
        working, threshold1=float(threshold1), threshold2=float(threshold2)
    ) > 0


def line_art_mask(grayscale, threshold: int = 210):
    """Convert clean black-on-white generated line art to a centerline mask."""
    gray = np.asarray(grayscale, dtype=np.uint8)
    # Qwen line art uses a white background with anti-aliased gray pixels at
    # the edges of its thinnest marks. Otsu commonly chooses a much darker
    # cutoff and erases those pixels, so the configured value is the exact ink
    # threshold rather than merely an upper bound on an adaptive threshold.
    return gray <= int(threshold)


def edge_strength_map(grayscale):
    """Normalized Sobel magnitude used to rank original-photo details."""
    gray = np.asarray(grayscale, dtype=np.uint8)
    grad_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    magnitude = cv2.magnitude(grad_x, grad_y)
    maximum = float(magnitude.max())
    return magnitude / maximum if maximum > 0 else magnitude


def _cluster_nodes(degree, neighbors):
    """Group adjacent junction pixels into logical graph vertices."""
    junctions = {point for point, value in degree.items() if value >= 3}
    cluster_of = {}
    next_id = 0
    for start in junctions:
        if start in cluster_of:
            continue
        stack = [start]
        cluster_of[start] = next_id
        while stack:
            current = stack.pop()
            for neighbor in neighbors(current):
                if neighbor in junctions and neighbor not in cluster_of:
                    cluster_of[neighbor] = next_id
                    stack.append(neighbor)
        next_id += 1
    return cluster_of


def _cluster_anchors(cluster_of):
    """Choose a central real skeleton pixel as each cluster's shared anchor."""
    members_by_cluster = {}
    for pixel, cluster_id in cluster_of.items():
        members_by_cluster.setdefault(cluster_id, []).append(pixel)

    anchors = {}
    for cluster_id, members in members_by_cluster.items():
        center_y = sum(point[0] for point in members) / len(members)
        center_x = sum(point[1] for point in members) / len(members)
        anchors[cluster_id] = min(
            members,
            key=lambda point: (
                (point[0] - center_y) ** 2 + (point[1] - center_x) ** 2,
                point,
            ),
        )
    return anchors


def _trace_skeleton(skeleton):
    """Walk a skeleton graph and return node-clustered centerline chains."""
    all_pixels = set(map(tuple, np.argwhere(skeleton)))

    def neighbors(pixel):
        y, x = pixel
        return [
            (y + dy, x + dx)
            for dy, dx in NEIGHBORS_8
            if (y + dy, x + dx) in all_pixels
        ]

    degree = {point: len(neighbors(point)) for point in all_pixels}
    nodes = {point for point, value in degree.items() if value != 2}
    cluster_of = _cluster_nodes(degree, neighbors)
    cluster_anchors = _cluster_anchors(cluster_of)

    def logical_id(pixel):
        return cluster_of.get(pixel, pixel)

    def anchor(pixel):
        cluster_id = cluster_of.get(pixel)
        return cluster_anchors.get(cluster_id, pixel)

    visited_edges = set()
    strokes_yx = []

    def walk(start, first_step):
        path = [start, first_step]
        visited_edges.add(frozenset((start, first_step)))
        previous, current = start, first_step
        while degree.get(current) == 2 and current not in nodes:
            next_point = next((point for point in neighbors(current) if point != previous), None)
            if next_point is None or frozenset((current, next_point)) in visited_edges:
                break
            visited_edges.add(frozenset((current, next_point)))
            path.append(next_point)
            previous, current = current, next_point
        return path

    for node in nodes:
        for neighbor in neighbors(node):
            edge = frozenset((node, neighbor))
            if edge in visited_edges:
                continue
            if neighbor in nodes and logical_id(neighbor) == logical_id(node):
                visited_edges.add(edge)
                continue
            path = walk(node, neighbor)
            end = path[-1]
            closed = end in nodes and logical_id(end) == logical_id(node)
            path[0] = anchor(path[0])
            path[-1] = anchor(path[-1])
            strokes_yx.append((path, closed))

    for pixel in all_pixels - nodes:
        unvisited = [
            point
            for point in neighbors(pixel)
            if frozenset((pixel, point)) not in visited_edges
        ]
        if unvisited:
            strokes_yx.append((walk(pixel, unvisited[0]), True))

    return [
        ([(x, y) for y, x in points_yx], closed)
        for points_yx, closed in strokes_yx
    ]


def trace_mask(mask):
    """Skeletonize a binary ink/edge mask and trace independent paths."""
    skeleton = skeletonize(np.asarray(mask, dtype=bool))
    return _trace_skeleton(skeleton)


def extract_strokes(image_path, threshold1: float, threshold2: float):
    """Run Canny edge detection + skeletonization on `image_path`.

    Returns (strokes, image_size) where `strokes` is a list of
    (points_xy, closed) - one entry per independent traced line, points as
    (x, y) pixel coordinates - and `image_size` is (width, height).
    """
    img = read_grayscale(image_path)
    edges = cv2.Canny(img, threshold1=threshold1, threshold2=threshold2)
    strokes_xy = trace_mask(edges > 0)
    height, width = img.shape
    return strokes_xy, (width, height)


def simplify(points_xy, closed: bool, epsilon_ratio: float, min_epsilon_px: float = 1.0):
    """Reduce point count with Douglas-Peucker while keeping the stroke shape."""
    pts = np.array(points_xy, dtype=np.int32).reshape(-1, 1, 2)
    if len(pts) < 3:
        return [tuple(p) for p in pts.reshape(-1, 2)]
    epsilon = max(min_epsilon_px, epsilon_ratio * cv2.arcLength(pts, closed))
    approx = cv2.approxPolyDP(pts, epsilon, closed)
    return [tuple(int(v) for v in p) for p in approx.reshape(-1, 2)]
