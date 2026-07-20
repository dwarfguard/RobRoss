"""Scanline fill for an arbitrary-shape region mask: erode inward (so
strokes don't bleed into a neighboring color's territory), then fill each
row with one straight paint_stroke per contiguous on-interval - handling
concave shapes that have more than one interval per row.

Row spacing mirrors Image_Process/mondrian/generate_painting_paths.py's
compute_stripe_row_centers() (copied and adapted to pixel-row space rather
than mm rectangle height, per this repo's copy-for-self-containment
convention - see path_validation.py).
"""

import cv2
import numpy as np


def erode_mask(mask: np.ndarray, erosion_px: int) -> np.ndarray:
    """Shrink a boolean region mask inward by ~erosion_px, so a stroke of
    the tool's width painted along the shrunk mask stays inside the
    original region instead of bleeding into a neighboring color."""
    if erosion_px <= 0:
        return mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * erosion_px + 1, 2 * erosion_px + 1))
    eroded = cv2.erode(mask.astype(np.uint8), kernel)
    return eroded.astype(bool)


def find_row_intervals(mask_row: np.ndarray) -> list:
    """Return every contiguous (start_col, end_col) run of True values in a
    1-D boolean row - there may be more than one for a concave shape."""
    padded = np.concatenate(([0], mask_row.astype(np.int8), [0]))
    diff = np.diff(padded)
    starts = np.where(diff == 1)[0]
    ends = np.where(diff == -1)[0]
    return list(zip(starts.tolist(), ends.tolist()))


def compute_stripe_rows(top: int, height: int, tool_width_px: float, stroke_overlap_ratio: float) -> list:
    """Row indices spaced so the tool covers the full mask height, always
    including the exact bottom row so the bottom edge gets covered too.
    Same spacing formula as mondrian's compute_stripe_row_centers()."""
    if height <= tool_width_px:
        return [round(top + height / 2)]

    stripe_step = tool_width_px * (1 - stroke_overlap_ratio)
    last_row = top + height - tool_width_px / 2

    rows = []
    row = top + tool_width_px / 2
    while row < last_row - 1e-9:
        rows.append(row)
        row += stripe_step

    rows.append(last_row)
    return sorted({round(r) for r in rows})


def region_to_pixel_strokes(
    mask: np.ndarray,
    tool_width_px: float,
    stroke_overlap_ratio: float,
    mask_erosion_px: int,
    min_interval_px: int = 1,
) -> list:
    """Convert one region's mask into a list of ((x0,y0), (x1,y1)) pixel
    strokes that fully fill it. No manual left-right alternation here -
    a concave shape's per-row interval count varies, so travel-order
    optimization is deferred entirely to the ordering pass downstream."""
    eroded = erode_mask(mask, mask_erosion_px)
    ys, xs = np.where(eroded)
    if ys.size == 0:
        return []

    top, bottom = int(ys.min()), int(ys.max())
    rows = compute_stripe_rows(top, bottom - top + 1, tool_width_px, stroke_overlap_ratio)

    strokes = []
    for row_y in rows:
        row_y = max(0, min(row_y, eroded.shape[0] - 1))
        for start_x, end_x in find_row_intervals(eroded[row_y, :]):
            if end_x - start_x < min_interval_px:
                continue
            strokes.append(((start_x, row_y), (end_x, row_y)))

    return strokes
