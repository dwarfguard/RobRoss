"""Per-color connected-component labeling of a quantized label image, with
small-region (noise speck) filtering.
"""

import cv2
import numpy as np


def label_connected_regions(label_image: np.ndarray, palette: list) -> list:
    """For each palette color, find its independent connected blobs.

    Returns a list of dicts: {color_index, color_name, color_hex, mask, area_px}.
    """
    regions = []

    for color_index, color in enumerate(palette):
        color_mask = (label_image == color_index).astype(np.uint8)
        if not np.any(color_mask):
            continue

        num_labels, labels_im, stats, _ = cv2.connectedComponentsWithStats(
            color_mask, connectivity=8
        )
        for component_id in range(1, num_labels):  # 0 is background
            regions.append(
                {
                    "color_index": color_index,
                    "color_name": color["name"],
                    "color_hex": color["hex"],
                    "mask": labels_im == component_id,
                    "area_px": int(stats[component_id, cv2.CC_STAT_AREA]),
                }
            )

    return regions


def filter_small_regions(regions: list, min_area_px: float) -> tuple:
    """Drop regions smaller than min_area_px. Returns (kept, dropped_count).

    Dropped pixels stay unpainted rather than being reassigned to a
    neighboring region - a documented v1 simplification.
    """
    kept = [r for r in regions if r["area_px"] >= min_area_px]
    dropped_count = len(regions) - len(kept)
    return kept, dropped_count


def _morph_per_color(label_image: np.ndarray, palette: list, op: int, kernel_px: int) -> np.ndarray:
    """Apply a binary morphological op (MORPH_OPEN/MORPH_CLOSE) to each
    color's own mask independently, then recompose into a label image.
    Pixels no color claims afterward are -1 ("unclaimed") - distinct from
    any real palette index (including 0), so affected pixels don't
    silently get counted as palette[0]'s territory."""
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_px, kernel_px))
    result = np.full(label_image.shape, -1, dtype=label_image.dtype)
    for color_index in range(len(palette)):
        mask = (label_image == color_index).astype(np.uint8)
        processed = cv2.morphologyEx(mask, op, kernel)
        result[processed > 0] = color_index
    return result


def clean_label_image(
    label_image: np.ndarray,
    palette: list,
    morph_open_kernel_px: int,
    morph_close_kernel_px: int = 0,
    protected_mask: np.ndarray = None,
) -> np.ndarray:
    """Morphologically clean every color's mask before labeling/border-tracing.

    Closing runs first (if enabled): it bridges gaps cut into a single
    region by internal lighting/shadow variation (e.g. a light object's
    shadowed fold reading as a different, more saturated color than its
    lit surface) - without this, one real object can fragment into several
    disconnected blobs even though it's visually one coherent shape.
    Opening runs after (if enabled): it strips small isolated speckle -
    running it first would treat a gap's ragged edge pixels as noise and
    erase them before closing gets a chance to bridge the gap, making the
    gap harder to close, so the order matters.

    protected_mask (optional boolean array, same shape as label_image): a
    closing kernel large enough to bridge a real gap in one object is also
    large enough to merge small nearby semantic features (e.g. an eye) into
    an adjacent region - closing has no notion of "this is an eye", only
    pixel geometry. Pixels marked True always keep their original
    pre-morphology classification, overriding whatever any color's
    close/open decided for that pixel, regardless of processing order.
    See face_protection.py for how this mask gets built.
    """
    working = label_image

    if morph_close_kernel_px and morph_close_kernel_px > 0:
        working = _morph_per_color(working, palette, cv2.MORPH_CLOSE, morph_close_kernel_px)

    if morph_open_kernel_px and morph_open_kernel_px > 0:
        working = _morph_per_color(working, palette, cv2.MORPH_OPEN, morph_open_kernel_px)

    if protected_mask is not None:
        working = working.copy()
        working[protected_mask] = label_image[protected_mask]

    return working


def segment_image(
    label_image: np.ndarray,
    palette: list,
    morph_open_kernel_px: int,
    min_area_px: float,
    morph_close_kernel_px: int = 0,
    protected_mask: np.ndarray = None,
) -> tuple:
    """Full per-color segmentation pipeline: labeling with morphological
    close + open passes to bridge lighting-induced gaps and strip speckle,
    then small-region filtering.

    Returns (kept_regions, dropped_count, cleaned_label_image). The cleaned
    label image (closed/opened, but not small-region-filtered) is also the
    right input for border tracing - see generate_painting_paths.py.
    """
    working_label_image = clean_label_image(
        label_image, palette, morph_open_kernel_px, morph_close_kernel_px, protected_mask
    )
    regions = label_connected_regions(working_label_image, palette)
    kept, dropped_count = filter_small_regions(regions, min_area_px)
    return kept, dropped_count, working_label_image
