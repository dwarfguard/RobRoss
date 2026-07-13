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


def clean_label_image(label_image: np.ndarray, palette: list, morph_open_kernel_px: int) -> np.ndarray:
    """Morphologically open every color's mask to strip single-pixel
    speckle before labeling/border-tracing. Pixels no color claims after
    opening are set to -1 ("unclaimed") - distinct from any real palette
    index (including 0), so removed speckle doesn't silently get counted
    as palette[0]'s territory."""
    if not morph_open_kernel_px or morph_open_kernel_px <= 0:
        return label_image

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_open_kernel_px, morph_open_kernel_px))
    cleaned = np.full(label_image.shape, -1, dtype=label_image.dtype)
    for color_index in range(len(palette)):
        mask = (label_image == color_index).astype(np.uint8)
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        cleaned[opened > 0] = color_index
    return cleaned


def segment_image(label_image: np.ndarray, palette: list, morph_open_kernel_px: int, min_area_px: float) -> tuple:
    """Full per-color segmentation pipeline: labeling with a morphological
    open pass to strip speckle, then small-region filtering.

    Returns (kept_regions, dropped_count, cleaned_label_image). The cleaned
    label image (speckle-opened, but not small-region-filtered) is also the
    right input for border tracing - see generate_painting_paths.py.
    """
    working_label_image = clean_label_image(label_image, palette, morph_open_kernel_px)
    regions = label_connected_regions(working_label_image, palette)
    kept, dropped_count = filter_small_regions(regions, min_area_px)
    return kept, dropped_count, working_label_image
