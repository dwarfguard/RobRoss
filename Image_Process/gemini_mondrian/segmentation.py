"""Per-color connected-component labeling of a quantized label image, with
small-region (noise speck) filtering.

Deliberately simpler than Image_Process/image_to_mondrian/segmentation.py:
that version's morphological *closing* exists to bridge gaps a real photo's
lighting/shadow cuts into one object - a problem specific to photographing a
physical scene. Gemini's output is already flat, bold-colored regions with
no such gaps, so there's nothing to bridge; running that same large-kernel
closing over an already-clean image just rounds off real detail (confirmed
by testing - it turned a recognizable portrait into a blob). Only a small
opening pass survives here, to strip anti-aliasing/compression speckle at
region edges. No face protection either - that machinery exists solely to
counteract closing's side effects, so with no closing there's nothing to
protect against.
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


def _morph_open_per_color(label_image: np.ndarray, palette: list, kernel_px: int) -> np.ndarray:
    """Apply a binary morphological opening to each color's own mask
    independently, then recompose into a label image. Pixels no color
    claims afterward are -1 ("unclaimed") - distinct from any real palette
    index (including 0), so affected pixels don't silently get counted as
    palette[0]'s territory."""
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_px, kernel_px))
    result = np.full(label_image.shape, -1, dtype=label_image.dtype)
    for color_index in range(len(palette)):
        mask = (label_image == color_index).astype(np.uint8)
        processed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        result[processed > 0] = color_index
    return result


def clean_label_image(
    label_image: np.ndarray,
    palette: list,
    morph_open_kernel_px: int,
) -> np.ndarray:
    """Strips small isolated speckle (anti-aliasing/compression artifacts at
    region boundaries) from every color's mask. No closing pass - Gemini's
    regions don't have lighting-induced gaps to bridge, see module docstring."""
    if morph_open_kernel_px and morph_open_kernel_px > 0:
        return _morph_open_per_color(label_image, palette, morph_open_kernel_px)
    return label_image


def segment_image(
    label_image: np.ndarray,
    palette: list,
    morph_open_kernel_px: int,
    min_area_px: float,
) -> tuple:
    """Full per-color segmentation pipeline: a light opening pass to strip
    speckle, then small-region filtering.

    Returns (kept_regions, dropped_count, cleaned_label_image). The cleaned
    label image (opened, but not small-region-filtered) is also the right
    input for border tracing - see generate_painting_paths.py.
    """
    working_label_image = clean_label_image(label_image, palette, morph_open_kernel_px)
    regions = label_connected_regions(working_label_image, palette)
    kept, dropped_count = filter_small_regions(regions, min_area_px)
    return kept, dropped_count, working_label_image
