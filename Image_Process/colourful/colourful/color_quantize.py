"""Load a source photo and quantize it down to a fixed palette (the robot's
physical pen colors), fully vectorized with numpy/opencv.
"""

import cv2
import numpy as np


def load_image(path) -> np.ndarray:
    """Read an image as BGR. Raises FileNotFoundError if it can't be read."""
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"Could not read image: {path}")
    return image


def preprocess(
    image_bgr: np.ndarray,
    blur_kernel_size: int = 5,
    blur_sigma: float = 0,
    downscale_max_dimension_px: int = None,
    bilateral_d: int = 0,
    bilateral_sigma_color: float = 75,
    bilateral_sigma_space: float = 75,
) -> np.ndarray:
    """Downscale (bounding the longest side) and smooth to merge single-pixel
    noise before quantization, so it doesn't fragment into tiny regions.

    bilateral_d > 0 uses edge-preserving bilateral filtering instead of
    Gaussian blur - it smooths flat/gradient areas (e.g. a lit-to-shadow
    transition across one object) while keeping real high-contrast edges
    (e.g. an eye against skin) sharp, which plain Gaussian blur can't
    distinguish. Takes priority over blur_kernel_size when enabled; 0
    (default) disables it and matches prior Gaussian-blur-only behavior
    exactly.
    """
    result = image_bgr

    if downscale_max_dimension_px:
        height, width = result.shape[:2]
        longest_side = max(height, width)
        if longest_side > downscale_max_dimension_px:
            scale = downscale_max_dimension_px / longest_side
            new_size = (max(1, round(width * scale)), max(1, round(height * scale)))
            result = cv2.resize(result, new_size, interpolation=cv2.INTER_AREA)

    if bilateral_d and bilateral_d > 0:
        result = cv2.bilateralFilter(result, bilateral_d, bilateral_sigma_color, bilateral_sigma_space)
    elif blur_kernel_size and blur_kernel_size > 1:
        result = cv2.GaussianBlur(result, (blur_kernel_size, blur_kernel_size), blur_sigma)

    return result


def compute_adaptive_chroma_threshold(image_bgr: np.ndarray, percentile: float) -> float:
    """Return the Nth percentile of this image's own Lab chroma distribution,
    for use as neutral_chroma_threshold - the threshold adapts to each
    photo's actual color distribution instead of being a fixed number picked
    for one specific photo."""
    lab = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    chroma = np.sqrt((lab[..., 1] - 128.0) ** 2 + (lab[..., 2] - 128.0) ** 2)
    return float(np.percentile(chroma, percentile))


def hex_to_bgr(hex_color: str) -> tuple:
    hex_color = hex_color.lstrip("#")
    r, g, b = (int(hex_color[i : i + 2], 16) for i in (0, 2, 4))
    return (b, g, r)


def _to_color_space(pixels_bgr: np.ndarray, color_space: str) -> np.ndarray:
    """pixels_bgr: (N, 1, 3) uint8 array -> (N, 3) float array in the target space."""
    if color_space == "lab":
        converted = cv2.cvtColor(pixels_bgr, cv2.COLOR_BGR2LAB)
    else:
        converted = pixels_bgr
    return converted.reshape(-1, 3).astype(np.float32)


# For 8-bit cv2 Lab, a/b are stored as true_a*/b* + 128 (neutral point at
# 128, not 0) - chroma (colorfulness) is the distance from that neutral
# point, not from the origin.
def _lab_chroma(lab_space: np.ndarray) -> np.ndarray:
    return np.sqrt((lab_space[:, 1] - 128.0) ** 2 + (lab_space[:, 2] - 128.0) ** 2)


# White/black have chroma ~0; the palette's own chromatic colors (red/blue/
# yellow) are all comfortably above this - used to auto-split the palette
# into a "neutral" bucket and a "chromatic" bucket from its hex values.
_SWATCH_NEUTRAL_EPSILON = 10.0


def quantize_to_palette(
    image_bgr: np.ndarray,
    palette_colors: list,
    color_space: str = "lab",
    neutral_chroma_threshold: float = 0.0,
) -> np.ndarray:
    """Map every pixel to the index of its nearest palette color.

    palette_colors: list of {"name": str, "hex": str} dicts, in config order.
    Returns an (H, W) int array of palette indices.

    neutral_chroma_threshold (Lab only): pixels with Lab chroma below this
    are only matched against the palette's own neutral (near-zero-chroma)
    colors - e.g. white/black - never a chromatic one, regardless of raw
    distance. Pixels at or above it are only matched against the palette's
    chromatic colors. This keeps desaturated/mid-tone content (skin, hair,
    shadow) from being forced into a bold color just because it happens to
    be marginally closer than white/black - real Mondrian paintings are
    mostly white canvas with sparingly-used bold color blocks, not every
    pixel colored in. 0 (default) disables this and matches prior
    behavior exactly (plain nearest-neighbor over the whole palette).
    """
    height, width = image_bgr.shape[:2]

    pixels = image_bgr.reshape(-1, 1, 3).astype(np.uint8)
    pixel_space = _to_color_space(pixels, color_space)

    swatch_bgr = np.array(
        [hex_to_bgr(c["hex"]) for c in palette_colors], dtype=np.uint8
    ).reshape(-1, 1, 3)
    palette_space = _to_color_space(swatch_bgr, color_space)

    # (N, 1, 3) - (1, P, 3) -> (N, P, 3) -> (N, P) squared distance.
    diffs = pixel_space[:, None, :] - palette_space[None, :, :]
    distances = np.sum(diffs * diffs, axis=2)

    if neutral_chroma_threshold and color_space == "lab":
        swatch_chroma = _lab_chroma(palette_space)
        neutral_mask = swatch_chroma <= _SWATCH_NEUTRAL_EPSILON
        # Gating only makes sense with both a neutral and a chromatic
        # bucket to choose between - otherwise fall through unchanged.
        if neutral_mask.any() and (~neutral_mask).any():
            pixel_chroma = _lab_chroma(pixel_space)
            is_neutral_pixel = pixel_chroma < neutral_chroma_threshold
            allowed = np.where(
                is_neutral_pixel[:, None], neutral_mask[None, :], ~neutral_mask[None, :]
            )
            distances = np.where(allowed, distances, np.inf)

    labels = np.argmin(distances, axis=1)
    return labels.reshape(height, width)
