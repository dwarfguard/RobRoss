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
) -> np.ndarray:
    """Downscale (bounding the longest side) and blur to merge single-pixel
    noise before quantization, so it doesn't fragment into tiny regions."""
    result = image_bgr

    if downscale_max_dimension_px:
        height, width = result.shape[:2]
        longest_side = max(height, width)
        if longest_side > downscale_max_dimension_px:
            scale = downscale_max_dimension_px / longest_side
            new_size = (max(1, round(width * scale)), max(1, round(height * scale)))
            result = cv2.resize(result, new_size, interpolation=cv2.INTER_AREA)

    if blur_kernel_size and blur_kernel_size > 1:
        result = cv2.GaussianBlur(result, (blur_kernel_size, blur_kernel_size), blur_sigma)

    return result


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


def quantize_to_palette(image_bgr: np.ndarray, palette_colors: list, color_space: str = "lab") -> np.ndarray:
    """Map every pixel to the index of its nearest palette color.

    palette_colors: list of {"name": str, "hex": str} dicts, in config order.
    Returns an (H, W) int array of palette indices.
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

    labels = np.argmin(distances, axis=1)
    return labels.reshape(height, width)
