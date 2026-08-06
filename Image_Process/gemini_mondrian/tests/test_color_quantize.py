import unittest

import context  # noqa: F401
import numpy as np

from color_quantize import hex_to_bgr, preprocess, quantize_to_palette

PALETTE = [
    {"name": "white", "hex": "#ffffff"},
    {"name": "yellow", "hex": "#f1c40f"},
    {"name": "red", "hex": "#d62828"},
    {"name": "blue", "hex": "#1d3557"},
    {"name": "black", "hex": "#000000"},
]


class TestQuantizeToPalette(unittest.TestCase):
    def test_pure_palette_colors_map_to_their_own_index(self):
        for color_space in ("rgb", "lab"):
            with self.subTest(color_space=color_space):
                swatches = np.array(
                    [hex_to_bgr(c["hex"]) for c in PALETTE], dtype=np.uint8
                ).reshape(1, len(PALETTE), 3)
                labels = quantize_to_palette(swatches, PALETTE, color_space)
                self.assertEqual(labels.tolist(), [list(range(len(PALETTE)))])

    def test_near_pure_color_still_maps_to_nearest_palette_entry(self):
        # Slightly off pure red - still closer to red than to any other swatch.
        near_red = np.array(hex_to_bgr("#d02020"), dtype=np.uint8).reshape(1, 1, 3)
        labels = quantize_to_palette(near_red, PALETTE, "lab")
        self.assertEqual(labels[0, 0], 2)  # red is index 2

    def test_output_shape_matches_input(self):
        image = np.zeros((5, 7, 3), dtype=np.uint8)
        labels = quantize_to_palette(image, PALETTE, "lab")
        self.assertEqual(labels.shape, (5, 7))


class TestPreprocess(unittest.TestCase):
    def test_no_args_is_a_noop(self):
        rng = np.random.default_rng(0)
        image = rng.integers(0, 255, (30, 30, 3), dtype=np.uint8)
        np.testing.assert_array_equal(preprocess(image), image)

    def test_downscales_when_over_the_max_dimension(self):
        image = np.zeros((200, 100, 3), dtype=np.uint8)
        result = preprocess(image, downscale_max_dimension_px=100)
        self.assertEqual(max(result.shape[:2]), 100)

    def test_leaves_image_alone_when_already_under_the_max_dimension(self):
        image = np.zeros((50, 30, 3), dtype=np.uint8)
        result = preprocess(image, downscale_max_dimension_px=100)
        self.assertEqual(result.shape, image.shape)

    def test_blur_kernel_size_zero_is_disabled_by_default(self):
        rng = np.random.default_rng(1)
        image = rng.integers(0, 255, (20, 20, 3), dtype=np.uint8)
        np.testing.assert_array_equal(preprocess(image, blur_kernel_size=0), image)


if __name__ == "__main__":
    unittest.main()
