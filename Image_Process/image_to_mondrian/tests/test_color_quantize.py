import unittest

import context  # noqa: F401
import cv2
import numpy as np

from color_quantize import compute_adaptive_chroma_threshold, hex_to_bgr, preprocess, quantize_to_palette

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


class TestNeutralChromaGating(unittest.TestCase):
    # Mid-tone skin-like BGR value: moderately saturated (Lab chroma ~35),
    # closer to red than white/black under plain nearest-neighbor.
    SKIN_LIKE_BGR = hex_to_bgr("#c68863")

    def test_gating_disabled_by_default_matches_legacy_behavior(self):
        image = np.array(self.SKIN_LIKE_BGR, dtype=np.uint8).reshape(1, 1, 3)
        without_param = quantize_to_palette(image, PALETTE, "lab")
        with_zero_threshold = quantize_to_palette(image, PALETTE, "lab", 0.0)
        self.assertEqual(without_param.tolist(), with_zero_threshold.tolist())
        # Confirms the pre-gating baseline: this skin-like pixel is closer
        # to red (index 2) than to white under plain nearest-neighbor.
        self.assertEqual(without_param[0, 0], 2)

    def test_moderately_saturated_skin_tone_gates_to_white_when_threshold_set(self):
        image = np.array(self.SKIN_LIKE_BGR, dtype=np.uint8).reshape(1, 1, 3)
        labels = quantize_to_palette(image, PALETTE, "lab", neutral_chroma_threshold=40.0)
        # White is index 0; this skin tone is light, so lightness should
        # pick white over black within the neutral bucket.
        self.assertEqual(labels[0, 0], 0)

    def test_saturated_red_stays_red_regardless_of_threshold(self):
        near_red = np.array(hex_to_bgr("#d02020"), dtype=np.uint8).reshape(1, 1, 3)
        for threshold in (20.0, 30.0, 40.0):
            with self.subTest(threshold=threshold):
                labels = quantize_to_palette(near_red, PALETTE, "lab", threshold)
                self.assertEqual(labels[0, 0], 2)  # red

    def test_gating_noop_when_palette_lacks_a_neutral_or_a_chromatic_bucket(self):
        all_chromatic = [c for c in PALETTE if c["name"] in ("red", "blue", "yellow")]
        image = np.array(hex_to_bgr("#c68863"), dtype=np.uint8).reshape(1, 1, 3)
        gated = quantize_to_palette(image, all_chromatic, "lab", neutral_chroma_threshold=40.0)
        ungated = quantize_to_palette(image, all_chromatic, "lab", 0.0)
        self.assertEqual(gated.tolist(), ungated.tolist())


class TestPreprocessBilateral(unittest.TestCase):
    def test_bilateral_disabled_by_default_matches_legacy_behavior(self):
        rng = np.random.default_rng(0)
        image = rng.integers(0, 255, (30, 30, 3), dtype=np.uint8)
        without_param = preprocess(image, blur_kernel_size=5)
        with_zero_bilateral = preprocess(image, blur_kernel_size=5, bilateral_d=0)
        np.testing.assert_array_equal(without_param, with_zero_bilateral)

    def test_bilateral_smooths_flat_region_but_preserves_a_strong_edge(self):
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        image[:, :50] = (40, 40, 210)  # BGR red-ish, left half
        image[:, 50:] = (180, 90, 40)  # BGR blue-ish, right half
        rng = np.random.default_rng(1)
        noise = rng.normal(0, 8, image.shape).astype(np.int16)
        noisy = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        result = preprocess(noisy, bilateral_d=9, bilateral_sigma_color=75, bilateral_sigma_space=75)

        # Noise within the flat left region should be substantially reduced.
        self.assertLess(result[:, :45].astype(np.float32).std(), noisy[:, :45].astype(np.float32).std())

        # The real edge between the two halves should still be a strong,
        # sharp color jump - not smoothed into a gradual gradient.
        left_of_edge = result[:, 45:49].mean(axis=(0, 1))
        right_of_edge = result[:, 51:55].mean(axis=(0, 1))
        self.assertGreater(np.linalg.norm(left_of_edge.astype(np.float32) - right_of_edge.astype(np.float32)), 100)


class TestAdaptiveChromaThreshold(unittest.TestCase):
    def test_matches_numpy_percentile_directly(self):
        rng = np.random.default_rng(2)
        image = rng.integers(0, 255, (20, 20, 3), dtype=np.uint8)

        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB).astype(np.float32)
        expected_chroma = np.sqrt((lab[..., 1] - 128.0) ** 2 + (lab[..., 2] - 128.0) ** 2)

        for percentile in (10, 50, 90):
            with self.subTest(percentile=percentile):
                expected = float(np.percentile(expected_chroma, percentile))
                actual = compute_adaptive_chroma_threshold(image, percentile)
                self.assertAlmostEqual(actual, expected, places=3)

    def test_all_neutral_image_gives_near_zero_threshold(self):
        image = np.full((10, 10, 3), 128, dtype=np.uint8)  # gray, ~zero chroma
        threshold = compute_adaptive_chroma_threshold(image, 50)
        self.assertLess(threshold, 5.0)


if __name__ == "__main__":
    unittest.main()
