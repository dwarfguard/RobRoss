import unittest

import context  # noqa: F401
import numpy as np

from segmentation import (
    clean_label_image,
    filter_small_regions,
    label_connected_regions,
    segment_image,
)

PALETTE = [
    {"name": "white", "hex": "#ffffff"},
    {"name": "red", "hex": "#d62828"},
]


class TestLabelConnectedRegions(unittest.TestCase):
    def test_two_disconnected_blobs_are_two_regions(self):
        label_image = np.zeros((10, 10), dtype=np.int64)  # all "white" (index 0)
        label_image[1:3, 1:3] = 1  # red blob A, 4px
        label_image[6:9, 6:9] = 1  # red blob B, 9px

        regions = label_connected_regions(label_image, PALETTE)
        red_regions = [r for r in regions if r["color_name"] == "red"]

        self.assertEqual(len(red_regions), 2)
        self.assertEqual(sorted(r["area_px"] for r in red_regions), [4, 9])

    def test_touching_pixels_are_one_region(self):
        label_image = np.zeros((10, 10), dtype=np.int64)
        label_image[1:5, 1:5] = 1  # one solid 16px blob

        regions = label_connected_regions(label_image, PALETTE)
        red_regions = [r for r in regions if r["color_name"] == "red"]

        self.assertEqual(len(red_regions), 1)
        self.assertEqual(red_regions[0]["area_px"], 16)


class TestFilterSmallRegions(unittest.TestCase):
    def test_drops_below_threshold_keeps_above(self):
        regions = [{"area_px": 2}, {"area_px": 50}]
        kept, dropped_count = filter_small_regions(regions, min_area_px=10)
        self.assertEqual(kept, [{"area_px": 50}])
        self.assertEqual(dropped_count, 1)


class TestSegmentImage(unittest.TestCase):
    def test_speck_removed_by_morph_open_is_not_a_kept_region(self):
        label_image = np.zeros((20, 20), dtype=np.int64)
        label_image[5, 5] = 1  # single-pixel red speck
        label_image[10:15, 10:15] = 1  # solid 25px red block

        kept, dropped_count, cleaned = segment_image(
            label_image, PALETTE, morph_open_kernel_px=3, min_area_px=1
        )

        red_kept = [r for r in kept if r["color_name"] == "red"]
        self.assertEqual(len(red_kept), 1)
        # An elliptical structuring element can round off a square block's
        # corners (opening doesn't perfectly regrow sharp corners) - assert
        # it's substantially preserved rather than exactly 25px.
        self.assertGreaterEqual(red_kept[0]["area_px"], 15)
        self.assertLessEqual(red_kept[0]["area_px"], 25)
        self.assertEqual(cleaned.shape, label_image.shape)


def _two_blocks_with_gap(gap_width: int) -> np.ndarray:
    """A red region split into two blocks by a white gap - simulates a
    single real object (e.g. a hat) getting cut apart by internal
    lighting/shadow variation during quantization."""
    label_image = np.zeros((30, 40), dtype=np.int64)
    left_end = 10
    gap_start = left_end
    gap_end = gap_start + gap_width
    label_image[5:25, 2:left_end] = 1
    label_image[5:25, gap_end : gap_end + 8] = 1
    return label_image


class TestCloseKernel(unittest.TestCase):
    def test_close_bridges_a_gap_cut_into_one_region(self):
        label_image = _two_blocks_with_gap(gap_width=4)
        without_close = label_connected_regions(label_image, PALETTE)
        self.assertEqual(len([r for r in without_close if r["color_name"] == "red"]), 2)

        closed = clean_label_image(label_image, PALETTE, morph_open_kernel_px=0, morph_close_kernel_px=9)
        red_regions = [r for r in label_connected_regions(closed, PALETTE) if r["color_name"] == "red"]
        self.assertEqual(len(red_regions), 1)

    def test_close_does_not_bridge_gaps_wider_than_kernel(self):
        label_image = _two_blocks_with_gap(gap_width=10)
        closed = clean_label_image(label_image, PALETTE, morph_open_kernel_px=0, morph_close_kernel_px=5)
        red_regions = [r for r in label_connected_regions(closed, PALETTE) if r["color_name"] == "red"]
        self.assertEqual(len(red_regions), 2)

    def test_close_disabled_by_default_matches_legacy_behavior(self):
        label_image = _two_blocks_with_gap(gap_width=4)
        without_param = clean_label_image(label_image, PALETTE, morph_open_kernel_px=0)
        with_zero = clean_label_image(label_image, PALETTE, morph_open_kernel_px=0, morph_close_kernel_px=0)
        np.testing.assert_array_equal(without_param, with_zero)
        np.testing.assert_array_equal(without_param, label_image)

    def test_segment_image_accepts_close_kernel(self):
        label_image = _two_blocks_with_gap(gap_width=4)
        kept, dropped_count, cleaned = segment_image(
            label_image, PALETTE, morph_open_kernel_px=0, min_area_px=1, morph_close_kernel_px=9
        )
        red_kept = [r for r in kept if r["color_name"] == "red"]
        self.assertEqual(len(red_kept), 1)


class TestProtectedMask(unittest.TestCase):
    def test_protected_mask_overrides_closing_result(self):
        label_image = _two_blocks_with_gap(gap_width=6)  # gap columns 10:16

        protected_mask = np.zeros(label_image.shape, dtype=bool)
        protected_mask[10:20, 12:14] = True  # a strip in the middle of the gap

        closed = clean_label_image(
            label_image, PALETTE, morph_open_kernel_px=0, morph_close_kernel_px=13,
            protected_mask=protected_mask,
        )

        # The protected strip must keep its original (white) classification,
        # regardless of what closing decided for that pixel.
        self.assertTrue(np.all(closed[protected_mask] == label_image[protected_mask]))

        # The rest of the gap (not protected) should still get bridged to red.
        red_regions = [r for r in label_connected_regions(closed, PALETTE) if r["color_name"] == "red"]
        self.assertEqual(len(red_regions), 1)

    def test_protected_mask_none_is_a_no_op(self):
        label_image = _two_blocks_with_gap(gap_width=4)
        with_none = clean_label_image(
            label_image, PALETTE, morph_open_kernel_px=0, morph_close_kernel_px=9, protected_mask=None
        )
        without_param = clean_label_image(label_image, PALETTE, morph_open_kernel_px=0, morph_close_kernel_px=9)
        np.testing.assert_array_equal(with_none, without_param)


if __name__ == "__main__":
    unittest.main()
