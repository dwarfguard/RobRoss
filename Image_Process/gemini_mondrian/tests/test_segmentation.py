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


class TestCleanLabelImage(unittest.TestCase):
    def test_zero_kernel_is_a_noop(self):
        label_image = np.zeros((10, 10), dtype=np.int64)
        label_image[2:5, 2:5] = 1
        cleaned = clean_label_image(label_image, PALETTE, morph_open_kernel_px=0)
        np.testing.assert_array_equal(cleaned, label_image)


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

    def test_a_gap_between_two_blocks_is_not_bridged(self):
        # Unlike image_to_mondrian's segmentation.py, this module has no
        # closing pass - a gap stays a gap, since Gemini's output doesn't
        # have lighting-induced gaps to bridge (see module docstring).
        label_image = np.zeros((30, 40), dtype=np.int64)
        label_image[5:25, 2:10] = 1
        label_image[5:25, 14:22] = 1  # 4px white gap between the two blocks

        kept, dropped_count, cleaned = segment_image(
            label_image, PALETTE, morph_open_kernel_px=0, min_area_px=1
        )
        red_kept = [r for r in kept if r["color_name"] == "red"]
        self.assertEqual(len(red_kept), 2)


if __name__ == "__main__":
    unittest.main()
