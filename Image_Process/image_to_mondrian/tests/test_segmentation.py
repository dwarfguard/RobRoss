import unittest

import context  # noqa: F401
import numpy as np

from segmentation import filter_small_regions, label_connected_regions, segment_image

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


if __name__ == "__main__":
    unittest.main()
