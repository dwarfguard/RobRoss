import unittest

import context  # noqa: F401
import numpy as np

from region_fill import (
    compute_stripe_rows,
    erode_mask,
    find_row_intervals,
    region_to_pixel_strokes,
)


class TestFindRowIntervals(unittest.TestCase):
    def test_single_interval(self):
        row = np.array([0, 0, 1, 1, 1, 0, 0], dtype=bool)
        self.assertEqual(find_row_intervals(row), [(2, 5)])

    def test_multiple_disjoint_intervals(self):
        row = np.array([1, 1, 0, 0, 1, 1, 1, 0], dtype=bool)
        self.assertEqual(find_row_intervals(row), [(0, 2), (4, 7)])

    def test_all_off_row_has_no_intervals(self):
        row = np.zeros(5, dtype=bool)
        self.assertEqual(find_row_intervals(row), [])

    def test_all_on_row_is_one_interval(self):
        row = np.ones(5, dtype=bool)
        self.assertEqual(find_row_intervals(row), [(0, 5)])


class TestErodeMask(unittest.TestCase):
    def test_zero_erosion_is_a_no_op(self):
        mask = np.zeros((10, 10), dtype=bool)
        mask[2:8, 2:8] = True
        np.testing.assert_array_equal(erode_mask(mask, 0), mask)

    def test_erosion_shrinks_solid_square(self):
        mask = np.zeros((20, 20), dtype=bool)
        mask[5:15, 5:15] = True  # 10x10 square
        eroded = erode_mask(mask, erosion_px=2)
        self.assertLess(eroded.sum(), mask.sum())
        # Everything left should still be inside the original mask.
        self.assertTrue(np.all(mask[eroded]))


class TestComputeStripeRows(unittest.TestCase):
    def test_short_region_gets_single_center_row(self):
        rows = compute_stripe_rows(top=10, height=3, tool_width_px=5, stroke_overlap_ratio=0.0)
        self.assertEqual(rows, [round(10 + 3 / 2)])

    def test_rows_cover_top_and_bottom_edges(self):
        rows = compute_stripe_rows(top=0, height=20, tool_width_px=3, stroke_overlap_ratio=0.0)
        self.assertGreaterEqual(rows[0], 0)
        self.assertLessEqual(rows[-1], 19)
        # Last row must be within half a tool-width of the bottom edge.
        self.assertLessEqual(19 - rows[-1], 1.5 + 1e-6)


class TestRegionToPixelStrokes(unittest.TestCase):
    def test_solid_rectangle_mask_is_fully_covered_with_no_gaps(self):
        mask = np.zeros((30, 30), dtype=bool)
        mask[5:25, 5:25] = True  # 20x20 solid square

        strokes = region_to_pixel_strokes(
            mask, tool_width_px=4, stroke_overlap_ratio=0.1, mask_erosion_px=0
        )
        self.assertGreater(len(strokes), 0)

        rows_hit = sorted({p0[1] for p0, _ in strokes})
        # Consecutive stripe rows should never be more than one tool-width
        # apart, or a gap would be left unpainted.
        for a, b in zip(rows_hit, rows_hit[1:]):
            self.assertLessEqual(b - a, 4 + 1e-6)

    def test_empty_mask_produces_no_strokes(self):
        mask = np.zeros((10, 10), dtype=bool)
        strokes = region_to_pixel_strokes(mask, tool_width_px=3, stroke_overlap_ratio=0.0, mask_erosion_px=0)
        self.assertEqual(strokes, [])

    def test_concave_shape_produces_multiple_intervals_on_some_row(self):
        # A "U" shape: two vertical bars connected only at the bottom.
        mask = np.zeros((20, 20), dtype=bool)
        mask[0:15, 2:5] = True
        mask[0:15, 12:15] = True
        mask[15:18, 2:15] = True

        strokes = region_to_pixel_strokes(
            mask, tool_width_px=2, stroke_overlap_ratio=0.0, mask_erosion_px=0
        )
        # At least one row (in the two-armed section) must have produced
        # two separate strokes rather than one that bridges the gap.
        by_row = {}
        for p0, p1 in strokes:
            by_row.setdefault(p0[1], []).append((p0, p1))
        self.assertTrue(any(len(v) >= 2 for v in by_row.values()))


if __name__ == "__main__":
    unittest.main()
