import unittest

import context  # noqa: F401
import cv2
import numpy as np

from border_tracing import simplify, trace_region_contours


class TestTraceRegionContours(unittest.TestCase):
    def test_solid_rectangle_has_one_closed_contour(self):
        mask = np.zeros((20, 20), dtype=bool)
        mask[5:15, 5:15] = True

        contours = trace_region_contours(mask)

        self.assertEqual(len(contours), 1)
        points, closed = contours[0]
        self.assertTrue(closed)
        self.assertGreaterEqual(len(points), 3)

    def test_donut_shape_has_two_closed_contours(self):
        mask = np.zeros((40, 40), dtype=np.uint8)
        cv2.circle(mask, (20, 20), 15, 1, -1)
        cv2.circle(mask, (20, 20), 6, 0, -1)  # punch a hole out of the middle

        contours = trace_region_contours(mask.astype(bool))

        self.assertEqual(len(contours), 2)
        self.assertTrue(all(closed for _points, closed in contours))

    def test_empty_mask_has_no_contours(self):
        mask = np.zeros((10, 10), dtype=bool)
        self.assertEqual(trace_region_contours(mask), [])

    def test_two_disconnected_blobs_produce_two_contours(self):
        mask = np.zeros((20, 20), dtype=bool)
        mask[1:4, 1:4] = True
        mask[10:15, 10:15] = True

        contours = trace_region_contours(mask)
        self.assertEqual(len(contours), 2)


class TestSimplify(unittest.TestCase):
    def test_reduces_point_count_on_a_straight_line(self):
        points = [(x, 0) for x in range(50)]
        simplified = simplify(points, closed=False, epsilon_ratio=0.02)
        self.assertLess(len(simplified), len(points))
        self.assertGreaterEqual(len(simplified), 2)

    def test_short_lines_are_returned_unchanged(self):
        points = [(0, 0), (1, 1)]
        simplified = simplify(points, closed=False, epsilon_ratio=0.02)
        self.assertEqual(simplified, points)


if __name__ == "__main__":
    unittest.main()
