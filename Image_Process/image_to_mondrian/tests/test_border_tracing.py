import unittest

import context  # noqa: F401
import numpy as np

from border_tracing import compute_boundary_mask, simplify, trace_boundary_strokes


class TestComputeBoundaryMask(unittest.TestCase):
    def test_flags_the_vertical_split_between_two_colors(self):
        label_image = np.zeros((10, 10), dtype=np.int64)
        label_image[:, 5:] = 1  # right half is a different color

        boundary = compute_boundary_mask(label_image)
        # Column 5 (where the color changes) should be flagged; column 2
        # (deep inside one color) should not.
        self.assertTrue(boundary[:, 5].all())
        self.assertFalse(boundary[:, 2].any())

    def test_uniform_image_has_no_boundary(self):
        label_image = np.full((10, 10), 3, dtype=np.int64)
        boundary = compute_boundary_mask(label_image)
        self.assertFalse(boundary.any())


class TestTraceBoundaryStrokes(unittest.TestCase):
    def test_traces_a_single_straight_boundary(self):
        label_image = np.zeros((20, 20), dtype=np.int64)
        label_image[:, 10:] = 1
        boundary = compute_boundary_mask(label_image)

        strokes, image_size = trace_boundary_strokes(boundary)

        self.assertEqual(image_size, (20, 20))
        self.assertGreater(len(strokes), 0)
        # Every traced point should lie on the boundary column.
        for points_xy, _closed in strokes:
            for x, _y in points_xy:
                self.assertIn(x, (9, 10))

    def test_no_boundary_produces_no_strokes(self):
        boundary = np.zeros((10, 10), dtype=bool)
        strokes, _ = trace_boundary_strokes(boundary)
        self.assertEqual(strokes, [])


class TestSimplify(unittest.TestCase):
    def test_reduces_point_count_on_a_straight_line(self):
        # A perfectly straight line with many redundant intermediate points.
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
