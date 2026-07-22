import tempfile
import unittest
from pathlib import Path

import context  # noqa: F401
import cv2
import numpy as np

from line_tracing import binarize, extract_strokes, prune_spurs, simplify, skeletonize_mask


class TestBinarize(unittest.TestCase):
    def test_dark_pixels_below_threshold_are_true(self):
        # 10x10 white image with a 4x4 black square in the middle.
        image = np.full((10, 10), 255, dtype=np.uint8)
        image[3:7, 3:7] = 0

        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "square.png"
            cv2.imwrite(str(path), image)
            mask, image_size = binarize(path, threshold=128)

        self.assertEqual(image_size, (10, 10))
        self.assertTrue(mask[5, 5])
        self.assertFalse(mask[0, 0])

    def test_missing_file_raises_file_not_found(self):
        with self.assertRaises(FileNotFoundError):
            binarize("/nonexistent/path/does_not_exist.png", threshold=128)

    def test_rgba_composites_transparent_pixels_onto_white(self):
        # A fully transparent black pixel should read as white background
        # (not a stroke), not as a dark foreground pixel.
        image = np.zeros((4, 4, 4), dtype=np.uint8)
        image[:, :, 3] = 255  # opaque
        image[1, 1] = [0, 0, 0, 0]  # fully transparent "black" pixel

        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "rgba.png"
            cv2.imwrite(str(path), image)
            mask, _ = binarize(path, threshold=128)

        self.assertFalse(mask[1, 1])


class TestSkeletonizeMask(unittest.TestCase):
    def test_thick_bar_thins_to_single_pixel_rows(self):
        mask = np.zeros((10, 20), dtype=bool)
        mask[4:7, 2:18] = True  # a 3px-tall horizontal bar

        skeleton = skeletonize_mask(mask)

        # Every column that has any skeleton pixel should have exactly one.
        for col in range(20):
            column_hits = skeleton[:, col].sum()
            self.assertLessEqual(column_hits, 1)
        self.assertGreater(skeleton.sum(), 0)


class TestExtractStrokes(unittest.TestCase):
    def test_straight_line_is_one_open_stroke(self):
        skeleton = np.zeros((5, 10), dtype=bool)
        skeleton[2, 1:9] = True

        strokes = extract_strokes(skeleton)

        self.assertEqual(len(strokes), 1)
        points, closed = strokes[0]
        self.assertFalse(closed)
        self.assertEqual(len(points), 8)
        xs = sorted(p[0] for p in points)
        self.assertEqual(xs, list(range(1, 9)))

    def test_ring_with_no_junctions_is_closed(self):
        skeleton = np.zeros((7, 7), dtype=bool)
        # A simple 4-connected ring (no branches, every pixel has degree 2).
        ring_cells = [
            (1, 2), (1, 3), (1, 4),
            (2, 5), (3, 5),
            (4, 4), (4, 3), (4, 2),
            (3, 1), (2, 1),
        ]
        for y, x in ring_cells:
            skeleton[y, x] = True

        strokes = extract_strokes(skeleton)

        self.assertEqual(len(strokes), 1)
        points, closed = strokes[0]
        self.assertTrue(closed)
        # A closed loop's walk returns to its starting pixel, so the point
        # list re-visits the start once at the end (ring_cells + 1).
        self.assertEqual(len(points), len(ring_cells) + 1)
        self.assertEqual(points[0], points[-1])

    def test_empty_skeleton_returns_no_strokes(self):
        skeleton = np.zeros((5, 5), dtype=bool)
        self.assertEqual(extract_strokes(skeleton), [])


class TestPruneSpurs(unittest.TestCase):
    def test_short_open_branch_is_dropped(self):
        strokes = [
            ([(0, 0), (1, 0), (2, 0)], False),  # length 2px, open
            ([(0, 0), (10, 0)], False),  # length 10px, open
        ]
        pruned = prune_spurs(strokes, min_length_px=5.0)
        self.assertEqual(len(pruned), 1)
        self.assertEqual(pruned[0][0], [(0, 0), (10, 0)])

    def test_short_closed_loop_is_kept(self):
        strokes = [
            ([(0, 0), (1, 0), (1, 1), (0, 1)], True),  # short, but closed
        ]
        pruned = prune_spurs(strokes, min_length_px=100.0)
        self.assertEqual(len(pruned), 1)


class TestSimplify(unittest.TestCase):
    def test_colinear_points_reduce_to_endpoints(self):
        points = [(x, 0) for x in range(10)]
        simplified = simplify(points, closed=False, epsilon_ratio=0.1)
        self.assertEqual(len(simplified), 2)
        self.assertEqual(simplified[0], (0, 0))
        self.assertEqual(simplified[-1], (9, 0))

    def test_fewer_than_three_points_returned_unchanged(self):
        points = [(0, 0), (5, 5)]
        simplified = simplify(points, closed=False, epsilon_ratio=0.1)
        self.assertEqual(simplified, [(0, 0), (5, 5)])


if __name__ == "__main__":
    unittest.main()
