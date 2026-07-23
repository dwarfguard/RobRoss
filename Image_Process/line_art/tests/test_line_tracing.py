import tempfile
import unittest
from pathlib import Path

import context
import cv2
import numpy as np

from line_tracing import (
    binarize,
    close_mask,
    extract_strokes,
    prune_skeleton_spurs,
    prune_spurs,
    simplify,
    skeletonize_mask,
)


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


class TestCloseMask(unittest.TestCase):
    def test_zero_kernel_returns_mask_unchanged(self):
        mask = np.zeros((5, 5), dtype=bool)
        mask[2, 1] = True
        mask[2, 3] = True  # a 1px gap at (2, 2)

        result = close_mask(mask, kernel_px=0)

        self.assertIs(result, mask)

    def test_positive_kernel_bridges_a_small_gap(self):
        mask = np.zeros((5, 7), dtype=bool)
        mask[2, 1:3] = True
        mask[2, 4:6] = True  # a 1px gap at column 3

        result = close_mask(mask, kernel_px=3)

        self.assertTrue(result[2, 3])
        self.assertEqual(result.dtype, bool)


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

    def test_adjacent_junction_pixels_cluster_into_one_closed_loop(self):
        # Regression test using the actual pixel region that motivated this
        # fix: the inner triangular counter of the "A" in
        # Image_Process/assets/ai.png. Before node clustering, the sharp
        # apex and a rounded corner each rasterize into node pixels
        # adjacent to each other, fragmenting what should be one closed
        # triangle into several open arcs (see line_art plan history). With
        # clustering, extract_strokes() on its own (no skeleton spur
        # pruning yet) should reduce that fragmentation - the corner that
        # rasterized into 2-3 adjacent junction pixels no longer produces
        # its own spurious near-duplicate paths between them.
        image_path = context.REPO_ROOT / "Image_Process" / "assets" / "ai.png"
        mask, _ = binarize(image_path, threshold=200)
        mask = close_mask(mask, kernel_px=2)
        skeleton = skeletonize_mask(mask)
        roi = skeleton[140:215, 1000:1060]  # the "A" counter's bounding box

        strokes = extract_strokes(roi)

        # Without clustering (the pre-fix extract_strokes) this region
        # fragments into 7 open strokes, 0 closed. Clustering the corner's
        # 2-3 adjacent junction pixels together removes several of the
        # spurious inter-cluster micro-edges, cutting that down to 4 -
        # still fragmented (the sharp apex needs skeleton-level spur
        # pruning too, see TestPruneSkeletonSpurs and
        # generate_line_art_paths.py's pipeline), but clustering alone is
        # already a measurable improvement in isolation.
        self.assertLessEqual(len(strokes), 4)


class TestPruneSkeletonSpurs(unittest.TestCase):
    def test_short_dead_end_branch_is_removed(self):
        # Two diagonal arms meeting at a corner (like the sharp apex that
        # motivated this function - see line_art plan history), plus a
        # short 2px whisker continuing straight past the corner. The
        # whisker should be removed; both arms stay intact and the corner
        # pixel itself is kept (it's still a real junction, not a spur).
        skeleton = np.zeros((8, 12), dtype=bool)
        for point in [(4, 5), (3, 4), (2, 3), (1, 2), (0, 1)]:
            skeleton[point] = True  # arm 1
        for point in [(4, 5), (3, 6), (2, 7), (1, 8), (0, 9)]:
            skeleton[point] = True  # arm 2
        skeleton[5, 5] = True
        skeleton[6, 5] = True  # whisker off the corner, length 2px

        pruned = prune_skeleton_spurs(skeleton, min_length_px=5.0)

        self.assertFalse(pruned[5, 5])
        self.assertFalse(pruned[6, 5])
        self.assertTrue(pruned[4, 5])  # the corner itself stays
        for point in [(3, 4), (2, 3), (1, 2), (0, 1), (3, 6), (2, 7), (1, 8), (0, 9)]:
            self.assertTrue(pruned[point])

    def test_long_branch_is_kept(self):
        skeleton = np.zeros((10, 10), dtype=bool)
        skeleton[5, 1:9] = True
        skeleton[1:5, 5] = True  # a 4px branch off the main line

        pruned = prune_skeleton_spurs(skeleton, min_length_px=3.0)

        for y in range(1, 5):
            self.assertTrue(pruned[y, 5])

    def test_pruning_a_whisker_lets_a_ring_close(self):
        # The corner pixel (1, 3) has a whisker attached; once the whisker
        # is pruned, extract_strokes() should treat that corner as a
        # simple pass-through again and recover the full closed ring.
        skeleton = np.zeros((7, 7), dtype=bool)
        ring_cells = [
            (1, 2), (1, 3), (1, 4),
            (2, 5), (3, 5),
            (4, 4), (4, 3), (4, 2),
            (3, 1), (2, 1),
        ]
        for y, x in ring_cells:
            skeleton[y, x] = True
        skeleton[0, 3] = True  # whisker off (1, 3), length ~1px

        pruned = prune_skeleton_spurs(skeleton, min_length_px=3.0)
        strokes = extract_strokes(pruned)

        self.assertEqual(len(strokes), 1)
        self.assertTrue(strokes[0][1])


class TestPruneSpurs(unittest.TestCase):
    def test_short_open_branch_is_dropped(self):
        strokes = [
            ([(0, 0), (1, 0), (2, 0)], False),  # length 2px, open
            ([(0, 0), (10, 0)], False),  # length 10px, open
        ]
        pruned = prune_spurs(strokes, min_length_px=5.0)
        self.assertEqual(len(pruned), 1)
        self.assertEqual(pruned[0][0], [(0, 0), (10, 0)])

    def test_long_closed_loop_is_kept(self):
        strokes = [
            ([(0, 0), (10, 0), (10, 10), (0, 10)], True),  # long, closed
        ]
        pruned = prune_spurs(strokes, min_length_px=5.0)
        self.assertEqual(len(pruned), 1)

    def test_short_closed_loop_is_dropped(self):
        # A closed loop this short is degenerate pixel-level noise (see
        # extract_strokes' node clustering), not a real small feature.
        strokes = [
            ([(0, 0), (1, 0), (1, 1), (0, 1)], True),  # short, closed
        ]
        pruned = prune_spurs(strokes, min_length_px=100.0)
        self.assertEqual(len(pruned), 0)


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
