import unittest

import context  # noqa: F401
import numpy as np

from canny_edges import _trace_skeleton, simplify, suppress_border_artifacts


class TestTraceSkeleton(unittest.TestCase):
    def test_clustered_junction_uses_one_shared_endpoint_anchor(self):
        skeleton = np.zeros((9, 9), dtype=bool)
        skeleton[1:8, 4] = True
        skeleton[4, 1:8] = True

        strokes = _trace_skeleton(skeleton)

        self.assertEqual(len(strokes), 4)
        endpoint_counts = {}
        for points, _ in strokes:
            for endpoint in (points[0], points[-1]):
                endpoint_counts[endpoint] = endpoint_counts.get(endpoint, 0) + 1
        self.assertEqual(max(endpoint_counts.values()), 4)


class TestSimplify(unittest.TestCase):
    def test_minimum_epsilon_removes_pixel_scale_zigzag(self):
        points = [(0, 0), (1, 1), (2, 0), (3, 1), (4, 0)]

        ratio_only = simplify(
            points, closed=False, epsilon_ratio=0.0001, min_epsilon_px=0.0
        )
        with_floor = simplify(
            points, closed=False, epsilon_ratio=0.0001, min_epsilon_px=1.0
        )

        self.assertGreater(len(ratio_only), len(with_floor))


class TestBorderSuppression(unittest.TestCase):
    def test_removes_band_only_artifact_but_preserves_boundary_crossing_contour(self):
        mask = np.zeros((30, 30), dtype=bool)
        mask[2:28, 28] = True  # isolated right-edge artifact
        mask[0:16, 8] = True  # legitimate contour entering the interior

        cleaned, metrics = suppress_border_artifacts(mask, band_px=3)

        self.assertFalse(cleaned[:, 28].any())
        self.assertTrue(cleaned[:, 8].any())
        self.assertEqual(metrics["border_components_filtered"], 1)


if __name__ == "__main__":
    unittest.main()
