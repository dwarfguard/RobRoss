import tempfile
import unittest
from pathlib import Path

import context  # noqa: F401
import cv2
import numpy as np
from PIL import Image

from canny_edges import (
    _trace_skeleton,
    line_art_mask,
    read_grayscale,
    read_grayscale_with_metadata,
    simplify,
    suppress_border_artifacts,
)


class TestImageLoading(unittest.TestCase):
    def test_orientation_6_jpeg_is_rotated_before_grayscale_use(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "orientation-6.jpg"
            pixels = np.full((20, 40, 3), 255, dtype=np.uint8)
            pixels[2:8, 4:12] = (0, 0, 0)
            image = Image.fromarray(pixels)
            exif = image.getexif()
            exif[274] = 6
            image.save(path, exif=exif, quality=100, subsampling=0)

            grayscale, metadata = read_grayscale_with_metadata(path)

        self.assertEqual(grayscale.shape, (40, 20))
        self.assertEqual(metadata["source_exif_orientation"], 6)
        self.assertTrue(metadata["source_orientation_normalized"])
        # The raw upper-left mark moves to the oriented upper-right.
        self.assertLess(float(grayscale[4:12, 12:19].mean()), 60.0)

    def test_ordinary_jpeg_and_grayscale_inputs_keep_their_dimensions(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            color_path = root / "ordinary.jpg"
            gray_path = root / "gray.png"
            cv2.imwrite(str(color_path), np.full((12, 21, 3), 80, dtype=np.uint8))
            cv2.imwrite(str(gray_path), np.full((9, 17), 120, dtype=np.uint8))

            color, color_metadata = read_grayscale_with_metadata(color_path)
            gray = read_grayscale(gray_path)

        self.assertEqual(color.shape, (12, 21))
        self.assertEqual(gray.shape, (9, 17))
        self.assertEqual(color_metadata["source_exif_orientation"], 1)
        self.assertFalse(color_metadata["source_orientation_normalized"])

    def test_transparent_png_is_composited_on_white(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "transparent.png"
            bgra = np.zeros((10, 12, 4), dtype=np.uint8)
            bgra[3:7, 4:8, :3] = 0
            bgra[3:7, 4:8, 3] = 255
            cv2.imwrite(str(path), bgra)

            grayscale = read_grayscale(path)

        self.assertEqual(grayscale.shape, (10, 12))
        self.assertEqual(int(grayscale[0, 0]), 255)
        self.assertEqual(int(grayscale[4, 5]), 0)


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


class TestLineArtMask(unittest.TestCase):
    def test_configured_threshold_is_exact_and_preserves_antialiasing(self):
        grayscale = np.array([[0, 164, 209, 210, 211, 254, 255]], dtype=np.uint8)

        mask = line_art_mask(grayscale, threshold=210)

        np.testing.assert_array_equal(
            mask,
            np.array([[True, True, True, True, False, False, False]]),
        )

    def test_explicit_threshold_override_is_honored(self):
        grayscale = np.array([[179, 180, 181, 255]], dtype=np.uint8)

        mask = line_art_mask(grayscale, threshold=180)

        np.testing.assert_array_equal(
            mask, np.array([[True, True, False, False]])
        )


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
