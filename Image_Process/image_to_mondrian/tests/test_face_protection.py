import unittest

import context  # noqa: F401
import numpy as np

from face_protection import detect_protected_face_mask

LENNA_PATH = context.MODULE_DIR.parent / "sketch" / "assets" / "lenna.png"


def _mediapipe_available() -> bool:
    try:
        import mediapipe  # noqa: F401
    except ImportError:
        return False
    return True


class TestDetectProtectedFaceMask(unittest.TestCase):
    def test_returns_none_when_no_face_in_image(self):
        if not _mediapipe_available():
            self.skipTest("mediapipe not installed - face protection is an optional dependency")
        # A flat solid-color image has no face for mediapipe to detect.
        image = np.full((50, 50, 3), 200, dtype=np.uint8)
        mask = detect_protected_face_mask(image)
        self.assertIsNone(mask)

    def test_detects_face_landmarks_on_a_real_photo(self):
        if not _mediapipe_available():
            self.skipTest("mediapipe not installed - face protection is an optional dependency")
        import cv2

        image = cv2.imread(str(LENNA_PATH))
        self.assertIsNotNone(image, f"fixture image not found: {LENNA_PATH}")

        mask = detect_protected_face_mask(image, margin_px=3)

        self.assertIsNotNone(mask)
        self.assertEqual(mask.shape, image.shape[:2])
        self.assertEqual(mask.dtype, bool)

        total_pixels = mask.size
        protected_pixels = int(mask.sum())
        # Should mark a real, non-trivial chunk of the image (eyes/eyebrows/
        # lips) but nowhere near the whole photo.
        self.assertGreater(protected_pixels, 100)
        self.assertLess(protected_pixels, total_pixels * 0.2)

        # Landmarks should be roughly in the upper-middle of the frame,
        # where a portrait's face typically sits - not a hardcoded
        # Lenna-specific check, just a sanity bound on plausible face
        # position within the frame.
        ys, _xs = np.where(mask)
        self.assertLess(ys.mean(), image.shape[0] * 0.75)


if __name__ == "__main__":
    unittest.main()
