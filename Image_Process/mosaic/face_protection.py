"""Optional face-aware protection: uses a pretrained MediaPipe Face Mesh
model (no training, CPU inference) to find eye/eyebrow/lip landmark regions,
so morphological closing (see segmentation.py) can be run with a large
enough kernel to properly bridge gaps in large objects (e.g. a hat split by
shadow) without risking merging small facial features into nearby regions -
closing has no notion of "this is an eye", only pixel geometry, so a purely
classical fix can't protect a semantically-small-but-topologically-large
feature (an eye's own pixel blob is often already connected to nearby hair
through eyebrow/lash strands before any morphology runs at all).

This is an opt-in dependency (mediapipe) - the rest of the module works
without it; only import it when this feature is actually requested.
"""

import cv2
import numpy as np


def detect_protected_face_mask(image_bgr: np.ndarray, margin_px: int = 3):
    """Returns a boolean (H, W) mask marking eye/eyebrow/lip landmark regions
    across all detected faces, dilated by margin_px - or None if mediapipe
    isn't installed or no face is detected (not every photo has one; the
    caller should treat that as "nothing to protect", not an error).
    """
    try:
        import mediapipe as mp
    except ImportError:
        return None

    mp_face_mesh = mp.solutions.face_mesh
    with mp_face_mesh.FaceMesh(
        static_image_mode=True, max_num_faces=5, refine_landmarks=True
    ) as face_mesh:
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        result = face_mesh.process(rgb)

    if not result.multi_face_landmarks:
        return None

    height, width = image_bgr.shape[:2]
    mask = np.zeros((height, width), dtype=np.uint8)

    # Each landmark group's own convex hull is filled separately - combining
    # every point from every group into one shared hull first would connect
    # the left eye, right eye, and lips into a single blob, defeating the
    # point of tracking them as distinct protected regions.
    landmark_groups = [
        mp_face_mesh.FACEMESH_LEFT_EYE,
        mp_face_mesh.FACEMESH_RIGHT_EYE,
        mp_face_mesh.FACEMESH_LEFT_EYEBROW,
        mp_face_mesh.FACEMESH_RIGHT_EYEBROW,
        mp_face_mesh.FACEMESH_LIPS,
    ]

    for face_landmarks in result.multi_face_landmarks:
        points = [(landmark.x * width, landmark.y * height) for landmark in face_landmarks.landmark]
        for group in landmark_groups:
            indices = sorted({index for pair in group for index in pair})
            polygon = np.array([points[i] for i in indices], dtype=np.int32)
            if len(polygon) >= 3:
                cv2.fillConvexPoly(mask, cv2.convexHull(polygon), 1)

    if margin_px > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (margin_px * 2 + 1, margin_px * 2 + 1))
        mask = cv2.dilate(mask, kernel)

    return mask.astype(bool)
