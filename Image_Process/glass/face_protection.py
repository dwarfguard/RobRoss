"""Optional face-aware protection for morphological operations.

Two backends, tried in order:

1. **MediaPipe** (if installed) – precise eye/eyebrow/lip landmark masks via
   Face Mesh.  Same as the original implementation.

2. **OpenCV native** (always available) – protects the most visually
   significant small-detail regions using Laplacian-based local contrast
   analysis.  Eyes naturally produce extreme local contrast (dark pupil vs.
   white sclera) and survive the high-percentile threshold, while regular
   texture detail passes through the morphological filter.  No ML model,
   no extra dependencies – just numpy + OpenCV.

Either way the function returns a boolean (H, W) mask, or ``None`` when
nothing worth protecting is found.  The caller treats ``None`` as "nothing
to protect", never as an error.
"""

import cv2
import numpy as np


# ── MediaPipe backend (original) ───────────────────────────────────────

def _mediapipe_protected_mask(image_bgr: np.ndarray, margin_px: int):
    """Return a boolean mask of eye/eyebrow/lip regions, or None."""
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

    landmark_groups = [
        mp_face_mesh.FACEMESH_LEFT_EYE,
        mp_face_mesh.FACEMESH_RIGHT_EYE,
        mp_face_mesh.FACEMESH_LEFT_EYEBROW,
        mp_face_mesh.FACEMESH_RIGHT_EYEBROW,
        mp_face_mesh.FACEMESH_LIPS,
    ]

    for face_landmarks in result.multi_face_landmarks:
        points = [(lm.x * width, lm.y * height) for lm in face_landmarks.landmark]
        for group in landmark_groups:
            indices = sorted({idx for pair in group for idx in pair})
            polygon = np.array([points[i] for i in indices], dtype=np.int32)
            if len(polygon) >= 3:
                cv2.fillConvexPoly(mask, cv2.convexHull(polygon), 1)

    if margin_px > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (margin_px * 2 + 1, margin_px * 2 + 1)
        )
        mask = cv2.dilate(mask, kernel)

    return mask.astype(bool)


# ── OpenCV-native backend (no extra deps) ──────────────────────────────

def _opencv_detail_mask(image_bgr: np.ndarray, margin_px: int):
    """Protect the most visually significant small-detail regions.

    Eyes have extreme local contrast (dark pupil next to white sclera) and
    therefore sit in the very top percentile of the Laplacian magnitude
    distribution – far above ordinary texture.  We threshold at the 99.5th
    percentile and then discard large connected components so only small,
    concentrated detail spots (eyes, nostrils, lip corners, …) are
    protected.

    Returns a boolean mask or ``None`` when no significant detail is found
    (low-contrast / uniformly textured images).
    """
    height, width = image_bgr.shape[:2]
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)

    # 1. Local contrast via Laplacian (2nd-derivative magnitude).
    lap = cv2.Laplacian(gray, cv2.CV_64F)
    lap_abs = np.abs(lap)

    # 2. Smooth to get a local "detail density" map.
    window = max(21, margin_px * 6 + 1)
    if window % 2 == 0:
        window += 1
    detail_map = cv2.GaussianBlur(lap_abs, (window, window), 0)

    # 3. Keep only the most extreme detail spots (99.5th percentile).
    #    Eyes / facial features have far higher contrast than random texture.
    threshold = np.percentile(detail_map, 99.5)
    if threshold <= 0:
        return None

    mask = detail_map >= threshold

    # 4. Discard large detail blobs – we only care about *small* features.
    #    Large high-detail regions are typically texture patches or strong
    #    object edges, not semantically important small details.
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8
    )
    max_feature_area = max(1, (width * height) * 0.003)  # ≤ 0.3% of image
    small_mask = np.zeros_like(mask)
    for i in range(1, num_labels):
        if stats[i, cv2.CC_STAT_AREA] <= max_feature_area:
            small_mask[labels == i] = True

    if not np.any(small_mask):
        return None

    # 5. Dilate to include surrounding context.
    if margin_px > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (margin_px * 2 + 1, margin_px * 2 + 1)
        )
        small_mask = cv2.dilate(small_mask.astype(np.uint8), kernel)

    return small_mask.astype(bool)


# ── Public API ─────────────────────────────────────────────────────────

def detect_protected_face_mask(image_bgr: np.ndarray, margin_px: int = 3):
    """Returns a boolean (H, W) mask marking regions that should survive
    morphological closing/opening unchanged.

    Tries the MediaPipe backend first (precise facial landmarks).  If
    MediaPipe is not installed or no face is found, falls back to a pure
    OpenCV detail-preservation heuristic that protects high-contrast
    small features (eyes naturally qualify).

    Returns ``None`` when nothing is detected – the caller should treat
    that as "nothing to protect", not an error.
    """
    # 1. MediaPipe (precise, requires pip install mediapipe).
    mask = _mediapipe_protected_mask(image_bgr, margin_px)
    if mask is not None:
        return mask

    # 2. OpenCV-native fallback (always available).
    return _opencv_detail_mask(image_bgr, margin_px)
