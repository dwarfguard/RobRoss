"""Unit tests for the pure teaching math in scripts/teach_canvas.py."""

import importlib.util
from pathlib import Path

import numpy as np

_MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "teach_canvas.py"
)
_spec = importlib.util.spec_from_file_location("teach_canvas", _MODULE_PATH)
teach_canvas = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(teach_canvas)


def test_still_samples_average_to_mean():
    positions = [
        [0.3000, 0.1000, 0.5000],
        [0.3002, 0.0998, 0.5001],
        [0.2998, 0.1002, 0.4999],
    ]
    mean, spread_mm = teach_canvas.average_still_samples(positions, tol_mm=0.5)
    assert mean is not None
    np.testing.assert_allclose(mean, [0.3, 0.1, 0.5], atol=1e-9)
    assert 0.0 < spread_mm <= 0.5


def test_moving_samples_are_rejected():
    # 2 mm drift within the window: hand still loading the arm.
    positions = [[0.3, 0.1, 0.5], [0.3, 0.1, 0.502]]
    mean, spread_mm = teach_canvas.average_still_samples(positions, tol_mm=0.5)
    assert mean is None
    assert spread_mm > 0.5


def test_single_sample_passes_with_zero_spread():
    mean, spread_mm = teach_canvas.average_still_samples(
        [[0.3, 0.1, 0.5]], tol_mm=0.5
    )
    assert mean is not None
    assert spread_mm == 0.0


def test_perfect_rectangle_has_zero_residual():
    tl = [0.4, 0.105, 0.6]
    tr = [0.4, -0.105, 0.6]
    bl = [0.4, 0.105, 0.303]
    br = [0.4, -0.105, 0.303]
    assert teach_canvas.rectangle_residual_mm(tl, tr, bl, br) < 1e-9


def test_offset_fourth_corner_reports_residual_in_mm():
    tl = [0.4, 0.105, 0.6]
    tr = [0.4, -0.105, 0.6]
    bl = [0.4, 0.105, 0.303]
    br = [0.4, -0.105, 0.303 - 0.005]  # 5 mm low
    residual = teach_canvas.rectangle_residual_mm(tl, tr, bl, br)
    np.testing.assert_allclose(residual, 5.0, atol=1e-9)


# Vertical A4 wall canvas facing the robot: x runs right (-y in base),
# y runs down (-z in base), so the canvas normal x cross y is +x in base
# (into the wall, away from the robot).
_WALL_TL = [0.4, 0.105, 0.6]
_WALL_TR = [0.4, -0.105, 0.6]
_WALL_BL = [0.4, 0.105, 0.303]


def test_canvas_pose_zero_bias_keeps_top_left_as_origin():
    origin, quat, width_m, height_m, skew_deg = teach_canvas.compute_canvas_pose(
        _WALL_TL, _WALL_TR, _WALL_BL
    )
    np.testing.assert_allclose(origin, _WALL_TL, atol=1e-12)
    np.testing.assert_allclose(width_m, 0.210, atol=1e-9)
    np.testing.assert_allclose(height_m, 0.297, atol=1e-9)
    np.testing.assert_allclose(skew_deg, 0.0, atol=1e-9)
    rot = teach_canvas.quat_to_matrix(*quat)
    np.testing.assert_allclose(rot[:, 0], [0.0, -1.0, 0.0], atol=1e-9)  # xc
    np.testing.assert_allclose(rot[:, 1], [0.0, 0.0, -1.0], atol=1e-9)  # yc
    np.testing.assert_allclose(rot[:, 2], [1.0, 0.0, 0.0], atol=1e-9)   # zc


def test_plane_bias_shifts_origin_into_wall_only():
    unbiased = teach_canvas.compute_canvas_pose(_WALL_TL, _WALL_TR, _WALL_BL)
    biased = teach_canvas.compute_canvas_pose(
        _WALL_TL, _WALL_TR, _WALL_BL, plane_bias_mm=1.8
    )
    # Origin moves exactly 1.8 mm along +zc (base +x here, into the wall).
    np.testing.assert_allclose(
        biased[0] - unbiased[0], [0.0018, 0.0, 0.0], atol=1e-12
    )
    # Orientation and measured size are untouched by the bias.
    np.testing.assert_allclose(biased[1], unbiased[1], atol=1e-12)
    assert biased[2:] == unbiased[2:]


def test_plane_bias_follows_a_slanted_canvas_normal():
    # 45 deg easel: y runs down-and-away from the robot, x still right.
    s = 0.297 / np.sqrt(2.0)
    tl = np.array([0.4, 0.105, 0.6])
    tr = np.array([0.4, -0.105, 0.6])
    bl = tl + np.array([s, 0.0, -s])
    origin, _quat, width_m, height_m, _skew = teach_canvas.compute_canvas_pose(
        tl, tr, bl, plane_bias_mm=2.0
    )
    normal = np.array([1.0, 0.0, 1.0]) / np.sqrt(2.0)
    np.testing.assert_allclose(origin - tl, 0.002 * normal, atol=1e-12)
    np.testing.assert_allclose(width_m, 0.210, atol=1e-9)
    np.testing.assert_allclose(height_m, 0.297, atol=1e-9)


def test_degenerate_corners_raise():
    try:
        teach_canvas.compute_canvas_pose(
            _WALL_TL, _WALL_TL, _WALL_BL, plane_bias_mm=1.8
        )
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for coincident corners")


# --- Multi-point plane fit + Z-correction surface -----------------------

# Wall frame axes (see the _WALL_* corners above): canvas x is base -y,
# canvas y is base -z, and the into-paper normal z is base +x.
_WALL_XC = np.array([0.0, -1.0, 0.0])
_WALL_YC = np.array([0.0, 0.0, -1.0])
_WALL_ZC = np.array([1.0, 0.0, 0.0])
_WALL_BR = [0.4, -0.105, 0.303]


def _wall_point(x_mm, y_mm, warp_mm=0.0):
    """Base-frame point on the wall canvas at canvas (x_mm, y_mm), pushed
    warp_mm out of the plane along the into-paper normal."""
    return (
        np.array(_WALL_TL)
        + (x_mm / 1000.0) * _WALL_XC
        + (y_mm / 1000.0) * _WALL_YC
        + (warp_mm / 1000.0) * _WALL_ZC
    )


def test_calibration_flat_matches_flat_plane():
    # Four planar corners, no interior samples: the best-fit plane must equal
    # the old three-corner plane, with a zero correction surface.
    origin, quat, width_m, height_m, coeffs, before, after = (
        teach_canvas.compute_canvas_calibration(
            _WALL_TL, _WALL_TR, _WALL_BL, _WALL_BR
        )
    )
    ref_origin, ref_quat, ref_w, ref_h, _skew = teach_canvas.compute_canvas_pose(
        _WALL_TL, _WALL_TR, _WALL_BL
    )
    np.testing.assert_allclose(origin, ref_origin, atol=1e-12)
    np.testing.assert_allclose(np.abs(quat), np.abs(ref_quat), atol=1e-9)
    np.testing.assert_allclose([width_m, height_m], [ref_w, ref_h], atol=1e-9)
    np.testing.assert_allclose(coeffs, np.zeros(6), atol=1e-9)
    assert before < 1e-9 and after < 1e-9


def test_calibration_absorbs_pure_tilt_without_correction():
    # A planar-but-tilted canvas: the plane fit should recover it (residual
    # ~0), so the correction surface stays ~flat.
    tilt = np.array([1.0, 0.0, 1.0]) / np.sqrt(2.0)  # normal of a 45-deg tilt
    tl = np.array([0.4, 0.105, 0.6])
    # Build tr/bl/br in a genuine plane with that normal.
    xc = np.array([0.0, -1.0, 0.0])
    yc = np.cross(tilt, xc)
    tr = tl + 0.210 * xc
    bl = tl + 0.297 * yc
    br = tl + 0.210 * xc + 0.297 * yc
    _o, _q, _w, _h, coeffs, before, after = (
        teach_canvas.compute_canvas_calibration(tl, tr, bl, br)
    )
    assert before < 1e-9   # already planar
    assert after < 1e-9
    np.testing.assert_allclose(coeffs, np.zeros(6), atol=1e-9)


def test_calibration_models_quadratic_warp():
    # Corners + a 3x3 interior grid lying on a warped (non-planar) surface.
    # The flat model would miss the bulge; the correction surface must absorb
    # nearly all of it.
    def warp(x, y):  # quadratic bowl, mm; ~0 at corners, peak mid-sheet
        return 1.2 * (1.0 - 0.5 * (
            ((x - 105.0) / 105.0) ** 2 + ((y - 148.5) / 148.5) ** 2
        ))

    grid = [(x, y) for x in (52.5, 105.0, 157.5)
            for y in (74.25, 148.5, 222.75)]
    tl = _wall_point(0, 0, warp(0, 0))
    tr = _wall_point(210, 0, warp(210, 0))
    bl = _wall_point(0, 297, warp(0, 297))
    br = _wall_point(210, 297, warp(210, 297))
    samples = [_wall_point(x, y, warp(x, y)) for x, y in grid]

    _o, _q, _w, _h, coeffs, before, after = (
        teach_canvas.compute_canvas_calibration(tl, tr, bl, br, samples)
    )
    # The warp is a genuine out-of-plane error the flat plane cannot remove...
    assert before > 0.3
    # ...but the quadratic surface reconstructs every touched point.
    assert after < 0.01
    assert np.max(np.abs(coeffs)) > 0.0


def test_fit_z_correction_roundtrip():
    coeffs_true = [0.2, -0.001, 0.0008, 5e-6, -3e-6, 2e-6]
    xs = np.array([x for x in (0.0, 105.0, 210.0) for _ in range(3)])
    ys = np.array([y for _ in range(3) for y in (0.0, 148.5, 297.0)])
    resid = teach_canvas._corr_design(xs, ys) @ np.array(coeffs_true)
    fitted = teach_canvas.fit_z_correction(xs, ys, resid)
    # The fitted quadratic reproduces the residuals at every point (comparing
    # coefficients directly is ill-conditioned given the large x^2/y^2 terms).
    recon = teach_canvas._corr_design(xs, ys) @ np.array(fitted)
    np.testing.assert_allclose(recon, resid, atol=1e-6)
    # Two quadratics agreeing at a non-degenerate grid are identical, so it
    # also matches away from the sampled points.
    np.testing.assert_allclose(
        teach_canvas.evaluate_z_correction(fitted, 70.0, 200.0),
        teach_canvas.evaluate_z_correction(coeffs_true, 70.0, 200.0),
        atol=1e-5,
    )


def test_fit_z_correction_too_few_points_is_flat():
    coeffs = teach_canvas.fit_z_correction([0.0, 10.0], [0.0, 5.0], [0.1, 0.2])
    assert coeffs == [0.0] * 6
