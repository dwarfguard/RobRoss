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
