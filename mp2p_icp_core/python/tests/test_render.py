"""Tests of mp2p_icp.render(). Skipped if off-screen OpenGL is not available."""

from __future__ import annotations

import numpy as np
import pytest

import mp2p_icp


def _render_or_skip(*args, **kwargs) -> np.ndarray:
    pytest.importorskip("mrpt.opengl")
    try:
        return mp2p_icp.render(*args, **kwargs)
    except RuntimeError as e:
        if "EGL" in str(e) or "OpenGL" in str(e) or "context" in str(e):
            pytest.skip(f"Off-screen OpenGL rendering not available: {e}")
        raise


def test_render_map(demos_dir, tmp_path):
    m = mp2p_icp.MetricMap.from_file(demos_dir / "global_001.mm")
    out = tmp_path / "map.png"
    img = _render_or_skip(m, 320, 240, point_size=3.0, background=(1, 1, 1), output=out)
    assert img.shape == (240, 320, 3) and img.dtype == np.uint8
    assert out.is_file()
    # Mostly white background, with some non-white (point) pixels:
    nonwhite = np.any(img < 200, axis=2)
    assert 0 < nonwhite.mean() < 0.5


def test_render_colormap_and_points(bunny_xyz):
    img = _render_or_skip(
        bunny_xyz,
        200,
        200,
        azimuth=30,
        elevation=20,
        colormap="jet",
        point_size=2.0,
        background=(0, 0, 0),
    )
    assert np.any(img > 50)
    with pytest.raises(ValueError):
        _render_or_skip(bunny_xyz, 10, 10, colormap="nope")
    with pytest.raises(KeyError):
        _render_or_skip(bunny_xyz, 10, 10, layers=["nope"])
