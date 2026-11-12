"""Tests of mp2p_icp.interop. Each test is skipped if its library is missing."""

from __future__ import annotations

import numpy as np
import pytest

import mp2p_icp
from mp2p_icp import interop


def test_missing_dependency_message(monkeypatch):
    import sys

    monkeypatch.setitem(sys.modules, "pandas", None)  # makes "import pandas" fail
    with pytest.raises(ImportError, match="pip install pandas"):
        interop.to_dataframe({"x": np.zeros(1), "y": np.zeros(1), "z": np.zeros(1)})


def test_pandas_round_trip(all_fields):
    pytest.importorskip("pandas")
    layer = mp2p_icp.layer_from_numpy(all_fields)
    df = interop.to_dataframe(layer)
    assert list(df.columns)[:3] == ["x", "y", "z"] and len(df) == len(all_fields["x"])
    back = mp2p_icp.layer_to_numpy(interop.from_dataframe(df))
    for k, v in all_fields.items():
        np.testing.assert_array_equal(back[k], v)
        assert back[k].dtype == v.dtype
    # int64 columns, as pandas creates them, become float64 fields:
    df["n"] = np.arange(len(df))
    assert mp2p_icp.layer_to_numpy(interop.from_dataframe(df))["n"].dtype == np.float64


def test_parquet_round_trip(tmp_path, all_fields):
    pytest.importorskip("pandas")
    pytest.importorskip("pyarrow")
    path = tmp_path / "cloud.parquet"
    interop.to_parquet(mp2p_icp.layer_from_numpy(all_fields), path)
    back = mp2p_icp.layer_to_numpy(interop.from_parquet(path))
    for k, v in all_fields.items():
        np.testing.assert_array_equal(back[k], v)


def test_open3d(all_fields):
    o3d = pytest.importorskip("open3d")
    fields = dict(all_fields)
    for i, n in enumerate(("normal_x", "normal_y", "normal_z")):
        fields[n] = np.full(len(fields["x"]), float(i == 2), np.float32)
    layer = mp2p_icp.layer_from_numpy(fields)

    pc = interop.to_open3d(layer)
    assert isinstance(pc, o3d.geometry.PointCloud)
    assert pc.has_colors() and pc.has_normals()
    np.testing.assert_allclose(np.asarray(pc.colors)[:, 0], fields["color_r"] / 255.0)
    back = mp2p_icp.layer_to_numpy(interop.from_open3d(pc))
    np.testing.assert_allclose(back["x"], fields["x"])
    np.testing.assert_allclose(back["color_rf"], fields["color_r"] / 255.0, rtol=1e-6)
    np.testing.assert_array_equal(back["normal_z"], fields["normal_z"])

    t = interop.to_open3d(layer, tensor=True)
    assert "intensity" in t.point and "ring" in t.point
    back = mp2p_icp.layer_to_numpy(interop.from_open3d(t))
    np.testing.assert_array_equal(back["intensity"], fields["intensity"])
    np.testing.assert_array_equal(back["ring"], fields["ring"])


def test_pyvista(all_fields):
    pytest.importorskip("pyvista")
    layer = mp2p_icp.layer_from_numpy(all_fields)
    poly = interop.to_pyvista(layer)
    assert poly.n_points == len(all_fields["x"])
    assert "intensity" in poly.point_data and "RGB" in poly.point_data
    back = mp2p_icp.layer_to_numpy(interop.from_pyvista(poly))
    for k, v in all_fields.items():
        np.testing.assert_array_equal(back[k], v)
    assert "RGB_0" not in back


def test_laspy(tmp_path, all_fields):
    laspy = pytest.importorskip("laspy")
    fields = dict(all_fields)
    fields["classification"] = (np.arange(len(fields["x"])) % 5).astype(np.uint8)
    las = interop.to_laspy(mp2p_icp.layer_from_numpy(fields))
    path = tmp_path / "cloud.las"
    las.write(path)
    back = mp2p_icp.layer_to_numpy(interop.from_laspy(path))
    np.testing.assert_allclose(back["x"], fields["x"], atol=1e-3)
    np.testing.assert_array_equal(back["color_r"], fields["color_r"])
    np.testing.assert_array_equal(back["classification"], fields["classification"])
    np.testing.assert_array_equal(back["ring"], fields["ring"])
    np.testing.assert_allclose(back["gps_time"], fields["gps_time"])
    np.testing.assert_allclose(back["intensity"], np.round(fields["intensity"]))

    # Files written by mp2p_icp.save_las() are read too:
    las_path = tmp_path / "native.las"
    mp2p_icp.save_las(mp2p_icp.layer_from_numpy(fields), las_path)
    native = mp2p_icp.layer_to_numpy(interop.from_laspy(laspy.read(las_path), origin=[1, 0, 0]))
    np.testing.assert_allclose(native["x"], fields["x"] - 1, atol=1e-3)
    np.testing.assert_array_equal(native["color_g"], fields["color_g"])
