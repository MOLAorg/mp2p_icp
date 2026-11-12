"""Tests of point layer fields to and from NumPy."""

from __future__ import annotations

import numpy as np
import pytest

import mrpt.maps

import mp2p_icp


def test_round_trip_all_types(all_fields):
    layer = mp2p_icp.layer_from_numpy(all_fields)
    assert isinstance(layer, mrpt.maps.CGenericPointsMap)
    assert layer.size() == len(all_fields["x"])
    out = mp2p_icp.layer_to_numpy(layer)
    assert set(out) == set(all_fields)
    for name, a in all_fields.items():
        assert out[name].dtype == a.dtype, name
        np.testing.assert_array_equal(out[name], a)
    # Key order: x, y, z first
    assert list(out)[:3] == ["x", "y", "z"]


def test_arrays_are_copies(all_fields):
    layer = mp2p_icp.layer_from_numpy(all_fields)
    out = mp2p_icp.layer_to_numpy(layer)
    out["x"][:] = 0
    assert mp2p_icp.layer_to_numpy(layer)["x"][0] == all_fields["x"][0]


def test_from_xyz_arrays():
    xyz = np.random.rand(50, 3)  # float64 coordinates become float32
    f = mp2p_icp.layer_to_numpy(mp2p_icp.layer_from_numpy(xyz))
    assert f["x"].dtype == np.float32
    np.testing.assert_allclose(f["z"], xyz[:, 2], rtol=1e-6)

    xy = np.random.rand(10, 2).tolist()  # lists work too; z=0
    f = mp2p_icp.layer_to_numpy(mp2p_icp.layer_from_numpy(xy))
    np.testing.assert_array_equal(f["z"], np.zeros(10, dtype=np.float32))

    f = mp2p_icp.layer_to_numpy(mp2p_icp.layer_from_numpy({"x": [1.0], "y": [2.0]}))
    assert f["z"][0] == 0.0


def test_empty():
    f = mp2p_icp.layer_to_numpy(mp2p_icp.layer_from_numpy(np.zeros((0, 3))))
    assert len(f["x"]) == 0


def test_errors():
    with pytest.raises(ValueError):
        mp2p_icp.layer_from_numpy(np.zeros((5, 4)))
    with pytest.raises(ValueError):
        mp2p_icp.layer_from_numpy({"x": np.zeros(3), "y": np.zeros(4)})
    with pytest.raises(ValueError):
        mp2p_icp.layer_from_numpy({"x": np.zeros(3)})
    with pytest.raises(TypeError):
        mp2p_icp.layer_from_numpy({"x": np.zeros(3), "y": np.zeros(3), "i": np.zeros(3, np.int32)})
    with pytest.raises(TypeError):
        mp2p_icp.layer_from_numpy(42)
    with pytest.raises(TypeError):
        mp2p_icp.layer_to_numpy(mrpt.maps.COccupancyGridMap2D())


def test_mrpt_field_api_interop(all_fields):
    # The created layers work with the mrpt.maps field API:
    layer = mp2p_icp.layer_from_numpy(all_fields)
    assert layer.hasPointField("intensity")
    assert layer.getPointField_float(3, "intensity") == pytest.approx(all_fields["intensity"][3])
    assert layer.getPointField_uint16(3, "ring") == all_fields["ring"][3]
    assert "gps_time" in layer.getPointFieldNames_double()
