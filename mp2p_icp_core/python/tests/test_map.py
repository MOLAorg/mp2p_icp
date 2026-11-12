"""Tests of MetricMap: loading, saving, layers, metadata, copies."""

from __future__ import annotations

import copy
import pickle

import numpy as np
import pytest

import mrpt.maps
import mrpt.math
import mrpt.poses
import mrpt.topography
import mrpt.viz

import mp2p_icp


def _same_points(a, b) -> None:
    fa = mp2p_icp.layer_to_numpy(a)
    fb = mp2p_icp.layer_to_numpy(b)
    assert fa.keys() == fb.keys()
    for k in fa:
        np.testing.assert_array_equal(fa[k], fb[k])


def test_load_demo_map(demos_dir):
    m = mp2p_icp.MetricMap.from_file(demos_dir / "local_001.mm")
    assert m.id == 7
    assert m.label is None
    assert list(m.layers) == ["2d_lidar"]
    assert m.size() == 544
    assert m.size_points_only() == 544
    assert not m.empty()
    assert "544" in m.contents_summary()
    layer = m.layers["2d_lidar"]
    assert isinstance(layer, mrpt.maps.CSimplePointsMap)
    assert layer.size() == 544
    assert isinstance(m.point_layer("2d_lidar"), mrpt.maps.CPointsMap)


def test_load_errors(tmp_path):
    with pytest.raises(FileNotFoundError):
        mp2p_icp.MetricMap.from_file(tmp_path / "does_not_exist.mm")
    bad = tmp_path / "bad.mm"
    bad.write_bytes(b"this is not a metric map")
    with pytest.raises(RuntimeError):
        mp2p_icp.MetricMap.from_file(bad)
    m = mp2p_icp.MetricMap()
    with pytest.raises(RuntimeError):
        m.save(tmp_path / "no_such_dir" / "x.mm")
    with pytest.raises(ValueError):
        m.save(tmp_path / "x.mm", compression="lzma")


@pytest.mark.parametrize("compression", ["zstd", "gzip", "none"])
def test_save_load_round_trip(tmp_path, all_fields, compression):
    m = mp2p_icp.MetricMap()
    m.layers["raw"] = all_fields
    m.layers["xyz"] = np.zeros((5, 3))
    m.id = 12345
    m.label = "a label"
    m.metadata = {"timestamp": "2026-11-12", "sequence": [1, 2.5, "x"], "nested": {"a": True}}
    geo = mp2p_icp.Georeferencing()
    geo.geo_coord = mrpt.topography.TGeodeticCoords(36.83, -2.40, 70.0)
    geo.T_enu_to_map = mrpt.poses.CPose3DPDFGaussian(mrpt.poses.CPose3D(1, 2, 3, 0.1, 0, 0))
    m.georeferencing = geo
    m.lines = [mrpt.math.TLine3D(mrpt.math.TPoint3D(0, 0, 0), mrpt.math.TPoint3D(1, 0, 0))]
    m.planes = [mp2p_icp.PlanePatch(mrpt.math.TPlane(0, 0, 1, -1), mrpt.math.TPoint3D(0.5, 0.5, 1))]

    path = tmp_path / "map.mm"
    m.save(path, compression=compression)
    m2 = mp2p_icp.MetricMap.from_file(path)

    assert m2.id == 12345
    assert m2.label == "a label"
    assert m2.metadata == m.metadata
    assert sorted(m2.layers.keys()) == ["raw", "xyz"]
    _same_points(m.layers["raw"], m2.layers["raw"])
    assert m2.georeferencing is not None
    assert m2.georeferencing.geo_coord.lat.decimal_value == pytest.approx(36.83)
    assert m2.georeferencing.T_enu_to_map.mean.x == pytest.approx(1.0)
    assert len(m2.lines) == 1 and m2.lines[0].director.x == pytest.approx(1.0)
    assert len(m2.planes) == 1 and m2.planes[0].centroid.z == pytest.approx(1.0)

    # load() into an existing object replaces its contents:
    m3 = mp2p_icp.MetricMap()
    m3.layers["other"] = np.ones((3, 3))
    m3.load(str(path))
    assert sorted(m3.layers.keys()) == ["raw", "xyz"]


def test_layers_view():
    m = mp2p_icp.MetricMap()
    assert len(m.layers) == 0
    m.layers["a"] = np.random.rand(10, 3)
    m.layers["b"] = mrpt.maps.CSimplePointsMap()
    assert "a" in m.layers and "c" not in m.layers
    assert m.layers.keys() == ["a", "b"]
    assert [n for n, _ in m.layers.items()] == ["a", "b"]
    assert len(m.layers.values()) == 2
    assert m.layers.get("c") is None
    assert isinstance(m.layers["a"], mrpt.maps.CGenericPointsMap)
    with pytest.raises(KeyError):
        m.layers["c"]
    with pytest.raises(KeyError):
        del m.layers["c"]
    del m.layers["b"]
    assert list(m.layers) == ["a"]

    # Layers are shared, not copied:
    pts = mrpt.maps.CSimplePointsMap()
    m.layers["shared"] = pts
    pts.insertPoint(1, 2, 3)
    assert m.layers["shared"].size() == 1

    # The view keeps the map alive:
    view = mp2p_icp.MetricMap().layers
    view["x"] = np.zeros((2, 3))
    assert len(view) == 1


def test_metadata_is_a_copy():
    m = mp2p_icp.MetricMap()
    assert m.metadata == {}
    md = m.metadata
    md["x"] = 1
    assert m.metadata == {}
    m.metadata = {"x": 1}
    assert m.metadata == {"x": 1}
    with pytest.raises(TypeError):
        m.metadata = [1, 2]


def test_copies_and_pickle(all_fields):
    m = mp2p_icp.MetricMap()
    m.layers["raw"] = all_fields
    m.layers["xyz"] = mrpt.maps.CSimplePointsMap()
    m.label = "orig"

    shallow = copy.copy(m)
    shallow.layers["new"] = np.zeros((1, 3))
    assert "new" not in m.layers
    shallow.layers["xyz"].insertPoint(0, 0, 0)
    assert m.layers["xyz"].size() == 1

    for c in (m.copy(), copy.deepcopy(m), pickle.loads(pickle.dumps(m))):
        assert c.label == "orig"
        _same_points(c.layers["raw"], m.layers["raw"])
        _same_points(c.layers["xyz"], m.layers["xyz"])
        # Deep copies: changing one does not change the other
        c.layers["xyz"].insertPoint(1, 2, 3)
        assert c.layers["xyz"].size() == m.layers["xyz"].size() + 1


def test_merge_with():
    a = mp2p_icp.MetricMap()
    a.layers["raw"] = np.array([[0.0, 0, 0], [1, 0, 0]])
    b = mp2p_icp.MetricMap()
    b.layers["raw"] = np.array([[0.0, 0, 0]])
    b.layers["other"] = np.array([[5.0, 5, 5]])
    a.merge_with(b, [10, 0, 0, 0, 0, 0])
    assert a.layers["raw"].size() == 3
    xs = mp2p_icp.layer_to_numpy(a.layers["raw"])["x"]
    assert 10.0 in xs
    assert a.layers["other"].size() == 1


def test_visualization(demos_dir):
    m = mp2p_icp.MetricMap.from_file(demos_dir / "local_001.mm")
    assert isinstance(m.get_visualization(), mrpt.viz.CSetOfObjects)
    p = mp2p_icp.RenderParams()
    p.points.allLayers.pointSize = 3.0
    assert p.points.allLayers.pointSize == 3.0
    cm = mp2p_icp.ColorMode()
    cm.recolorizeByField = "z"
    p.points.allLayers.colorMode = cm
    p.points.perLayer = {"2d_lidar": mp2p_icp.RenderParamsPointLayer()}
    assert isinstance(m.get_visualization(p), mrpt.viz.CSetOfObjects)
    assert isinstance(mp2p_icp.layer_visualization(m.layers["2d_lidar"]), mrpt.viz.CSetOfObjects)


def test_georeferencing_dict():
    g = mp2p_icp.Georeferencing()
    g.geo_coord = mrpt.topography.TGeodeticCoords(10.0, 20.0, 30.0)
    d = g.as_dict()
    g2 = mp2p_icp.Georeferencing.from_dict(d)
    assert g2.geo_coord.lon.decimal_value == pytest.approx(20.0)
    assert "lat=10" in repr(g2)


def test_voxel_layer_points(datasets_dir):
    m = mp2p_icp.MetricMap.from_file(datasets_dir / "loop_closures" / "kitti00_000.mm")
    vox = m.layers["localmap_voxels"]
    assert isinstance(vox, mrpt.maps.CVoxelMap)
    f = mp2p_icp.layer_to_numpy(vox)
    assert len(f["x"]) > 1000
    pts = mp2p_icp.layer_points_view(vox)
    assert isinstance(pts, mrpt.maps.CPointsMap) and pts.size() == len(f["x"])
