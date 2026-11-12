"""Tests of filter pipelines, generators and simplemap_to_metricmap()."""

from __future__ import annotations

import pytest

import mrpt.maps
import mrpt.obs
import mrpt.poses

import mp2p_icp

DECIMATE = {
    "filters": [
        {
            "class_name": "mp2p_icp_filters::FilterDecimateVoxels",
            "params": {
                "input_pointcloud_layer": "raw",
                "output_pointcloud_layer": "decimated",
                "voxel_filter_resolution": "${RES|0.01}",
                "decimate_method": "DecimateMethod::FirstPoint",
            },
        }
    ]
}


def test_filter_pipeline(bunny_xyz):
    pipeline = mp2p_icp.filter_pipeline_from_yaml(DECIMATE)
    assert len(pipeline) == 1
    assert pipeline.filters[0][0] == "mp2p_icp_filters::FilterDecimateVoxels"
    m = mp2p_icp.to_metric_map(bunny_xyz)
    pipeline.apply(m)
    n_coarse = m.layers["decimated"].size()
    assert 0 < n_coarse < len(bunny_xyz)

    # ${VAR} expansion, and the list form (without the 'filters' key):
    fine = mp2p_icp.FilterPipeline.from_yaml(DECIMATE["filters"], {"RES": 0.003})
    m2 = mp2p_icp.to_metric_map(bunny_xyz)
    mp2p_icp.apply_filter_pipeline(fine, m2)
    assert m2.layers["decimated"].size() > n_coarse


def test_filter_formula_variables(bunny_xyz):
    cfg = {"filters": [dict(DECIMATE["filters"][0])]}
    cfg["filters"][0]["params"] = dict(cfg["filters"][0]["params"], voxel_filter_resolution="2*R")
    pipeline = mp2p_icp.FilterPipeline.from_yaml(cfg)
    m = mp2p_icp.to_metric_map(bunny_xyz)
    with pytest.raises(RuntimeError):
        pipeline.apply(m)  # R undefined
    sizes = []
    for r in (0.005, 0.0015):
        pipeline.update_variables({"R": r})
        m = mp2p_icp.to_metric_map(bunny_xyz)
        pipeline.apply(m)
        sizes.append(m.layers["decimated"].size())
    assert sizes[0] < sizes[1]


def test_filter_pipeline_from_demo_file(demos_dir):
    # A pipeline file from the demos, with a real map:
    pipeline = mp2p_icp.filter_pipeline_from_yaml(demos_dir / "mm-filter_keep_mls_xyzrgb.yaml")
    assert len(pipeline) > 0


def test_generators():
    gens = mp2p_icp.generators_from_yaml(
        [{"class_name": "mp2p_icp_filters::Generator", "params": {"target_layer": "raw"}}]
    )
    assert len(gens) == 1
    pc = mrpt.maps.CSimplePointsMap()
    for i in range(10):
        pc.insertPoint(float(i), 0.0, 0.0)
    obs = mrpt.maps.CObservationPointCloud()
    obs.pointcloud = pc
    m = gens.apply(obs)
    assert m.layers["raw"].size() == 10

    # With a robot pose, appended to an existing map:
    m2 = mp2p_icp.apply_generators(gens, obs, robot_pose=[0, 5, 0, 0, 0, 0], output=m)
    assert m2.layers["raw"].size() == 20
    assert mp2p_icp.layer_to_numpy(m2.layers["raw"])["y"].max() == pytest.approx(5.0)
    with pytest.raises(TypeError):
        gens.apply(42)


def test_simplemap_to_metricmap(demos_dir, datasets_dir):
    sm = datasets_dir / "simplemaps" / "mvsim-warehouse01.simplemap"
    m = mp2p_icp.simplemap_to_metricmap(
        sm,
        demos_dir / "sm2mm_pointcloud_voxelize_no_deskew.yaml",
        decimate_maximum_frame_count=10,
        verbosity="ERROR",
    )
    assert isinstance(m, mp2p_icp.MetricMap)
    assert m.size_points_only() > 1000
    layer = next(iter(m.layers.values()))
    assert "intensity" in mp2p_icp.layer_to_numpy(layer)

    with pytest.raises(FileNotFoundError):
        mp2p_icp.simplemap_to_metricmap(datasets_dir / "missing.simplemap")
