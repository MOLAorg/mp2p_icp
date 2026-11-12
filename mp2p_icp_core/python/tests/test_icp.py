"""Tests of ICP: pipelines from YAML, align(), results, iteration hooks, logs."""

from __future__ import annotations

import math

import numpy as np
import pytest

import mrpt.math
import mrpt.poses

import mp2p_icp


def _transform(xyz: np.ndarray, pose: mrpt.poses.CPose3D) -> np.ndarray:
    H = np.asarray(pose.getHomogeneousMatrix())
    return xyz @ H[:3, :3].T + H[:3, 3]


def _demo_2d(demos_dir):
    local = mp2p_icp.MetricMap.from_file(demos_dir / "local_001.mm")
    glob = mp2p_icp.MetricMap.from_file(demos_dir / "global_001.mm")
    icp, params = mp2p_icp.icp_pipeline_from_yaml(
        demos_dir / "icp-settings-2d-lidar-example-point2point.yaml"
    )
    params.debugPrintIterationProgress = False
    return icp, params, local, glob


def test_demo_2d_known_result(demos_dir):
    # Same result as: icp-run --input-local local_001.mm --input-global global_001.mm
    #   -c icp-settings-2d-lidar-example-point2point.yaml
    icp, params, local, glob = _demo_2d(demos_dir)
    assert params is icp.parameters
    r = icp.align(local, glob)
    p = r.optimal_tf.mean
    assert p.x == pytest.approx(0.1285, abs=1e-4)
    assert p.y == pytest.approx(0.0472, abs=1e-4)
    assert math.degrees(p.yaw) == pytest.approx(5.69, abs=0.01)
    assert r.quality == pytest.approx(0.739, abs=1e-3)
    assert r.nIterations == 26
    assert r.terminationReason == mp2p_icp.IterTermReason.Stalled
    assert r.finalPairings.num_pt2pt == 402
    assert r.optimal_pose_matrix.shape == (4, 4)
    assert "quality" in str(r)

    pairs = r.finalPairings.pt2pt_to_numpy()
    assert pairs["global"].shape == (402, 3) and pairs["local_idx"].shape == (402,)


@pytest.mark.parametrize(
    "guess",
    [
        None,
        [0, 0, 0, 0, 0, 0],
        np.eye(4),
        mrpt.math.TPose3D(0, 0, 0, 0, 0, 0),
        mrpt.poses.CPose3D(),
    ],
)
def test_initial_guess_types(demos_dir, guess):
    icp, _, local, glob = _demo_2d(demos_dir)
    r = icp.align(local, glob, guess)
    assert r.optimal_tf.mean.x == pytest.approx(0.1285, abs=1e-4)


def test_bad_initial_guess(demos_dir):
    icp, _, local, glob = _demo_2d(demos_dir)
    with pytest.raises(ValueError):
        icp.align(local, glob, [1, 2, 3])
    with pytest.raises(TypeError):
        icp.align(local, glob, "nope")


@pytest.mark.parametrize("config", ["3d-point2point", "3d-point2plane"])
def test_bunny_synthetic_transform(bunny_xyz, config):
    gt = mrpt.poses.CPose3D(0.02, -0.01, 0.015, math.radians(10), math.radians(-5), math.radians(3))
    glob = _transform(bunny_xyz, gt)
    r = mp2p_icp.align(
        bunny_xyz,
        glob,
        config,
        variables={
            "ICP_THRESHOLD": 0.05,
            "ICP_DECIMATION": 0.002,
            "ICP_PLANE_MAX_DEVIATION": 0.002,
        },
    )
    H = np.asarray(gt.getHomogeneousMatrix())
    np.testing.assert_allclose(r.optimal_pose_matrix, H, atol=2e-4)
    assert r.quality > 0.95


@pytest.mark.parametrize("config", ["3d-point2point", "3d-point2plane"])
def test_bundled_3d_quality_is_the_overlap(bunny_xyz, config):
    # A small overlap: the global cloud only has the 10% of the points with
    # the smallest x. The quality is that overlap (it was 0, below the
    # default minimum pairing ratio of 0.2).
    rng = np.random.default_rng(1)
    local = bunny_xyz + rng.normal(0, 2e-4, bunny_xyz.shape)
    in_global = bunny_xyz[:, 0] < np.quantile(bunny_xyz[:, 0], 0.1)
    glob = bunny_xyz[in_global]
    r = mp2p_icp.align(
        local,
        glob,
        config,
        variables={
            "ICP_THRESHOLD": 0.01,
            "ICP_DECIMATION": 0.002,
            "ICP_PLANE_MAX_DEVIATION": 0.002,
            "ICP_QUALITY_THRESHOLD": 0.002,
        },
    )
    assert r.terminationReason == mp2p_icp.IterTermReason.Stalled
    assert np.abs(r.optimal_pose_matrix[:3, 3]).max() < 1e-3
    assert r.quality == pytest.approx(np.mean(in_global), abs=0.03)


def test_bundled_2d_quality_dense_clouds(bunny_xyz):
    # A perfect alignment of the same (noisy) cloud has quality 1, even if the
    # ICP pairings pair each global point only once:
    rng = np.random.default_rng(2)
    noisy = bunny_xyz + rng.normal(0, 1e-3, bunny_xyz.shape)
    for local, glob in [(noisy, bunny_xyz), (bunny_xyz, bunny_xyz[::2])]:
        r = mp2p_icp.align(
            local,
            glob,
            "2d-point2point",
            variables={"ICP_THRESHOLD": 0.02, "ICP_QUALITY_THRESHOLD": 0.01},
        )
        assert r.quality > 0.99


def test_align_inputs_unchanged(bunny_xyz):
    local = mp2p_icp.to_metric_map(bunny_xyz)
    glob = mp2p_icp.to_metric_map(bunny_xyz + [0.01, 0, 0])
    mp2p_icp.align(local, glob, variables={"ICP_THRESHOLD": 0.05, "ICP_DECIMATION": 0.003})
    # The filters run on copies: no "decimated" layers in the inputs
    assert list(local.layers) == ["raw"] and list(glob.layers) == ["raw"]


def test_align_parameters_override(bunny_xyz):
    r = mp2p_icp.align(
        bunny_xyz,
        bunny_xyz + [0.01, 0, 0],
        variables={"ICP_THRESHOLD": 0.05, "ICP_DECIMATION": 0.003},
        parameters={"maxIterations": 3},
    )
    assert r.nIterations == 3
    with pytest.raises(AttributeError):
        mp2p_icp.align(bunny_xyz, bunny_xyz, parameters={"noSuchParam": 1})


def test_bundled_configs():
    names = mp2p_icp.list_configs()
    assert {"2d-point2point", "3d-point2point", "3d-point2plane"} <= set(names)
    for name in names:
        assert mp2p_icp.config_path(name).is_file()
        icp, params = mp2p_icp.icp_pipeline_from_yaml(mp2p_icp.config_path(name))
        assert params.maxIterations > 0
        # No early abort for low quality (partial overlaps also have it):
        assert params.quality_checkpoints == {}
    with pytest.raises(ValueError):
        mp2p_icp.config_path("nope")


def test_pipeline_from_dict_and_variables(demos_dir):
    cfg = mp2p_icp.load_yaml(mp2p_icp.config_path("2d-point2point"), {"ICP_MAX_ITERATIONS": 7})
    assert cfg["params"]["maxIterations"] == 7
    icp = mp2p_icp.ICP.from_yaml(cfg)
    assert icp.parameters.maxIterations == 7
    assert "Matcher_Points_DistanceThreshold" in repr(icp)

    # A dict with variables:
    cfg2 = mp2p_icp.load_yaml(mp2p_icp.config_path("2d-point2point"))
    cfg2["params"]["maxIterations"] = "${N_ITERS|9}"
    assert mp2p_icp.ICP.from_yaml(cfg2).parameters.maxIterations == 9
    assert mp2p_icp.ICP.from_yaml(cfg2, {"N_ITERS": 4}).parameters.maxIterations == 4

    with pytest.raises(FileNotFoundError):
        mp2p_icp.ICP.from_yaml(demos_dir / "missing.yaml")
    with pytest.raises(TypeError):
        mp2p_icp.ICP.from_yaml(42)


def test_formula_variables(demos_dir):
    # Parameters as formulas of variables, updated at run time:
    cfg = mp2p_icp.load_yaml(demos_dir / "icp-settings-2d-lidar-example-point2point.yaml")
    cfg["params"]["debugPrintIterationProgress"] = False
    cfg["matchers"][0]["params"]["threshold"] = "2*HALF_THRESHOLD"
    icp = mp2p_icp.ICP.from_yaml(cfg)
    icp.update_variables({"HALF_THRESHOLD": 0.075})
    assert icp.variables["HALF_THRESHOLD"] == 0.075
    _, _, local, glob = _demo_2d(demos_dir)
    r = icp.align(local, glob)
    assert r.optimal_tf.mean.x == pytest.approx(0.1285, abs=1e-4)


def test_parameters():
    p = mp2p_icp.Parameters()
    assert p.as_dict()["quality_checkpoints"] == {50: 0.05, 100: 0.1}
    p.maxIterations = 12
    p.quality_checkpoints = {10: 0.2}
    d = p.as_dict()
    assert d["maxIterations"] == 12
    assert d["quality_checkpoints"] == {10: 0.2}
    # Round trip, and the YAML format of the checkpoints:
    assert mp2p_icp.Parameters.from_dict(d).as_dict() == d
    p3 = mp2p_icp.Parameters.from_dict(
        {"maxIterations": 5, "quality_checkpoints": [{"iteration": 3, "minimum_quality": 0.5}]}
    )
    assert p3.quality_checkpoints == {3: 0.5}
    no_checkpoints = mp2p_icp.Parameters.from_dict({"maxIterations": 5, "quality_checkpoints": {}})
    assert no_checkpoints.quality_checkpoints == {}
    p2 = mp2p_icp.Parameters.from_dict({"maxIterations": 5, "minAbsStep_trans": 1e-3})
    assert p2.maxIterations == 5 and p2.minAbsStep_trans == pytest.approx(1e-3)
    assert "maxIterations" in repr(p2)


def test_iteration_hook(demos_dir):
    icp, _, local, glob = _demo_2d(demos_dir)
    seen = []

    def hook(info: mp2p_icp.IterationInfo):
        seen.append((info.iteration, info.num_pairings, info.pose.x))
        return info.iteration >= 3  # stop

    icp.set_iteration_hook(hook)
    r = icp.align(local, glob)
    assert r.terminationReason == mp2p_icp.IterTermReason.HookRequest
    assert [s[0] for s in seen] == [0, 1, 2, 3]
    assert all(s[1] > 0 for s in seen)

    # Exceptions in the hook are raised by align():
    def bad_hook(info):
        raise ValueError("stop here")

    icp.set_iteration_hook(bad_hook)
    with pytest.raises(ValueError, match="stop here"):
        icp.align(local, glob)

    icp.set_iteration_hook(None)
    assert icp.align(local, glob).nIterations == 26
    with pytest.raises(TypeError):
        icp.set_iteration_hook(42)


def test_iteration_hook_sees_every_iteration(demos_dir):
    # The hook runs once per executed iteration, including the last one, as
    # the recorded iteration details:
    icp, params, local, glob = _demo_2d(demos_dir)
    params.saveIterationDetails = True
    params.decimationIterationDetails = 1
    seen = []
    icp.set_iteration_hook(lambda info: seen.append((info.iteration, info.pose)))
    log = mp2p_icp.LogRecord()
    r = icp.align(local, glob, log=log)
    assert r.terminationReason == mp2p_icp.IterTermReason.Stalled
    assert [s[0] for s in seen] == list(range(r.nIterations))
    assert sorted(log.iterationsDetails) == list(range(r.nIterations))
    last, result = seen[-1][1], r.optimal_tf.mean
    assert [last.x, last.y, last.z, last.yaw, last.pitch, last.roll] == [
        result.x,
        result.y,
        result.z,
        result.yaw,
        result.pitch,
        result.roll,
    ]

    # Stopping at the first iteration: it is counted.
    icp.set_iteration_hook(lambda info: True)
    r = icp.align(local, glob)
    assert r.terminationReason == mp2p_icp.IterTermReason.HookRequest
    assert r.nIterations == 1


def test_log_record(demos_dir, tmp_path):
    icp, params, local, glob = _demo_2d(demos_dir)
    params.saveIterationDetails = True
    params.decimationIterationDetails = 1
    log = mp2p_icp.LogRecord()
    r = icp.align(local, glob, log=log)
    assert log.icpResult.quality == pytest.approx(r.quality)
    assert log.pcLocal is not None and log.pcLocal.size() == 544
    assert log.iterationsDetails is not None and len(log.iterationsDetails) > 0

    path = tmp_path / "run.icplog"
    log.save(path)
    log2 = mp2p_icp.LogRecord.from_file(path)
    assert log2.icpResult.nIterations == r.nIterations
    assert log2.pcGlobal.size() == glob.size()
    with pytest.raises(FileNotFoundError):
        mp2p_icp.LogRecord.from_file(tmp_path / "missing.icplog")


def test_debug_files(demos_dir, tmp_path):
    icp, params, local, glob = _demo_2d(demos_dir)
    params.generateDebugFiles = True
    params.debugFileNameFormat = str(tmp_path / "log-$UNIQUE_ID.icplog")
    icp.align(local, glob)
    files = list(tmp_path.glob("*.icplog"))
    assert len(files) == 1
    assert mp2p_icp.LogRecord.from_file(files[0]).icpResult.quality > 0.7


def test_gravity_prior_and_prior(bunny_xyz):
    gp = mp2p_icp.GravityPrior()
    gp.sigma_rad = 0.01
    prior = mrpt.poses.CPose3DPDFGaussianInf()
    r = mp2p_icp.align(
        bunny_xyz,
        bunny_xyz + [0.005, 0, 0],
        variables={"ICP_THRESHOLD": 0.05, "ICP_DECIMATION": 0.003},
        gravity_prior=gp,
        prior=prior,
    )
    assert r.optimal_tf.mean.x == pytest.approx(0.005, abs=1e-3)


def test_kitti_loop_closure(datasets_dir):
    # Known good registration listed in the dataset.yaml of these maps:
    lc = datasets_dir / "loop_closures"
    glob = mp2p_icp.MetricMap.from_file(lc / "kitti00_000.mm")
    local = mp2p_icp.MetricMap.from_file(lc / "kitti00_040.mm")
    guess = [71.0, 4.0, 0.0, math.radians(3.0), 0.0, 0.0]
    r = mp2p_icp.align(
        local.layers["localmap_down"], glob.layers["localmap_down"], initial_guess=guess
    )
    p = r.optimal_tf.mean
    assert p.x == pytest.approx(71.30, abs=0.3)
    assert p.y == pytest.approx(4.69, abs=0.3)
    assert math.degrees(p.yaw) == pytest.approx(3.90, abs=0.5)
    assert r.quality > 0.5
