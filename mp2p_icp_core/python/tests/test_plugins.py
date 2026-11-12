"""Tests of maps with layers of classes from other libraries (plugins).

Skipped if the dataset (mola_test_datasets) or the plugin library
(libmola_metric_maps.so) are not available.
"""

from __future__ import annotations

import subprocess
import sys

import numpy as np
import pytest

import mp2p_icp

# A map built with MOLA, whose layer "localmap" is a mola::IncrementalPointCloud:
PLUGIN_MAP = "ual/husky_benches/husky_benches_map.mm"
PLUGIN = "libmola_metric_maps.so"


@pytest.fixture(scope="module")
def plugin_map_file(datasets_dir):
    f = datasets_dir / PLUGIN_MAP
    if not f.is_file():
        pytest.skip(f"Dataset file not found: {f}")
    return f


@pytest.fixture(scope="module")
def plugin_map(plugin_map_file):
    try:
        mp2p_icp.load_plugins(PLUGIN)
    except RuntimeError as e:
        pytest.skip(f"{PLUGIN} not available: {e}")
    return mp2p_icp.MetricMap.from_file(plugin_map_file)


def test_plugin_map_without_plugin(plugin_map_file, tmp_path):
    # Once loaded, a plugin stays registered for the rest of the process: try
    # without it in a new one.
    code = (
        "import sys\n"
        "import mp2p_icp\n"
        "try:\n"
        "    mp2p_icp.MetricMap.from_file(sys.argv[1])\n"
        "except RuntimeError as e:\n"
        "    print(e)\n"
        "    sys.exit(0)\n"
        "sys.exit(1)\n"
    )
    r = subprocess.run(
        [sys.executable, "-c", code, str(plugin_map_file)],
        cwd=tmp_path,
        capture_output=True,
        text=True,
    )
    assert r.returncode == 0, f"No RuntimeError without the plugin:\n{r.stdout}\n{r.stderr}"
    assert "mp2p_icp.load_plugins(" in r.stdout


def test_plugin_map_contents(plugin_map):
    assert list(plugin_map.layers) == ["localmap"]
    assert plugin_map.size() == 217841
    fields = mp2p_icp.layer_to_numpy(plugin_map.layers["localmap"])
    for name in ["x", "y", "z"]:
        assert fields[name].shape == (217841,)
        assert np.all(np.isfinite(fields[name]))
    assert plugin_map.georeferencing is not None


def test_plugin_map_render(plugin_map, render_or_skip):
    img = render_or_skip(plugin_map, 320, 240, background=(1, 1, 1))
    assert img.shape == (240, 320, 3)
    # Some non-white (point) pixels:
    assert np.any(img < 200, axis=2).mean() > 0.01
