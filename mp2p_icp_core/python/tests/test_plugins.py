"""Tests of maps with layers of classes from other libraries (plugins), and of
the automatic import of the `mola_metric_maps` Python module when a `mola::`
class is not registered.

The tests with a real MOLA map are skipped if the dataset (mola_test_datasets)
or the plugin library (libmola_metric_maps.so) are not available. The others
use a fake `mola_metric_maps` module.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

import mp2p_icp

# A map built with MOLA, whose layer "localmap" is a mola::IncrementalPointCloud:
PLUGIN_MAP = "ual/husky_benches/husky_benches_map.mm"
PLUGIN = "libmola_metric_maps.so"

# Printed by the fake mola_metric_maps module when imported:
FAKE_IMPORTED = "FAKE mola_metric_maps imported"


def _run_with_fake_plugin_module(tmp_path: Path, code: str, *args, module_code: str = ""):
    """Runs Python code in a new process (plugins and imports stay for the rest
    of a process) where `import mola_metric_maps` finds a fake module, which
    prints FAKE_IMPORTED and then runs `module_code`."""
    fake_dir = tmp_path / "fake_modules"
    fake_dir.mkdir(exist_ok=True)
    (fake_dir / "mola_metric_maps.py").write_text(f"print({FAKE_IMPORTED!r})\n{module_code}")
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(filter(None, [str(fake_dir), env.get("PYTHONPATH")]))
    return subprocess.run(
        [sys.executable, "-c", code, *map(str, args)],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
    )


def _map_file_with_unregistered_class(demos_dir: Path, tmp_path: Path) -> Path:
    """A copy of a demo map whose layer class name is replaced by an unknown
    `mola::` class of the same length."""
    m = mp2p_icp.MetricMap.from_file(demos_dir / "local_001.mm")
    (layer,) = m.layers.values()
    name = f"mrpt::maps::{type(layer).__name__}".encode()
    fake = b"mola::Fake" + b"X" * (len(name) - len("mola::Fake"))
    f = tmp_path / "unregistered.mm"
    m.save(f, compression="none")
    data = f.read_bytes()
    assert name in data
    f.write_bytes(data.replace(name, fake))
    return f


def test_auto_import_on_map_load(demos_dir, tmp_path):
    code = (
        "import sys\n"
        "import mp2p_icp\n"
        "for _ in range(2):\n"
        "    try:\n"
        "        mp2p_icp.MetricMap.from_file(sys.argv[1])\n"
        "    except RuntimeError as e:\n"
        "        print('ERROR:', e)\n"
    )
    r = _run_with_fake_plugin_module(
        tmp_path, code, _map_file_with_unregistered_class(demos_dir, tmp_path)
    )
    assert r.returncode == 0, r.stderr
    # Imported once, before retrying the first load; both loads fail with a hint:
    assert r.stdout.count(FAKE_IMPORTED) == 1, r.stdout
    assert r.stdout.index(FAKE_IMPORTED) < r.stdout.index("ERROR:")
    assert r.stdout.count("ERROR:") == 2, r.stdout
    assert r.stdout.count("pip install mola-metric-maps") == 2, r.stdout
    assert "mola::FakeX" in r.stdout


def test_auto_import_on_pipeline(tmp_path):
    code = (
        "import mp2p_icp\n"
        "try:\n"
        "    mp2p_icp.filter_pipeline_from_yaml(\n"
        "        {'filters': [{'class_name': 'mola::FakeFilter', 'params': {}}]})\n"
        "except RuntimeError as e:\n"
        "    print('ERROR:', e)\n"
        "# Classes of other namespaces do not import it:\n"
        "try:\n"
        "    mp2p_icp.filter_pipeline_from_yaml([{'class_name': 'foo::Bar', 'params': {}}])\n"
        "except RuntimeError as e:\n"
        "    print('ERROR:', e)\n"
    )
    r = _run_with_fake_plugin_module(tmp_path, code)
    assert r.returncode == 0, r.stderr
    assert r.stdout.count(FAKE_IMPORTED) == 1, r.stdout
    assert "Class 'mola::FakeFilter' is not registered" in r.stdout
    assert "pip install mola-metric-maps" in r.stdout
    assert r.stdout.count("ERROR:") == 2, r.stdout

    # Without any `mola::` class, the module is not imported:
    code = "import mp2p_icp\nmp2p_icp.filter_pipeline_from_yaml([])\n"
    r = _run_with_fake_plugin_module(tmp_path, code)
    assert r.returncode == 0, r.stderr
    assert FAKE_IMPORTED not in r.stdout


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
    # without it in a new one, where a fake mola_metric_maps module (that
    # registers nothing) shadows any real one.
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
    r = _run_with_fake_plugin_module(tmp_path, code, plugin_map_file)
    assert r.returncode == 0, f"No RuntimeError without the plugin:\n{r.stdout}\n{r.stderr}"
    assert "mp2p_icp.load_plugins(" in r.stdout


def test_plugin_map_auto_import(plugin_map, plugin_map_file, tmp_path):
    # A mola_metric_maps module that registers the classes, as the real one
    # does, is imported when needed:
    code = (
        "import sys\n"
        "import mp2p_icp\n"
        "print(mp2p_icp.MetricMap.from_file(sys.argv[1]).size())\n"
    )
    module_code = f"import mp2p_icp\nmp2p_icp.load_plugins({PLUGIN!r})\n"
    r = _run_with_fake_plugin_module(tmp_path, code, plugin_map_file, module_code=module_code)
    assert r.returncode == 0, f"{r.stdout}\n{r.stderr}"
    assert FAKE_IMPORTED in r.stdout
    assert str(plugin_map.size()) in r.stdout


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
