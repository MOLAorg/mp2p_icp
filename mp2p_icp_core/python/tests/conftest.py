"""Common fixtures for the mp2p_icp Python tests."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

import numpy as np
import pytest

import mp2p_icp

# mp2p_icp_core/demos, with the example maps and pipelines:
DEMOS_DIR = Path(__file__).resolve().parents[2] / "demos"


def _mola_test_datasets_dir() -> Optional[Path]:
    """Datasets of the optional `mola_test_datasets` package, if available."""
    env = os.environ.get("MOLA_TEST_DATASETS_DIR")
    if env:
        return Path(env)
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("mola_test_datasets")) / "datasets"
    except Exception:
        return None


@pytest.fixture(scope="session")
def demos_dir() -> Path:
    return DEMOS_DIR


@pytest.fixture(scope="session")
def datasets_dir() -> Path:
    d = _mola_test_datasets_dir()
    if d is None or not d.is_dir():
        pytest.skip("mola_test_datasets not found (set MOLA_TEST_DATASETS_DIR)")
    return d


@pytest.fixture(scope="session")
def bunny_xyz() -> np.ndarray:
    """The Stanford bunny (decimated), as an (N,3) array."""
    f = mp2p_icp.layer_to_numpy(mp2p_icp.load_xyz_file(DEMOS_DIR / "bunny_decim.xyz.gz"))
    return np.column_stack([f["x"], f["y"], f["z"]])


@pytest.fixture
def all_fields() -> dict:
    """Per-point fields of all the supported types."""
    rng = np.random.default_rng(42)
    n = 200
    return {
        "x": rng.uniform(-10, 10, n).astype(np.float32),
        "y": rng.uniform(-10, 10, n).astype(np.float32),
        "z": rng.uniform(-1, 3, n).astype(np.float32),
        "intensity": rng.uniform(0, 255, n).astype(np.float32),
        "t": rng.uniform(0, 0.1, n).astype(np.float32),
        "gps_time": rng.uniform(1.7e9, 1.7e9 + 100, n),
        "ring": rng.integers(0, 128, n).astype(np.uint16),
        "color_r": rng.integers(0, 256, n).astype(np.uint8),
        "color_g": rng.integers(0, 256, n).astype(np.uint8),
        "color_b": rng.integers(0, 256, n).astype(np.uint8),
        "label": rng.integers(0, 1 << 30, n).astype(np.uint32),
    }
