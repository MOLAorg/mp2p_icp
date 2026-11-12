#!/usr/bin/env python3
"""Checks an installed mp2p-icp wheel: it imports, it uses the MRPT and oneTBB
libraries of the mrpt3 wheel (no bundled copies), and it works (.mm files, ICP,
NumPy). Then it runs the Python tests of the source tree, if pytest is found.

Usage: python test_wheel.py   (with the wheel installed in the environment)

The example data comes from mp2p_icp_core/demos in the source tree.
"""

import importlib.metadata
import importlib.util
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
DEMOS_DIR = REPO_ROOT / "mp2p_icp_core" / "demos"
TESTS_DIR = REPO_ROOT / "mp2p_icp_core" / "python" / "tests"

# Libraries that must come from the mrpt3 wheel, never from this one:
SDK_LIB_PREFIXES = ("libmrpt_", "libtbb")


def check_install(failed: list) -> None:
    import mrpt.sdk
    import mp2p_icp

    pkg_dir = Path(mp2p_icp.__file__).resolve().parent
    sdk_lib_dir = Path(mrpt.sdk.get_library_dir()).resolve()
    print("mp2p_icp", mp2p_icp.__version__, "from", pkg_dir)
    print("MRPT SDK", mrpt.sdk.MRPT_VERSION, "from", sdk_lib_dir)

    dist_version = importlib.metadata.version("mp2p-icp")
    if mp2p_icp.__version__ != dist_version:
        failed.append(f"__version__ {mp2p_icp.__version__} != wheel version {dist_version}")

    # No copies of the SDK libraries in the wheel:
    for f in importlib.metadata.files("mp2p-icp") or []:
        if Path(f).name.startswith(SDK_LIB_PREFIXES):
            failed.append(f"SDK library bundled in the wheel: {f}")

    # Where the shared libraries of this process were loaded from:
    loaded = set()
    for line in Path("/proc/self/maps").read_text().splitlines():
        path = line.split()[-1]
        if path.startswith("/") and ".so" in path:
            loaded.add(Path(path).resolve())
    for lib in sorted(loaded):
        if lib.name.startswith(SDK_LIB_PREFIXES) and lib.parent != sdk_lib_dir:
            failed.append(f"{lib.name} loaded from {lib.parent}, not from the MRPT SDK")
        if lib.name.startswith("libmp2p_icp") and lib.parent != pkg_dir:
            failed.append(f"{lib.name} loaded from {lib.parent}, not from the wheel")
    names = {lib.name for lib in loaded}
    for expected in ["libmp2p_icp.so", "libmp2p_icp_map.so", "libmrpt_maps.so", "libtbb.so.12"]:
        if expected not in names:
            failed.append(f"{expected} not loaded")


def check_metric_map_files(failed: list, tmp: Path) -> None:
    import mp2p_icp

    m = mp2p_icp.MetricMap.from_file(DEMOS_DIR / "local_001.mm")
    n = {name: layer.size() for name, layer in m.layers.items()}
    m.metadata = {"note": "wheel test"}
    m.save(tmp / "copy.mm")
    m2 = mp2p_icp.MetricMap.from_file(tmp / "copy.mm")
    n2 = {name: layer.size() for name, layer in m2.layers.items()}
    if not n or n != n2 or m2.metadata.get("note") != "wheel test":
        failed.append(f".mm round trip: {n} vs {n2}, metadata {m2.metadata}")
    print("[OK] .mm round trip:", n2)


def check_icp(failed: list) -> None:
    import mp2p_icp

    # Same result as: icp-run --input-local local_001.mm --input-global global_001.mm
    #   -c icp-settings-2d-lidar-example-point2point.yaml
    local = mp2p_icp.MetricMap.from_file(DEMOS_DIR / "local_001.mm")
    glob = mp2p_icp.MetricMap.from_file(DEMOS_DIR / "global_001.mm")
    icp, params = mp2p_icp.icp_pipeline_from_yaml(
        DEMOS_DIR / "icp-settings-2d-lidar-example-point2point.yaml"
    )
    params.debugPrintIterationProgress = False
    r = icp.align(local, glob)
    p = r.optimal_tf.mean
    ok = (
        abs(p.x - 0.1285) < 1e-4
        and abs(p.y - 0.0472) < 1e-4
        and abs(math.degrees(p.yaw) - 5.69) < 0.01
        and r.nIterations == 25
    )
    if not ok:
        failed.append(f"ICP on the demo maps: unexpected result {p}, {r.nIterations} iterations")
    print(f"[OK] ICP on the demo maps: {p}, quality={r.quality:.3f}")


def check_numpy(failed: list) -> None:
    import mp2p_icp

    rng = np.random.default_rng(0)
    fields = {
        "x": rng.uniform(-5, 5, 100).astype(np.float32),
        "y": rng.uniform(-5, 5, 100).astype(np.float32),
        "z": rng.uniform(-1, 1, 100).astype(np.float32),
        "intensity": rng.uniform(0, 100, 100).astype(np.float32),
        "ring": rng.integers(0, 64, 100).astype(np.uint16),
        "gps_time": rng.uniform(1e9, 1e9 + 10, 100),
    }
    back = mp2p_icp.layer_to_numpy(mp2p_icp.layer_from_numpy(fields))
    for name, a in fields.items():
        if name not in back or back[name].dtype != a.dtype or not np.array_equal(back[name], a):
            failed.append(f"NumPy round trip: field '{name}' differs")
    print("[OK] NumPy round trip:", sorted(back))


def run_pytest(failed: list) -> None:
    if importlib.util.find_spec("pytest") is None:
        print("[SKIP] Python tests of the source tree: pytest not found")
        return
    # Run from elsewhere, so the package in the source tree is not imported
    # instead of the installed one:
    with tempfile.TemporaryDirectory() as cwd:
        r = subprocess.run(
            [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", str(TESTS_DIR)],
            cwd=cwd,
        )
    if r.returncode != 0:
        failed.append(f"Python tests of the source tree failed ({TESTS_DIR})")


def main() -> int:
    failed = []
    with tempfile.TemporaryDirectory() as tmp:
        for check in [
            lambda: check_metric_map_files(failed, Path(tmp)),
            lambda: check_icp(failed),
            lambda: check_numpy(failed),
            # Last, so libraries loaded on demand are checked too:
            lambda: check_install(failed),
        ]:
            try:
                check()
            except Exception as e:  # noqa: BLE001
                failed.append(f"{type(e).__name__}: {e}")
    run_pytest(failed)

    if failed:
        print("\nFAILURES:", *failed, sep="\n- ")
        return 1
    print("\nAll checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
