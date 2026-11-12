#!/usr/bin/env python3
"""Checks an installed mp2p-icp wheel: it imports, it uses the MRPT and oneTBB
libraries of the mrpt3 wheel (no bundled copies), and it works (.mm files, ICP,
NumPy). Then it builds a C++ program and a Python module against its C++ SDK
(if cmake and a compiler are found), and runs the Python tests of the source
tree (if pytest is found).

Usage: python test_wheel.py   (with the wheel installed in the environment)

The example data comes from mp2p_icp_core/demos in the source tree.
"""

import importlib.metadata
import importlib.util
import math
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
DEMOS_DIR = REPO_ROOT / "mp2p_icp_core" / "demos"
TESTS_DIR = REPO_ROOT / "mp2p_icp_core" / "python" / "tests"
SDK_CONSUMER_DIR = Path(__file__).resolve().parent / "sdk_consumer"

# Libraries that must come from the mrpt3 wheel, never from this one:
SDK_LIB_PREFIXES = ("libmrpt_", "libtbb")


def check_install(failed: list) -> None:
    import mrpt.sdk
    import mp2p_icp
    import mp2p_icp.sdk

    pkg_dir = Path(mp2p_icp.__file__).resolve().parent
    lib_dir = Path(mp2p_icp.sdk.get_library_dir()).resolve()
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
        if lib.name.startswith("libmp2p_icp") and lib.parent != lib_dir:
            failed.append(f"{lib.name} loaded from {lib.parent}, not from the wheel SDK")
    names = {lib.name for lib in loaded}
    for expected in ["libmp2p_icp.so", "libmp2p_icp_map.so", "libmrpt_maps.so", "libtbb.so.12"]:
        if expected not in names:
            failed.append(f"{expected} not loaded")


def check_sdk_files(failed: list) -> None:
    import mp2p_icp
    import mp2p_icp.sdk

    sdk = Path(mp2p_icp.sdk.get_cmake_prefix_path())
    version = mp2p_icp.sdk.MP2P_ICP_VERSION
    print(f"mp2p_icp SDK {version} (pybind11 {mp2p_icp.sdk.PYBIND11_VERSION}) from {sdk}")
    if version != mp2p_icp.__version__:
        failed.append(f"SDK version {version} != {mp2p_icp.__version__}")
    files = [
        "include/mp2p_icp/metricmap.h",
        "include/mp2p_icp/ICP.h",
        "include/mp2p_icp/Parameterizable.h",
        "include/mp2p_icp_filters/FilterBase.h",
        "share/mola_common/cmake/mola_common-config.cmake",
        "share/mola_common/cmake/mola_cmake_functions.cmake",
    ]
    for lib in ["mp2p_icp_common", "mp2p_icp_map", "mp2p_icp", "mp2p_icp_filters"]:
        files += [f"lib/lib{lib}.so", f"lib/{lib}/cmake/{lib}-config.cmake"]
    for f in files:
        if not (sdk / f).is_file():
            failed.append(f"SDK file missing: {f}")
    print("[OK] SDK files")


def check_sdk_consumer(failed: list, tmp: Path) -> None:
    """Builds a C++ program, a library (with mola_add_library()) and a Python
    module against the SDKs of the mp2p-icp and mrpt3 wheels, as a downstream
    wheel would, and runs them."""
    import mrpt.sdk
    import mp2p_icp.sdk

    if not shutil.which("cmake") or not (shutil.which("c++") or shutil.which("g++")):
        print("[SKIP] SDK consumer: cmake or a C++ compiler not found")
        return
    bld = tmp / "sdk_consumer"
    cfg = [
        "cmake",
        "-S",
        str(SDK_CONSUMER_DIR),
        "-B",
        str(bld),
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DCMAKE_PREFIX_PATH={mp2p_icp.sdk.get_cmake_prefix_path()};"
        f"{mrpt.sdk.get_cmake_prefix_path()}",
        # Only the SDKs, as on a user system: not libraries registered by the
        # wheel builds in ~/.cmake/packages, nor a sourced ROS environment.
        "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF",
        "-DCMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH=OFF",
        "-DMOLA_NO_SOVERSION=ON",
        f"-DPython_EXECUTABLE={sys.executable}",
    ]
    if shutil.which("ninja"):
        cfg += ["-G", "Ninja"]
    # The Python module needs the pybind11 version of the wheel:
    try:
        import pybind11

        if pybind11.__version__ == mp2p_icp.sdk.PYBIND11_VERSION:
            cfg.append(f"-Dpybind11_DIR={pybind11.get_cmake_dir()}")
        else:
            print(
                f"pybind11 {pybind11.__version__} != {mp2p_icp.sdk.PYBIND11_VERSION}: "
                "no Python module"
            )
    except ImportError:
        print("pybind11 not found: no Python module")

    for cmd in [
        cfg,
        ["cmake", "--build", str(bld)],
        [str(bld / "bin" / "sdk_consumer"), str(DEMOS_DIR / "local_001.mm")],
    ]:
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            failed.append(f"SDK consumer, {' '.join(cmd)}:\n{r.stdout[-3000:]}\n{r.stderr[-3000:]}")
            return
    print("[OK] SDK consumer program:", r.stdout.strip().replace("\n", "; "))

    # mola_add_library() with MOLA_NO_SOVERSION: a single, unversioned file
    libs = sorted(p.name for p in (bld / "lib").glob("libsdk_consumer_lib.so*"))
    if libs != ["libsdk_consumer_lib.so"]:
        failed.append(f"SDK consumer: unexpected library files {libs}")

    modules = list(bld.rglob("sdk_consumer_py*.so"))
    if not modules:
        print("[SKIP] SDK consumer Python module: not built")
        return
    code = (
        "import mp2p_icp, sdk_consumer_py as c\n"
        "m = c.make_map(5)\n"
        "assert isinstance(m, mp2p_icp.MetricMap), type(m)\n"
        "assert m.layers['raw'].size() == 5\n"
        "m.layers['more'] = [[0.0, 0.0, 0.0]]\n"
        "assert c.count_points(m) == 6\n"
        f"assert c.count_points(mp2p_icp.MetricMap.from_file({str(DEMOS_DIR / 'local_001.mm')!r})) > 0\n"
    )
    env = dict(os.environ, PYTHONPATH=str(modules[0].parent))
    r = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, cwd=tmp, env=env
    )
    if r.returncode != 0:
        failed.append(f"SDK consumer Python module:\n{r.stdout[-3000:]}\n{r.stderr[-3000:]}")
        return
    print("[OK] SDK consumer Python module: shares mp2p_icp.MetricMap")


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
        and r.nIterations == 26
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
            lambda: check_sdk_files(failed),
            lambda: check_sdk_consumer(failed, Path(tmp)),
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
