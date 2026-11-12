# mp2p-icp Python wheel: developer notes

PyPI name `mp2p-icp`, import name `mp2p_icp`. `pyproject.toml`
(scikit-build-core) runs the superbuild in `CMakeLists.txt`, which builds as
ExternalProjects:

1. `mola_common` (CMake scripts only), downloaded from GitHub at a fixed commit.
2. `mp2p_icp_core` from this repository: its four C++ libraries and the Python
   bindings (`mp2p_icp_core/python`), without apps, tests nor
   `mola_imu_preintegration` (so `FilterDeskew` has no IMU-based methods).

Both are built against the C++ SDK of the [`mrpt3`](https://pypi.org/project/mrpt3/)
wheel (`site-packages/mrpt/sdk`: MRPT headers, `libmrpt_*.so`, CMake configs,
Eigen and oneTBB), which is a build requirement, and installed into a staging
tree with the layout of the wheel:

```
site-packages/
  mp2p_icp/        Python package, .pyi stubs, configs/, _core.*.so
  mp2p_icp/sdk/    C++ SDK: include/, lib/libmp2p_icp{,_common,_map,_filters}.so,
                   lib/<library>/cmake/ (CMake configs), share/mola_common/
                   (mola_common CMake functions) and __init__.py
  mp2p_icp.libs/   other third-party libs bundled by auditwheel (if any)
```

* **MRPT and oneTBB are never bundled.** MRPT keeps a single class registry per
  process (used, e.g., to deserialize `.mm` files), and Python types are shared
  with the `mrpt` Python modules, so the wheel must load the very same
  `libmrpt_*.so` and `libtbb.so.12` as the installed `mrpt3` wheel. RPATHs
  point there (`$ORIGIN/../../../mrpt/sdk/lib` for the libraries in
  `mp2p_icp/sdk/lib`, `$ORIGIN/sdk/lib:$ORIGIN/../mrpt/sdk/lib` for the
  extension), and auditwheel is told to exclude `libmrpt_*`, `libtbb*` and the
  OpenGL (libglvnd) libraries (`repair-wheel-command` in `pyproject.toml`).
* pybind11 is pinned to the version of the mrpt3 wheel
  (`mrpt.sdk.PYBIND11_VERSION`), or Python types would not be shared; the
  superbuild checks it.
* Libraries are built without SOVERSION (`MOLA_NO_SOVERSION=ON`), since wheels
  cannot hold symlinks. That option is only in the `feat/no-soversion-option`
  branch of mola_common for now (MOLAorg/mola_common#1): `CMakeLists.txt` pins a
  commit of it; switch to a release once merged (see the TODO there).
* The version comes from `mp2p_icp_core/package.xml` (as `mp2p_icp.__version__`).
* The CMake files of the SDK must be relocatable: the superbuild fails if they
  hold absolute paths of the build machine.

## The C++ SDK, for downstream wheels

Other wheels with C++ code using mp2p_icp (e.g. `mola-metric-maps`) build
against `mp2p_icp/sdk`, as this one builds against `mrpt/sdk`. `mp2p_icp.sdk`
is a `cmake.prefix` entry point, so with scikit-build-core, and `mp2p-icp`,
`mrpt3` and `pybind11` (the version in `mp2p_icp.sdk.PYBIND11_VERSION`) in
`[build-system] requires`, this just works:

```cmake
find_package(mola_common REQUIRED)   # mola_add_library() & co.
find_package(mp2p_icp_map REQUIRED)  # it also finds the MRPT modules it needs
target_link_libraries(my_lib PUBLIC mola::mp2p_icp_map)
```

Other build systems pass `mp2p_icp.sdk.get_cmake_prefix_path()` and
`mrpt.sdk.get_cmake_prefix_path()` in `CMAKE_PREFIX_PATH` (see also
`get_include()` and `get_library_dir()`). Downstream wheels must:

* Build their libraries with `-DMOLA_NO_SOVERSION=ON` (if they use
  `mola_add_library()`), and set an RPATH to the SDK libraries, e.g.
  `$ORIGIN/../mp2p_icp/sdk/lib` and `$ORIGIN/../mrpt/sdk/lib` for files in
  `site-packages/<package>/`.
* Tell auditwheel not to bundle `libmp2p_icp*.so`, `libmrpt_*.so` nor
  `libtbb*.so*`, for the same reason as above.
* Require `mp2p-icp>=X.Y.Z,<X.(Y+1)` and the mrpt3 range of the mp2p-icp wheel
  they build against (no SOVERSION here either, see below).
* To return or accept the Python types bound here (`mp2p_icp.MetricMap`...) in
  pybind11 functions, build with the same pybind11 version, and `import
  mp2p_icp` before (e.g. `py::module_::import("mp2p_icp")` in their module
  init).

`tests/sdk_consumer` is such a downstream project: a library defined with
`mola_add_library()`, a program, and a pybind11 module that returns a
`mp2p_icp.MetricMap`. `tests/test_wheel.py` builds and runs it.

## The mrpt3 requirement and its ABI

The wheel requires `mrpt3>=X.Y.Z,<X.(Y+1)`, where `X.Y.Z` is the mrpt3 version
it was built against (computed at build time by
`build_helpers/mrpt3_requirement.py`). The build uses the exact mrpt3 version
pinned in `[build-system] requires`: update it (and the pybind11 pin, if it
changed) to build against a newer mrpt3.

This assumes that mrpt3 patch releases of one minor series keep the C++ ABI of
the `libmrpt_*.so` files, which MRPT does not formally guarantee: no
SOVERSION, so a patch release that changes a class layout or removes a symbol
used by mp2p_icp would make `import mp2p_icp` fail (undefined symbol) or
misbehave. If that happens, rebuild against the new mrpt3 and publish a new
mp2p-icp version, or tighten the range by hand. A new mrpt3 minor version
always needs a new mp2p-icp build.

## Local build (quick check, manylinux image)

The reference build is the one in CI (next section). To reproduce it locally
(needs Docker), from the repository root, with
`git submodule update --init` done:

```bash
python3 -m venv /tmp/cibw && /tmp/cibw/bin/pip install cibuildwheel
CIBW_BUILD=cp312-manylinux_x86_64 /tmp/cibw/bin/cibuildwheel --platform linux \
  --output-dir /tmp/wheelhouse packaging/pip
```

That builds, repairs and tests the wheel. To test it again in a clean system:

```bash
docker run --rm -v $PWD:/src:ro -v /tmp/wheelhouse:/wh:ro python:3.12-slim bash -c \
  "apt-get update -qq && apt-get install -y -qq g++ > /dev/null && \
   pip install pytest cmake ninja pybind11==3.1.0 /wh/mp2p_icp-*-cp312-*.whl && \
   python /src/packaging/pip/tests/test_wheel.py"
```

Building outside the manylinux images (`python -m build --wheel packaging/pip`)
also works, if the compiler is ABI compatible with the one used for the mrpt3
wheel, but do not source a ROS environment first: the superbuild ignores
`$CMAKE_PREFIX_PATH`, yet other variables may still leak in. The build dir
(`packaging/pip/build/<wheel tag>`) is kept for incremental rebuilds.

`tests/test_wheel.py` (run by cibuildwheel) checks that the libraries come from
the mrpt3 wheel and are not bundled, a `.mm` round trip, ICP between the demo
maps, NumPy conversions and the SDK files, builds and runs the SDK consumer
(if cmake and a C++ compiler are found; also install `pybind11==3.1.0` for its
Python module), and then runs the pytest suite of
`mp2p_icp_core/python/tests`. Both use the example data in
`mp2p_icp_core/demos`, so they need the source tree.

To check the binaries of a wheel:

```bash
auditwheel show mp2p_icp-*.whl   # libmrpt_*, libtbb: "not found" here is expected
unzip -o -d /tmp/w mp2p_icp-*.whl && readelf -d /tmp/w/mp2p_icp/*.so /tmp/w/mp2p_icp/sdk/lib/*.so | grep -E "NEEDED|RUNPATH"
```

## Type stubs

The committed `mp2p_icp_core/python/mp2p_icp/_core.pyi` is generated by
`mp2p_icp_core/python/scripts/generate_stubs.py` (pybind11-stubgen) from a
module built with the pybind11 version of the mrpt3 wheel: other versions
write different annotations. With a wheel built as above, from the repository
root:

```bash
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v $PWD:/src -v /tmp/wheelhouse:/wh:ro \
  python:3.12-slim bash -c "pip install --user mrpt3 /wh/mp2p_icp-*-cp312-*.whl \
  pybind11-stubgen==3.0.0 && cd / && python /src/mp2p_icp_core/python/scripts/generate_stubs.py"
```

## CI and publishing

`.github/workflows/python-wheels.yml` builds and tests the wheels with
cibuildwheel, in `manylinux_2_28` containers, on native x86_64 and aarch64
runners:

* Pull requests touching the bindings, the packaging or the workflow: one
  Python version per platform.
* Weekly, on release tags and by hand: Python 3.10 to 3.14.
* Release tags (`X.Y.Z`, as the rest of the release process) upload the wheels
  to PyPI. A manual run can also upload to TestPyPI or PyPI (`publish` input).
  The tag must match the version in `mp2p_icp_core/package.xml`.

Uploads use PyPI Trusted Publishing (no tokens). **One-time setup**, by a
maintainer: on pypi.org, create the `mp2p-icp` project, or a "pending
publisher" for it, with owner `MOLAorg`, repository `mp2p_icp`, workflow
`python-wheels.yml` and environment `pypi`; then create that `pypi`
environment in the GitHub repository settings (optionally, with required
reviewers). For TestPyPI, the same on test.pypi.org with environment
`testpypi`. PyPI never accepts the same version twice: try changes on TestPyPI
first.
