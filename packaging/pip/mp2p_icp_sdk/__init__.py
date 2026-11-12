"""C++ SDK shipped inside the mp2p-icp wheel: headers, shared libraries and
CMake config files of the mp2p_icp libraries (mp2p_icp_common, mp2p_icp_map,
mp2p_icp, mp2p_icp_filters) and the CMake functions of mola_common
(mola_add_library()...), so other wheels can build C++ code against them.

With scikit-build-core, ``find_package(mp2p_icp_map)`` (or any other of them)
works out of the box, since this package is registered as a ``cmake.prefix``
entry point, as the MRPT SDK of the ``mrpt3`` wheel (``mrpt.sdk``) they depend
on. Other build systems must pass both ``get_cmake_prefix_path()`` and
``mrpt.sdk.get_cmake_prefix_path()`` in ``CMAKE_PREFIX_PATH``.

C++ extensions built this way must use the same pybind11 version as this
wheel (``PYBIND11_VERSION``), and ``import mp2p_icp`` before using them, to
share Python types (``mp2p_icp.MetricMap``...) with it.
"""

from pathlib import Path

from ._build_info import MP2P_ICP_VERSION, PYBIND11_VERSION  # noqa: F401

_ROOT = Path(__file__).resolve().parent


def get_cmake_prefix_path() -> str:
    """Directory to add to CMAKE_PREFIX_PATH to find the mp2p_icp and
    mola_common CMake packages."""
    return str(_ROOT)


def get_include() -> str:
    """Directory with the mp2p_icp C++ headers."""
    return str(_ROOT / "include")


def get_library_dir() -> str:
    """Directory with the mp2p_icp shared libraries."""
    return str(_ROOT / "lib")
