# ------------------------------------------------------------------------------
#        Multi primitive-to-primitive (MP2P) ICP C++ library
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco, contributors (AUTHORS)
# All rights reserved.
# Released under BSD 3-Clause License. See COPYING.
# ------------------------------------------------------------------------------

# Superbuild for the mp2p-icp Python wheel, driven by scikit-build-core (see
# pyproject.toml). It builds, as ExternalProjects, mola_common (CMake scripts)
# and then mp2p_icp_core (C++ libraries and Python bindings, no apps nor
# tests), against the MRPT C++ SDK of the mrpt3 wheel (mrpt/sdk), and puts
# into the wheel:
#
#   mp2p_icp/     Python package, the _core extension and libmp2p_icp*.so
#
# The MRPT and oneTBB libraries are never bundled: they are loaded from the
# installed mrpt3 wheel (RPATH), since there must be only one MRPT class
# registry per process.

cmake_minimum_required(VERSION 3.24)
project(mp2p_icp_wheel LANGUAGES C CXX)

include(ExternalProject)

set(MP2P_ICP_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../mp2p_icp_core")

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# The MRPT C++ SDK, from the mrpt3 wheel installed in the build environment:
execute_process(
  COMMAND "${Python_EXECUTABLE}" -c
    "import mrpt.sdk as s; print(s.get_cmake_prefix_path(), s.MRPT_VERSION, s.PYBIND11_VERSION, sep=';')"
  OUTPUT_VARIABLE _sdk_info
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE _sdk_result
)
if (NOT _sdk_result EQUAL 0)
  message(FATAL_ERROR "Cannot import mrpt.sdk: the mrpt3 wheel must be installed in the build environment")
endif()
list(GET _sdk_info 0 MRPT_SDK_DIR)
list(GET _sdk_info 1 MRPT_SDK_VERSION)
list(GET _sdk_info 2 MRPT_SDK_PYBIND11_VERSION)
message(STATUS "MRPT SDK ${MRPT_SDK_VERSION} (pybind11 ${MRPT_SDK_PYBIND11_VERSION}): ${MRPT_SDK_DIR}")

# Python types are only shared between extensions built with the same pybind11:
if (NOT pybind11_VERSION VERSION_EQUAL MRPT_SDK_PYBIND11_VERSION)
  message(FATAL_ERROR
    "pybind11 ${pybind11_VERSION} found, but the mrpt3 wheel uses ${MRPT_SDK_PYBIND11_VERSION}: "
    "update the pybind11 pin in pyproject.toml")
endif()

# Install prefix of the ExternalProjects. The Python package goes to
# ${MP2P_PREFIX}/mp2p_icp, the C++ libraries to ${MP2P_PREFIX}/lib.
set(MP2P_PREFIX "${CMAKE_BINARY_DIR}/prefix")

# The C++ libraries end up next to the extension, in site-packages/mp2p_icp/,
# and the MRPT ones are in site-packages/mrpt/sdk/lib/:
set(_rpath "\$ORIGIN;\$ORIGIN/../mrpt/sdk/lib")

set(_cache_args
  -DCMAKE_BUILD_TYPE:STRING=Release
  -DCMAKE_INSTALL_PREFIX:PATH=${MP2P_PREFIX}
  -DCMAKE_INSTALL_LIBDIR:STRING=lib
  -DCMAKE_PREFIX_PATH:STRING=${MP2P_PREFIX};${MRPT_SDK_DIR}
  -DCMAKE_INSTALL_RPATH:STRING=${_rpath}
  # Wheels cannot hold the libfoo.so -> libfoo.so.X.Y symlinks:
  -DMOLA_NO_SOVERSION:BOOL=ON
  # No C++20 modules are used, and the module scanning of C++20 targets
  # (with Ninja) fails with ccache:
  -DCMAKE_CXX_SCAN_FOR_MODULES:BOOL=OFF
  -DPython_EXECUTABLE:FILEPATH=${Python_EXECUTABLE}
  -DPython3_EXECUTABLE:FILEPATH=${Python_EXECUTABLE}
  -Dpybind11_DIR:PATH=${pybind11_DIR}
  # Only use the MRPT SDK and the packages built here, not others from the
  # user package registry (~/.cmake/packages) or from a sourced ROS
  # environment ($CMAKE_PREFIX_PATH): they would not be in the wheel.
  -DCMAKE_FIND_USE_PACKAGE_REGISTRY:BOOL=OFF
  -DCMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH:BOOL=OFF
  -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY:BOOL=ON
)
# (CMAKE_MAKE_PROGRAM changes on each build: pip uses temporary environments)
foreach(_var
    CMAKE_MAKE_PROGRAM CMAKE_C_COMPILER CMAKE_CXX_COMPILER
    CMAKE_C_COMPILER_LAUNCHER CMAKE_CXX_COMPILER_LAUNCHER
    CMAKE_TOOLCHAIN_FILE)
  if (DEFINED ${_var} AND NOT "${${_var}}" STREQUAL "")
    list(APPEND _cache_args "-D${_var}:STRING=${${_var}}")
  endif()
endforeach()

# mola_common: CMake scripts only (mola_add_library()...).
# TODO: switch to a mola_common release once MOLA_NO_SOVERSION is merged
# (https://github.com/MOLAorg/mola_common/pull/1). Until then, this is a commit
# of its branch "feat/no-soversion-option".
set(MP2P_WHEEL_MOLA_COMMON_COMMIT 3e95c2bccf064506994ab2e6b0ed1c466c442ae8)
ExternalProject_Add(ep_mola_common
  URL "https://github.com/MOLAorg/mola_common/archive/${MP2P_WHEEL_MOLA_COMMON_COMMIT}.tar.gz"
  URL_HASH SHA256=b9be16cbffe28772ec2e0aed41c94827dc02b72a6a50d525358e5ac7920fb802
  DOWNLOAD_EXTRACT_TIMESTAMP ON
  BINARY_DIR "${CMAKE_BINARY_DIR}/mola_common"
  CMAKE_CACHE_ARGS ${_cache_args}
  USES_TERMINAL_CONFIGURE ON
  USES_TERMINAL_BUILD ON
  USES_TERMINAL_INSTALL ON
)

# mp2p_icp_core: the C++ libraries and the Python bindings, with oneTBB from
# the MRPT SDK, and without mola_imu_preintegration (advanced FilterDeskew
# methods are not available in the wheel).
ExternalProject_Add(ep_mp2p_icp_core
  SOURCE_DIR "${MP2P_ICP_CORE_DIR}"
  BINARY_DIR "${CMAKE_BINARY_DIR}/mp2p_icp_core"
  CMAKE_CACHE_ARGS ${_cache_args}
  CMAKE_ARGS
    -DMP2PICP_BUILD_APPLICATIONS=OFF
    -DMP2PICP_BUILD_TESTING=OFF
    -DMP2PICP_BUILD_PYTHON=ON
    -DMP2PICP_USE_TBB=ON
    -DCMAKE_DISABLE_FIND_PACKAGE_mola_imu_preintegration=ON
    # The package goes to ${MP2P_PREFIX}/mp2p_icp:
    -DMP2P_ICP_PYTHON_INSTALL_DIR=.
  DEPENDS ep_mola_common
  # Always re-run the build tool, so source edits are picked up:
  BUILD_ALWAYS ON
  USES_TERMINAL_CONFIGURE ON
  USES_TERMINAL_BUILD ON
  USES_TERMINAL_INSTALL ON
)

# Into the wheel: the Python package, plus the C++ libraries next to the
# extension (headers and CMake configs are left out). The bindings are skipped
# without error if pybind11 or Python are not found: make sure they are there.
install(CODE "
  file(GLOB _ext \"${MP2P_PREFIX}/mp2p_icp/_core*.so\")
  if (NOT _ext)
    message(FATAL_ERROR \"The mp2p_icp._core extension was not built\")
  endif()
  file(GLOB _libs \"${MP2P_PREFIX}/lib/libmp2p_icp*.so\")
  file(INSTALL \${_libs} DESTINATION \"\${CMAKE_INSTALL_PREFIX}/mp2p_icp\" USE_SOURCE_PERMISSIONS)
")
install(DIRECTORY "${MP2P_PREFIX}/mp2p_icp" DESTINATION .
  USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE
)
