# ------------------------------------------------------------------------------
#        Multi primitive-to-primitive (MP2P) ICP C++ library
#
# Copyright (C) 2018-2026, Jose Luis Blanco-Claraco, contributors (AUTHORS)
# All rights reserved.
# Released under BSD 3-Clause License. See COPYING.
# ------------------------------------------------------------------------------

# Minimal downstream project, built against the C++ SDK inside the mp2p-icp
# wheel and the MRPT one of the mrpt3 wheel, as a downstream wheel would
# (CMAKE_PREFIX_PATH: mp2p_icp.sdk.get_cmake_prefix_path() and
# mrpt.sdk.get_cmake_prefix_path()). Run by test_wheel.py.

cmake_minimum_required(VERSION 3.22)
project(mp2p_icp_sdk_consumer LANGUAGES CXX)

# The CMake functions of MOLA modules (mola_add_library()...):
find_package(mola_common REQUIRED)
find_package(mp2p_icp_map REQUIRED)
find_package(mp2p_icp_filters REQUIRED)  # it also finds mp2p_icp, mp2p_icp_common

# A library defined as MOLA modules do (headers in include/, version from
# package.xml). Wheels need -DMOLA_NO_SOVERSION=ON.
mola_add_library(
  TARGET sdk_consumer_lib
  SOURCES sdk_consumer_lib.cpp include/sdk_consumer_lib.h
  PUBLIC_LINK_LIBRARIES mola::mp2p_icp_map
  CMAKE_DEPENDENCIES mp2p_icp_map
)

add_executable(sdk_consumer main.cpp)
target_link_libraries(sdk_consumer PRIVATE sdk_consumer_lib mola::mp2p_icp mola::mp2p_icp_filters)

# A Python module that returns and accepts types bound by mp2p_icp._core
# (needs the same pybind11 version as the wheel, see mp2p_icp.sdk):
find_package(Python COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG)
if (pybind11_FOUND AND Python_FOUND)
  pybind11_add_module(sdk_consumer_py MODULE py_module.cpp)
  target_link_libraries(sdk_consumer_py PRIVATE sdk_consumer_lib)
else()
  message(STATUS "pybind11 not found: the Python module will not be built")
endif()
