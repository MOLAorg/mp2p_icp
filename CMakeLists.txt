# ------------------------------------------------------------------------------
#        Multi primitive-to-primitive (MP2) ICP C++ library
#
# Copyright (C) 2018-2020, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under BSD 3-Clause License. See COPYING.
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

# Tell CMake we'll use C++ for use in its tests/flags
project(mp2p_icp LANGUAGES CXX)

# find dependencies:
find_package(mrpt-tfest REQUIRED)
find_package(mrpt-containers REQUIRED)
find_package(mrpt-maps REQUIRED)
find_package(mrpt-gui REQUIRED) # for tests/demos

# MOLA CMake scripts: "mola_xxx()"
# to allow stand-along builds outside of MOLA, use local copy instead:
find_package(mola-common QUIET)
if (NOT mola-common_FOUND)
	include(${mp2p_icp_SOURCE_DIR}/localcopy_mola_cmake_functions.cmake)
endif()

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES
		mrpt::containers
		mrpt::tfest
		mrpt::maps
)

# -----------------------
# define tests:
enable_testing()

add_subdirectory(tests)
