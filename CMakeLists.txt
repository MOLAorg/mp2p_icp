# ------------------------------------------------------------------------------
#        Multi primitive-to-primitive (MP2P) ICP C++ library
#
# Copyright (C) 2018-2020, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under BSD 3-Clause License. See COPYING.
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

# Tell CMake we'll use C++ for use in its tests/flags
project(mp2p_icp_all LANGUAGES CXX)

# 3rd party:
add_subdirectory(3rdparty)

# find dependencies:
find_package(MRPT 2.4.0 REQUIRED COMPONENTS containers tfest maps gui)

# MOLA CMake scripts: "mola_xxx()"
# to allow stand-along builds outside of MOLA, use local copy instead:
find_package(mola-common QUIET)
if (NOT mola-common_FOUND)
	message(STATUS "Standalone ${PROJECT_NAME} build: cloning required mola-common repository...")
	execute_process(COMMAND git clone git@github.com:MOLAorg/mola-common.git "${CMAKE_CURRENT_BINARY_DIR}/mola-common-src")
	file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/mola-common-build")
	execute_process(COMMAND "${CMAKE_COMMAND}" "${CMAKE_CURRENT_BINARY_DIR}/mola-common-src"
	                WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/mola-common-build")
	# 2nd attempt:
	set(mola-common_DIR "${CMAKE_BINARY_DIR}/mola-common-build")
	find_package(mola-common REQUIRED)
endif()

# -----------------------
# define library targets:
# -----------------------

# Main mp2p_icp library:
add_subdirectory(mp2p_icp)

# Basic filters library:
add_subdirectory(mp2p_icp_filters)

# -----------------------
# define tests:
# -----------------------
option(MP2PICP_BUILD_TESTING "Build unit tests" ON)
if (MP2PICP_BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()

# -----------------------
# define apps:
# -----------------------
option(MP2PICP_BUILD_APPLICATIONS "Build mp2p_icp applications" ON)
if (MP2PICP_BUILD_APPLICATIONS)
  add_subdirectory(apps)
endif()
