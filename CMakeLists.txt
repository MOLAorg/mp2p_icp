# ------------------------------------------------------------------------------
#        Multi primitive-to-primitive (MP2P) ICP C++ library
#
# Copyright (C) 2018-2020, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under BSD 3-Clause License. See COPYING.
# ------------------------------------------------------------------------------

# Minimum CMake version: deprecated if <3.5 (as of Apr 2024)
cmake_minimum_required(VERSION 3.5)

# Tell CMake we'll use C++ for use in its tests/flags
project(mp2p_icp_all LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
# to allow stand-along builds outside of MOLA, use local copy instead:
find_package(mola_common QUIET)
if (NOT mola_common_FOUND)
	message(STATUS "Standalone ${PROJECT_NAME} build: using mola_common from submodule.")
	file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/mola_common-build")
	execute_process(COMMAND "${CMAKE_COMMAND}" "${PROJECT_SOURCE_DIR}/3rdparty/mola_common"
	                WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/mola_common-build")
	# 2nd attempt:
	set(mola_common_DIR "${CMAKE_BINARY_DIR}/mola_common-build")
	find_package(mola_common REQUIRED)
endif()

# 3rd party:
add_subdirectory(3rdparty)

# find dependencies:
find_package(MRPT 2.11.5 REQUIRED COMPONENTS
	containers
	tfest
	maps
	gui
	topography
)
find_package(Threads REQUIRED)
find_package(TBB) # optional

if (TBB_FOUND)
	option(MP2PICP_USE_TBB "If found TBB, this option controls whether to use it or not" ON)
endif()

#----
# Extract version from package.xml
# Example line:" <version>0.3.2</version>"
file(READ package.xml contentPackageXML)
string(REGEX MATCH "<version>([0-9\.]*)</version>" _ ${contentPackageXML})
set(MP2P_ICP_VERSION ${CMAKE_MATCH_1})
message(STATUS "MP2P_ICP version: ${MP2P_ICP_VERSION} (detected in package.xml)")
string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)" _ ${MP2P_ICP_VERSION})
set(MP2P_ICP_MAJOR_VERSION ${CMAKE_MATCH_1})
set(MP2P_ICP_MINOR_VERSION ${CMAKE_MATCH_2})
set(MP2P_ICP_PATCH_VERSION ${CMAKE_MATCH_3})
#----


# -----------------------
# define library targets:
# -----------------------

# Metric map container library:
add_subdirectory(mp2p_icp_map)

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
