# ------------------------------------------------------------------------------
#        mp2p_icp_viz: GUI applications for mp2p_icp
#
# Copyright (C) 2018-2020, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under BSD 3-Clause License. See COPYING.
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+

if("$ENV{ROS_VERSION}" STREQUAL "2")
  set(DETECTED_ROS2 TRUE)
elseif("$ENV{ROS_VERSION}" STREQUAL "1")
  set(DETECTED_ROS1 TRUE)
endif()

project(mp2p_icp_viz LANGUAGES CXX)

# This package builds as a plain "cmake" build_type under colcon (not
# ament_cmake or catkin), but colcon still passes these catkin-only
# variables when invoked with --symlink-install. Consume them here to
# silence the "Manually-specified variables were not used" CMake warning.
set(_mp2p_icp_ignored_vars "${CATKIN_INSTALL_INTO_PREFIX_ROOT}" "${CATKIN_SYMLINK_INSTALL}")
unset(_mp2p_icp_ignored_vars)

find_package(mola_common REQUIRED)

# The mp2p_icp (ICP algorithms) library, built by the sibling mp2p_icp_core
# package:
find_package(mp2p_icp REQUIRED)

find_package(Threads REQUIRED)

# -- Dear ImGui, from the MRPT vendor package --------------------------------
#
# mrpt_imgui_vendor ships a single vendored copy of Dear ImGui (docking branch)
# plus ImPlot and portable-file-dialogs, and exports it as a static target
# aliased to the upstream-style name imgui::imgui. Shared by both apps.
find_package(mrpt_imgui_vendor REQUIRED)

#----
# Extract version from package.xml
file(READ package.xml contentPackageXML)
string(REGEX MATCH "<version>([0-9\.]*)</version>" _ ${contentPackageXML})
set(MP2P_ICP_VERSION ${CMAKE_MATCH_1})
message(STATUS "MP2P_ICP_VIZ version: ${MP2P_ICP_VERSION} (detected in package.xml)")
#----

add_subdirectory(apps)

if(DETECTED_ROS1)
  find_package(catkin REQUIRED)
  catkin_package()
endif()
