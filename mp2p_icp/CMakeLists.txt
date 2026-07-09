# ------------------------------------------------------------------------------
#  mp2p_icp metapackage
#
#  Kept for backward compatibility: depends on mp2p_icp_core and
#  mp2p_icp_viz, so existing <depend>mp2p_icp</depend> consumers keep
#  building/installing the exact same set of libraries and applications.
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22...3.31)

project(mp2p_icp NONE)

if("$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED)
  catkin_metapackage()
else()
  # ROS2: this metapackage has no code of its own, but colcon expects an
  # 'install' target to exist for "cmake" build_type packages; declare a
  # no-op one to avoid a spurious colcon warning.
  install(CODE "")
endif()
