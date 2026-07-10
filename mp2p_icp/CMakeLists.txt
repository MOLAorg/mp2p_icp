# ------------------------------------------------------------------------------
#  mp2p_icp metapackage
#
#  Kept for backward compatibility: depends on mp2p_icp_core and
#  mp2p_icp_viz, so existing <depend>mp2p_icp</depend> consumers keep
#  building/installing the exact same set of libraries and applications.
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22...3.31)

project(mp2p_icp NONE)

# This is a meta package, there are no targets.
# Install files:
install(DIRECTORY
  #rviz2
  DESTINATION
    share/${PROJECT_NAME}
)

# Silent warning on unused variable:
set(ignored ${CMAKE_EXPORT_COMPILE_COMMANDS})
unset(ignored)

