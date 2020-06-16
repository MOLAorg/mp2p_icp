
set(MP2PICP_BUILD_POINTMATCHER ON CACHE BOOL "Build libpointmatcher modules")

# Check if we have the dependencies of pointmatcher, and silently skip it if we don't have:
find_package(Boost COMPONENTS thread filesystem system program_options date_time QUIET)
if (NOT Boost_FOUND)
  message(STATUS "mp2p_icp: Boost not found: building without libpointmatcher")
  set(MP2PICP_BUILD_POINTMATCHER OFF CACHE BOOL "Build libpointmatcher modules" FORCE)
  return()
endif()

if (MP2PICP_BUILD_POINTMATCHER)
  # libnabo
  # ---------------
  option(LIBNABO_BUILD_EXAMPLES "Build libnabo examples" OFF)
  option(LIBNABO_BUILD_TESTS "Build libnabo tests" OFF)
  option(LIBNABO_BUILD_PYTHON "Build libnabo python" OFF)
  option(LIBNABO_BUILD_DOXYGEN "Build libnabo doxygen documentation" OFF)
  mark_as_advanced(LIBNABO_BUILD_EXAMPLES)
  mark_as_advanced(LIBNABO_BUILD_TESTS)
  mark_as_advanced(LIBNABO_BUILD_PYTHON)
  mark_as_advanced(LIBNABO_BUILD_DOXYGEN)
  
  add_subdirectory(libnabo)
  
  # libpointmatcher:
  # ---------------
  option(POINTMATCHER_BUILD_EXAMPLES "Build libpointmatcher examples" OFF)
  option(POINTMATCHER_BUILD_EVALUATIONS "Build libpointmatcher evaluations" OFF)
  mark_as_advanced(POINTMATCHER_BUILD_EXAMPLES)
  mark_as_advanced(POINTMATCHER_BUILD_EVALUATIONS)
  
  add_subdirectory(libpointmatcher)
endif()
  
