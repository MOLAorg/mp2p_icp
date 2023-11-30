
set(MP2PICP_BUILD_POINTMATCHER OFF CACHE BOOL "Build libpointmatcher modules")

if (MP2PICP_BUILD_POINTMATCHER)
    # Check if we have the dependencies of pointmatcher, and silently skip it if we don't have:
    find_package(Boost COMPONENTS thread filesystem system program_options date_time QUIET)
    if (NOT Boost_FOUND)
        message(STATUS "mp2p_icp: Boost not found: building without libpointmatcher")
        set(MP2PICP_BUILD_POINTMATCHER OFF CACHE BOOL "Build libpointmatcher modules" FORCE)
    endif()
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
  
# ================================
# robin-map
# ================================
# This defines the target "tsl::robin_map"
add_subdirectory(robin-map)

# Install lib:
install(TARGETS robin_map EXPORT robin_map-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
# Install hdrs:
install(
  DIRECTORY robin-map/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

# make project importable from build_dir:
export(
    TARGETS robin_map
    # export to ROOT cmake directory (when building MOLA as a superproject)
    FILE ${CMAKE_BINARY_DIR}/robin_map-targets.cmake
    NAMESPACE tsl::
)

# And generate the -config.cmake file:
set(ALL_DEPS_LIST "") # used in xxx-config.cmake.in
set(MOLA_MODULE_NAME robin_map)
configure_file(
"${_MOLACOMMON_MODULE_BASE_DIR}/mola-xxx-config.cmake.in"
"${CMAKE_BINARY_DIR}/robin_map-config.cmake" IMMEDIATE @ONLY
)
# Install cmake config module
install(
	EXPORT
		robin_map-targets
	DESTINATION
		${CMAKE_INSTALL_LIBDIR}/robin_map/cmake
)
install(
	FILES
		${CMAKE_BINARY_DIR}/robin_map-config.cmake
		#${CMAKE_BINARY_DIR}/robin_map-config-version.cmake
	DESTINATION
		${CMAKE_INSTALL_LIBDIR}/robin_map/cmake
)
