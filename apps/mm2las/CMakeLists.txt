project(mm2las)

find_package(mrpt-tclap REQUIRED)
find_package(mrpt-topography REQUIRED)

mola_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    main.cpp
  LINK_LIBRARIES
    mp2p_icp_map
    mrpt::tclap
    mrpt::topography
)

