project(txt2mm)

find_package(mrpt-maps REQUIRED)
find_package(mrpt-tclap REQUIRED)

mola_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    main.cpp
  LINK_LIBRARIES
    mp2p_icp_map
    mrpt::maps
    mrpt::tclap
)

