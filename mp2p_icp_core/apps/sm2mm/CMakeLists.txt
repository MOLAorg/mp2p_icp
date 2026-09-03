project(sm2mm)

find_package(mrpt_maps REQUIRED)
find_package(mrpt_obs REQUIRED)
find_package(CLI11 REQUIRED)

mola_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    main.cpp
  LINK_LIBRARIES
    mp2p_icp_filters
    mrpt::mrpt_maps
    mrpt::mrpt_obs
    CLI11::CLI11
)

