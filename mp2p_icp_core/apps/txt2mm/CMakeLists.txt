project(txt2mm)

find_package(mrpt_maps REQUIRED)
find_package(CLI11 REQUIRED)

mola_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    main.cpp
  LINK_LIBRARIES
    mp2p_icp_map
    mrpt::mrpt_maps
    CLI11::CLI11
)

