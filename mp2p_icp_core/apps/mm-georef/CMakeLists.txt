project(mm-georef)

find_package(CLI11 REQUIRED)

mola_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    main.cpp
  LINK_LIBRARIES
    mp2p_icp_map
    mp2p_icp_common
    CLI11::CLI11
)

