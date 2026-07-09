project(icp-run)

find_package(CLI11 REQUIRED)

mola_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    main.cpp
  LINK_LIBRARIES
    mp2p_icp
    mp2p_icp_filters
    CLI11::CLI11
  )

