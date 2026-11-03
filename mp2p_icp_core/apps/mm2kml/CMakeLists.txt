project(mm2kml)

find_package(CLI11 REQUIRED)
find_package(ZLIB REQUIRED)

mola_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    main.cpp
    kmz_writer.cpp
    kmz_writer.h
  LINK_LIBRARIES
    mp2p_icp_map
    CLI11::CLI11
    ZLIB::ZLIB
)
