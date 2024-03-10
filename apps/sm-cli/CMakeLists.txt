project(sm-cli)

find_package(mrpt-maps REQUIRED)
find_package(mrpt-gui REQUIRED)
find_package(mrpt-obs REQUIRED)
find_package(mrpt-tclap REQUIRED)

mola_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    sm-cli-main.cpp
    sm-cli.h
    sm-cli-cut.cpp
    sm-cli-export-kfs.cpp
    sm-cli-export-rawlog.cpp
    sm-cli-info.cpp
    sm-cli-level.cpp
  LINK_LIBRARIES
    mp2p_icp_filters
    mrpt::maps
    mrpt::gui
    mrpt::obs
    mrpt::tclap
)

target_compile_definitions(${PROJECT_NAME} PRIVATE MP2P_ICP_VERSION=\"${MP2P_ICP_VERSION}\")
