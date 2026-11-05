project(mm-render)

find_package(CLI11 REQUIRED)
find_package(mrpt_opengl REQUIRED)  # CFBORender

mola_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    main.cpp
  LINK_LIBRARIES
    mola::mp2p_icp_map
    CLI11::CLI11
    mrpt::mrpt_opengl
  )
