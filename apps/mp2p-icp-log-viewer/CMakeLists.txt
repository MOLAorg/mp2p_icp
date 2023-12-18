project(mp2p-icp-log-viewer)

find_package(mrpt-tclap REQUIRED)
find_package(mrpt-gui REQUIRED)

mola_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    main.cpp
  LINK_LIBRARIES
    mp2p_icp
    mrpt::tclap
    mrpt::gui
    Threads::Threads
  )
