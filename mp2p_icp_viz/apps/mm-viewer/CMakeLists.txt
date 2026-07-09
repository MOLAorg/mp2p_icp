project(mm-viewer)

find_package(CLI11 REQUIRED)
find_package(mrpt-gui REQUIRED)

mola_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    main.cpp
  LINK_LIBRARIES
    mola::mp2p_icp
    CLI11::CLI11
    mrpt::gui
    Threads::Threads
  )
