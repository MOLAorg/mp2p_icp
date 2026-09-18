project(mm-viewer)

find_package(CLI11 REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(mrpt_imgui REQUIRED)  # CImGuiSceneView

mola_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    main.cpp
  LINK_LIBRARIES
    mola::mp2p_icp
    CLI11::CLI11
    mrpt::mrpt_gui
    mrpt::mrpt_imgui
    Threads::Threads
    imgui_app_common
  )
