project(mm-viewer)

find_package(CLI11 REQUIRED)
find_package(mrpt_gui REQUIRED)
find_package(mrpt_imgui REQUIRED)  # CImGuiSceneView
find_package(mrpt_opengl REQUIRED)  # CFBORender

mola_add_executable(
  TARGET ${PROJECT_NAME}
  SOURCES
    main.cpp
    CameraTravelling.cpp
  LINK_LIBRARIES
    mola::mp2p_icp
    CLI11::CLI11
    mrpt::mrpt_gui
    mrpt::mrpt_imgui
    mrpt::mrpt_opengl
    Threads::Threads
    imgui_app_common
  )
