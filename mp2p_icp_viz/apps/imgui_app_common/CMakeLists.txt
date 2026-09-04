# Shared GLFW + Dear ImGui (docking) application shell used by both
# mm-viewer and icp-log-viewer. Internal build detail only: not installed
# or exported.
project(imgui_app_common LANGUAGES CXX)

add_library(${PROJECT_NAME} STATIC
  src/ImGuiAppShell.cpp
  src/SimpleFileDialog.cpp
  include/imgui_app_common/AsyncTask.h
  include/imgui_app_common/ImGuiAppShell.h
  include/imgui_app_common/SimpleFileDialog.h
)

target_include_directories(${PROJECT_NAME} PUBLIC include)

find_package(Threads REQUIRED)

target_link_libraries(${PROJECT_NAME} PUBLIC imgui::imgui Threads::Threads)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
