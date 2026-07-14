
# ================================
# robin-map
# ================================
# This defines the target "tsl::robin_map"
# (No rosdep key exists for this header-only library, so it stays vendored
# as a git submodule; it is a private-only dependency of mp2p_icp_filters.)
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/robin-map/CMakeLists.txt")
  message(FATAL_ERROR
    "The git submodule 'robin-map' is missing.\n"
    "It looks like you cloned this repository without its submodules.\n"
    "Please run:\n"
    "  git submodule update --init --recursive\n"
    "or clone again with:\n"
    "  git clone --recursive <repo-url>\n")
endif()
add_subdirectory(robin-map)

# do NOT install the robin-map library: we will use it as PIMP only
# to prevent problems from exposing it in the public API:

