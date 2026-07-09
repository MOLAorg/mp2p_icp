
# ================================
# robin-map
# ================================
# This defines the target "tsl::robin_map"
# (No rosdep key exists for this header-only library, so it stays vendored
# as a git submodule; it is a private-only dependency of mp2p_icp_filters.)
add_subdirectory(robin-map)

# do NOT install the robin-map library: we will use it as PIMP only
# to prevent problems from exposing it in the public API:

