
# ================================
# robin-map
# ================================
# This defines the target "tsl::robin_map"
add_subdirectory(robin-map)

# do NOT install the robin-map library: we will use it as PIMP only 
# to prevent problems from exposing it in the public API:

