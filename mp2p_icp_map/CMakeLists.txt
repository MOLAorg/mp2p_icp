project(mp2p_icp_map LANGUAGES CXX)

# -----------------------
# define lib:
set(LIB_SRCS
  src/estimate_points_eigen.cpp
  src/IcpPrepareCapable.cpp
  src/load_xyz_file.cpp
  src/metricmap.cpp
  src/MetricMapMergeCapable.cpp
  src/NearestPlaneCapable.cpp
  src/NearestPointWithCovCapable.cpp
  src/point_with_cov_pair_t.cpp
  src/pointcloud_sanity_check.cpp
  #
  src/register.cpp # This must be last
)

set(LIB_PUBLIC_HDRS
  include/mp2p_icp/estimate_points_eigen.h
  include/mp2p_icp/IcpPrepareCapable.h
  include/mp2p_icp/layer_name_t.h
  include/mp2p_icp/load_xyz_file.h
  include/mp2p_icp/metricmap.h
  include/mp2p_icp/MetricMapMergeCapable.h
  include/mp2p_icp/NearestPlaneCapable.h
  include/mp2p_icp/NearestPointWithCovCapable.h
  include/mp2p_icp/plane_patch.h
  include/mp2p_icp/point_plane_pair_t.h
  include/mp2p_icp/point_with_cov_pair_t.h
  include/mp2p_icp/pointcloud_bitfield.h
  include/mp2p_icp/pointcloud_sanity_check.h
  include/mp2p_icp/render_params.h
)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::maps
    mrpt::opengl
    mrpt::topography
  CMAKE_DEPENDENCIES
    mrpt-maps
    mrpt-opengl
    mrpt-topography
)
