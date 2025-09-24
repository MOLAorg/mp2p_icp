project(mp2p_icp_common LANGUAGES CXX)

# -----------------------
# define lib:
set(LIB_SRCS
  src/load_plugin.cpp
  src/Parameterizable.cpp
  #
  src/register.cpp # This must be last
)

set(LIB_PUBLIC_HDRS
  include/mp2p_icp/load_plugin.h
  include/mp2p_icp/Parameterizable.h
)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::system
    mola::mola_imu_preintegration
  CMAKE_DEPENDENCIES
    mrpt-system
    mola_imu_preintegration
)
