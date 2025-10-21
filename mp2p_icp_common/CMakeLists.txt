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

if (mola_imu_preintegration_FOUND)
  set(imu_pub_link_ mola::mola_imu_preintegration)
  set(imu_cmake_dep_ mola_imu_preintegration)
endif()

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::expr
    mrpt::system
    ${imu_pub_link_}
  CMAKE_DEPENDENCIES
    mrpt-system
    mrpt-expr
    ${imu_cmake_dep_}
)

if (mola_imu_preintegration_FOUND)
  target_compile_definitions(${PROJECT_NAME} PUBLIC MP2P_ICP_HAS_MOLA_IMU_PREINTEGRATION)
endif()