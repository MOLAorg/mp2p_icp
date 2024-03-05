project(mp2p_icp LANGUAGES CXX)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES
		mp2p_icp_map
		mrpt::containers
		mrpt::tfest
		mrpt::maps
		mrpt::gui
	CMAKE_DEPENDENCIES
		mp2p_icp_map
		mrpt-containers
		mrpt-maps
		mrpt-tfest
		mrpt-gui
)

if (MP2PICP_BUILD_POINTMATCHER)
	target_compile_definitions(${PROJECT_NAME} PRIVATE MP2P_HAS_LIBPOINTMATCHER)
	target_link_libraries(${PROJECT_NAME} PRIVATE libpointmatcher::libpointmatcher)
endif()

if (TBB_FOUND AND MP2PICP_USE_TBB)
	target_compile_definitions(${PROJECT_NAME} PRIVATE MP2P_HAS_TBB)
	target_link_libraries(${PROJECT_NAME} PRIVATE TBB::tbb)
endif()
