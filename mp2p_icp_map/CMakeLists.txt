project(mp2p_icp_map LANGUAGES CXX)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

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
