project(mp2p_icp_filters LANGUAGES CXX)

# -----------------------
# define lib:
set(LIB_SRCS
	src/FilterAdjustTimestamps.cpp
	src/FilterDeleteLayer.cpp
	src/FilterCurvature.cpp
	src/FilterByIntensity.cpp
	src/FilterDecimateAdaptive.cpp
	src/FilterByRange.cpp
	src/GeneratorEdgesFromCurvature.cpp
	src/FilterVoxelSlice.cpp
	src/PointCloudToVoxelGridSingle.cpp
	src/Generator.cpp
	src/FilterDeskew.cpp
	src/GetOrCreatePointLayer.cpp
	src/PointCloudToVoxelGrid.cpp
	src/GeneratorEdgesFromRangeImage.cpp
	src/FilterRemoveByVoxelOccupancy.cpp
	src/sm2mm.cpp
	src/FilterEdgesPlanes.cpp
	src/FilterDecimateVoxels.cpp
	src/FilterNormalizeIntensity.cpp
	src/FilterBase.cpp
	src/FilterDecimateVoxelsQuadratic.cpp
	src/FilterByRing.cpp
	src/FilterMerge.cpp
	src/FilterBoundingBox.cpp
	#
	src/register.cpp # This must be last
)

set(LIB_PUBLIC_HDRS
	include/mp2p_icp_filters/FilterDecimateVoxels.h
	include/mp2p_icp_filters/GeneratorEdgesFromCurvature.h
	include/mp2p_icp_filters/FilterMerge.h
	include/mp2p_icp_filters/Generator.h
	include/mp2p_icp_filters/FilterEdgesPlanes.h
	include/mp2p_icp_filters/FilterDeskew.h
	include/mp2p_icp_filters/GetOrCreatePointLayer.h
	include/mp2p_icp_filters/FilterVoxelSlice.h
	include/mp2p_icp_filters/FilterByRing.h
	include/mp2p_icp_filters/FilterDecimateVoxelsQuadratic.h
	include/mp2p_icp_filters/FilterDeleteLayer.h
	include/mp2p_icp_filters/FilterAdjustTimestamps.h
	include/mp2p_icp_filters/FilterByIntensity.h
	include/mp2p_icp_filters/FilterRemoveByVoxelOccupancy.h
	include/mp2p_icp_filters/FilterCurvature.h
	include/mp2p_icp_filters/PointCloudToVoxelGrid.h
	include/mp2p_icp_filters/FilterByRange.h
	include/mp2p_icp_filters/sm2mm.h
	include/mp2p_icp_filters/GeneratorEdgesFromRangeImage.h
	include/mp2p_icp_filters/FilterNormalizeIntensity.h
	include/mp2p_icp_filters/FilterBase.h
	include/mp2p_icp_filters/PointCloudToVoxelGridSingle.h
	include/mp2p_icp_filters/FilterBoundingBox.h
	include/mp2p_icp_filters/FilterDecimateAdaptive.h
)

mola_add_library(
	TARGET ${PROJECT_NAME}
	SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
	PUBLIC_LINK_LIBRARIES
		mp2p_icp_map
	PRIVATE_LINK_LIBRARIES
		tsl::robin_map
	CMAKE_DEPENDENCIES
		mp2p_icp
		#robin_map
)

if (TBB_FOUND AND MP2PICP_USE_TBB)
	target_compile_definitions(${PROJECT_NAME} PRIVATE MP2P_HAS_TBB)
	target_link_libraries(${PROJECT_NAME} PRIVATE TBB::tbb)
endif()
