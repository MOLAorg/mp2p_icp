project(mp2p_icp_lib LANGUAGES CXX)

set(lib_name mp2p_icp) # we cannot use PROJECT_NAME here, since catkin requires that name for the top-level project()

# -----------------------
# define lib:
set(LIB_SRCS
	src/errorTerms.cpp
	src/Results.cpp
	src/Solver_GaussNewton.cpp
	src/Parameters.cpp
	src/Matcher_Point2Plane.cpp
	src/optimal_tf_olae.cpp
	src/LogRecord.cpp
	src/Matcher_Points_DistanceThreshold.cpp
	src/Solver.cpp
	src/pt2ln_pl_to_pt2pt.cpp
	src/WeightParameters.cpp
	src/Matcher_Point2Line.cpp
	src/QualityEvaluator_PairedRatio.cpp
	src/QualityEvaluator_Voxels.cpp
	src/QualityEvaluator.cpp
	src/Solver_Horn.cpp
	src/QualityEvaluator_RangeImageSimilarity.cpp
	src/ICP_LibPointmatcher.cpp
	src/Solver_OLAE.cpp
	src/covariance.cpp
	src/optimal_tf_gauss_newton.cpp
	src/icp_pipeline_from_yaml.cpp
	src/Matcher_Adaptive.cpp
	src/Matcher_Planes_Normals.cpp
	src/ICP.cpp
	src/optimal_tf_horn.cpp
	src/Pairings.cpp
	src/PairWeights.cpp
	src/Matcher_Points_InlierRatio.cpp
	src/Matcher_Points_Base.cpp
	src/Matcher.cpp
	src/visit_correspondences.h
	#
	src/register.cpp # This must be last
)
set(LIB_PUBLIC_HDRS
	include/mp2p_icp/Pairings.h
	include/mp2p_icp/Matcher.h
	include/mp2p_icp/Matcher_Points_DistanceThreshold.h
	include/mp2p_icp/optimal_tf_horn.h
	include/mp2p_icp/QualityEvaluator.h
	include/mp2p_icp/QualityEvaluator_Voxels.h
	include/mp2p_icp/Solver_Horn.h
	include/mp2p_icp/Matcher_Point2Line.h
	include/mp2p_icp/optimal_tf_gauss_newton.h
	include/mp2p_icp/Matcher_Points_InlierRatio.h
	include/mp2p_icp/QualityEvaluator_RangeImageSimilarity.h
	include/mp2p_icp/Solver_GaussNewton.h
	include/mp2p_icp/Solver_OLAE.h
	include/mp2p_icp/optimal_tf_olae.h
	include/mp2p_icp/IterTermReason.h
	include/mp2p_icp/Matcher_Points_Base.h
	include/mp2p_icp/pt2ln_pl_to_pt2pt.h
	include/mp2p_icp/LogRecord.h
	include/mp2p_icp/Matcher_Adaptive.h
	include/mp2p_icp/Matcher_Point2Plane.h
	include/mp2p_icp/icp_pipeline_from_yaml.h
	include/mp2p_icp/Parameters.h
	include/mp2p_icp/ICP.h
	include/mp2p_icp/PairWeights.h
	include/mp2p_icp/OptimalTF_Result.h
	include/mp2p_icp/QualityEvaluator_PairedRatio.h
	include/mp2p_icp/WeightParameters.h
	include/mp2p_icp/covariance.h
	include/mp2p_icp/errorTerms.h
	include/mp2p_icp/ICP_LibPointmatcher.h
	include/mp2p_icp/Solver.h
	include/mp2p_icp/robust_kernels.h
	include/mp2p_icp/Results.h
)

mola_add_library(
	TARGET ${lib_name}
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
	target_compile_definitions(${lib_name} PRIVATE MP2P_HAS_LIBPOINTMATCHER)
	target_link_libraries(${lib_name} PRIVATE libpointmatcher::libpointmatcher)
endif()

if (TBB_FOUND AND MP2PICP_USE_TBB)
	target_compile_definitions(${lib_name} PRIVATE MP2P_HAS_TBB)
	target_link_libraries(${lib_name} PRIVATE TBB::tbb)
endif()


