# ------------------------------------------------------------------------------
#        Multi primitive-to-primitive (MP2P) ICP C++ library
#
# Copyright (C) 2018-2020, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under BSD 3-Clause License. See COPYING.
# ------------------------------------------------------------------------------

include_directories(".") # for "test-common.h"

find_package(mola_test_datasets QUIET)

# ---------------------------------------------------------------------------
# Usage: mp2p_add_test(foo) to define a test named "test_mp2p_icp_foo"
#   from "test-foo.cpp". Additional extra .cpp files can be added as argv
#   Example with EXTRA_LIBS:
#     mp2p_add_test(mp2p_example_test EXTRA_LIBS some_extra_lib)
# ---------------------------------------------------------------------------
function(mp2p_add_test NAME)
  set(options)
  set(oneValueArgs EXTRA_LIBS)
  set(multiValueArgs)
  cmake_parse_arguments(MP2P_ADD_TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})


  mola_add_test(
    TARGET  test-${NAME}
    SOURCES test-${NAME}.cpp ${MP2P_ADD_TEST_UNPARSED_ARGUMENTS}
    LINK_LIBRARIES
    mp2p_icp
    ${MP2P_ADD_TEST_EXTRA_LIBS}
  )
  target_compile_definitions(test-${NAME}
    PRIVATE
    MP2P_DATASET_DIR="${mp2p_icp_SOURCE_DIR}/demos/")
  if (mola_test_datasets_FOUND)
      target_compile_definitions(test-${NAME}
          PRIVATE
          MOLA_TEST_DATASET_DIR="${mola_test_datasets_DIR}/../datasets/")
      #message(STATUS "mola_test_datasets_DIR: ${mola_test_datasets_DIR}")
  endif()
endfunction()


if (mola_imu_preintegration_FOUND)
mp2p_add_test(mp2p_deskew EXTRA_LIBS mp2p_icp_filters)
endif()

#mp2p_add_test(mp2p_matcher_pt2pl)  # TODO: This now requires a NP metric map to run the test
mp2p_add_test(mp2p_cov2cov)
mp2p_add_test(mp2p_error_terms_jacobians)
mp2p_add_test(mp2p_estimate_points_eigen)
mp2p_add_test(mp2p_georef_yaml)
mp2p_add_test(mp2p_icp_algos)
mp2p_add_test(mp2p_LogRecord)
mp2p_add_test(mp2p_map_serialization)
mp2p_add_test(mp2p_matcher_pt2ln)
mp2p_add_test(mp2p_matcher_pt2pt_parameterizable)
mp2p_add_test(mp2p_matcher_pt2pt)
mp2p_add_test(mp2p_optimal_tf_algos)
mp2p_add_test(mp2p_optimize_cov2cov)
mp2p_add_test(mp2p_optimize_pt2ln)
mp2p_add_test(mp2p_optimize_pt2pl)
mp2p_add_test(mp2p_optimize_with_prior)
mp2p_add_test(mp2p_Pairings)
mp2p_add_test(mp2p_PairWeights)
mp2p_add_test(mp2p_quality_reproject_ranges)
mp2p_add_test(mp2p_WeightParameters)

mp2p_add_test(mp2p_class_factory EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterAbsoluteTimestamp EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterByExpression EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterByIntensity EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterByRange EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterByRing EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterClear EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterDecimate EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterDecimateAdaptive EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterDecimateVoxels EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterEdgesPlanes EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterFartherPointSampling EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterPoleDetector EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterRemovePointCloudField EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterRenameLayer EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_Filters EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterSOR EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_FilterVoxelSOR EXTRA_LIBS mp2p_icp_filters)
mp2p_add_test(mp2p_generator EXTRA_LIBS mp2p_icp_filters)


if (mola_test_datasets_FOUND)
  mp2p_add_test(mp2p_quality_voxels)
endif()
