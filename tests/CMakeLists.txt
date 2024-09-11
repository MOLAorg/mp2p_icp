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
# ---------------------------------------------------------------------------
function(mp2p_add_test NAME)
  mola_add_test(
    TARGET  test-${NAME}
    SOURCES test-${NAME}.cpp ${ARGN}
    LINK_LIBRARIES
    mp2p_icp
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

mp2p_add_test(mp2p_error_terms_jacobians)
mp2p_add_test(mp2p_icp_algos)
#mp2p_add_test(mp2p_matcher_pt2pl)  # TODO: This now requires a NP metric map to run the test
mp2p_add_test(mp2p_matcher_pt2pt_parameterizable)
mp2p_add_test(mp2p_matcher_pt2pt)
mp2p_add_test(mp2p_optimal_tf_algos)
mp2p_add_test(mp2p_optimize_pt2ln)
mp2p_add_test(mp2p_optimize_pt2pl)
mp2p_add_test(mp2p_optimize_with_prior)
mp2p_add_test(mp2p_quality_reproject_ranges)

if (mola_test_datasets_FOUND)
  mp2p_add_test(mp2p_quality_voxels)
endif()
