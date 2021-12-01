# ------------------------------------------------------------------------------
#        Multi primitive-to-primitive (MP2P) ICP C++ library
#
# Copyright (C) 2018-2020, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under BSD 3-Clause License. See COPYING.
# ------------------------------------------------------------------------------

include_directories(".") # for "test-common.h"

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
    MP2P_DATASET_DIR="${mp2p_icp_all_SOURCE_DIR}/demos/")
endfunction()

mp2p_add_test(mp2p_optimal_tf_algos)
mp2p_add_test(mp2p_icp_algos)
mp2p_add_test(mp2p_error_terms_jacobians)
mp2p_add_test(mp2p_quality_reproject_ranges)
mp2p_add_test(mp2p_matcher_pt2pt)
mp2p_add_test(mp2p_matcher_pt2pl)
mp2p_add_test(mp2p_optimize_pt2ln)
