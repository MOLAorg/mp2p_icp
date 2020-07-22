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
  mola_add_executable(
	  TARGET  test-${NAME}
	  SOURCES test-${NAME}.cpp ${ARGN}
	  LINK_LIBRARIES
		  mp2p_icp
  )
  add_test(mp2p_icp_${NAME} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test-${NAME})
  target_compile_definitions(test-${NAME}
	  PRIVATE
	  MP2P_DATASET_DIR="${mp2p_icp_SOURCE_DIR}/test-datasets/")
endfunction()

mp2p_add_test(mp2p_optimal_tf_algos)
mp2p_add_test(mp2p_icp_algos test-common.cpp)
mp2p_add_test(mp2p_jacobians)
mp2p_add_test(mp2p_quality_reproject_ranges test-common.cpp)
mp2p_add_test(mp2p_matcher_pt2pt)
mp2p_add_test(mp2p_matcher_pt2pl)

