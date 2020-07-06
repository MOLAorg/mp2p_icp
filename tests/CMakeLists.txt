# ------------------------------------------------------------------------------
#        Multi primitive-to-primitive (MP2) ICP C++ library
#
# Copyright (C) 2018-2020, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under BSD 3-Clause License. See COPYING.
# ------------------------------------------------------------------------------

mola_add_executable(
    TARGET  test-mp2p_optimal_tf_algos
    SOURCES test-mp2p_optimal_tf_algos.cpp
	LINK_LIBRARIES
	    mp2p_icp
)
add_test(MP2P_optimal_tf_algorithms ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test-mp2p_optimal_tf_algos)


mola_add_executable(
    TARGET  test-mp2p_icp_algos
    SOURCES test-mp2p_icp_algos.cpp
    LINK_LIBRARIES
        mp2p_icp
)
add_test(MP2P_ICP_algorithms ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test-mp2p_icp_algos)

target_compile_definitions(test-mp2p_icp_algos
    PRIVATE
    MP2P_DATASET_DIR="${mp2p_icp_SOURCE_DIR}/test-datasets/")


mola_add_executable(
    TARGET  test-mp2p_jacobians
    SOURCES test-mp2p_jacobians.cpp
	LINK_LIBRARIES
	    mp2p_icp
)
add_test(MP2P_jacobians ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test-mp2p_jacobians)


mola_add_executable(
    TARGET  test-mp2p_quality_reproject_ranges
    SOURCES test-mp2p_quality_reproject_ranges.cpp
	LINK_LIBRARIES
	    mp2p_icp
)
add_test(MP2P_QualityReproject ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test-mp2p_quality_reproject_ranges)
target_compile_definitions(test-mp2p_quality_reproject_ranges
    PRIVATE
    MP2P_DATASET_DIR="${mp2p_icp_SOURCE_DIR}/test-datasets/")
