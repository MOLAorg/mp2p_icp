/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   register.cpp
 * @brief  RTTI registry
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#include <mp2p_icp_filters/FilterBoundingBox.h>
#include <mp2p_icp_filters/FilterDecimateVoxels.h>
#include <mp2p_icp_filters/FilterDeleteLayer.h>
#include <mp2p_icp_filters/FilterEdgesPlanes.h>
#include <mp2p_icp_filters/Generator.h>
#include <mrpt/core/initializer.h>

MRPT_INITIALIZER(register_mola_lidar_segmentation)
{
    using mrpt::rtti::registerClass;

    // Generators:
    registerClass(CLASS_ID(mp2p_icp_filters::Generator));

    // Filters:
    registerClass(CLASS_ID(mp2p_icp_filters::FilterBase));
    registerClass(CLASS_ID(mp2p_icp_filters::FilterDecimateVoxels));
    registerClass(CLASS_ID(mp2p_icp_filters::FilterDeleteLayer));
    registerClass(CLASS_ID(mp2p_icp_filters::FilterEdgesPlanes));
    registerClass(CLASS_ID(mp2p_icp_filters::FilterBoundingBox));
}
