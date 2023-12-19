/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterNormalizeIntensity.cpp
 * @brief  Normalizes the intensity channel of a point cloud layer
 * @author Jose Luis Blanco Claraco
 * @date   Dec 19, 2023
 */

#include <mp2p_icp_filters/FilterNormalizeIntensity.h>
#include <mrpt/containers/yaml.h>

IMPLEMENTS_MRPT_OBJECT(
    FilterNormalizeIntensity, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterNormalizeIntensity::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, pointcloud_layer);
}

FilterNormalizeIntensity::FilterNormalizeIntensity()
{
    mrpt::system::COutputLogger::setLoggerName("FilterNormalizeIntensity");
}

void FilterNormalizeIntensity::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c);

    MRPT_END
}

void FilterNormalizeIntensity::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    // In/out:
    auto pcPtr = inOut.point_layer(params_.pointcloud_layer);
    ASSERTMSG_(
        pcPtr, mrpt::format(
                   "Input point cloud layer '%s' was not found.",
                   params_.pointcloud_layer.c_str()));

    auto& pc = *pcPtr;

    auto* IsPtr = pc.getPointsBufferRef_intensity();

    ASSERTMSG_(
        IsPtr != nullptr && !IsPtr->empty(),
        mrpt::format(
            "Input point cloud layer '%s' (%s) seems not to have an intensity "
            "channel or it is empty.",
            params_.pointcloud_layer.c_str(), pc.GetRuntimeClass()->className));

    auto& Is = *IsPtr;

    std::optional<float> minI, maxI;

    for (size_t i = 0; i < Is.size(); i++)
    {
        const float I = Is[i];

        if (!minI || I < *minI) minI = I;
        if (!maxI || I > *maxI) maxI = I;
    }
    ASSERT_(minI && maxI);

    float delta = *maxI - *minI;
    if (delta == 0) delta = 1;
    const float delta_inv = 1.0f / delta;

    for (size_t i = 0; i < Is.size(); i++)
    {
        float& I = Is[i];

        I = (I - *minI) * delta_inv;
    }

    MRPT_END
}
