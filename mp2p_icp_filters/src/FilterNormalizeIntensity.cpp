/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   FilterNormalizeIntensity.cpp
 * @brief  Normalizes the intensity channel of a point cloud layer
 * @author Jose Luis Blanco Claraco
 * @date   Dec 19, 2023
 */

#include <mp2p_icp_filters/FilterNormalizeIntensity.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/lock_helper.h>

IMPLEMENTS_MRPT_OBJECT(FilterNormalizeIntensity, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterNormalizeIntensity::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, pointcloud_layer);
    MCP_LOAD_OPT(c, remember_intensity_range);
    MCP_LOAD_OPT(c, fixed_minimum_intensity);
    MCP_LOAD_OPT(c, fixed_maximum_intensity);
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
        pcPtr,
        mrpt::format(
            "Input point cloud layer '%s' was not found.", params_.pointcloud_layer.c_str()));

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

    if (params_.fixed_maximum_intensity > 0)
    {
        maxI = params_.fixed_maximum_intensity;
        minI = params_.fixed_minimum_intensity;
    }
    else
    {
        for (size_t i = 0; i < Is.size(); i++)
        {
            const float I = Is[i];

            if (!minI || I < *minI)
            {
                minI = I;
            }
            if (!maxI || I > *maxI)
            {
                maxI = I;
            }
        }
        ASSERT_(minI && maxI);

        // Merge with range memory?
        if (params_.remember_intensity_range)
        {
            auto lck = mrpt::lockHelper(minMaxMtx_);
            if (!minI_ || *minI < *minI_)
            {
                minI_ = minI;
            }
            if (!maxI_ || *maxI > *maxI_)
            {
                maxI_ = maxI;
            }
            minI = minI_;
            maxI = maxI_;
        }
    }

    float delta = *maxI - *minI;
    if (delta == 0)
    {
        delta = 1;
    }
    const float delta_inv = 1.0f / delta;

    for (size_t i = 0; i < Is.size(); i++)
    {
        float& I = Is[i];

        I = std::clamp((I - *minI) * delta_inv, 0.0f, 1.0f);
    }

    MRPT_LOG_DEBUG_STREAM("Normalized with minI=" << *minI << " maxI=" << *maxI);

    MRPT_END
}
