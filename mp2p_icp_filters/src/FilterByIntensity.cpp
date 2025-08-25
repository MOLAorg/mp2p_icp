/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterByIntensity.cpp
 * @brief  Thresholds an input cloud by intensity values.
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2024
 */

#include <mp2p_icp_filters/FilterByIntensity.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>

IMPLEMENTS_MRPT_OBJECT(FilterByIntensity, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterByIntensity::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);

    MCP_LOAD_OPT(c, output_layer_low_intensity);
    MCP_LOAD_OPT(c, output_layer_high_intensity);
    MCP_LOAD_OPT(c, output_layer_mid_intensity);

    ASSERTMSG_(
        !output_layer_low_intensity.empty() || !output_layer_low_intensity.empty() ||
            !output_layer_mid_intensity.empty(),
        "At least one of 'output_layer_low_intensity' or "
        "'output_layer_low_intensity' or 'output_layer_mid_intensity' must be "
        "provided.");

    MCP_LOAD_REQ(c, low_threshold);
    MCP_LOAD_REQ(c, high_threshold);
}

FilterByIntensity::FilterByIntensity() = default;

void FilterByIntensity::initialize(const mrpt::containers::yaml& c)
{
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c);
}

void FilterByIntensity::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    // In:
    const auto& pcPtr = inOut.point_layer(params_.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr,
        mrpt::format(
            "Input point cloud layer '%s' was not found.", params_.input_pointcloud_layer.c_str()));

    const auto& pc = *pcPtr;

    // Outputs:
    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outLow = GetOrCreatePointLayer(
        inOut, params_.output_layer_low_intensity, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    if (outLow) outLow->reserve(outLow->size() + pc.size() / 10);

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outHigh = GetOrCreatePointLayer(
        inOut, params_.output_layer_high_intensity, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    if (outHigh) outHigh->reserve(outHigh->size() + pc.size() / 10);

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outMid = GetOrCreatePointLayer(
        inOut, params_.output_layer_mid_intensity, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    if (outMid) outMid->reserve(outMid->size() + pc.size() / 10);

    ASSERTMSG_(
        outLow || outHigh || outMid,
        "At least one of 'output_layer_low_intensity' or "
        "'output_layer_low_intensity' or 'output_layer_mid_intensity' must be "
        "provided.");

    const auto& xs = pc.getPointsBufferRef_x();
    // const auto& ys   = pc.getPointsBufferRef_y();
    // const auto& zs   = pc.getPointsBufferRef_z();
    const auto* ptrI = pc.getPointsBufferRef_intensity();
    if (!ptrI || ptrI->empty())
    {
        THROW_EXCEPTION_FMT(
            "Error: this filter needs the input layer '%s' to has an "
            "'intensity' "
            "point channel.",
            params_.input_pointcloud_layer.c_str());
    }

    const auto& Is = *ptrI;
    ASSERT_EQUAL_(Is.size(), xs.size());
    const size_t N = xs.size();

    size_t countLow = 0, countMid = 0, countHigh = 0;

    for (size_t i = 0; i < N; i++)
    {
        const float I = Is[i];

        mrpt::maps::CPointsMap* trg = nullptr;

        if (I < params_.low_threshold)
        {
            trg = outLow.get();
            ++countLow;
        }
        else if (I > params_.high_threshold)
        {
            trg = outHigh.get();
            ++countHigh;
        }
        else
        {
            trg = outMid.get();
            ++countMid;
        }

        if (trg) trg->insertPointFrom(pc, i);
    }

    MRPT_LOG_DEBUG_STREAM(
        "[FilterByIntensity] Input points=" << N << " low=" << countLow << " mid=" << countMid
                                            << " high=" << countHigh);

    MRPT_END
}
