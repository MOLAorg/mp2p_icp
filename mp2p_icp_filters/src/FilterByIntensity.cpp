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
 * @file   FilterByIntensity.cpp
 * @brief  Thresholds an input cloud by intensity values.
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2024
 */

#include <mp2p_icp_filters/FilterByIntensity.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/version.h>

IMPLEMENTS_MRPT_OBJECT(FilterByIntensity, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterByIntensity::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);

    MCP_LOAD_OPT(c, output_layer_low_intensity);
    MCP_LOAD_OPT(c, output_layer_high_intensity);
    MCP_LOAD_OPT(c, output_layer_mid_intensity);

    ASSERTMSG_(
        !output_layer_low_intensity.empty() || !output_layer_high_intensity.empty() ||
            !output_layer_mid_intensity.empty(),
        "At least one of 'output_layer_low_intensity' or "
        "'output_layer_high_intensity' or 'output_layer_mid_intensity' must be "
        "provided.");

    MCP_LOAD_REQ(c, low_threshold);
    MCP_LOAD_REQ(c, high_threshold);
}

FilterByIntensity::FilterByIntensity() = default;

void FilterByIntensity::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c);
}

void FilterByIntensity::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    // In:
    const auto& pcPtr = inOut.point_layer(params.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr,
        mrpt::format(
            "Input point cloud layer '%s' was not found.", params.input_pointcloud_layer.c_str()));

    const auto& pc = *pcPtr;

    // Outputs:
    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outLow = GetOrCreatePointLayer(
        inOut, params.output_layer_low_intensity, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    if (outLow)
    {
        outLow->reserve(outLow->size() + pc.size() / 10);
    }

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outHigh = GetOrCreatePointLayer(
        inOut, params.output_layer_high_intensity, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    if (outHigh)
    {
        outHigh->reserve(outHigh->size() + pc.size() / 10);
    }

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outMid = GetOrCreatePointLayer(
        inOut, params.output_layer_mid_intensity, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    if (outMid)
    {
        outMid->reserve(outMid->size() + pc.size() / 10);
    }

    ASSERTMSG_(
        outLow || outHigh || outMid,
        "At least one of 'output_layer_low_intensity' or "
        "'output_layer_low_intensity' or 'output_layer_mid_intensity' must be "
        "provided.");

    const auto& xs = pc.getPointsBufferRef_x();

#if MRPT_VERSION >= 0x020f00  // 2.15.0
    const auto* ptrI = pc.getPointsBufferRef_float_field("intensity");
#else
    const auto* ptrI = pc.getPointsBufferRef_intensity();
#endif
    if (!ptrI || ptrI->empty())
    {
        THROW_EXCEPTION_FMT(
            "Error: this filter needs the input layer '%s' to has an 'intensity' point channel.",
            params.input_pointcloud_layer.c_str());
    }

    const auto& Is = *ptrI;
    ASSERT_EQUAL_(Is.size(), xs.size());
    const size_t N = xs.size();

#if MRPT_VERSION >= 0x020f00  // 2.15.0
    mrpt::maps::CPointsMap::InsertCtx ctxLow, ctxHigh, ctxMid;
    if (outLow)
    {
        ctxLow = outLow->prepareForInsertPointsFrom(pc);
    }
    if (outHigh)
    {
        ctxHigh = outHigh->prepareForInsertPointsFrom(pc);
    }
    if (outMid)
    {
        ctxMid = outMid->prepareForInsertPointsFrom(pc);
    }
#endif

    size_t countLow = 0, countMid = 0, countHigh = 0;

    for (size_t i = 0; i < N; i++)
    {
        const float I = Is[i];

        mrpt::maps::CPointsMap* trg = nullptr;

#if MRPT_VERSION >= 0x020f00  // 2.15.0
        mrpt::maps::CPointsMap::InsertCtx* ctx = nullptr;
#endif
        if (I < params.low_threshold)
        {
            trg = outLow.get();
            ++countLow;
#if MRPT_VERSION >= 0x020f00  // 2.15.0
            ctx = &ctxLow;
#endif
        }
        else if (I > params.high_threshold)
        {
            trg = outHigh.get();
            ++countHigh;
#if MRPT_VERSION >= 0x020f00  // 2.15.0
            ctx = &ctxHigh;
#endif
        }
        else
        {
            trg = outMid.get();
            ++countMid;
#if MRPT_VERSION >= 0x020f00  // 2.15.0
            ctx = &ctxMid;
#endif
        }

        if (trg)
        {
#if MRPT_VERSION >= 0x020f00  // 2.15.0
            trg->insertPointFrom(pc, i, *ctx);
#else
            trg->insertPointFrom(pc, i);
#endif
        }
    }

    MRPT_LOG_DEBUG_STREAM(
        "[FilterByIntensity] Input points=" << N << " low=" << countLow << " mid=" << countMid
                                            << " high=" << countHigh);

    MRPT_END
}
