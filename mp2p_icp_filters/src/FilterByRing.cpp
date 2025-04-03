/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterByRing.cpp
 * @brief  Keeps only a given subset of an input cloud by LiDAR "ring_id"
 * @author Jose Luis Blanco Claraco
 * @date   Jun 20, 2024
 */

#include <mp2p_icp_filters/FilterByRing.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>

IMPLEMENTS_MRPT_OBJECT(FilterByRing, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterByRing::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);

    MCP_LOAD_OPT(c, output_layer_selected);
    MCP_LOAD_OPT(c, output_layer_non_selected);

    selected_ring_ids.clear();

    auto cfgIn = c["selected_ring_ids"];
    if (cfgIn.isScalar())
    {
        // only one:
        selected_ring_ids.insert(cfgIn.as<int>());
    }
    else
    {
        ASSERTMSG_(
            cfgIn.isSequence(),
            "YAML configuration must have an entry `selected_ring_ids` "
            "with a scalar or sequence.");

        for (const auto& n : cfgIn.asSequenceRange()) selected_ring_ids.insert(n.as<int>());
    }
    ASSERT_(!selected_ring_ids.empty());
}

FilterByRing::FilterByRing() = default;

void FilterByRing::initialize(const mrpt::containers::yaml& c)
{
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c);
}

void FilterByRing::filter(mp2p_icp::metric_map_t& inOut) const
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
    mrpt::maps::CPointsMap::Ptr outSelected = GetOrCreatePointLayer(
        inOut, params_.output_layer_selected, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    if (outSelected) outSelected->reserve(outSelected->size() + pc.size() / 10);

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outNonSel = GetOrCreatePointLayer(
        inOut, params_.output_layer_non_selected, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    if (outNonSel) outNonSel->reserve(outNonSel->size() + pc.size() / 10);

    ASSERTMSG_(
        outSelected || outNonSel,
        "At least one of 'output_layer_selected' or "
        "'output_layer_non_selected' must be provided.");

    const auto& xs = pc.getPointsBufferRef_x();
    // const auto& ys   = pc.getPointsBufferRef_y();
    // const auto& zs   = pc.getPointsBufferRef_z();
    const auto* ptrR = pc.getPointsBufferRef_ring();
    if (!ptrR || ptrR->empty())
    {
        THROW_EXCEPTION_FMT(
            "Error: this filter needs the input layer '%s' to has an "
            "'ring' point channel.",
            params_.input_pointcloud_layer.c_str());
    }

    const auto& Rs = *ptrR;
    ASSERT_EQUAL_(Rs.size(), xs.size());
    const size_t N = xs.size();

    size_t countSel = 0, countNon = 0;

    for (size_t i = 0; i < N; i++)
    {
        const auto R = Rs[i];

        mrpt::maps::CPointsMap* trg = nullptr;

        if (params_.selected_ring_ids.count(R) != 0)
        {
            trg = outSelected.get();
            ++countSel;
        }
        else
        {
            trg = outNonSel.get();
            ++countNon;
        }

        if (trg) trg->insertPointFrom(pc, i);
    }

    MRPT_LOG_DEBUG_STREAM(
        "[FilterByRing] Input points=" << N << " selected=" << countSel
                                       << " non-selected=" << countNon);

    MRPT_END
}
