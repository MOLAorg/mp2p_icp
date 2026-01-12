/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   FilterDecimate.cpp
 * @brief  Naive point cloud downsampling (every N-th point).
 * @author Jose Luis Blanco Claraco
 * @date   Jan 12, 2026
 */

#include <mp2p_icp_filters/FilterDecimate.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/version.h>

IMPLEMENTS_MRPT_OBJECT(FilterDecimate, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

FilterDecimate::FilterDecimate() = default;

void FilterDecimate::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, input_layer);
    MCP_LOAD_REQ(c, output_layer);
    MCP_LOAD_OPT(c, decimation);
    MCP_LOAD_OPT(c, target_max_size);
}

void FilterDecimate::initialize_filter(const mrpt::containers::yaml& c)
{
    params.load_from_yaml(c);
}

void FilterDecimate::filter(mp2p_icp::metric_map_t& inOut) const
{
    auto pcIn = inOut.layer<mrpt::maps::CPointsMap>(params.input_layer);
    if (!pcIn || pcIn->empty())
    {
        return;
    }

    const size_t nIn = pcIn->size();

    // 1. Determine decimation factor N
    size_t N = 1;
    if (params.decimation > 0)
    {
        N = params.decimation;
    }
    else if (params.target_max_size > 0 && nIn > params.target_max_size)
    {
        N = nIn / params.target_max_size;
    }

    // Shortcut if no decimation is needed and layers are different
    if (N <= 1)
    {
        if (params.input_layer != params.output_layer)
        {
            // warning: this makes a shallow copy... more efficient but users should be aware of
            // side effects.
            inOut.layers[params.output_layer] = pcIn;
        }
        return;
    }

    // 2. Prepare output layer
    auto outPc = GetOrCreatePointLayer(
        inOut, params.output_layer, false /*don't allow move if same name*/,
        pcIn->GetRuntimeClass()->className);

    // Efficiently copy points including all extra fields
#if MRPT_VERSION >= 0x020f00
    outPc->registerPointFieldsFrom(*pcIn);
    auto ctx = outPc->prepareForInsertPointsFrom(*pcIn);
#endif

    outPc->reserve(outPc->size() + (nIn / N));

    for (size_t i = 0; i < nIn; i += N)
    {
#if MRPT_VERSION >= 0x020f03
        outPc->insertPointFrom(i, ctx);
#elif MRPT_VERSION >= 0x020f00
        outPc->insertPointFrom(*pcIn, i, ctx);
#else
        outPc->insertPointFrom(*pcIn, i);
#endif
    }
}