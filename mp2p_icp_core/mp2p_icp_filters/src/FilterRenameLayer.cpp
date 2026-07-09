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
 * @file   FilterRenameLayer.cpp
 * @brief  Renames a layer within a metric map.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 7, 2026
 */

#include <mp2p_icp_filters/FilterRenameLayer.h>
#include <mrpt/containers/yaml.h>

IMPLEMENTS_MRPT_OBJECT(FilterRenameLayer, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterRenameLayer::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, input_layer);
    MCP_LOAD_REQ(c, output_layer);
    MCP_LOAD_OPT(c, fail_if_input_layer_does_not_exist);
}

void FilterRenameLayer::initialize_filter(const mrpt::containers::yaml& c)
{
    params.load_from_yaml(c);
}

void FilterRenameLayer::filter(mp2p_icp::metric_map_t& inOut) const
{
    if (params.input_layer == params.output_layer)
    {
        return;
    }
    ASSERT_(!params.input_layer.empty());
    ASSERT_(!params.output_layer.empty());

    auto it = inOut.layers.find(params.input_layer);
    if (it == inOut.layers.end())
    {
        if (params.fail_if_input_layer_does_not_exist)
        {
            THROW_EXCEPTION_FMT(
                "FilterRenameLayer input layer '%s' does not exist", params.input_layer.c_str());
        }

        return;  // silently
    }

    // Move the pointer to the new name, effectively overwriting any existing layer
    inOut.layers[params.output_layer] = std::move(it->second);

    // Remove the old entry
    inOut.layers.erase(it);
}