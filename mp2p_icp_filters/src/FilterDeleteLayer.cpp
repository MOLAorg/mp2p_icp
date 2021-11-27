/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterDeleteLayer.cpp
 * @brief  Builds a new layer with a decimated version of an input layer.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 10, 2021
 */

#include <mp2p_icp_filters/FilterDeleteLayer.h>
#include <mrpt/containers/yaml.h>

IMPLEMENTS_MRPT_OBJECT(
    FilterDeleteLayer, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterDeleteLayer::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, pointcloud_layer_to_remove);
    MCP_LOAD_OPT(c, error_on_missing_input_layer);
}

FilterDeleteLayer::FilterDeleteLayer() = default;

void FilterDeleteLayer::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c);

    MRPT_END
}

void FilterDeleteLayer::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    const auto nRemoved =
        inOut.layers.erase(params_.pointcloud_layer_to_remove);

    if (params_.error_on_missing_input_layer)
    {
        ASSERTMSG_(
            nRemoved != 0, mrpt::format(
                               "Point cloud layer '%s' was not found.",
                               params_.pointcloud_layer_to_remove.c_str()));
    }
    MRPT_END
}
