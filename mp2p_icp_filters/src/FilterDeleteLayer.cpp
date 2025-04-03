/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
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

IMPLEMENTS_MRPT_OBJECT(FilterDeleteLayer, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterDeleteLayer::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    ASSERTMSG_(
        c.has("pointcloud_layer_to_remove"),
        "YAML configuration must have an entry `pointcloud_layer_to_remove` "
        "with a "
        "scalar or sequence.");

    pointcloud_layer_to_remove.clear();

    auto cfgIn = c["pointcloud_layer_to_remove"];
    if (cfgIn.isScalar()) { pointcloud_layer_to_remove.push_back(cfgIn.as<std::string>()); }
    else
    {
        ASSERTMSG_(
            cfgIn.isSequence(),
            "YAML configuration must have an entry "
            "`pointcloud_layer_to_remove` "
            "with a scalar or sequence.");

        for (const auto& s : cfgIn.asSequence())
            pointcloud_layer_to_remove.push_back(s.as<std::string>());
    }
    ASSERT_(!pointcloud_layer_to_remove.empty());

    MCP_LOAD_OPT(c, error_on_missing_input_layer);
}

FilterDeleteLayer::FilterDeleteLayer()
{
    mrpt::system::COutputLogger::setLoggerName("FilterDeleteLayer");
}

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

    for (const auto& layer : params_.pointcloud_layer_to_remove)
    {
        const auto nRemoved = inOut.layers.erase(layer);

        if (params_.error_on_missing_input_layer)
        {
            ASSERTMSG_(
                nRemoved != 0,
                mrpt::format("Point cloud layer '%s' was not found.", layer.c_str()));
        }
        MRPT_LOG_DEBUG_STREAM("Deleted layer: '" << layer << "'");
    }
    MRPT_END
}
