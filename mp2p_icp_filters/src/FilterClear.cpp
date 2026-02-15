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
 * @file   FilterClear.cpp
 * @brief  Clears (empties) a given metric map layer
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2026
 */

#include <mp2p_icp_filters/FilterClear.h>
#include <mrpt/containers/yaml.h>

IMPLEMENTS_MRPT_OBJECT(FilterClear, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterClear::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, target_layer);
}

FilterClear::FilterClear() { mrpt::system::COutputLogger::setLoggerName("FilterClear"); }

void FilterClear::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c);

    MRPT_END
}

void FilterClear::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    auto it = inOut.layers.find(params.target_layer);

    ASSERTMSG_(
        it != inOut.layers.end(),
        mrpt::format("Layer '%s' was not found in the metric map.", params.target_layer.c_str()));

    ASSERT_(it->second);

    // Call the virtual clear() method from CMetricMap
    it->second->clear();

    MRPT_LOG_DEBUG_STREAM("Cleared layer: '" << params.target_layer << "'");

    MRPT_END
}
