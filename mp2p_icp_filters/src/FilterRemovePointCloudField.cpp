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
 * @file   FilterRemovePointCloudField.cpp
 * @brief  Unregisters (removes) a custom point cloud field
 * @author Jose Luis Blanco Claraco
 * @date   Feb 15, 2026
 */

#include <mp2p_icp_filters/FilterRemovePointCloudField.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CGenericPointsMap.h>

IMPLEMENTS_MRPT_OBJECT(FilterRemovePointCloudField, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterRemovePointCloudField::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_OPT(c, pointcloud_layer);
    MCP_LOAD_REQ(c, field_name);
    MCP_LOAD_OPT(c, throw_on_missing_field);
}

FilterRemovePointCloudField::FilterRemovePointCloudField()
{
    mrpt::system::COutputLogger::setLoggerName("FilterRemovePointCloudField");
}

void FilterRemovePointCloudField::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c);

    MRPT_END
}

void FilterRemovePointCloudField::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    auto it = inOut.layers.find(params.pointcloud_layer);
    if (it == inOut.layers.end())
    {
        MRPT_LOG_WARN_STREAM(
            "Layer '" << params.pointcloud_layer << "' not found in input map, skipping.");
        return;
    }

    auto pcPtr = it->second;
    ASSERT_(pcPtr);

    // Try to cast to CGenericPointsMap to access unregisterField method
    auto* pc = dynamic_cast<mrpt::maps::CGenericPointsMap*>(pcPtr.get());
    if (!pc)
    {
        MRPT_LOG_WARN_STREAM(
            "Layer '" << params.pointcloud_layer << "' is not a CGenericPointsMap, skipping.");
        return;
    }

    // Try to unregister the field
    const bool removed = pc->unregisterField(params.field_name);

    if (!removed && params.throw_on_missing_field)
    {
        THROW_EXCEPTION(mrpt::format(
            "Field '%s' does not exist in layer '%s'", params.field_name.c_str(),
            params.pointcloud_layer.c_str()));
    }

    if (removed)
    {
        MRPT_LOG_DEBUG_STREAM(
            "Removed field '" << params.field_name << "' from layer '" << params.pointcloud_layer
                              << "'");
    }
    else
    {
        MRPT_LOG_DEBUG_STREAM(
            "Field '" << params.field_name << "' not found in layer '" << params.pointcloud_layer
                      << "' (ignored)");
    }

    MRPT_END
}
