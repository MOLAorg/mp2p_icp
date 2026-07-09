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
 * @file   FilterPolygon2D.cpp
 * @brief  Leaves or removes the points inside an arbitrary 2D polygon footprint
 * @author Jose Luis Blanco Claraco
 * @date   May 26, 2026
 */

#include <mp2p_icp_filters/FilterPolygon2D.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/version.h>

IMPLEMENTS_MRPT_OBJECT(FilterPolygon2D, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterPolygon2D::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterPolygon2D& /*parent*/)
{
    MCP_LOAD_OPT(c, input_pointcloud_layer);
    MCP_LOAD_OPT(c, inside_pointcloud_layer);
    MCP_LOAD_OPT(c, outside_pointcloud_layer);
    MCP_LOAD_OPT(c, z_min);
    MCP_LOAD_OPT(c, z_max);

    ASSERTMSG_(
        !inside_pointcloud_layer.empty() || !outside_pointcloud_layer.empty(),
        "At least one 'inside_pointcloud_layer' or 'outside_pointcloud_layer' must be provided.");

    ASSERTMSG_(
        c.has("polygon") && c["polygon"].isSequence(),
        "A 'polygon' sequence of [x, y] vertices is required.");

    const auto vertices = c["polygon"].asSequence();
    ASSERTMSG_(vertices.size() >= 3, "'polygon' must have at least 3 vertices.");

    polygon.clear();
    for (const auto& v : vertices)
    {
        ASSERTMSG_(
            v.isSequence() && v.asSequence().size() == 2,
            "Each 'polygon' vertex must be a [x, y] pair.");
        const auto xy = v.asSequence();
        polygon.emplace_back(xy.at(0).as<double>(), xy.at(1).as<double>());
    }
}

FilterPolygon2D::FilterPolygon2D()
{
    mrpt::system::COutputLogger::setLoggerName("FilterPolygon2D");
}

void FilterPolygon2D::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c, *this);

    MRPT_END
}

void FilterPolygon2D::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    // In:
    const auto pcPtr = inOut.point_layer(params.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr,
        mrpt::format(
            "Input point cloud layer '%s' was not found.", params.input_pointcloud_layer.c_str()));

    const auto& pc = *pcPtr;

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr insidePc = GetOrCreatePointLayer(
        inOut, params.inside_pointcloud_layer, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    if (insidePc)
    {
        insidePc->reserve(insidePc->size() + pc.size() / 10);
    }

    mrpt::maps::CPointsMap::Ptr outsidePc = GetOrCreatePointLayer(
        inOut, params.outside_pointcloud_layer, true /*allow empty for nullptr*/,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    if (outsidePc)
    {
        outsidePc->reserve(outsidePc->size() + pc.size() / 10);
    }

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    mrpt::maps::CPointsMap::InsertCtx ctxInside;
    if (insidePc)
    {
        insidePc->registerPointFieldsFrom(pc);
        ctxInside = insidePc->prepareForInsertPointsFrom(pc);
    }

    mrpt::maps::CPointsMap::InsertCtx ctxOutside;
    if (outsidePc)
    {
        outsidePc->registerPointFieldsFrom(pc);
        ctxOutside = outsidePc->prepareForInsertPointsFrom(pc);
    }

    for (size_t i = 0; i < xs.size(); i++)
    {
        const bool isInsideZ = (zs[i] >= params.z_min) && (zs[i] <= params.z_max);

        const bool isInside = isInsideZ && params.polygon.contains({xs[i], ys[i]});

        auto*       targetPc = isInside ? insidePc.get() : outsidePc.get();
        const auto* ctx      = isInside ? &ctxInside : &ctxOutside;
        if (targetPc)
        {
#if MRPT_VERSION >= 0x020f03  // 2.15.3
            targetPc->insertPointFrom(i, *ctx);
#elif MRPT_VERSION >= 0x020f00  // 2.15.0
            targetPc->insertPointFrom(pc, i, *ctx);
#else
            targetPc->insertPointFrom(pc, i);
#endif
        }
    }

    MRPT_END
}
