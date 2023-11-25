/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterByRange.cpp
 * @brief  Leaves or removes points by min/max range from the origin.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 14, 2023
 */

#include <mp2p_icp_filters/FilterByRange.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>

IMPLEMENTS_MRPT_OBJECT(
    FilterByRange, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterByRange::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_REQ(c, output_pointcloud_layer);
    MCP_LOAD_REQ(c, range_min);
    MCP_LOAD_REQ(c, range_max);
    MCP_LOAD_REQ(c, keep_between);
}

FilterByRange::FilterByRange() = default;

void FilterByRange::initialize(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params_.load_from_yaml(c);

    MRPT_END
}

void FilterByRange::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    // In:
    const auto pcPtr = inOut.point_layer(params_.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr, mrpt::format(
                   "Input point cloud layer '%s' was not found.",
                   params_.input_pointcloud_layer.c_str()));

    const auto& pc = *pcPtr;

    // Out:
    ASSERT_(!params_.output_pointcloud_layer.empty());

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap* outPc = nullptr;
    if (auto itLy = inOut.layers.find(params_.output_pointcloud_layer);
        itLy != inOut.layers.end())
    {
        outPc = mp2p_icp::MapToPointsMap(*itLy->second);
        if (!outPc)
            THROW_EXCEPTION_FMT(
                "Layer '%s' must be of point cloud type.",
                params_.output_pointcloud_layer.c_str());
    }
    else
    {
        auto newMap = mrpt::maps::CSimplePointsMap::Create();
        outPc       = newMap.get();
        inOut.layers[params_.output_pointcloud_layer] = newMap;
    }

    outPc->reserve(outPc->size() + pc.size() / 10);

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    const float sqrMin = mrpt::square(params_.range_min);
    const float sqrMax = mrpt::square(params_.range_max);

    for (size_t i = 0; i < xs.size(); i++)
    {
        const float sqrNorm =
            mrpt::math::TPoint3Df(xs[i], ys[i], zs[i]).sqrNorm();

        const bool isInside = sqrNorm >= sqrMin && sqrNorm <= sqrMax;

        if ((isInside && !params_.keep_between) ||
            (!isInside && params_.keep_between))
            continue;  // remove this point.

        // Otherwise, add it:
        outPc->insertPointFast(xs[i], ys[i], zs[i]);
    }
    outPc->mark_as_modified();

    MRPT_END
}
