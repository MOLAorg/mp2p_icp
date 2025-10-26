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
 * @file   FilterFartherPointSampling.cpp
 * @brief  Farther point sampling (FPS) algorithm
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2025
 */

#include <mp2p_icp_filters/FilterFartherPointSampling.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/round.h>
#include <mrpt/version.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <queue>
#include <vector>

IMPLEMENTS_MRPT_OBJECT(FilterFartherPointSampling, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

namespace
{

// Heap element
struct HeapItem
{
    HeapItem() = default;
    HeapItem(const double dist_, std::size_t idx_) : dist(dist_), idx(idx_) {}

    double      dist = 0;
    std::size_t idx  = 0;
    bool        operator<(const HeapItem& other) const
    {
        return dist < other.dist;  // max-heap
    }
};

}  // namespace

void FilterFartherPointSampling::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterFartherPointSampling& parent)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_REQ(c, output_pointcloud_layer);
    DECLARE_PARAMETER_IN_REQ(c, desired_output_point_count, parent);
}

FilterFartherPointSampling::FilterFartherPointSampling()
{
    mrpt::system::COutputLogger::setLoggerName("FilterFartherPointSampling");
}

void FilterFartherPointSampling::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c, *this);

    MRPT_END
}

void FilterFartherPointSampling::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    // In:
    ASSERTMSG_(
        inOut.layers.count(params.input_pointcloud_layer) != 0,
        mrpt::format(
            "Input point cloud layer '%s' was not found.", params.input_pointcloud_layer.c_str()));

    auto pcPtr = mp2p_icp::MapToPointsMap(*inOut.layers.at(params.input_pointcloud_layer));
    if (!pcPtr)
    {
        THROW_EXCEPTION_FMT(
            "Layer '%s' must be of point cloud type.", params.input_pointcloud_layer.c_str());
    }

    const auto& pc = *pcPtr;

    // Create if new: Append to existing layer, if already existed.
    mrpt::maps::CPointsMap::Ptr outPc =
        GetOrCreatePointLayer(inOut, params.output_pointcloud_layer);

    outPc->reserve(outPc->size() + params.desired_output_point_count);

#if MRPT_VERSION >= 0x020f00  // 2.15.0
    mrpt::maps::CPointsMap::InsertCtx ctx = outPc->prepareForInsertPointsFrom(pc);
#endif

    const auto input_size = pc.size();
    if (params.desired_output_point_count > input_size)
    {
        // Just copy all points:
        outPc->insertAnotherMap(pcPtr, mrpt::poses::CPose3D::Identity());
        return;
    }

    std::vector<double>           minDist(input_size, std::numeric_limits<double>::max());
    std::priority_queue<HeapItem> heap;

    // Pick random start
    srand((unsigned)time(NULL));
    std::size_t idx = rand() % input_size;

#if MRPT_VERSION >= 0x020f00  // 2.15.0
    outPc->insertPointFrom(pc, idx, ctx);
#else
    outPc->insertPointFrom(pc, idx);
#endif

    // Initialize distances to first point
    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    const auto dist2 = [&](const size_t i, const size_t j)
    {
        const float dx = xs[i] - xs[j];
        const float dy = ys[i] - ys[j];
        const float dz = zs[i] - zs[j];
        return dx * dx + dy * dy + dz * dz;
    };

    for (std::size_t j = 0; j < input_size; j++)
    {
        minDist[j] = dist2(j, idx);
        heap.push({minDist[j], j});
    }

    for (std::size_t i = 1; i < params.desired_output_point_count; i++)
    {
        // Get farthest valid point from heap
        HeapItem top;
        do
        {
            top = heap.top();
            heap.pop();
        } while (top.dist != minDist[top.idx] && !heap.empty());

        const auto farthestIdx = top.idx;
#if MRPT_VERSION >= 0x020f00  // 2.15.0
        outPc->insertPointFrom(pc, farthestIdx, ctx);
#else
        outPc->insertPointFrom(pc, farthestIdx);
#endif
        // Update distances relative to new point
        for (std::size_t j = 0; j < input_size; j++)
        {
            double d = dist2(j, farthestIdx);
            if (d < minDist[j])
            {
                minDist[j] = d;
                heap.push({d, j});
            }
        }
    }

    MRPT_END
}
