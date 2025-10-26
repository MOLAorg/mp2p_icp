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
 * @file   FilterDecimateAdaptive.cpp
 * @brief  An adaptive sampler of pointclouds
 * @author Jose Luis Blanco Claraco
 * @date   Nov 24, 2023
 */

#include <mp2p_icp_filters/FilterDecimateAdaptive.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/round.h>
#include <mrpt/version.h>

#if defined(MP2P_HAS_TBB)
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#endif

IMPLEMENTS_MRPT_OBJECT(FilterDecimateAdaptive, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterDecimateAdaptive::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_REQ(c, output_pointcloud_layer);

    MCP_LOAD_REQ(c, desired_output_point_count);

    MCP_LOAD_OPT(c, voxel_size);
    MCP_LOAD_OPT(c, minimum_input_points_per_voxel);
    MCP_LOAD_OPT(c, parallelization_grain_size);
}

struct FilterDecimateAdaptive::Impl
{
#if defined(MP2P_HAS_TBB)
    tbb::enumerable_thread_specific<PointCloudToVoxelGrid> tls;
#else
    PointCloudToVoxelGrid filter_grid;
#endif
};

FilterDecimateAdaptive::FilterDecimateAdaptive() : impl_(mrpt::make_impl<Impl>())
{
    mrpt::system::COutputLogger::setLoggerName("FilterDecimateAdaptive");
}

void FilterDecimateAdaptive::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_START

    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c);

    MRPT_END
}

void FilterDecimateAdaptive::filter(mp2p_icp::metric_map_t& inOut) const
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
    mrpt::maps::CPointsMap::Ptr outPc = GetOrCreatePointLayer(
        inOut, params.output_pointcloud_layer,
        /*do not allow empty*/
        false,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    const auto& _ = params;  // shortcut

    outPc->reserve(outPc->size() + _.desired_output_point_count);

    struct DataPerVoxel
    {
        const PointCloudToVoxelGrid::voxel_t* voxel     = nullptr;
        uint32_t                              nextIdx   = 0;
        bool                                  exhausted = false;
    };

    // A list of all "valid" voxels:
    std::vector<DataPerVoxel> voxels;

    std::size_t nTotalVoxels = 0;

    const auto lambdaVisitVoxel =
        [&](const PointCloudToVoxelGrid::indices_t&, const PointCloudToVoxelGrid::voxel_t& data)
    {
        if (!data.indices.empty())
        {
            nTotalVoxels++;
        }
        if (data.indices.size() < _.minimum_input_points_per_voxel)
        {
            return;
        }

        voxels.emplace_back().voxel = &data;
    };

    // Parse input cloud through subsampling:
#if defined(MP2P_HAS_TBB)

    // Clear from past runs:
    for (auto& grid : impl_->tls)
    {
        grid.setConfiguration(params.voxel_size, true);
    }

    const auto pointCount = pc.size();

    const size_t grainsize = params.parallelization_grain_size;  // minimum block size hint

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, pointCount, grainsize),
        [&](const tbb::blocked_range<size_t>& r)
        {
            const auto start  = r.begin();
            const auto length = r.end() - r.begin();

            bool  tls_exists;
            auto& grid = impl_->tls.local(tls_exists);
            if (!tls_exists)
            {
                // Configure for first time:
                grid.setConfiguration(params.voxel_size, true);
            }
            grid.processPointCloud(pc, start, length);
        });

    for (auto& grid : impl_->tls)
    {
        grid.visit_voxels(lambdaVisitVoxel);
    }

#else
    impl_->filter_grid.clear();
    impl_->filter_grid.setConfiguration(params.voxel_size, true);
    impl_->filter_grid.processPointCloud(pc);

    voxels.reserve(impl_->filter_grid.size());

    impl_->filter_grid.visit_voxels(lambdaVisitVoxel);

#endif

#if MRPT_VERSION >= 0x020f00  // 2.15.0
    mrpt::maps::CPointsMap::InsertCtx ctx = outPc->prepareForInsertPointsFrom(pc);
#endif

    // Perform resampling:
    // -------------------
    constexpr int FRACTIONARY_BIT_COUNT = 12;

    const size_t nVoxels           = voxels.size();
    float        voxelIdxIncrement = 1.0f;
    if (nVoxels > params.desired_output_point_count)
    {
        voxelIdxIncrement =
            static_cast<float>(nVoxels) / static_cast<float>(params.desired_output_point_count);
    }

    const auto voxelIdxIncrement_frac =
        static_cast<std::size_t>(voxelIdxIncrement * (1 << FRACTIONARY_BIT_COUNT));

    bool anyInsertInTheRound = false;

    std::size_t i_frac = 0;
    while (outPc->size() < params.desired_output_point_count)
    {
        std::size_t i = i_frac >> FRACTIONARY_BIT_COUNT;

        if (i >= nVoxels)
        {
            i_frac = i_frac % (nVoxels << FRACTIONARY_BIT_COUNT);
            i      = i_frac >> FRACTIONARY_BIT_COUNT;

            if (!anyInsertInTheRound)
            {
                // This means there is no more points and we must end
                // despite we didn't reached the user's desired number of
                // points:
                break;
            }

            anyInsertInTheRound = false;
        }

        auto& ith = voxels[i];
        if (!ith.exhausted)
        {
            auto ptIdx = ith.voxel->indices[ith.nextIdx++];
#if MRPT_VERSION >= 0x020f00  // 2.15.0
            outPc->insertPointFrom(pc, ptIdx, ctx);
#else
            outPc->insertPointFrom(pc, ptIdx);
#endif
            anyInsertInTheRound = true;

            if (ith.nextIdx >= ith.voxel->indices.size())
            {
                ith.exhausted = true;
            }
        }

        i_frac += voxelIdxIncrement_frac;
    }

    MRPT_LOG_DEBUG_STREAM("used voxels=" << nTotalVoxels);

    MRPT_END
}
