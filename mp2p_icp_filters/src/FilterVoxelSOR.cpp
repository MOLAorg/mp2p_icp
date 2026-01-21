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
 * @file   FilterVoxelSOR.cpp
 * @brief  Voxel-based Statistical Outlier Removal (SOR) filter.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 21, 2026
 */

#include <mp2p_icp_filters/FilterVoxelSOR.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mp2p_icp_filters/PointCloudToVoxelGrid.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/ops_containers.h>  // meanAndStd
#include <mrpt/version.h>

#if defined(MP2P_HAS_TBB)
#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#endif

IMPLEMENTS_MRPT_OBJECT(FilterVoxelSOR, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

FilterVoxelSOR::FilterVoxelSOR() = default;

void FilterVoxelSOR::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, input_layer);
    MCP_LOAD_REQ(c, output_layer_inliers);
    MCP_LOAD_OPT(c, output_layer_outliers);
    MCP_LOAD_REQ(c, voxel_size);
    MCP_LOAD_OPT(c, mean_k);
    MCP_LOAD_OPT(c, std_dev_mul);
    MCP_LOAD_OPT(c, use_tsl_robin_map);
    MCP_LOAD_OPT(c, parallelization_grain_size);
}

void FilterVoxelSOR::initialize_filter(const mrpt::containers::yaml& c)
{
    params.load_from_yaml(c);

    ASSERT_GT_(params.voxel_size, 0.0f);
    ASSERT_GE_(params.mean_k, 1);
    ASSERT_GE_(params.parallelization_grain_size, 1);
}

void FilterVoxelSOR::filter(mp2p_icp::metric_map_t& inOut) const
{
    auto pcIn = inOut.layer<mrpt::maps::CPointsMap>(params.input_layer);
    if (!pcIn || pcIn->empty())
    {
        return;
    }

    if (params.output_layer_inliers == params.input_layer ||
        (!params.output_layer_outliers.empty() &&
         (params.output_layer_outliers == params.input_layer ||
          params.output_layer_outliers == params.output_layer_inliers)))
    {
        throw std::invalid_argument(
            "FilterVoxelSOR: output layers must differ from input_layer and each other");
    }

    MRPT_LOG_DEBUG("FilterVoxelSOR: Starting SOR filtering...");

    // 1. Partition into Voxel Grid
    PointCloudToVoxelGrid grid;
    grid.setConfiguration(params.voxel_size, params.use_tsl_robin_map);
    grid.processPointCloud(*pcIn);

    MRPT_LOG_DEBUG("FilterVoxelSOR: Done voxels.");

    // 2. Prepare output layers
    auto outInliers = GetOrCreatePointLayer(
        inOut, params.output_layer_inliers, true, pcIn->GetRuntimeClass()->className);

    mrpt::maps::CPointsMap::Ptr outOutliers;
    if (!params.output_layer_outliers.empty())
    {
        outOutliers = GetOrCreatePointLayer(
            inOut, params.output_layer_outliers, false, pcIn->GetRuntimeClass()->className);
        outOutliers->clear();
    }

#if MRPT_VERSION >= 0x020f00
    outInliers->registerPointFieldsFrom(*pcIn);
    auto ctxI = outInliers->prepareForInsertPointsFrom(*pcIn);
    std::optional<mrpt::maps::CPointsMap::InsertCtx> ctxO;
    if (outOutliers)
    {
        outOutliers->registerPointFieldsFrom(*pcIn);
        ctxO = outOutliers->prepareForInsertPointsFrom(*pcIn);
    }
#endif

    const auto& xs = pcIn->getPointsBufferRef_x();
    const auto& ys = pcIn->getPointsBufferRef_y();
    const auto& zs = pcIn->getPointsBufferRef_z();

    std::atomic<size_t> inliersCount{0};
    std::atomic<size_t> outliersCount{0};

    // 3. Collect all voxels into a vector for parallel processing
    struct VoxelData
    {
        PointCloudToVoxelGrid::indices_t idx;
        PointCloudToVoxelGrid::voxel_t   vxl;
    };

    std::vector<VoxelData> voxels;
    grid.visit_voxels([&](const auto& idx, const auto& vxl) { voxels.push_back({idx, vxl}); });

    const size_t numVoxels = voxels.size();
    MRPT_LOG_DEBUG_FMT("FilterVoxelSOR: Processing %zu voxels...", numVoxels);

    // Thread-safe storage for results: each thread accumulates its own results
    struct ThreadResults
    {
        std::vector<size_t> inlierIndices;
        std::vector<size_t> outlierIndices;
    };

#if defined(MP2P_HAS_TBB)
    // Use thread-local storage for each thread to avoid mutex contention
    tbb::enumerable_thread_specific<ThreadResults> threadResults;

    // 4. Process voxels in parallel
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, numVoxels, params.parallelization_grain_size),
        [&](const tbb::blocked_range<size_t>& r)
        {
            // Get thread-local storage
            auto& localResults = threadResults.local();

            for (size_t voxelIdx = r.begin(); voxelIdx < r.end(); ++voxelIdx)
            {
                const auto&  vxlData      = voxels[voxelIdx];
                const auto&  vxl          = vxlData.vxl;
                const size_t nVoxelPoints = vxl.indices.size();

                // If too few points for SOR, keep them all as inliers
                if (nVoxelPoints <= params.mean_k)
                {
                    for (const auto originalIdx : vxl.indices)
                    {
                        localResults.inlierIndices.push_back(originalIdx);
                    }
                    continue;
                }

                // Build a small local map for k-NN
                mrpt::maps::CSimplePointsMap localMap;
                localMap.reserve(nVoxelPoints);
                for (const auto originalIdx : vxl.indices)
                {
                    localMap.insertPointFast(xs[originalIdx], ys[originalIdx], zs[originalIdx]);
                }

                // Calculate avg distance to k-neighbors for each point in voxel
                std::vector<double> avg_distances(nVoxelPoints);
                std::vector<size_t> nn_indices(params.mean_k + 1);
                std::vector<float>  nn_dists_sq(params.mean_k + 1);

                for (size_t i = 0; i < nVoxelPoints; ++i)
                {
                    localMap.kdTreeNClosestPoint3DIdx(
                        xs[vxl.indices[i]], ys[vxl.indices[i]], zs[vxl.indices[i]],
                        params.mean_k + 1, nn_indices, nn_dists_sq);

                    double sum_dist = 0;
                    // index 0 is the point itself
                    for (size_t k = 1; k < nn_dists_sq.size(); ++k)
                    {
                        sum_dist += std::sqrt(static_cast<double>(nn_dists_sq[k]));
                    }

                    avg_distances[i] = sum_dist / static_cast<double>(params.mean_k);
                }

                // Compute voxel-local stats
                double mu, sigma;
                mrpt::math::meanAndStd(avg_distances, mu, sigma);
                const double threshold = mu + params.std_dev_mul * sigma;

                // Classify points
                for (size_t i = 0; i < nVoxelPoints; ++i)
                {
                    const auto originalIdx = vxl.indices[i];
                    if (avg_distances[i] <= threshold)
                    {
                        localResults.inlierIndices.push_back(originalIdx);
                    }
                    else
                    {
                        localResults.outlierIndices.push_back(originalIdx);
                    }
                }
            }
        });

    // 5. Merge results from all threads
    for (const auto& result : threadResults)
    {
        for (const auto idx : result.inlierIndices)
        {
#if MRPT_VERSION >= 0x020f03  // 2.15.3
            outInliers->insertPointFrom(idx, ctxI);
#else
            outInliers->insertPointFrom(*pcIn, idx, ctxI);
#endif
        }
        inliersCount += result.inlierIndices.size();

        if (outOutliers)
        {
            for (const auto idx : result.outlierIndices)
            {
#if MRPT_VERSION >= 0x020f03  // 2.15.3
                outOutliers->insertPointFrom(idx, *ctxO);
#else
                outOutliers->insertPointFrom(*pcIn, idx, *ctxO);
#endif
            }
            outliersCount += result.outlierIndices.size();
        }
    }

#else
    // Sequential fallback (no TBB)
    for (size_t voxelIdx = 0; voxelIdx < numVoxels; ++voxelIdx)
    {
        const auto&  vxlData      = voxels[voxelIdx];
        const auto&  vxl          = vxlData.vxl;
        const size_t nVoxelPoints = vxl.indices.size();

        // If too few points for SOR, keep them all as inliers
        if (nVoxelPoints <= params.mean_k)
        {
            inliersCount += nVoxelPoints;
            for (const auto originalIdx : vxl.indices)
            {
#if MRPT_VERSION >= 0x020f03  // 2.15.3
                outInliers->insertPointFrom(originalIdx, ctxI);
#else
                outInliers->insertPointFrom(*pcIn, originalIdx, ctxI);
#endif
            }
            continue;
        }

        // Build a small local map for k-NN
        mrpt::maps::CSimplePointsMap localMap;
        localMap.reserve(nVoxelPoints);
        for (const auto originalIdx : vxl.indices)
        {
            localMap.insertPointFast(xs[originalIdx], ys[originalIdx], zs[originalIdx]);
        }

        // Calculate avg distance to k-neighbors for each point in voxel
        std::vector<double> avg_distances(nVoxelPoints);
        std::vector<size_t> nn_indices(params.mean_k + 1);
        std::vector<float>  nn_dists_sq(params.mean_k + 1);

        for (size_t i = 0; i < nVoxelPoints; ++i)
        {
            localMap.kdTreeNClosestPoint3DIdx(
                xs[vxl.indices[i]], ys[vxl.indices[i]], zs[vxl.indices[i]], params.mean_k + 1,
                nn_indices, nn_dists_sq);

            double sum_dist = 0;
            // index 0 is the point itself
            for (size_t k = 1; k < nn_dists_sq.size(); ++k)
            {
                sum_dist += std::sqrt(static_cast<double>(nn_dists_sq[k]));
            }

            avg_distances[i] = sum_dist / static_cast<double>(params.mean_k);
        }

        // Compute voxel-local stats
        double mu, sigma;
        mrpt::math::meanAndStd(avg_distances, mu, sigma);
        const double threshold = mu + params.std_dev_mul * sigma;

        // Dispatch
        for (size_t i = 0; i < nVoxelPoints; ++i)
        {
            const auto originalIdx = vxl.indices[i];
            if (avg_distances[i] <= threshold)
            {
                inliersCount++;
#if MRPT_VERSION >= 0x020f03  // 2.15.3
                outInliers->insertPointFrom(originalIdx, ctxI);
#else
                outInliers->insertPointFrom(*pcIn, originalIdx, ctxI);
#endif
            }
            else
            {
                outliersCount++;
                if (outOutliers)
                {
#if MRPT_VERSION >= 0x020f03  // 2.15.3
                    outOutliers->insertPointFrom(originalIdx, *ctxO);
#else
                    outOutliers->insertPointFrom(*pcIn, originalIdx, *ctxO);
#endif
                }
            }
        }
    }
#endif

    MRPT_LOG_DEBUG_FMT(
        "FilterVoxelSOR: Inliers kept: %zu, Outliers removed: %zu", inliersCount.load(),
        outliersCount.load());
}