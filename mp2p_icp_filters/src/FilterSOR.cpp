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
 * @file   FilterSOR.cpp
 * @brief  Statistical Outlier Removal (SOR) filter.
 * @author Jose Luis Blanco Claraco
 * @date   Dec 28, 2025
 */

#include <mp2p_icp_filters/FilterSOR.h>
#include <mp2p_icp_filters/GetOrCreatePointLayer.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/ops_containers.h>  // meanAndCov
#include <mrpt/version.h>

#if defined(MP2P_HAS_TBB)
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#endif

IMPLEMENTS_MRPT_OBJECT(FilterSOR, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

void FilterSOR::Parameters::load_from_yaml(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);
    MCP_LOAD_OPT(c, output_layer_inliers);
    MCP_LOAD_OPT(c, output_layer_outliers);
    MCP_LOAD_OPT(c, mean_k);
    MCP_LOAD_OPT(c, std_dev_mul);
    MCP_LOAD_OPT(c, parallelization_grain_size);

    ASSERTMSG_(
        !output_layer_inliers.empty() || !output_layer_outliers.empty(),
        "FilterSOR: At least one output layer must be specified.");
}

void FilterSOR::initialize_filter(const mrpt::containers::yaml& c)
{  // Load params:
    params.load_from_yaml(c);
}

void FilterSOR::filter(mp2p_icp::metric_map_t& inOut) const
{
#if MRPT_VERSION < 0x020f04
    throw std::runtime_error("FilterSOR requires MRPT >= 2.15.4");
#endif

    auto pcPtr = inOut.layer<mrpt::maps::CPointsMap>(params.input_pointcloud_layer);
    ASSERT_(pcPtr);
    const auto& pc = *pcPtr;

    // Define the output layers:
    mrpt::maps::CPointsMap::Ptr outInliers = GetOrCreatePointLayer(
        inOut, params.output_layer_inliers,
        /*do allow empty*/
        true,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    mrpt::maps::CPointsMap::Ptr outOutliers = GetOrCreatePointLayer(
        inOut, params.output_layer_outliers,
        /*do allow empty*/
        true,
        /* create cloud of the same type */
        pcPtr->GetRuntimeClass()->className);

    ASSERTMSG_(
        outInliers || outOutliers,
        "At least one of 'output_layer_inliers' or 'output_layer_outliers' must be provided.");

    std::optional<mrpt::maps::CPointsMap::InsertCtx> ctxI, ctxO;
    if (outInliers)
    {
        outInliers->registerPointFieldsFrom(pc);
        ctxI = outInliers->prepareForInsertPointsFrom(pc);
    }
    if (outOutliers)
    {
        outOutliers->registerPointFieldsFrom(pc);
        ctxO = outOutliers->prepareForInsertPointsFrom(pc);
    }

    if (pcPtr->empty())
    {
        return;
    }

    const size_t N = pc.size();

    // 1. Prepare KD-Tree for neighbor search
    pc.nn_prepare_for_3d_queries();

    // 2. Pass 1: Compute mean distances to K neighbors
    std::vector<float> avg_distances(N, 0.0f);

    auto lambda_process = [&](size_t i)
    {
        mrpt::math::TPoint3Df q;
        pc.getPoint(i, q.x, q.y, q.z);

        std::vector<float>  dists_sq;
        std::vector<size_t> indices;
        // PCL logic: nearestKSearch includes the query point itself at index 0
        // So we search for K+1 points.
        pc.kdTreeNClosestPoint3DIdx(q, params.mean_k + 1, indices, dists_sq);

        const auto found = indices.size();

        if (found <= 1)
        {
            avg_distances[i] = 0.0f;  // Isolated points are treated as inliers
            return;
        }

        double sum_dist = 0;
        for (size_t k = 1; k < found; ++k)
        {
            sum_dist += std::sqrt(dists_sq[k]);
        }
        avg_distances[i] = static_cast<float>(sum_dist / static_cast<float>(found - 1));
    };

#if defined(MP2P_HAS_TBB)
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, N, params.parallelization_grain_size),
        [&](const tbb::blocked_range<size_t>& r)
        {
            for (size_t i = r.begin(); i < r.end(); ++i)
            {
                lambda_process(i);
            }
        });
#else
    for (size_t i = 0; i < N; ++i)
    {
        lambda_process(i);
    }
#endif

    // 3. Compute Global Statistics
    double mean_dist, std_dev;
    mrpt::math::meanAndStd(avg_distances, mean_dist, std_dev);
    const double threshold = mean_dist + params.std_dev_mul * std_dev;

    size_t num_inliers  = 0;
    size_t num_outliers = 0;

    // 4. Pass 2: Dispatch points to output layers
    for (size_t i = 0; i < N; ++i)
    {
        const bool isInlier = (avg_distances[i] <= threshold);
        if (isInlier)
        {
#if MRPT_VERSION >= 0x020f03  // 2.15.3
            ++num_inliers;
            if (outInliers)
            {
                outInliers->insertPointFrom(i, *ctxI);
            }
        }
        else
        {
            ++num_outliers;
            if (outOutliers)
            {
                outOutliers->insertPointFrom(i, *ctxO);
            }
#elif MRPT_VERSION >= 0x020f00  // 2.15.0
            ++num_inliers;
            if (outInliers)
            {
                outInliers->insertPointFrom(pc, i, *ctxI);
            }
        }
        else
        {
            ++num_outliers;
            if (outOutliers)
            {
                outOutliers->insertPointFrom(pc, i, *ctxO);
            }
#else
            ++num_inliers;
            if (outInliers)
            {
                outInliers->insertPointFrom(pc, i);
            }
        }
        else
        {
            ++num_outliers;
            if (outOutliers)
            {
                outOutliers->insertPointFrom(pc, i);
            }
#endif
        }
    }

    // Debug stats:
    MRPT_LOG_DEBUG_FMT(
        "FilterSOR: Total points: %zu, Inliers: %zu, Outliers: %zu", N, num_inliers, num_outliers);
}