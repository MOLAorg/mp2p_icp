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
 * @file   FilterVoxelSOR.h
 * @brief  Voxel-based Statistical Outlier Removal (SOR) filter.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 21, 2026
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/maps/CPointsMap.h>

namespace mp2p_icp_filters
{
/**
 * @brief Voxel-based Statistical Outlier Removal (SOR) filter.
 *
 * This filter partitions the point cloud into voxels. Inside each voxel, it
 * computes the average distance of each point to its k-nearest neighbors.
 * Points are considered outliers if their average distance is significantly
 * higher than the mean of the distances within that specific voxel.
 *
 * This is much faster than global SOR for large clouds as the KD-tree searches
 * are localized and restricted to a small subset of points.
 *
 * When TBB is available, voxel processing is automatically parallelized for
 * improved performance on multi-core systems.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterVoxelSOR : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterVoxelSOR, mp2p_icp_filters)
   public:
    FilterVoxelSOR();

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        std::string input_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;
        std::string output_layer_inliers;
        std::string output_layer_outliers;

        /** Size of the voxel for local processing [meters]. */
        float voxel_size = 2.0f;

        /** Number of neighbors to analyze for each point locally. */
        uint32_t mean_k = 20;

        /** Standard deviation multiplier threshold. */
        double std_dev_mul = 2.0;

        /** Whether to use tsl::robin_map for voxel storage (faster in general) */
        bool use_tsl_robin_map = true;

        /** Grain size for TBB parallelization (number of voxels per thread block).
         *  Larger values reduce overhead but may cause load imbalance.
         *  Only used if TBB is available at build time.
         */
        size_t parallelization_grain_size = 10;
    };

    /** Algorithm parameters */
    Parameters params;

   protected:
    void initialize_filter(const mrpt::containers::yaml& c) override;
};

}  // namespace mp2p_icp_filters