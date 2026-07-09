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
 * @file   FilterSOR.h
 * @brief  Statistical Outlier Removal (SOR) filter.
 * @author Jose Luis Blanco Claraco
 * @date   Dec 28, 2025
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/maps/CPointsMap.h>

namespace mp2p_icp_filters
{
/**
 * @brief Statistical Outlier Removal (SOR) filter.
 * * For each point, it computes the average distance to its k-nearest neighbors.
 * Points are considered outliers if their average distance is greater than
 * (mean + std_dev_multiplier * std_dev) of the entire cloud's average distances.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterSOR : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterSOR, mp2p_icp_filters)

   public:
    FilterSOR() = default;

    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        std::string input_pointcloud_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;
        std::string output_layer_inliers;
        std::string output_layer_outliers;

        /** Number of neighbors to analyze for each point. */
        unsigned int mean_k = 20;

        /** Standard deviation multiplier threshold. */
        double std_dev_mul = 1.0;

        /** When TBB is enabled, the grainsize for parallel processing. */
        size_t parallelization_grain_size = 1024UL;
    };

    Parameters params;

   protected:
    void initialize_filter(const mrpt::containers::yaml& c) override;

   private:
    struct Impl;
    mutable mrpt::pimpl<Impl> impl_;
};

}  // namespace mp2p_icp_filters