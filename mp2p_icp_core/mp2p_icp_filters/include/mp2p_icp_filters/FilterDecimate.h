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
 * @file   FilterDecimate.h
 * @brief  Naive point cloud downsampling (every N-th point).
 * @author Jose Luis Blanco Claraco
 * @date   Jan 12, 2026
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

namespace mp2p_icp_filters
{
/**
 * @brief Naive point cloud downsampling.
 *
 * This filter reduces the number of points by keeping only one out of every N
 * points. It can be configured either with a fixed decimation factor or by
 * providing a target maximum size (in which case the decimation factor is
 * computed automatically).
 *
 * If the output layer exists, new points are accumulated on it: previous contents are not cleared.
 *
 * Note: This filter is non-deterministic regarding the spatial distribution
 * of points, as it relies purely on the internal storage order.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterDecimate : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterDecimate, mp2p_icp_filters)
   public:
    FilterDecimate();

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        /** Input layer name (must be a point cloud) */
        std::string input_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;

        /** Output layer name */
        std::string output_layer;

        /** Keep one out of every 'decimation' points. If > 0, this takes
         * precedence over target_max_size. */
        uint32_t decimation = 0;

        /** Target maximum number of points in the output cloud.
         * Used only if decimation == 0. */
        uint64_t target_max_size = 0;
    };

    /** Algorithm parameters */
    Parameters params;

   protected:
    void initialize_filter(const mrpt::containers::yaml& c) override;
};

}  // namespace mp2p_icp_filters