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
 * @file   FilterFartherPointSampling.h
 * @brief  Farther point sampling (FPS) algorithm
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2025
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/maps/CPointsMap.h>

namespace mp2p_icp_filters
{
/** Subsamples a cloud using Farther Point Sampling (FPS) with a desired point count.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterFartherPointSampling : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterFartherPointSampling, mp2p_icp_filters)
   public:
    FilterFartherPointSampling();

    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterFartherPointSampling& parent);

        std::string  input_pointcloud_layer;
        std::string  output_pointcloud_layer;
        unsigned int desired_output_point_count = 2000;
    };

    /** Algorithm parameters */
    Parameters params;
};

/** @} */

}  // namespace mp2p_icp_filters
