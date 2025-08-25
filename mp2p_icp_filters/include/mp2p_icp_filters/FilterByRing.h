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
 * @file   FilterByRing.h
 * @brief  Keeps only a given subset of an input cloud by LiDAR "ring_id"
 * @author Jose Luis Blanco Claraco
 * @date   Jun 20, 2024
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

#include <set>

namespace mp2p_icp_filters
{
/** Keeps only a given subset of an input cloud by LiDAR "ring_id".
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterByRing : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterByRing, mp2p_icp_filters)
   public:
    FilterByRing();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        std::string input_pointcloud_layer;

        /** If non-empty, points with the selected ring_id's will be stored here
         */
        std::string output_layer_selected;

        /** If non-empty, points WITHOUT the selected ring_id's will be stored
         * here */
        std::string output_layer_non_selected;

        std::set<int> selected_ring_ids;
    };

    /** Algorithm parameters */
    Parameters params_;
};

/** @} */

}  // namespace mp2p_icp_filters
