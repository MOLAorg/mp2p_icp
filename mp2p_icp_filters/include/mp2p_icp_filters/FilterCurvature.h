/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterCurvature.cpp
 * @brief  Classifies a sorted input point cloud by local curvature
 * @author Jose Luis Blanco Claraco
 * @date   Dec 11, 2023
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

namespace mp2p_icp_filters
{
/** Accepts as input a sorted point cloud, and classifies points by the local
 * curvature, estimated from the angle between each point and its immediate
 * former and posterior neigbors.
 *
 * Not compatible with calling from different threads simultaneously for
 * different input point clouds. Use independent instances for each thread if
 * needed.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterCurvature : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterCurvature, mp2p_icp_filters)
   public:
    FilterCurvature();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        std::string input_pointcloud_layer =
            mp2p_icp::metric_map_t::PT_LAYER_RAW;

        /** If non-empty, points with larger curvature ("edges") will be stored
         * here. */
        std::string output_layer_larger_curvature;

        /** If non-empty, points with smaller curvature ("flatter") will be
         * stored here. */
        std::string output_layer_smaller_curvature;

        /** If non-empty, points that do no fall in any of the two categories
         * are stored here. */
        std::string output_layer_other;

        float max_cosine    = 0.5f;
        float min_clearance = 0.02f;
        float max_gap       = 1.00f;
    };

    /** Algorithm parameters */
    Parameters params_;
};

/** @} */

}  // namespace mp2p_icp_filters
