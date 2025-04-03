/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterPoleDetector.h
 * @brief  Leaves or removes points that seem to belong to vertical structures
 * @author Jose Luis Blanco Claraco
 * @date   Jan 27, 2025
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

namespace mp2p_icp_filters
{
/** Filter points according to whether they seem to belong to a pole or vertical
 *  structure.
 *
 * There are two (optional) output target layers, one for the points that *are*
 * poles (`output_layer_poles`) and another for those
 * that are not (`output_layer_no_poles`). At least one must be provided.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterPoleDetector : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterPoleDetector, mp2p_icp_filters)
   public:
    FilterPoleDetector();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterPoleDetector& parent);

        std::string input_pointcloud_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;

        /** Optional output layer name for points that **are** poles */
        std::string output_layer_poles;

        /** Optional output layer name for points that **are not** poles */
        std::string output_layer_no_poles;

        float grid_size = 2.0f;

        float minimum_relative_height = 2.5f;
        float maximum_relative_height = 25.0f;

        uint32_t minimum_pole_points              = 5;
        uint32_t minimum_neighbors_checks_to_pass = 3;
    };

    /** Algorithm parameters */
    Parameters params_;
};

/** @} */

}  // namespace mp2p_icp_filters
