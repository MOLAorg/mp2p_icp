/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterByRange.h
 * @brief  Leaves or removes points by min/max range from the origin.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 14, 2023
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

namespace mp2p_icp_filters
{
/** Leaves (`keep_between`=true) or removes (`keep_between`=false)
 * the points with a range (distance from "center") in between a (min,max)
 * range. By default, "center" is at (0,0,0) in the point cloud frame of
 * reference.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterByRange : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterByRange, mp2p_icp_filters)
   public:
    FilterByRange();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(
            const mrpt::containers::yaml& c, FilterByRange& parent);

        std::string input_pointcloud_layer =
            mp2p_icp::metric_map_t::PT_LAYER_RAW;

        /** The output point cloud layer name */
        std::string output_pointcloud_layer;

        /** Optional output point cloud layer name for deleted points */
        std::string output_deleted_pointcloud_layer;

        /**
         * YAML loading format:
         * \code
         * range_min: 3.0
         * range_max: 90.0
         * \endcode
         */
        float range_min = 3.0f;
        float range_max = 90.0f;

        /** See explanation on top of FilterByRange */
        bool keep_between = true;

        /** Ranges are measured from this center point.
         * Can be defined as a function of the robot pose variables, e.g.:
         * \code
         * center: [0, 0, 0]
         * # or:
         * center: [robot_x, robot_y, robot_z]
         * \endcode
         */
        mrpt::math::TPoint3Df center = {0, 0, 0};
    };

    /** Algorithm parameters */
    Parameters params_;
};

/** @} */

}  // namespace mp2p_icp_filters
