/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterBoundingBox.h
 * @brief  Leaves or removes the points in a given bounding box.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 21, 2021
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

namespace mp2p_icp_filters
{
/** Split a point cloud into those points inside and outside a given bounding
 * box. Optionally, you can only keep one of those two clouds, by leaving the
 * non-used one undefined in your YAML file (or as an empty string).
 *
 * Bounding box coordinates can contain the variables `robot_x`, `robot_y`,
 * `robot_z` for usage of robocentric coordinates if used for simplemap
 * filtering as part of a pipeline for the sm2mm application.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterBoundingBox : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterBoundingBox, mp2p_icp_filters)
   public:
    FilterBoundingBox();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterBoundingBox& parent);

        std::string input_pointcloud_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;

        /** The output point cloud layer name for points INSIDE the bbox */
        std::string inside_pointcloud_layer;

        /** The output point cloud layer name for points OUTSIDE the bbox */
        std::string outside_pointcloud_layer;

        /**
         * YAML loading format:
         * \code
         * bounding_box_min: [-10, -10, -5]
         * bounding_box_max: [ 10,  10,  5]
         * \endcode
         */
        mrpt::math::TBoundingBoxf bounding_box = {{-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f}};
    };

    /** Algorithm parameters */
    Parameters params_;
};

/** @} */

}  // namespace mp2p_icp_filters
