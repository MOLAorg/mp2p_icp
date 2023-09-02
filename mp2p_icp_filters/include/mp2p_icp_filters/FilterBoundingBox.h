/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
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
/** Leaves (`keep_bbox_contents`=true) or removes (`keep_bbox_contents`=false)
 * the points in a given bounding box.
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
        void load_from_yaml(const mrpt::containers::yaml& c);

        std::string input_pointcloud_layer =
            mp2p_icp::metric_map_t::PT_LAYER_RAW;

        /** The output point cloud layer name */
        std::string output_pointcloud_layer;

        /**
         * YAML loading format:
         * \code
         * bounding_box_min: [-10, -10, -5]
         * bounding_box_max: [ 10,  10,  5]
         * \endcode
         */
        mrpt::math::TBoundingBoxf bounding_box = {
            {-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f}};

        bool keep_bbox_contents = true;
    };

    /** Algorithm parameters */
    Parameters params_;
};

/** @} */

}  // namespace mp2p_icp_filters
