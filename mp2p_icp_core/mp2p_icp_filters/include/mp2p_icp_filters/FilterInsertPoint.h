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
 * @file   FilterInsertPoint.h
 * @brief  Appends one point, given by (possibly variable) coordinates, to a layer
 * @author Jose Luis Blanco Claraco
 * @date   Oct 23, 2026
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

namespace mp2p_icp_filters
{
/** Appends one point to a point cloud layer, creating it if it does not exist.
 *
 * The coordinates may be formulas of the pipeline variables, e.g. to collect the
 * keyframe positions while building a map with sm2mm (its `filters` run once per
 * handled observation, so this is one point per keyframe if only one sensor is
 * processed):
 *
 * \code
 * - class_name: mp2p_icp_filters::FilterInsertPoint
 *   params:
 *     target_layer: "trajectory"
 *     point: [robot_x, robot_y, robot_z]
 * \endcode
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterInsertPoint : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterInsertPoint, mp2p_icp_filters)
   public:
    FilterInsertPoint();

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterInsertPoint& parent);

        /** Layer to append the point to. It is created as a
         *  mrpt::maps::CSimplePointsMap if it does not exist, and must be of that class
         *  otherwise. */
        std::string target_layer;

        /** Point coordinates [x, y, z] */
        mrpt::math::TPoint3Df point = {0, 0, 0};
    };

    /** Algorithm parameters */
    Parameters params;

   protected:
    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;
};

/** @} */

}  // namespace mp2p_icp_filters
