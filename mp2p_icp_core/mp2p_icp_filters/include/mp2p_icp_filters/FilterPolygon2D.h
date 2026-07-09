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
 * @file   FilterPolygon2D.h
 * @brief  Leaves or removes the points inside an arbitrary 2D polygon footprint
 * @author Jose Luis Blanco Claraco
 * @date   May 26, 2026
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/math/TPolygon2D.h>

#include <limits>

namespace mp2p_icp_filters
{
/** Split a point cloud into those points inside and outside an arbitrary 2D
 * polygon, defined in the XY plane. A point is "inside" when its (x, y)
 * projection lies within the polygon and its `z` coordinate is within the
 * (optional) range [`z_min`, `z_max`].
 *
 * Optionally, you can keep only one of those two clouds by leaving the unused
 * one undefined in your YAML file (or as an empty string).
 *
 * Unlike FilterBoundingBox, the cropping region is a general (possibly
 * non-convex) polygon, which is useful to match footprints that are not
 * axis-aligned rectangles.
 *
 * Example YAML configuration block:
 * \code
 * - class_name: mp2p_icp_filters::FilterPolygon2D
 *   params:
 *     input_pointcloud_layer: 'raw'
 *     # Keep at least one of these two output layers:
 *     inside_pointcloud_layer: 'inside'
 *     outside_pointcloud_layer: 'outside'
 *     # The polygon vertices, in the XY plane (map frame):
 *     polygon: [[0.0, 0.0], [10.0, 0.0], [10.0, 5.0], [0.0, 5.0]]
 *     # Optional Z crop (defaults to +/- infinity):
 *     z_min: 0.3
 *     z_max: 2.0
 * \endcode
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterPolygon2D : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterPolygon2D, mp2p_icp_filters)
   public:
    FilterPolygon2D();

    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterPolygon2D& parent);

        std::string input_pointcloud_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;

        /** The output point cloud layer name for points INSIDE the polygon */
        std::string inside_pointcloud_layer;

        /** The output point cloud layer name for points OUTSIDE the polygon */
        std::string outside_pointcloud_layer;

        /** Polygon vertices in the XY plane (map frame). At least 3 are
         * required. */
        mrpt::math::TPolygon2D polygon;

        /** Z range kept for points inside the polygon footprint. Defaults keep
         * all heights. */
        double z_min = -std::numeric_limits<double>::infinity();
        double z_max = std::numeric_limits<double>::infinity();
    };

    /** Algorithm parameters */
    Parameters params;
};

/** @} */

}  // namespace mp2p_icp_filters
