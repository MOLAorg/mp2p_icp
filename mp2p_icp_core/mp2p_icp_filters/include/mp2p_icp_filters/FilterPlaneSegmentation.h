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
 * @file   FilterPlaneSegmentation.h
 * @brief  Segments the large planes of a point cloud or map
 * @author Jose Luis Blanco Claraco
 * @date   Nov 25, 2026
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/plane_segmentation.h>

namespace mp2p_icp_filters
{
/** Segments the large planar surfaces (ground, walls, facades...) of a point
 *  layer with segment_planes(): region growing on voxel normals, conservative
 *  merging, and robust refinement. Meant for whole maps (1e6 to 1e7 points)
 *  as well as single scans; see PlaneSegmentationParams for the parameters.
 *
 * Outputs, all optional:
 * - `output_layer_planes`: the inliers of all planes, with all the input
 *   point fields.
 * - `output_layer_others`: the remaining points.
 * - `plane_id_field`: a `uint16` per-point field with the plane of each point:
 *   0 for none, `k` for the k-th plane (by decreasing number of points).
 *   It is written to `output_layer_planes` if that is set, otherwise to the
 *   input layer itself, which must then support extra fields
 *   (e.g. mrpt::maps::CGenericPointsMap).
 * - `store_planes`: append each plane, with its area and number of points, to
 *   `metric_map_t::planes`.
 *
 * Output layers are created if they do not exist (as
 * mrpt::maps::CGenericPointsMap when they must hold the plane id field, or as
 * the input class otherwise), or appended to.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterPlaneSegmentation : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterPlaneSegmentation, mp2p_icp_filters)
   public:
    FilterPlaneSegmentation();

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters : public PlaneSegmentationParams
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterPlaneSegmentation& parent);

        std::string input_pointcloud_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;

        /** Optional output layer for the points of all planes */
        std::string output_layer_planes;

        /** Optional output layer for the points not on any plane */
        std::string output_layer_others;

        /** Optional name of the per-point plane id field (uint16) */
        std::string plane_id_field;

        /** Append the planes to `metric_map_t::planes` */
        bool store_planes = true;

        /** Empty `metric_map_t::planes` first (only if `store_planes`) */
        bool clear_previous_planes = true;
    };

    /** Algorithm parameters */
    Parameters params;

   protected:
    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;
};

/** @} */

}  // namespace mp2p_icp_filters
