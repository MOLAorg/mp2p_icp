/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterMerge.h
 * @brief  Takes an input point cloud layer and inserts it into another one of
 * arbitrary metric map type.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 12, 2024
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/math/TPose3D.h>

namespace mp2p_icp_filters
{
/** Takes an input point cloud or mrpt::maps::CVoxelMap layer and inserts it
 * into another one of arbitrary metric map type.
 *
 * Insertion is done by converting the input layer into an
 * mrpt::obs::CObservationPointCloud, then invoking the target layer's
 * mrpt::maps::CMetricMap::insertObservation()
 *
 * If the input was a mrpt::maps::CVoxelMap, it is first converted into
 * a point cloud by generating points for each occupied voxel using
 * mrpt::maps::CVoxelMap::getOccupiedVoxels().
 *
 * Coordinate frames are:
 * - If `input_layer_in_local_coordinates` is `false` (default): both, input and
 *   output layers are in the same frame of reference. Even though, the variable
 *   `robot_pose` is important in cases where ray-tracing is necesary, e.g.
 *   voxel or grid maps.
 * - If `input_layer_in_local_coordinates` is `true`: input is in the vehicle
 *   frame, output is in another global frame, in which the vehicle is at pose
 *   `robot_pose`.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterMerge : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterMerge, mp2p_icp_filters)
   public:
    FilterMerge();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterMerge& parent);

        std::string input_pointcloud_layer;
        std::string target_layer;

        /** See discussion above for FilterMerge */
        bool input_layer_in_local_coordinates = false;

        // clang-format off
        /** In the context of a sm2mm pipeline, this should be set to the expression:
         * \code
         * robot_pose: [robot_x, robot_y, robot_z, robot_yaw, robot_pitch, robot_roll]
         * \endcode
         */
        // clang-format on
        mrpt::math::TPose3D robot_pose;
    };

    /** Algorithm parameters */
    Parameters params_;
};

/** @} */

}  // namespace mp2p_icp_filters
