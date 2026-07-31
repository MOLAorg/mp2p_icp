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
 * @file   FilterTransformPointCloud.h
 * @brief  Rigidly transforms a point cloud layer by a pose, or its inverse.
 * @author Jose Luis Blanco Claraco
 * @date   Jul 31, 2026
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/math/TPose3D.h>

namespace mp2p_icp_filters
{
/** Rigidly transforms an input point cloud layer into a different (new)
 * output layer, replacing each point \f$ p_i \f$ by \f$ p'_i = T \oplus p_i
 * \f$ (pose compounding operator), where \f$ T \f$ is `pose`, or its inverse
 * if `invert_pose` is `true`.
 *
 * Unlike FilterMerge, this does not insert into an existing metric map: it
 * always creates (or overwrites) a plain point-cloud output layer of the
 * same class as the input, making it composable with any other filter that
 * takes a point-cloud layer as input (e.g. FilterDecimateAdaptive).
 *
 * Typical use case: some algorithms (e.g. DLIO) voxelize incoming scans
 * *after* transforming them into the (approximately known, e.g. from a
 * motion-model prior) global/map frame, so voxel membership is anchored to
 * the map rather than to the vehicle's instantaneous local frame. This filter
 * lets a pipeline replicate that: transform local->global with `invert_pose:
 * false`, run the decimation filter, then transform back with `invert_pose:
 * true` and the *same* `pose` so the rest of the pipeline is unaffected.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterTransformPointCloud : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterTransformPointCloud, mp2p_icp_filters)
   public:
    FilterTransformPointCloud();

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterTransformPointCloud& parent);

        std::string input_pointcloud_layer;
        std::string output_pointcloud_layer;

        // clang-format off
        /** In the context of a mola_lidar_odometry pipeline, this is typically:
         * \code
         * pose: [robot_x, robot_y, robot_z, robot_yaw, robot_pitch, robot_roll]
         * \endcode
         * Each of the 6 YAML sequence entries is bound as-is into the
         * corresponding mrpt::math::TPose3D field, so angles must already be
         * in RADIANS (TPose3D's own convention) -- e.g. the `robot_yaw` /
         * `robot_pitch` / `robot_roll` dynamic variables above are, since
         * they come from mrpt::poses::CPose3D::yaw()/pitch()/roll().
         */
        // clang-format on
        mrpt::math::TPose3D pose;

        /** If `false` (default), each point is replaced by `pose \oplus p_i`.
         * If `true`, by `pose^{-1} \oplus p_i` instead.
         */
        bool invert_pose = false;
    };

    /** Algorithm parameters */
    Parameters params;

   protected:
    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;
};

/** @} */

}  // namespace mp2p_icp_filters
