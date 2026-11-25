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
 * @file   FilterBeamCorrection.h
 * @brief  Applies per-beam (per-ring) intrinsic corrections to LiDAR points.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 25, 2026
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mrpt/math/TPose3D.h>

#include <string>
#include <vector>

namespace mp2p_icp_filters
{
/** Corrects, in place, the points of a spinning LiDAR with a per-ring table
 *  of intrinsic corrections, e.g. the output of a calibration.
 *
 * Where FilterRangeBiasCorrection moves points along a surface normal by a
 * model of range and incidence shared by all beams, this filter corrects each
 * beam (ring) on its own ray, and needs no normals. Each point of ring \f$ k
 * \f$ is read, in the sensor frame, as a ray from the beam origin \f$ o \f$
 * with range \f$ \rho \f$, elevation \f$ e \f$ and azimuth \f$ a \f$, and
 * replaced by
 *
 * \f[ p' = o + v_k \hat{z} + h_k \hat{n}(a') + ((1 + s_k) \rho + r_k)
 *          \, \mathrm{dir}(e + \delta e_k, a'), \quad a' = a + \delta a_k \f]
 *
 * with \f$ \hat{n}(a) = (-\sin a, \cos a, 0) \f$. The six per-ring values are
 * the corrections to add to the raw ones (true minus reported):
 * `d_elevation_deg`, `d_azimuth_deg`, `d_range_m` (the 3-parameter model)
 * and, optionally, `d_range_scale`, `d_vertical_m`, `d_horizontal_m`. Missing
 * values are zero; points of rings without a row are left untouched.
 *
 * The beam origin is the sensor origin, unless `beam_origin_m` gives the
 * radius of an Ouster-style beam origin circle, \f$ o = b (\cos\theta,
 * \sin\theta, 0) \f$ at encoder angle \f$ \theta \f$ (recovered from the point
 * and, if given, the per-ring nominal `beam_azimuth_offsets_deg`).
 *
 * The table is given inline (`beams`) or in a YAML file (`beams_file`): a
 * file with a top-level `beams` sequence, or a calibration file whose
 * `sensors.<sensor>.intrinsics` hold `beams` (and optionally
 * `beam_origin_m`, `beam_azimuth_offset_deg`).
 *
 * YAML parameters:
 * \code
 * pointcloud_layer: 'raw'
 * # LiDAR pose in the cloud frame [x, y, z, yaw, pitch, roll] (m, rad):
 * sensor_pose: [0, 0, 0.3, 0, 0, 0]
 * beams:
 *   - {ring: 0, d_elevation_deg: 0.012, d_azimuth_deg: -0.004, d_range_m: 0.006}
 *   - {ring: 1, d_elevation_deg: -0.031, d_azimuth_deg: 0.002, d_range_m: -0.011}
 * # or:
 * # beams_file: 'calibration.yaml'
 * # sensor: 'lidar0'
 * \endcode
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterBeamCorrection : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterBeamCorrection, mp2p_icp_filters)
   public:
    FilterBeamCorrection();

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    /** Corrections of one ring, in SI units (radians, meters). */
    struct Beam
    {
        double d_elevation   = 0;
        double d_azimuth     = 0;
        double d_range       = 0;
        double d_range_scale = 0;
        double d_vertical    = 0;
        double d_horizontal  = 0;
        bool   defined       = false;  //!< the table has a row for this ring
    };

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterBeamCorrection& parent);

        /** The point cloud layer to correct, in place. */
        std::string pointcloud_layer;

        /** Name of the per-point ring field (uint16, or float). */
        std::string ring_field = "ring";

        /** Pose of the LiDAR in the frame of the point cloud: [x, y, z, yaw,
         * pitch, roll] in meters and radians, as in FilterTransformPointCloud.
         * Each entry may be an expression of the pipeline variables. */
        mrpt::math::TPose3D sensor_pose;

        /** Radius of the beam origin circle (Ouster-style sensors) [m]; 0: a
         * single origin. */
        double beam_origin_m = 0;

        /** Nominal azimuth of each ring minus the encoder angle [rad], used
         * with `beam_origin_m` to find the encoder angle (empty: zeros). */
        std::vector<double> beam_azimuth_offsets;

        /** Per-ring corrections, indexed by ring. */
        std::vector<Beam> beams;

        /** Points closer than this to their beam origin are left as they are [m]. */
        double min_range = 0.01;

        /** When TBB is enabled, the grainsize for parallel processing. */
        size_t parallelization_grain_size = 4096UL;
    };

    /** Algorithm parameters */
    Parameters params;

    /** Corrects one point given in the sensor frame, with the corrections of
     * its ring (exposed for testing and reuse). */
    [[nodiscard]] mrpt::math::TPoint3D correctPoint(
        const mrpt::math::TPoint3D& p_sensor, std::size_t ring) const;

   protected:
    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;
};

/** @} */

}  // namespace mp2p_icp_filters
