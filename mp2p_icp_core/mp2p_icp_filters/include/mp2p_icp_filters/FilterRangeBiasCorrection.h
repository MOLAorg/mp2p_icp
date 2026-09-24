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
 * @file   FilterRangeBiasCorrection.h
 * @brief  Removes a range- and incidence-dependent LiDAR surface bias.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 24, 2026
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

#include <array>

namespace mp2p_icp_filters
{
/** Corrects, in place, a systematic surface offset of LiDAR returns that
 *  depends on range and incidence angle.
 *
 * Some sensors measure surfaces slightly too near or too far, by an amount
 * that grows with range and with the obliquity of the surface. This filter
 * models that offset, measured along the surface normal, as:
 *
 * \f[ d = a + b \, r + c \, \sin^2 \theta \f]
 *
 * with \f$ r \f$ the range from `sensor_origin` and \f$ \theta \f$ the angle
 * between the viewing ray and the local surface normal. Each point is moved
 * by \f$ -d \f$ along the normal oriented away from the sensor, so a negative
 * \f$ d \f$ (surface measured too near) pushes the point outwards.
 *
 * Normals come from the \f$ k \f$ nearest neighbors in the same cloud. Points
 * whose neighborhood is not a well-defined plane are left untouched. The
 * planar ones are classified as ground, slanted or wall by the vertical
 * component of their normal in the cloud frame, and each class has its own
 * \f$ (a, b, c) \f$.
 *
 * The coefficients are sensor-specific and must be calibrated; with all of
 * them zero (the default) the filter does nothing.
 *
 * YAML parameters:
 * \code
 * pointcloud_layer: "deskewed"
 * sensor_origin: [0, 0, 0.124]  # sensor position in the cloud frame
 * ground: [0.0116, -0.00123, -0.0085]  # a [m], b [m/m], c [m]
 * slanted: [0.0007, -0.00058, -0.0110]
 * wall: [0.0044, -0.00052, -0.0172]
 * \endcode
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterRangeBiasCorrection : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterRangeBiasCorrection, mp2p_icp_filters)
   public:
    FilterRangeBiasCorrection();

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterRangeBiasCorrection& parent);

        /** The point cloud layer to correct, in place. */
        std::string pointcloud_layer;

        /** Sensor position in the frame of the point cloud. Can be defined as
         * a function of the robot pose variables, as in FilterByRange. */
        mrpt::math::TPoint3Df sensor_origin = {0, 0, 0};

        /** Model coefficients (a [m], b [m/m], c [m]) per surface class. */
        std::array<double, 3> ground  = {0, 0, 0};
        std::array<double, 3> slanted = {0, 0, 0};
        std::array<double, 3> wall    = {0, 0, 0};

        /** A normal with |n_z| above this is ground, below `wall_max_nz` a
         * wall, and anything in between slanted. */
        double ground_min_nz = 0.9;
        double wall_max_nz   = 0.3;

        /** Neighbors used to estimate each normal. */
        unsigned int k_neighbors = 16;

        /** Neighbors farther than this are ignored [m]. */
        double max_neighbor_distance = 1.0;

        /** Points with fewer usable neighbors are left untouched. */
        unsigned int min_neighbors = 6;

        /** The neighborhood is a plane only if its smallest covariance
         * eigenvalue is below this fraction of the middle one. */
        double max_planarity_ratio = 0.1;

        /** ...and it is not a line: the middle eigenvalue must be above this
         * fraction of the largest one. */
        double min_line_ratio = 0.05;

        /** Corrections are clamped to this magnitude [m]. */
        double max_correction = 0.05;

        /** When TBB is enabled, the grainsize for parallel processing. */
        size_t parallelization_grain_size = 1024UL;
    };

    /** Algorithm parameters */
    Parameters params;

   protected:
    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;
};

/** @} */

}  // namespace mp2p_icp_filters
