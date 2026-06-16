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
 * @file   FilterDecimateRangeAdaptive.h
 * @brief  EllipseLIO-style range-adaptive voxel decimation (arXiv:2605.21150).
 * @author Jose Luis Blanco Claraco
 * @date   Jun 2026
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/PointCloudToVoxelGrid.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/maps/CPointsMap.h>

namespace mp2p_icp_filters
{
/** Range-adaptive voxel decimation following EllipseLIO (arXiv:2605.21150, Eqs. 1-3).
 *
 * The input cloud is split into configurable-width radial bins (see
 * `bin_width`). Within bin `i` (points in [i*bin_width, (i+1)*bin_width) m),
 * the voxel size is set to the scan-line separation at that
 * range:
 *
 *   v_i = (i+1) * vertical_fov_rad / (num_scan_lines - 1)
 *
 * clamped to [min_voxel_size, max_voxel_size]. Each bin is independently
 * voxelised (first-point per voxel), and the filtered clouds are unioned.
 *
 * When vertical_fov_rad or num_scan_lines are zero, they are auto-derived
 * from the input cloud's ring channel (if present).
 *
 * Not compatible with calling from different threads simultaneously for
 * different input point clouds. Use independent instances for each thread if
 * needed.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterDecimateRangeAdaptive : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterDecimateRangeAdaptive, mp2p_icp_filters)
   public:
    FilterDecimateRangeAdaptive();

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        std::string input_pointcloud_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;
        std::string output_pointcloud_layer;

        /** LiDAR vertical field-of-view (rad). If 0, auto-derived from ring
         *  channel elevation spread. */
        double vertical_fov_rad = 0;

        /** Number of scan lines (rings). If 0, auto-derived from max ring id
         *  in the input cloud's ring channel. */
        unsigned int num_scan_lines = 0;

        /** Radial bin width [m]. */
        double bin_width = 1.0;

        /** Maximum range [m]. 0 means use the farthest point in the cloud. */
        double max_range = 0;

        /** Minimum voxel size clamp [m] to avoid degenerate voxels near the
         *  sensor. */
        double min_voxel_size = 0.05;

        /** Maximum voxel size clamp [m]. */
        double max_voxel_size = 2.0;

        /** Minimum number of points that a voxel must contain to produce an
         *  output point. */
        unsigned int min_input_points_per_voxel = 1;

        /** Grain size for TBB parallelization across bins. */
        size_t parallelization_grain_size = 16UL * 1024UL;
    };

    /** Algorithm parameters */
    Parameters params;

   protected:
    // See docs in base class.
    void initialize_filter(const mrpt::containers::yaml& c) override;

   private:
    struct Impl;
    mutable mrpt::pimpl<Impl> impl_;
};

/** @} */

}  // namespace mp2p_icp_filters
