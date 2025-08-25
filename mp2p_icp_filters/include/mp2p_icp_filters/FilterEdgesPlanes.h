/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   FilterEdgesPlanes.h
 * @brief  Classify pointcloud voxels into planes / "edges".
 * @author Jose Luis Blanco Claraco
 * @date   Jun 10, 2019
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/PointCloudToVoxelGrid.h>
#include <mrpt/maps/CPointsMap.h>

namespace mp2p_icp_filters
{
/** Accepts as input a point cloud, and classifies points into edges and planes.
 *
 * Based on the LOAM paper (Zhang, Ji, and Sanjiv Singh. "LOAM: Lidar odometry
 * and mapping in real-time." Robotics: Science and systems. Vol. 2. No. 9.
 * 2014.).
 *
 * Not compatible with calling from different threads simultaneously for
 * different input point clouds. Use independent instances for each thread if
 * needed.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterEdgesPlanes : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterEdgesPlanes, mp2p_icp_filters)
   public:
    FilterEdgesPlanes();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        std::string input_pointcloud_layer = mp2p_icp::metric_map_t::PT_LAYER_RAW;

        unsigned int full_pointcloud_decimation = 20;

        /** Size of each voxel edge [meters] */
        float voxel_filter_resolution = .5f;  // [m]
        bool  use_tsl_robin_map       = true;

        unsigned int voxel_filter_decimation = 1;
        float        voxel_filter_max_e2_e0  = 30.f;
        float        voxel_filter_max_e1_e0  = 30.f;
        float        voxel_filter_min_e2_e0  = 100.f;
        float        voxel_filter_min_e1_e0  = 100.f;
        float        voxel_filter_min_e1     = .0f;
    };

    /** Algorithm parameters */
    Parameters params_;

   private:
    mutable PointCloudToVoxelGrid filter_grid_;
};

/** @} */

}  // namespace mp2p_icp_filters
