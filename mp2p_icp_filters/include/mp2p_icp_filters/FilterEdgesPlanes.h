/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
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

        std::string input_pointcloud_layer =
            mp2p_icp::metric_map_t::PT_LAYER_RAW;

        unsigned int full_pointcloud_decimation = 20;

        /** Size of each voxel edge [meters] */
        double voxel_filter_resolution = .5;  // [m]

        unsigned int voxel_filter_decimation = 1;
        float        voxel_filter_max_e2_e0  = 30.f;
        float        voxel_filter_max_e1_e0  = 30.f;
        float        voxel_filter_min_e2_e0  = 100.f;
        float        voxel_filter_min_e1_e0  = 100.f;
        float        voxel_filter_min_e1     = .0f;

        float init_extension_min_x = -10.0f;
        float init_extension_min_y = -10.0f;
        float init_extension_min_z = -10.0f;
        float init_extension_max_x = 10.0f;
        float init_extension_max_y = 10.0f;
        float init_extension_max_z = 10.0f;
    };

    /** Algorithm parameters */
    Parameters params_;

   private:
    mutable PointCloudToVoxelGrid filter_grid_;
};

/** @} */

}  // namespace mp2p_icp_filters
