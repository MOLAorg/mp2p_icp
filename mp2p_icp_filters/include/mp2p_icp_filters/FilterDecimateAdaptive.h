/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterDecimateAdaptive.h
 * @brief  An adaptive sampler of pointclouds
 * @author Jose Luis Blanco Claraco
 * @date   Nov 24, 2023
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>
#include <mp2p_icp_filters/PointCloudToVoxelGrid.h>
#include <mrpt/maps/CPointsMap.h>

namespace mp2p_icp_filters
{
/** Accepts as input a point cloud layer, voxelizes it, and generates a new
 * point cloud layer with an adaptive sampling.
 *
 * Not compatible with calling from different threads simultaneously for
 * different input point clouds. Use independent instances for each thread if
 * needed.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterDecimateAdaptive : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterDecimateAdaptive, mp2p_icp_filters)
   public:
    FilterDecimateAdaptive();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c);

        bool enabled = true;

        std::string input_pointcloud_layer =
            mp2p_icp::metric_map_t::PT_LAYER_RAW;

        std::string output_pointcloud_layer;

        unsigned int desired_output_point_count = 1000;

        /** Voxels with less points will not generate anything at the output
         * layer */
        unsigned int minimum_input_points_per_voxel = 1;

        // These are used to automatically estimate the voxel size:
        double       assumed_minimum_pointcloud_bbox   = 10.0;  // [m]
        unsigned int maximum_voxel_count_per_dimension = 100;
    };

    /** Algorithm parameters */
    Parameters params_;

   private:
    mutable PointCloudToVoxelGrid filter_grid_;
};

/** @} */

}  // namespace mp2p_icp_filters
