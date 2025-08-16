/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FilterRemoveByVoxelOccupancy.h
 * @brief  Removes points from an input point cloud layer by occupancy of
 * another input voxel layer. Can be used to remove dynamic objects.
 * @author Jose Luis Blanco Claraco
 * @date   May 28, 2024
 */

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mp2p_icp_filters/FilterBase.h>

namespace mp2p_icp_filters
{
/** Removes points from an input point cloud layer by occupancy of
 * another input voxel layer. Can be used to remove dynamic objects.
 * Both input layers are assumed to be in the same frame of reference.
 *
 * There are two output layer (you can define one or both):
 *
 * - `output_layer_static_objects`: for those points within voxels with high
 * occupancy ("static objects").
 * - `output_layer_dynamic_objects`: for those points within voxels with low
 * occupancy
 * ("dynamic objects").
 *
 * The input layers are:
 *
 * - `input_pointcloud_layer`: The input cloud.
 * - `input_voxel_layer`: It must be of type mrpt::maps::CVoxelMap and contains
 * the occupancy of each volume of the map.
 *
 * \ingroup mp2p_icp_filters_grp
 */
class FilterRemoveByVoxelOccupancy : public mp2p_icp_filters::FilterBase
{
    DEFINE_MRPT_OBJECT(FilterRemoveByVoxelOccupancy, mp2p_icp_filters)
   public:
    FilterRemoveByVoxelOccupancy();

    // See docs in base class.
    void initialize(const mrpt::containers::yaml& c) override;

    // See docs in FilterBase
    void filter(mp2p_icp::metric_map_t& inOut) const override;

    struct Parameters
    {
        void load_from_yaml(const mrpt::containers::yaml& c, FilterRemoveByVoxelOccupancy& parent);

        std::string input_pointcloud_layer;
        std::string input_voxel_layer;
        std::string output_layer_static_objects;
        std::string output_layer_dynamic_objects;
        double      occupancy_threshold = 0.4;
    };

    /** Algorithm parameters */
    Parameters params_;
};

/** @} */

}  // namespace mp2p_icp_filters
