/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   PointCloudToVoxelGrid.h
 * @brief  Makes an index of a point cloud using a voxel grid.
 * @author Jose Luis Blanco Claraco
 * @date   Dec 17, 2018
 */

#pragma once

#include <mrpt/containers/CDynamicGrid3D.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TPoint3D.h>

#include <vector>

/** \ingroup mp2p_icp_filters_grp */
namespace mp2p_icp_filters
{
/** Auxiliary data structure: an index of points in a point cloud, organized by
 *  their 3D position according to a predefined regular-sized voxel grid.
 * \ingroup mp2p_icp_filters_grp
 */
class PointCloudToVoxelGrid
{
   public:
    PointCloudToVoxelGrid() = default;

    void resize(
        const mrpt::math::TPoint3D& min_corner,
        const mrpt::math::TPoint3D& max_corner, const float voxel_size);
    void processPointCloud(const mrpt::maps::CPointsMap& p);
    void clear();

    struct Parameters
    {
        /** Minimum distance (infinity norm) between **consecutive** points to
         * be accepted in a voxel. By looking at points in order, this allows
         * for a very fast discrimination of too-close consecutive points
         * without the need to query any KD-tree.
         *
         * (Default=0, i.e. disabled).
         */
        float min_consecutive_distance{.0f};
    };

    Parameters params_;

    /** The list of point indices in each voxel */
    struct voxel_t
    {
        std::vector<std::size_t> indices;
        bool                     is_empty{true};
    };
    using grid_t = mrpt::containers::CDynamicGrid3D<voxel_t, float>;

    /** The point indices in each voxel. Directly access to each desired cell,
     * use its iterator, etc. */
    grid_t pts_voxels;

    std::vector<uint32_t> used_voxel_indices;
};

}  // namespace mp2p_icp_filters
