/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/containers/CDynamicGrid3D.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mp2p_icp
{
/** Index of points in a point cloud by their 3D position according to a
 * predefined regular-sized voxel grid.
 *
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
        /** Minimum distance (infinity norm) between consecutive points to be
         * accepted in a voxel. */
        float min_consecutive_distance{.0f};
    };

    Parameters params_;

    /** The list of point indices in each voxel */
    struct voxel_t
    {
        std::vector<std::size_t> indices;
    };
    using grid_t = mrpt::containers::CDynamicGrid3D<voxel_t, float>;

    /** The point indices in each voxel. Directly access to each desired cell,
     * use its iterator, etc. */
    grid_t pts_voxels;

    std::vector<uint32_t> used_voxel_indices;

   protected:
    std::vector<bool> voxel_is_empty_;
};

}  // namespace mp2p_icp
