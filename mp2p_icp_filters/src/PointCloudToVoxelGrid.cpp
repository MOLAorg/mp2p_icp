/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   PointCloudToVoxelGrid.cpp
 * @brief  Makes an index of a point cloud using a voxel grid.
 * @author Jose Luis Blanco Claraco
 * @date   Dec 17, 2018
 */

/** \defgroup mp2p_icp_filters_grp mp2p_icp_filters library
 * 2D/3D LiDAR scan segmentation algorithms and utilities.
 *
 */

#include <mp2p_icp_filters/PointCloudToVoxelGrid.h>

using namespace mp2p_icp_filters;

void PointCloudToVoxelGrid::resize(
    const mrpt::math::TPoint3D& min_corner,
    const mrpt::math::TPoint3D& max_corner, const float voxel_size)
{
    MRPT_START

    pts_voxels.clear();
    pts_voxels.setSize(
        min_corner.x, max_corner.x, min_corner.y, max_corner.y, min_corner.z,
        max_corner.z, voxel_size, voxel_size);

    used_voxel_indices.clear();
    used_voxel_indices.reserve(pts_voxels.getVoxelCount() / 4);

    MRPT_END
}

void PointCloudToVoxelGrid::processPointCloud(const mrpt::maps::CPointsMap& p)
{
    using mrpt::max3;
    using std::abs;

    const auto& xs   = p.getPointsBufferRef_x();
    const auto& ys   = p.getPointsBufferRef_y();
    const auto& zs   = p.getPointsBufferRef_z();
    const auto  npts = xs.size();

    // Previous point:
    float x0, y0, z0;
    x0 = y0 = z0 = std::numeric_limits<float>::max();

    for (std::size_t i = 0; i < npts; i++)
    {
        // Skip this point?
        if (params_.min_consecutive_distance != .0f &&
            max3(abs(x0 - xs[i]), abs(y0 - ys[i]), abs(z0 - zs[i])) <
                params_.min_consecutive_distance)
            continue;

        // Save for the next point:
        x0 = xs[i];
        y0 = ys[i];
        z0 = zs[i];

        const auto cx      = pts_voxels.x2idx(x0);
        const auto cy      = pts_voxels.y2idx(y0);
        const auto cz      = pts_voxels.z2idx(z0);
        const auto vxl_idx = pts_voxels.cellAbsIndexFromCXCYCZ(cx, cy, cz);
        if (vxl_idx == grid_t::INVALID_VOXEL_IDX) continue;

        auto* c = pts_voxels.cellByIndex(vxl_idx);
        if (!c) continue;
        c->indices.push_back(i);  // only if not out of grid range

        if (c->is_empty)
        {
            c->is_empty = false;
            used_voxel_indices.push_back(vxl_idx);
        }
    }
}

void PointCloudToVoxelGrid::clear()
{
    for (auto idx : used_voxel_indices)
    {
        auto c = pts_voxels.cellByIndex(idx);
        c->indices.clear();
        c->is_empty = true;
    }

    used_voxel_indices.clear();
}
