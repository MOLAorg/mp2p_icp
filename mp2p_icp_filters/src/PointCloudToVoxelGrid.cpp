/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
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

void PointCloudToVoxelGrid::setResolution(const float voxel_size)
{
    MRPT_START

    pts_voxels.clear();
    resolution_ = voxel_size;

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

        const indices_t vxl_idx = {x2idx(x0), y2idx(y0), z2idx(z0)};

        auto& cell = pts_voxels[vxl_idx];
        cell.indices.push_back(i);  // only if not out of grid range
    }
}

void PointCloudToVoxelGrid::clear()
{
    //
    pts_voxels.clear();
}
