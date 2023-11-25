/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   PointCloudToVoxelGridSingle.cpp
 * @brief  Makes an index of a point cloud using a voxel grid.
 * @author Jose Luis Blanco Claraco
 * @date   Dec 17, 2018
 */

#include <mp2p_icp_filters/PointCloudToVoxelGridSingle.h>

using namespace mp2p_icp_filters;

void PointCloudToVoxelGridSingle::setResolution(const float voxel_size)
{
    MRPT_START

    pts_voxels.clear();
    resolution_ = voxel_size;

    MRPT_END
}

void PointCloudToVoxelGridSingle::processPointCloud(
    const mrpt::maps::CPointsMap& p)
{
    using mrpt::max3;
    using std::abs;

    const auto& xs   = p.getPointsBufferRef_x();
    const auto& ys   = p.getPointsBufferRef_y();
    const auto& zs   = p.getPointsBufferRef_z();
    const auto  npts = xs.size();

    for (std::size_t i = 0; i < npts; i++)
    {
        const auto x = xs[i];
        const auto y = ys[i];
        const auto z = zs[i];

        const indices_t vxl_idx = {coord2idx(x), coord2idx(y), coord2idx(z)};

        auto itVoxel = pts_voxels.find(vxl_idx);
        if (itVoxel != pts_voxels.end())
        {
            itVoxel->second.pointCount++;
            continue;  // already existed, do nothing
        }
        else
        {
            pts_voxels[vxl_idx] = {i, 0};  // insert new
        }
    }
}

void PointCloudToVoxelGridSingle::clear()
{
    //
    pts_voxels.clear();
}
