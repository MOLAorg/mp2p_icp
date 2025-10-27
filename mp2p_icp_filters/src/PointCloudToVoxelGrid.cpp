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
 * @file   PointCloudToVoxelGrid.cpp
 * @brief  Makes an index of a point cloud using a voxel grid.
 * @author Jose Luis Blanco Claraco
 * @date   Dec 17, 2018
 */

#include <mp2p_icp_filters/PointCloudToVoxelGrid.h>

// Used in the PIMP:
#include <tsl/robin_map.h>

#include <map>

using namespace mp2p_icp_filters;

struct PointCloudToVoxelGrid::Impl
{
    tsl::robin_map<indices_t, voxel_t, IndicesHash> pts_voxels;
    std::map<indices_t, voxel_t, IndicesHash>       pts_voxels_std_map;
};

PointCloudToVoxelGrid::PointCloudToVoxelGrid() : impl_(mrpt::make_impl<Impl>()) {}

void PointCloudToVoxelGrid::setConfiguration(const float voxel_size, bool use_tsl_robin_map)
{
    MRPT_START

    resolution_        = voxel_size;
    use_tsl_robin_map_ = use_tsl_robin_map;

    this->clear();

    MRPT_END
}

void PointCloudToVoxelGrid::processPointCloud(
    const mrpt::maps::CPointsMap& p, const std::size_t first_pt_idx,
    const std::size_t points_to_process)
{
    using mrpt::max3;
    using std::abs;

    const auto& xs = p.getPointsBufferRef_x();
    const auto& ys = p.getPointsBufferRef_y();
    const auto& zs = p.getPointsBufferRef_z();

    const auto last_pt_idx = points_to_process ? (first_pt_idx + points_to_process) : xs.size();

    // Duplicated code in the two if() branches for low-level efficiency.
    if (use_tsl_robin_map_)
    {
        auto& pts_voxels = impl_->pts_voxels;
        // Previous point:
        pts_voxels.reserve(pts_voxels.size() + last_pt_idx - first_pt_idx);

        for (std::size_t i = first_pt_idx; i < last_pt_idx; i++)
        {
            const indices_t vxl_idx = {coord2idx(xs[i]), coord2idx(ys[i]), coord2idx(zs[i])};

            auto& cell = pts_voxels[vxl_idx];
            cell.indices.push_back(i);
        }
    }
    else
    {
        auto& pts_voxels = impl_->pts_voxels_std_map;

        for (std::size_t i = first_pt_idx; i < last_pt_idx; i++)
        {
            const indices_t vxl_idx = {coord2idx(xs[i]), coord2idx(ys[i]), coord2idx(zs[i])};

            auto& cell = pts_voxels[vxl_idx];
            cell.indices.push_back(i);
        }
    }
}

void PointCloudToVoxelGrid::clear()
{
    if (use_tsl_robin_map_)
    {
        impl_->pts_voxels.clear();
    }
    else
    {
        impl_->pts_voxels_std_map.clear();
    }
}

void PointCloudToVoxelGrid::visit_voxels(
    const std::function<void(const indices_t idx, const voxel_t& vxl)>& userCode) const
{
    if (use_tsl_robin_map_)
    {
        for (const auto& [idx, vxl] : impl_->pts_voxels)
        {
            userCode(idx, vxl);
        }
    }
    else
    {
        for (const auto& [idx, vxl] : impl_->pts_voxels_std_map)
        {
            userCode(idx, vxl);
        }
    }
}

size_t PointCloudToVoxelGrid::size() const
{
    return use_tsl_robin_map_ ? impl_->pts_voxels.size() : impl_->pts_voxels_std_map.size();
}
