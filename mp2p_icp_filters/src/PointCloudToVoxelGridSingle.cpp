/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
/**
 * @file   PointCloudToVoxelGridSingle.cpp
 * @brief  Makes an index of a point cloud using a voxel grid.
 * @author Jose Luis Blanco Claraco
 * @date   Dec 17, 2018
 */

#include <mp2p_icp_filters/PointCloudToVoxelGridSingle.h>
// Used in the PIMP:
#include <tsl/robin_map.h>

#include <map>

using namespace mp2p_icp_filters;

struct PointCloudToVoxelGridSingle::Impl
{
    tsl::robin_map<indices_t, voxel_t, IndicesHash> pts_voxels;
    std::map<indices_t, voxel_t, IndicesHash>       pts_voxels_std_map;
};

PointCloudToVoxelGridSingle::PointCloudToVoxelGridSingle() : impl_(mrpt::make_impl<Impl>()) {}

void PointCloudToVoxelGridSingle::setConfiguration(const float voxel_size, bool use_tsl_robin_map)
{
    MRPT_START

    resolution_        = voxel_size;
    use_tsl_robin_map_ = use_tsl_robin_map;

    this->clear();

    MRPT_END
}

void PointCloudToVoxelGridSingle::processPointCloud(
    const mrpt::maps::CPointsMap& p, const std::size_t first_pt_idx,
    const std::size_t points_to_process)
{
    using mrpt::max3;
    using std::abs;

    const auto& xs = p.getPointsBufferRef_x();
    const auto& ys = p.getPointsBufferRef_y();
    const auto& zs = p.getPointsBufferRef_z();

    const auto last_pt_idx = points_to_process ? (first_pt_idx + points_to_process) : xs.size();

    const auto lambda_process = [&](auto& pts_voxels)
    {
        for (std::size_t i = first_pt_idx; i < last_pt_idx; i++)
        {
            const auto x = xs[i];
            const auto y = ys[i];
            const auto z = zs[i];

            const indices_t vxl_idx = {coord2idx(x), coord2idx(y), coord2idx(z)};

            // try_emplace: single hash lookup for both insert and existing-key cases
            auto [it, inserted] =
                pts_voxels.try_emplace(vxl_idx, mrpt::math::TPoint3Df(x, y, z), i, &p, 1);

            if (!inserted)
            {
                auto& vx = const_cast<voxel_t&>(it->second);

                if (vx.pointCount == 0)
                {
                    vx = {mrpt::math::TPoint3Df(x, y, z), i, &p, 1};
                }
                else
                {
                    vx.pointCount++;
                }
            }
        }
    };

    if (use_tsl_robin_map_)
    {
        // No pre-reservation: reserving for all input points (last_pt_idx -
        // first_pt_idx) is a gross over-estimate when voxels are large relative
        // to point spacing — e.g. 100k points at 0.5m voxels yields ~10k unique
        // voxels, so reserving 100k wastes ~10x RAM upfront. Alternatives:
        //   a) reserve(N / expected_pts_per_voxel) — needs a hint parameter.
        //   b) reserve(N) — correct worst-case but costly for big clouds.
        //   c) no reserve — O(log N) rehashes, fine for single-pass use.
        // We choose (c): let robin_map grow on demand.
        lambda_process(impl_->pts_voxels);
    }
    else
    {
        lambda_process(impl_->pts_voxels_std_map);
    }
}

void PointCloudToVoxelGridSingle::clear()
{
    if (use_tsl_robin_map_)
    {
        impl_->pts_voxels.min_load_factor(0.01f);
        impl_->pts_voxels.clear();
    }
    else
    {
        impl_->pts_voxels_std_map.clear();
    }
}

void PointCloudToVoxelGridSingle::visit_voxels(
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

size_t PointCloudToVoxelGridSingle::size() const
{
    return use_tsl_robin_map_ ? impl_->pts_voxels.size() : impl_->pts_voxels_std_map.size();
}
