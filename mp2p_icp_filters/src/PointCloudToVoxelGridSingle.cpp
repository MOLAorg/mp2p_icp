/* -------------------------------------------------------------------------
 * A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   PointCloudToVoxelGridSingle.cpp
 * @brief  Makes an index of a point cloud using a voxel grid.
 * @author Jose Luis Blanco Claraco
 * @date   Dec 17, 2018
 */

#include <mp2p_icp_filters/PointCloudToVoxelGridSingle.h>
// Used in the PIMP:
#include <tsl/robin_map.h>

using namespace mp2p_icp_filters;

struct PointCloudToVoxelGridSingle::Impl
{
    tsl::robin_map<indices_t, voxel_t, IndicesHash> pts_voxels;
};

PointCloudToVoxelGridSingle::PointCloudToVoxelGridSingle()
    : impl_(mrpt::make_impl<Impl>())
{
}

void PointCloudToVoxelGridSingle::setResolution(const float voxel_size)
{
    MRPT_START

    impl_->pts_voxels.clear();
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

    auto& pts_voxels = impl_->pts_voxels;

    pts_voxels.reserve(pts_voxels.size() + npts);

    for (std::size_t i = 0; i < npts; i++)
    {
        const auto x = xs[i];
        const auto y = ys[i];
        const auto z = zs[i];

        const indices_t vxl_idx = {coord2idx(x), coord2idx(y), coord2idx(z)};

        auto itVoxel = pts_voxels.find(vxl_idx);

        if (itVoxel != pts_voxels.end())
        {
            // (const cast: required for tsl::robin_map)
            auto& vx = const_cast<voxel_t&>(itVoxel->second);

            if (vx.pointCount == 0)
                vx = {mrpt::math::TPoint3Df(x, y, z), i, &p, 1};
            else
                vx.pointCount++;
        }
        else
        {
            // insert new
            pts_voxels[vxl_idx] = {mrpt::math::TPoint3Df(x, y, z), i, &p, 1};
        }
    }
}

void PointCloudToVoxelGridSingle::clear()
{
    //
    impl_->pts_voxels.min_load_factor(0.01f);
    impl_->pts_voxels.clear();
}

void PointCloudToVoxelGridSingle::visit_voxels(
    const std::function<void(const indices_t idx, const voxel_t& vxl)>&
        userCode) const
{
    for (const auto& [idx, vxl] : impl_->pts_voxels) userCode(idx, vxl);
}

size_t PointCloudToVoxelGridSingle::size() const
{
    return impl_->pts_voxels.size();
}
