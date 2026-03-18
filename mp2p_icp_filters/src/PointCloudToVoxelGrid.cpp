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

// Internal per-voxel bookkeeping: offset into flat_indices and final count.
// This is separate from the public voxel_t (which is a non-owning view) to
// keep the map value small and avoid exposing internals.
struct VoxelEntry
{
    uint32_t offset = 0;
    uint32_t count  = 0;
};

struct PointCloudToVoxelGrid::Impl
{
    tsl::robin_map<indices_t, VoxelEntry, IndicesHash> pts_voxels;
    std::map<indices_t, VoxelEntry, IndicesHash>       pts_voxels_std_map;

    /** Flat array of all point indices, partitioned by voxel.
     *  Each voxel's indices live at flat_indices[entry.offset .. entry.offset+entry.count).
     *  A single contiguous allocation replaces the previous per-voxel std::vector<size_t>,
     *  eliminating O(num_voxels) heap allocations and fragmentation.
     */
    std::vector<std::size_t> flat_indices;
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
    const auto& xs = p.getPointsBufferRef_x();
    const auto& ys = p.getPointsBufferRef_y();
    const auto& zs = p.getPointsBufferRef_z();

    const auto last_pt_idx = points_to_process ? (first_pt_idx + points_to_process) : xs.size();
    const auto n_pts       = last_pt_idx - first_pt_idx;

    // Two-pass flat-array construction: eliminates per-voxel heap allocations.
    //
    // Pass 1 — count how many points land in each voxel.
    // Pass 2 — scatter point indices into a single contiguous flat_indices
    //          array, using the per-voxel entry as a write cursor.
    //
    // Result: all indices live in one allocation; voxel_t exposes a
    // non-owning view (pointer + count) with zero additional malloc calls.

    auto pass = [&](auto& voxels)
    {
        auto& flat = impl_->flat_indices;

        // Pass 1: count
        for (std::size_t i = first_pt_idx; i < last_pt_idx; i++)
        {
            const indices_t key = {coord2idx(xs[i]), coord2idx(ys[i]), coord2idx(zs[i])};
            voxels[key].count++;
        }

        // Assign offsets via prefix sums and pre-size the flat array.
        const std::size_t base = flat.size();
        flat.resize(base + n_pts);

        // Use a lambda to get a mutable reference to the mapped value regardless
        // of whether the container is tsl::robin_map (uses it.value()) or
        // std::map (uses it->second).
        auto mutVal = [](auto& it) -> VoxelEntry&
        {
            if constexpr (requires { it.value(); })
            {
                return it.value();
            }
            else
            {
                return it->second;
            }
        };

        uint32_t next = static_cast<uint32_t>(base);
        for (auto it = voxels.begin(); it != voxels.end(); ++it)
        {
            mutVal(it).offset = next;
            next += mutVal(it).count;
            mutVal(it).count = 0;  // reset to use as fill cursor in pass 2
        }

        // Pass 2: scatter
        for (std::size_t i = first_pt_idx; i < last_pt_idx; i++)
        {
            const indices_t key   = {coord2idx(xs[i]), coord2idx(ys[i]), coord2idx(zs[i])};
            auto&           entry = voxels.at(key);
            flat[entry.offset + entry.count++] = i;
        }
    };

    if (use_tsl_robin_map_)
    {
        pass(impl_->pts_voxels);
    }
    else
    {
        pass(impl_->pts_voxels_std_map);
    }
}

void PointCloudToVoxelGrid::clear()
{
    if (use_tsl_robin_map_)
    {
        // Low min_load_factor lets clear() shrink the bucket array, freeing
        // peak memory rather than retaining it across filter invocations.
        impl_->pts_voxels.min_load_factor(0.01f);
        impl_->pts_voxels.clear();
    }
    else
    {
        impl_->pts_voxels_std_map.clear();
    }
    impl_->flat_indices.clear();
    impl_->flat_indices.shrink_to_fit();
}

void PointCloudToVoxelGrid::visit_voxels(
    const std::function<void(const indices_t idx, const voxel_t& vxl)>& userCode) const
{
    const std::size_t* flat_data = impl_->flat_indices.data();

    auto visit = [&](const auto& voxels)
    {
        for (const auto& [idx, entry] : voxels)
        {
            voxel_t view;
            view.begin_ptr = flat_data + entry.offset;
            view.count     = entry.count;
            userCode(idx, view);
        }
    };

    if (use_tsl_robin_map_)
    {
        visit(impl_->pts_voxels);
    }
    else
    {
        visit(impl_->pts_voxels_std_map);
    }
}

std::size_t PointCloudToVoxelGrid::size() const
{
    return use_tsl_robin_map_ ? impl_->pts_voxels.size() : impl_->pts_voxels_std_map.size();
}
