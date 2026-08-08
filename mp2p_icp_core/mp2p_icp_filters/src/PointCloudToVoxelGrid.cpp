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
#include <mrpt/core/exceptions.h>

// Used in the PIMP:
#include <tsl/robin_map.h>

#include <algorithm>
#include <limits>
#include <map>

using namespace mp2p_icp_filters;

/** Internal per-voxel bookkeeping. The point indices themselves live in one
 *  contiguous array shared by all voxels (see Impl::indices), so that adding a
 *  voxel does not require a heap allocation of its own.
 */
struct VoxelEntry
{
    /// Where this voxel's run starts within the flat index array.
    std::uint32_t offset = 0;

    /// How many indices of the run are already written.
    std::uint32_t filled = 0;

    /// Final length of the run, once the ongoing pass finishes.
    std::uint32_t count = 0;
};

struct PointCloudToVoxelGrid::Impl
{
    tsl::robin_map<indices_t, VoxelEntry, IndicesHash> pts_voxels;
    std::map<indices_t, VoxelEntry, IndicesHash>       pts_voxels_std_map;

    /// Flat storage for the point indices of every voxel, one contiguous run
    /// per voxel. Kept across calls (only its size is reset in clear()) to
    /// avoid reallocating it for each processed cloud.
    std::vector<std::uint32_t> indices;

    /// Scratch buffer used to rebuild `indices` when runs move around.
    std::vector<std::uint32_t> indices_aux;

    /// Sum of the `count` of all voxels, i.e. the size `indices` must have.
    std::uint32_t total_indices = 0;
};

namespace
{
using RobinMap = tsl::robin_map<
    PointCloudToVoxelGrid::indices_t, VoxelEntry, PointCloudToVoxelGrid::IndicesHash>;
using StdMap =
    std::map<PointCloudToVoxelGrid::indices_t, VoxelEntry, PointCloudToVoxelGrid::IndicesHash>;

// tsl::robin_map only exposes mutable values via value(), std::map via second:
VoxelEntry& valueRef(RobinMap::iterator& it) { return it.value(); }
VoxelEntry& valueRef(StdMap::iterator& it) { return it->second; }

/** Assigns every voxel a run of `count` entries in a freshly laid out index
 *  array, carrying over the `filled` indices each voxel already had.
 *  Needed whenever new points are added, since runs of existing voxels must
 *  grow in place.
 */
template <class MAP>
void relayoutVoxels(
    MAP& voxels, std::vector<std::uint32_t>& arena, std::vector<std::uint32_t>& aux,
    const std::uint32_t total)
{
    aux.resize(total);

    std::uint32_t next = 0;
    for (auto it = voxels.begin(); it != voxels.end(); ++it)
    {
        auto& e = valueRef(it);
        std::copy(
            arena.begin() + e.offset, arena.begin() + e.offset + e.filled, aux.begin() + next);
        e.offset = next;
        next += e.count;
    }
    arena.swap(aux);
}
}  // namespace

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

    // Point indices are stored as 32 bit to halve the memory traffic:
    ASSERT_LT_(last_pt_idx, static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()));

    auto pass = [&](auto& voxels)
    {
        // 1st pass: how many points fall into each voxel.
        for (std::size_t i = first_pt_idx; i < last_pt_idx; i++)
        {
            const indices_t key = {coord2idx(xs[i]), coord2idx(ys[i]), coord2idx(zs[i])};
            voxels[key].count++;
        }

        impl_->total_indices += static_cast<std::uint32_t>(last_pt_idx - first_pt_idx);

        relayoutVoxels(voxels, impl_->indices, impl_->indices_aux, impl_->total_indices);

        // 2nd pass: write each point index into the run of its voxel.
        for (std::size_t i = first_pt_idx; i < last_pt_idx; i++)
        {
            const indices_t key = {coord2idx(xs[i]), coord2idx(ys[i]), coord2idx(zs[i])};
            auto            it  = voxels.find(key);
            auto&           e   = valueRef(it);
            impl_->indices[e.offset + e.filled++] = static_cast<std::uint32_t>(i);
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

void PointCloudToVoxelGrid::mergeFrom(const PointCloudToVoxelGrid& other)
{
    mergeFrom(std::vector<const PointCloudToVoxelGrid*>{&other});
}

void PointCloudToVoxelGrid::mergeFrom(const std::vector<const PointCloudToVoxelGrid*>& others)
{
    MRPT_START

    for (const auto* o : others)
    {
        ASSERT_EQUAL_(resolution_, o->resolution_);
        ASSERT_EQUAL_(use_tsl_robin_map_, o->use_tsl_robin_map_);
    }

    auto append = [&](auto& dest, auto srcOf)
    {
        for (const auto* o : others)
        {
            impl_->total_indices += o->impl_->total_indices;

            for (const auto& [key, entry] : srcOf(*o))
            {
                dest[key].count += entry.count;
            }
        }

        // One single layout pass for all the merged grids:
        relayoutVoxels(dest, impl_->indices, impl_->indices_aux, impl_->total_indices);

        for (const auto* o : others)
        {
            const auto& srcIndices = o->impl_->indices;
            for (const auto& [key, entry] : srcOf(*o))
            {
                auto  it = dest.find(key);
                auto& e  = valueRef(it);
                std::copy(
                    srcIndices.begin() + entry.offset,
                    srcIndices.begin() + entry.offset + entry.count,
                    impl_->indices.begin() + e.offset + e.filled);
                e.filled += entry.count;
            }
        }
    };

    if (use_tsl_robin_map_)
    {
        append(
            impl_->pts_voxels,
            [](const PointCloudToVoxelGrid& g) -> const RobinMap& { return g.impl_->pts_voxels; });
    }
    else
    {
        append(
            impl_->pts_voxels_std_map,
            [](const PointCloudToVoxelGrid& g) -> const StdMap&
            { return g.impl_->pts_voxels_std_map; });
    }

    MRPT_END
}

void PointCloudToVoxelGrid::sortVoxelPointIndices()
{
    auto sortAll = [&](const auto& voxels)
    {
        for (const auto& [key, entry] : voxels)
        {
            const auto runBegin = impl_->indices.begin() + entry.offset;
            std::sort(runBegin, runBegin + entry.count);
        }
    };

    if (use_tsl_robin_map_)
    {
        sortAll(impl_->pts_voxels);
    }
    else
    {
        sortAll(impl_->pts_voxels_std_map);
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

    // Keep the capacity of the flat index array: it is refilled on every run
    // with a similar number of points.
    impl_->indices.clear();
    impl_->total_indices = 0;
}

void PointCloudToVoxelGrid::visit_voxels(
    const std::function<void(const indices_t idx, const voxel_t& vxl)>& userCode) const
{
    auto visit = [&](const auto& voxels)
    {
        for (const auto& [idx, entry] : voxels)
        {
            voxel_t view;
            view.begin_ptr = impl_->indices.data() + entry.offset;
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
