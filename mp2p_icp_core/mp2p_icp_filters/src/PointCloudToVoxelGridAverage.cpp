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
 * @file   PointCloudToVoxelGridAverage.cpp
 * @brief  Voxel grid that summarizes each voxel without keeping its points.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 22, 2026
 */

#include <mp2p_icp_filters/PointCloudToVoxelGridAverage.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/round.h>

// Used in the PIMP:
#include <tsl/robin_map.h>

#include <limits>
#include <map>

using namespace mp2p_icp_filters;

namespace
{
/** Running sums of one voxel, plus the best candidate found so far by the
 *  second pass. Kept in a flat vector, indexed by the slot the hash map assigns
 *  to the voxel key.
 */
struct alignas(32) Accumulator
{
    float sumX = 0, sumY = 0, sumZ = 0;
    float bestSqrErr = std::numeric_limits<float>::max();

    uint32_t bestIdx = 0;
    uint32_t count   = 0;
};
}  // namespace

struct PointCloudToVoxelGridAverage::Impl
{
    /// Voxel key -> slot within `accumulators`.
    tsl::robin_map<indices_t, uint32_t, IndicesHash> slot_of_voxel;
    std::map<indices_t, uint32_t, IndicesHash>       slot_of_voxel_std_map;

    std::vector<Accumulator> accumulators;

    /// Slot of the voxel each input point fell into, so that the second pass
    /// does not have to hash the coordinates again. Its capacity is kept across
    /// calls, since consecutive clouds have a similar number of points.
    std::vector<uint32_t> slot_of_point;
};

PointCloudToVoxelGridAverage::PointCloudToVoxelGridAverage() : impl_(mrpt::make_impl<Impl>()) {}

void PointCloudToVoxelGridAverage::setConfiguration(const float voxel_size, bool use_tsl_robin_map)
{
    MRPT_START

    resolution_        = voxel_size;
    use_tsl_robin_map_ = use_tsl_robin_map;

    this->clear();

    MRPT_END
}

void PointCloudToVoxelGridAverage::processPointCloud(
    const mrpt::maps::CPointsMap& p, bool findClosestToAverage)
{
    MRPT_START

    const auto& xs = p.getPointsBufferRef_x();
    const auto& ys = p.getPointsBufferRef_y();
    const auto& zs = p.getPointsBufferRef_z();

    const std::size_t nPts = xs.size();

    // Slots are stored as 32 bit:
    ASSERT_LT_(nPts, static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()));

    auto& accum = impl_->accumulators;
    auto& slots = impl_->slot_of_point;

    if (findClosestToAverage)
    {
        slots.resize(nPts);
    }

    // 1st pass: accumulate the sums of each voxel.
    auto firstPass = [&](auto& voxels)
    {
        for (std::size_t i = 0; i < nPts; i++)
        {
            const indices_t key = {coord2idx(xs[i]), coord2idx(ys[i]), coord2idx(zs[i])};

            // try_emplace: single hash lookup for both insert and existing-key
            // cases. The mapped value is the slot this voxel would take if it
            // turns out to be new.
            auto [it, inserted] = voxels.try_emplace(key, static_cast<uint32_t>(accum.size()));
            if (inserted)
            {
                accum.emplace_back();
            }

            const uint32_t slot = it->second;
            auto&          a    = accum[slot];

            a.sumX += xs[i];
            a.sumY += ys[i];
            a.sumZ += zs[i];
            a.count++;

            if (findClosestToAverage)
            {
                slots[i] = slot;
            }
        }
    };

    if (use_tsl_robin_map_)
    {
        firstPass(impl_->slot_of_voxel);
    }
    else
    {
        firstPass(impl_->slot_of_voxel_std_map);
    }

    // Turn the sums into averages:
    for (auto& a : accum)
    {
        const float inv_n = 1.0f / static_cast<float>(a.count);
        a.sumX *= inv_n;
        a.sumY *= inv_n;
        a.sumZ *= inv_n;
    }

    if (!findClosestToAverage)
    {
        return;
    }

    // 2nd pass: the point closest to its voxel average. No hashing here: the
    // slot of each point was recorded above.
    for (std::size_t i = 0; i < nPts; i++)
    {
        auto& a = accum[slots[i]];

        const float sqrErr = mrpt::square(xs[i] - a.sumX) + mrpt::square(ys[i] - a.sumY) +
                             mrpt::square(zs[i] - a.sumZ);

        if (sqrErr < a.bestSqrErr)
        {
            a.bestSqrErr = sqrErr;
            a.bestIdx    = static_cast<uint32_t>(i);
        }
    }

    MRPT_END
}

void PointCloudToVoxelGridAverage::clear()
{
    if (use_tsl_robin_map_)
    {
        // Low min_load_factor lets clear() shrink the bucket array, freeing
        // peak memory rather than retaining it across filter invocations.
        impl_->slot_of_voxel.min_load_factor(0.01f);
        impl_->slot_of_voxel.clear();
    }
    else
    {
        impl_->slot_of_voxel_std_map.clear();
    }

    // Keep the capacity of these: they are refilled on every run with a
    // similar number of entries.
    impl_->accumulators.clear();
    impl_->slot_of_point.clear();
}

void PointCloudToVoxelGridAverage::visit_voxels(
    const std::function<void(const indices_t idx, const voxel_t& vxl)>& userCode) const
{
    auto visit = [&](const auto& voxels)
    {
        for (const auto& [idx, slot] : voxels)
        {
            const auto& a = impl_->accumulators[slot];

            voxel_t v;
            v.average             = {a.sumX, a.sumY, a.sumZ};
            v.closestToAverageIdx = a.bestIdx;
            v.pointCount          = a.count;

            userCode(idx, v);
        }
    };

    if (use_tsl_robin_map_)
    {
        visit(impl_->slot_of_voxel);
    }
    else
    {
        visit(impl_->slot_of_voxel_std_map);
    }
}

std::size_t PointCloudToVoxelGridAverage::size() const
{
    return use_tsl_robin_map_ ? impl_->slot_of_voxel.size() : impl_->slot_of_voxel_std_map.size();
}
