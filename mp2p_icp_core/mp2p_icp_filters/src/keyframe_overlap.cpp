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
 * @file   keyframe_overlap.cpp
 * @brief  Shared perceptual content between keyframes, as a weighted graph.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 27, 2026
 */

#include <mp2p_icp_filters/keyframe_overlap.h>
#include <mrpt/core/exceptions.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

namespace mp2p_icp_filters
{
namespace
{
// Key layout: three voxel indices of IDX_BITS bits, then BUCKET_BITS of direction
constexpr int64_t IDX_BITS    = 19;
constexpr int64_t BUCKET_BITS = 5;
constexpr int64_t IDX_HALF    = int64_t(1) << (IDX_BITS - 1);

uint64_t voxelKey(int64_t ix, int64_t iy, int64_t iz)
{
    ASSERTMSG_(
        ix >= -IDX_HALF && ix < IDX_HALF && iy >= -IDX_HALF && iy < IDX_HALF && iz >= -IDX_HALF &&
            iz < IDX_HALF,
        "keyframe_signature(): point too far from the origin for this voxel size");
    const auto ux = static_cast<uint64_t>(ix + IDX_HALF);
    const auto uy = static_cast<uint64_t>(iy + IDX_HALF);
    const auto uz = static_cast<uint64_t>(iz + IDX_HALF);
    return (ux << (2 * IDX_BITS + BUCKET_BITS)) | (uy << (IDX_BITS + BUCKET_BITS)) |
           (uz << BUCKET_BITS);
}

// One of six directions: dominant axis and its sign
uint64_t axisBucket(const Eigen::Vector3d& v)
{
    Eigen::Index axis = 0;
    v.cwiseAbs().maxCoeff(&axis);
    return static_cast<uint64_t>(2 * axis + (v[axis] < 0 ? 1 : 0));
}

struct VoxelStats
{
    std::size_t     n = 0;
    Eigen::Vector3d sum{0, 0, 0};
    Eigen::Matrix3d sum2 = Eigen::Matrix3d::Zero();
};

}  // namespace

KeyframeSignature keyframe_signature(
    const mrpt::maps::CPointsMap& pc, const mrpt::math::TPoint3D& sensor,
    const KeyframeSignatureParams& params)
{
    MRPT_START
    ASSERT_GT_(params.voxel_size, 0.0);

    const auto&  xs        = pc.getPointsBufferRef_x();
    const auto&  ys        = pc.getPointsBufferRef_y();
    const auto&  zs        = pc.getPointsBufferRef_z();
    const double maxR2     = params.max_range > 0 ? params.max_range * params.max_range : -1.0;
    const double voxelSize = params.voxel_size;

    std::unordered_map<uint64_t, VoxelStats> voxels;
    voxels.reserve(pc.size() / 4 + 16);
    for (size_t k = 0; k < pc.size(); k++)
    {
        const Eigen::Vector3d p(xs[k], ys[k], zs[k]);
        if (!p.allFinite())
        {
            continue;
        }
        const Eigen::Vector3d rel(p.x() - sensor.x, p.y() - sensor.y, p.z() - sensor.z);
        if (maxR2 > 0 && rel.squaredNorm() > maxR2)
        {
            continue;
        }
        const auto key = voxelKey(
            static_cast<int64_t>(std::floor(p.x() / voxelSize)),
            static_cast<int64_t>(std::floor(p.y() / voxelSize)),
            static_cast<int64_t>(std::floor(p.z() / voxelSize)));
        auto& v = voxels[key];
        v.n++;
        if (params.oriented)
        {
            v.sum += rel;
            v.sum2 += rel * rel.transpose();
        }
    }

    KeyframeSignature sig;
    sig.reserve(voxels.size());
    for (const auto& [key, v] : voxels)
    {
        if (!params.oriented)
        {
            sig.push_back(key);
            continue;
        }
        // Mean relative to the sensor, so the viewing ray is the mean itself.
        // Voxels above and below the sensor are kept apart, so a wall seen
        // from two storeys does not join them where it crosses the slab.
        const Eigen::Vector3d mean   = v.sum / static_cast<double>(v.n);
        const uint64_t        side   = mean.z() > 0 ? 12 : 0;
        uint64_t              bucket = 6 + axisBucket(mean);
        if (v.n >= params.min_planar_points)
        {
            const Eigen::Matrix3d cov = v.sum2 / static_cast<double>(v.n) - mean * mean.transpose();
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
            es.computeDirect(cov);
            const auto& ev = es.eigenvalues();
            if (ev[0] < params.max_planarity_ratio * ev[1])
            {
                Eigen::Vector3d n = es.eigenvectors().col(0);
                if (n.dot(mean) > 0)
                {
                    n = -n;  // towards the sensor
                }
                bucket = axisBucket(n);
            }
        }
        sig.push_back(key | (bucket + side));
    }
    std::sort(sig.begin(), sig.end());
    return sig;
    MRPT_END
}

double signature_similarity(const KeyframeSignature& a, const KeyframeSignature& b)
{
    size_t shared = 0;
    auto   ia     = a.begin();
    auto   ib     = b.begin();
    while (ia != a.end() && ib != b.end())
    {
        if (*ia < *ib)
        {
            ++ia;
        }
        else if (*ib < *ia)
        {
            ++ib;
        }
        else
        {
            shared++;
            ++ia;
            ++ib;
        }
    }
    const size_t uni = a.size() + b.size() - shared;
    return uni > 0 ? static_cast<double>(shared) / static_cast<double>(uni) : 0.0;
}

std::vector<std::pair<uint32_t, uint32_t>> KeyframeOverlapGraph::sharedCounts(
    const KeyframeSignature& sig) const
{
    // Only the nodes reached through a shared voxel are counted:
    std::unordered_map<uint32_t, uint32_t> count;
    for (const uint64_t key : sig)
    {
        const auto it = index_.find(key);
        if (it == index_.end())
        {
            continue;
        }
        for (const uint32_t node : it->second)
        {
            count[node]++;
        }
    }
    std::vector<std::pair<uint32_t, uint32_t>> out(count.begin(), count.end());
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<std::pair<uint32_t, double>> KeyframeOverlapGraph::query(
    const KeyframeSignature& sig) const
{
    std::vector<std::pair<uint32_t, double>> out;
    for (const auto& [node, shared] : sharedCounts(sig))
    {
        const double uni = static_cast<double>(sig.size() + sizes_[node] - shared);
        const double sim = static_cast<double>(shared) / uni;
        if (sim >= params_.min_similarity)
        {
            out.emplace_back(node, sim);
        }
    }
    return out;
}

uint32_t KeyframeOverlapGraph::add_node(const KeyframeSignature& sig)
{
    ASSERTMSG_(
        std::is_sorted(sig.begin(), sig.end()) &&
            std::adjacent_find(sig.begin(), sig.end()) == sig.end(),
        "KeyframeOverlapGraph::add_node(): the signature must be sorted and unique");

    const auto id = static_cast<uint32_t>(sizes_.size());
    for (const auto& [node, sim] : query(sig))
    {
        edges_.push_back({node, id, sim});
    }
    sizes_.push_back(sig.size());
    for (const uint64_t key : sig)
    {
        index_[key].push_back(id);
    }
    return id;
}

}  // namespace mp2p_icp_filters
