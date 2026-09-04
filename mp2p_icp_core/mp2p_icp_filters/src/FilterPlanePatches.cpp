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
 * @file   FilterPlanePatches.cpp
 * @brief  Extracts large planar patches from a point layer into `planes`
 * @author Jose Luis Blanco Claraco
 * @date   Sep 1, 2026
 */

#include <mp2p_icp_filters/FilterPlanePatches.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TPlane.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

IMPLEMENTS_MRPT_OBJECT(FilterPlanePatches, mp2p_icp_filters::FilterBase, mp2p_icp_filters)

using namespace mp2p_icp_filters;

namespace
{
struct VoxelKey
{
    int64_t x, y, z;
    bool    operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct VoxelKeyHash
{
    std::size_t operator()(const VoxelKey& k) const
    {
        // Three odd 64-bit constants; collisions only cost a bucket walk.
        return static_cast<std::size_t>(k.x * 73856093LL ^ k.y * 19349663LL ^ k.z * 83492791LL);
    }
};

struct CellKey
{
    int64_t u, v;
    bool    operator==(const CellKey& o) const { return u == o.u && v == o.v; }
};

struct CellKeyHash
{
    std::size_t operator()(const CellKey& k) const
    {
        return static_cast<std::size_t>(k.u * 73856093LL ^ k.v * 19349663LL);
    }
};

/** Smallest-eigenvalue eigenvector of a 3x3 symmetric matrix, plus the
 *  eigenvalues, ascending. */
void smallestAxis(const Eigen::Matrix3d& C, Eigen::Vector3d& axis, Eigen::Vector3d& evals)
{
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
    evals = es.eigenvalues();
    axis  = es.eigenvectors().col(0);
}
}  // namespace

void FilterPlanePatches::Parameters::load_from_yaml(
    const mrpt::containers::yaml& c, FilterPlanePatches& parent)
{
    MCP_LOAD_REQ(c, input_pointcloud_layer);

    MCP_LOAD_OPT(c, clear_previous);
    MCP_LOAD_OPT(c, voxel_size);
    MCP_LOAD_OPT(c, distance_threshold);
    MCP_LOAD_OPT(c, normal_agreement_deg);
    MCP_LOAD_OPT(c, min_points);
    MCP_LOAD_OPT(c, min_span);
    MCP_LOAD_OPT(c, range_min);
    MCP_LOAD_OPT(c, range_max);
    MCP_LOAD_OPT(c, max_patches);
    MCP_LOAD_OPT(c, normal_knn);
    MCP_LOAD_OPT(c, seed_candidates);

    ASSERT_GT_(voxel_size, 0.0);
    ASSERT_GT_(distance_threshold, 0.0);
    ASSERT_GT_(min_points, 2U);
    ASSERT_GE_(min_span, 0.0);
    // A negative range_min would be squared below into a positive threshold,
    // silently rejecting everything nearer than |range_min|.
    ASSERT_GE_(range_min, 0.0);
    ASSERT_GT_(range_max, range_min);
    // Past 90 deg the cosine turns negative and the agreement test stops
    // rejecting anything at all.
    ASSERT_GE_(normal_agreement_deg, 0.0);
    ASSERT_LE_(normal_agreement_deg, 90.0);
    ASSERT_GT_(normal_knn, 3U);
    ASSERT_GT_(seed_candidates, 0U);

    (void)parent;
}

FilterPlanePatches::FilterPlanePatches() = default;

void FilterPlanePatches::initialize_filter(const mrpt::containers::yaml& c)
{
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << c);
    params.load_from_yaml(c, *this);
}

void FilterPlanePatches::filter(mp2p_icp::metric_map_t& inOut) const
{
    MRPT_START

    checkAllParametersAreRealized();

    const auto& pcPtr = inOut.point_layer(params.input_pointcloud_layer);
    ASSERTMSG_(
        pcPtr,
        mrpt::format(
            "Input point cloud layer '%s' was not found.", params.input_pointcloud_layer.c_str()));

    if (params.clear_previous) inOut.planes.clear();

    const auto&  pc  = *pcPtr;
    const auto&  xs  = pc.getPointsBufferRef_x();
    const auto&  ys  = pc.getPointsBufferRef_y();
    const auto&  zs  = pc.getPointsBufferRef_z();
    const size_t nIn = xs.size();
    if (nIn == 0) return;

    // 1) Range gate and voxel downsample in one pass. Keeping the first point
    // that lands in each voxel makes the result independent of any ordering
    // the caller's decimation happened to leave behind.
    const double r2min  = params.range_min * params.range_min;
    const double r2max  = params.range_max * params.range_max;
    const double invVox = 1.0 / params.voxel_size;

    // Keeping "the first point seen" in each voxel would make the whole result
    // depend on the order the caller happened to deliver the points in, which an
    // upstream parallel stage is free to change between runs. Instead every
    // survivor is sorted by (voxel, coordinates) and the smallest point of each
    // voxel is kept, so the downsampled cloud is a function of the SET of input
    // points and of nothing else.
    struct Candidate
    {
        VoxelKey        k;
        Eigen::Vector3d p;

        bool operator<(const Candidate& o) const
        {
            if (k.x != o.k.x) return k.x < o.k.x;
            if (k.y != o.k.y) return k.y < o.k.y;
            if (k.z != o.k.z) return k.z < o.k.z;
            if (p.x() != o.p.x()) return p.x() < o.p.x();
            if (p.y() != o.p.y()) return p.y() < o.p.y();
            return p.z() < o.p.z();
        }
    };

    std::vector<Candidate> cand;
    cand.reserve(nIn);
    for (size_t i = 0; i < nIn; i++)
    {
        const double x = xs[i], y = ys[i], z = zs[i];
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        const double r2 = x * x + y * y + z * z;
        if (r2 < r2min || r2 > r2max) continue;

        cand.push_back(Candidate{
            VoxelKey{
                static_cast<int64_t>(std::floor(x * invVox)),
                static_cast<int64_t>(std::floor(y * invVox)),
                static_cast<int64_t>(std::floor(z * invVox))},
            Eigen::Vector3d(x, y, z)});
    }
    std::sort(cand.begin(), cand.end());

    std::vector<Eigen::Vector3d> P;
    P.reserve(cand.size());
    for (size_t i = 0; i < cand.size(); i++)
    {
        if (i == 0 || !(cand[i].k == cand[i - 1].k)) P.push_back(cand[i].p);
    }

    const size_t N = P.size();
    if (N < params.min_points) return;

    // 2) One normal per point, from a k-NN PCA. Built through a temporary
    // points map purely for its kd-tree.
    auto kdMap = mrpt::maps::CSimplePointsMap::Create();
    kdMap->reserve(N);
    for (const auto& p : P) kdMap->insertPointFast(p.x(), p.y(), p.z());
    kdMap->mark_as_modified();

    const size_t                 K = std::min<size_t>(params.normal_knn, N);
    std::vector<Eigen::Vector3d> NRM(N);
    {
        std::vector<size_t> idx;
        std::vector<float>  dsq;
        for (size_t i = 0; i < N; i++)
        {
            kdMap->kdTreeNClosestPoint3DIdx(P[i].x(), P[i].y(), P[i].z(), K, idx, dsq);
            Eigen::Vector3d mean = Eigen::Vector3d::Zero();
            for (const auto j : idx) mean += P[j];
            mean /= static_cast<double>(idx.size());
            Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
            for (const auto j : idx)
            {
                const Eigen::Vector3d d = P[j] - mean;
                C.noalias() += d * d.transpose();
            }
            C /= static_cast<double>(idx.size());
            Eigen::Vector3d axis, ev;
            smallestAxis(C, axis, ev);
            NRM[i] = axis;
        }
    }

    // 3) Greedy extraction.
    const double cosTh       = std::cos(mrpt::DEG2RAD(params.normal_agreement_deg));
    const double areaPerCell = params.voxel_size * params.voxel_size;

    std::vector<uint8_t>  live(N, 1);
    std::vector<uint32_t> ids;  // live indices, rebuilt each round
    std::vector<uint8_t>  inl, best;
    ids.reserve(N);
    inl.reserve(N);
    best.reserve(N);

    for (uint32_t patch = 0; patch < params.max_patches; patch++)
    {
        ids.clear();
        for (uint32_t i = 0; i < N; i++)
            if (live[i]) ids.push_back(i);
        if (ids.size() < params.min_points) break;

        // Seeds: a fixed stride through the live set, so the choice does not
        // depend on a random state that a parallel or resumed run would not
        // reproduce.
        const size_t nSeeds = std::min<size_t>(params.seed_candidates, ids.size());
        const size_t stride = std::max<size_t>(1, ids.size() / nSeeds);

        size_t bestCount = 0;
        best.assign(ids.size(), 0);
        inl.resize(ids.size());

        for (size_t s = 0; s < ids.size(); s += stride)
        {
            const Eigen::Vector3d& n0  = NRM[ids[s]];
            const Eigen::Vector3d& p0  = P[ids[s]];
            size_t                 cnt = 0;
            for (size_t t = 0; t < ids.size(); t++)
            {
                const uint32_t j  = ids[t];
                const bool     ok = std::abs((P[j] - p0).dot(n0)) < params.distance_threshold &&
                                std::abs(NRM[j].dot(n0)) >= cosTh;
                inl[t] = ok ? 1 : 0;
                cnt += ok ? 1 : 0;
            }
            if (cnt > bestCount)
            {
                bestCount = cnt;
                best      = inl;
            }
        }
        if (bestCount < params.min_points) break;

        // Refit by PCA on the winning inliers, then re-select with the refined
        // plane: the seed's own normal is a k-NN estimate and is noisier than
        // the fit over hundreds of points.
        const auto fitAndSelect = [&](const std::vector<uint8_t>& sel, Eigen::Vector3d& c,
                                      Eigen::Vector3d& n, Eigen::Matrix3d& evecs)
        {
            c        = Eigen::Vector3d::Zero();
            size_t m = 0;
            for (size_t t = 0; t < ids.size(); t++)
                if (sel[t])
                {
                    c += P[ids[t]];
                    m++;
                }
            c /= static_cast<double>(m);
            Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
            for (size_t t = 0; t < ids.size(); t++)
                if (sel[t])
                {
                    const Eigen::Vector3d d = P[ids[t]] - c;
                    C.noalias() += d * d.transpose();
                }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
            evecs = es.eigenvectors();
            n     = evecs.col(0);
        };

        Eigen::Vector3d c, n;
        Eigen::Matrix3d evecs;
        fitAndSelect(best, c, n, evecs);

        std::vector<uint8_t> sel(ids.size());
        size_t               selCount = 0;
        for (size_t t = 0; t < ids.size(); t++)
        {
            const uint32_t j  = ids[t];
            const bool     ok = std::abs((P[j] - c).dot(n)) < params.distance_threshold &&
                            std::abs(NRM[j].dot(n)) >= cosTh;
            sel[t] = ok ? 1 : 0;
            selCount += ok ? 1 : 0;
        }
        if (selCount < params.min_points)
        {
            // The refit did not survive: retire the seed's own inliers so the
            // next round cannot pick the same hypothesis again.
            for (size_t t = 0; t < ids.size(); t++)
                if (best[t]) live[ids[t]] = 0;
            continue;
        }

        fitAndSelect(sel, c, n, evecs);

        // In-plane extent along the two major axes, and the occupied footprint.
        //
        // The area is counted on a grid laid out *in the plane*, not from the
        // number of inlier points: the 3-D voxel grid splits a surface that
        // happens to straddle one of its boundaries into two layers, which
        // would credit that surface with up to twice its real area purely
        // because of where the grid fell.
        const Eigen::Vector3d a1 = evecs.col(2), a2 = evecs.col(1);
        double                lo1 = 1e300, hi1 = -1e300, lo2 = 1e300, hi2 = -1e300;
        std::unordered_map<CellKey, uint8_t, CellKeyHash> cells;
        cells.reserve(selCount);
        for (size_t t = 0; t < ids.size(); t++)
        {
            if (!sel[t]) continue;
            const Eigen::Vector3d d = P[ids[t]] - c;
            const double          u = d.dot(a1), v = d.dot(a2);
            lo1 = std::min(lo1, u);
            hi1 = std::max(hi1, u);
            lo2 = std::min(lo2, v);
            hi2 = std::max(hi2, v);
            cells.emplace(
                CellKey{
                    static_cast<int64_t>(std::floor(u * invVox)),
                    static_cast<int64_t>(std::floor(v * invVox))},
                uint8_t{1});
        }
        const double span = std::min(hi1 - lo1, hi2 - lo2);

        for (size_t t = 0; t < ids.size(); t++)
            if (sel[t]) live[ids[t]] = 0;

        if (span < params.min_span) continue;

        mp2p_icp::plane_patch_t pp;
        pp.centroid = {c.x(), c.y(), c.z()};
        pp.plane    = mrpt::math::TPlane(
               mrpt::math::TPoint3D(c.x(), c.y(), c.z()), mrpt::math::TVector3D(n.x(), n.y(), n.z()));
        // Surface area, not the bounding box: a sparse plane fitted across a
        // cluttered volume must not outweigh a solid wall of the same extent.
        pp.area       = static_cast<double>(cells.size()) * areaPerCell;
        pp.num_points = static_cast<uint32_t>(selCount);
        inOut.planes.push_back(pp);
    }

    MRPT_LOG_DEBUG_STREAM(
        "[FilterPlanePatches] " << params.input_pointcloud_layer << ": " << nIn << " pts -> " << N
                                << " voxels -> " << inOut.planes.size() << " patches");

    MRPT_END
}
