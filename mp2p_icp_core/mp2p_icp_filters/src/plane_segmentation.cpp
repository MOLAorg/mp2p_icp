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
 * @file   plane_segmentation.cpp
 * @brief  Segmentation of large planes in point clouds and maps
 * @author Jose Luis Blanco Claraco
 * @date   Nov 25, 2026
 */

#include <mp2p_icp_filters/plane_segmentation.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <nanoflann.hpp>
#include <numeric>
#include <utility>

#if defined(MP2P_HAS_TBB)
#include <tbb/blocked_range.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>
#endif

namespace mp2p_icp_filters
{
namespace
{
constexpr uint32_t kNone = std::numeric_limits<uint32_t>::max();

// A neighborhood (or region) is planar if lambda0 < kMaxFlatness * lambda1.
constexpr double kMaxFlatness = 0.25;
// A neighborhood is a line, not a surface, if lambda1 < kMinSpread * lambda2
// (e.g. the points of one ring of a sparse scan).
constexpr double kMinSpread = 0.05;
// Minimum neighbors (including the point itself) for a normal.
constexpr std::size_t kMinNeighbors = 5;
// Wider neighborhood (times normal_knn) for normals of noisy maps.
constexpr std::size_t kWideScale = 3;
// Region growing tolerance, in standard deviations of the representatives.
constexpr double kGrowSigmas = 3.0;
// Minimum points in a voxel for a normal from its own points.
constexpr double kMinVoxelPoints = 8;
// Minimum fraction of the points of a plane in voxels whose own normal agrees.
constexpr double kMinNormalSupport = 0.5;
// Regions grown with fewer representatives are discarded.
constexpr std::size_t kMinRegionSize = 3;
// Regions with fewer representatives do not take part in merging.
constexpr std::size_t kMinMergeSize = 8;
// Robust refinement
constexpr int    kRefineIterations = 8;
constexpr double kCauchyScale      = 2.3849;  // times the MAD sigma
constexpr double kMinSigma         = 1e-4;  // [m]
// Larger planes are refitted on a fixed-stride subsample of their points.
constexpr std::size_t kMaxFitPoints = 200000;
// Voxel coordinates are packed in 21 bits per axis.
constexpr int64_t kMaxCellsPerAxis = (int64_t{1} << 21) - 1;

/** Runs f(begin, end) over blocks of [0, n), in parallel if TBB is enabled.
 *  Every caller writes only to the outputs of its own indices, so the result
 *  does not depend on the partition. */
template <typename F>
void parallel_blocks(std::size_t n, std::size_t grain, F&& f)
{
#if defined(MP2P_HAS_TBB)
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, n, grain),
        [&](const tbb::blocked_range<std::size_t>& r) { f(r.begin(), r.end()); });
#else
    (void)grain;
    f(std::size_t{0}, n);
#endif
}

template <typename It>
void sort_range(It first, It last)
{
#if defined(MP2P_HAS_TBB)
    tbb::parallel_sort(first, last);
#else
    std::sort(first, last);
#endif
}

/// nanoflann adaptor for the voxel representatives.
struct RepCloud
{
    const std::vector<Eigen::Vector3d>& pts;

    std::size_t kdtree_get_point_count() const { return pts.size(); }
    double      kdtree_get_pt(std::size_t i, std::size_t dim) const
    {
        return pts[i][static_cast<Eigen::Index>(dim)];
    }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const
    {
        return false;
    }
};

using RepTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, RepCloud>, RepCloud, 3, uint32_t>;

/// k-d tree parameters: built in parallel where supported (the tree does not
/// depend on the number of threads).
nanoflann::KDTreeSingleIndexAdaptorParams tree_params()
{
#if NANOFLANN_VERSION >= 0x150 && defined(MP2P_HAS_TBB)
    // Same limit as the TBB loops: the arena, and any global_control in force.
    const std::size_t nThreads = std::min(
        static_cast<std::size_t>(std::max(1, tbb::this_task_arena::max_concurrency())),
        tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism));
    return nanoflann::KDTreeSingleIndexAdaptorParams(
        10, nanoflann::KDTreeSingleIndexAdaptorFlags::None,
        static_cast<unsigned int>(std::max<std::size_t>(1, nThreads)));
#else
    return nanoflann::KDTreeSingleIndexAdaptorParams(10);
#endif
}

/// Count, mean and scatter matrix (about the mean) of a point set.
struct Moments
{
    double          n    = 0;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d m2   = Eigen::Matrix3d::Zero();

    void add(const Eigen::Vector3d& p)
    {
        n += 1;
        const Eigen::Vector3d d = p - mean;
        mean += d / n;
        m2.noalias() += d * (p - mean).transpose();
    }

    void merge(const Moments& o)
    {
        if (o.n == 0)
        {
            return;
        }
        if (n == 0)
        {
            *this = o;
            return;
        }
        const double          nt = n + o.n;
        const Eigen::Vector3d d  = o.mean - mean;
        mean += d * (o.n / nt);
        m2 += o.m2 + (d * d.transpose()) * (n * o.n / nt);
        n = nt;
    }
};

/// Plane fit from moments: normal, a point on it, eigenvalues (ascending) of
/// the covariance.
struct PlaneFit
{
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d point  = Eigen::Vector3d::Zero();
    Eigen::Vector3d eig    = Eigen::Vector3d::Zero();
    bool            valid  = false;

    double rms() const { return std::sqrt(std::max(0.0, eig[0])); }
};

PlaneFit fit_plane(const Moments& m)
{
    PlaneFit f;
    if (m.n < 3)
    {
        return f;
    }
    const Eigen::Matrix3d cov = (0.5 / m.n) * (m.m2 + m.m2.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    f.eig    = es.eigenvalues();
    f.normal = es.eigenvectors().col(0);
    f.point  = m.mean;
    f.valid  = f.eig[0] < kMaxFlatness * f.eig[1];
    return f;
}

struct KeyIdx
{
    uint64_t key;
    uint32_t idx;

    bool operator<(const KeyIdx& o) const
    {
        if (key != o.key)
        {
            return key < o.key;
        }
        return idx < o.idx;
    }
};

inline uint64_t pack_key(int64_t ix, int64_t iy, int64_t iz)
{
    return (static_cast<uint64_t>(ix) << 42) | (static_cast<uint64_t>(iy) << 21) |
           static_cast<uint64_t>(iz);
}

/// Cell index along one axis: divides by the resolution and rounds down.
inline int64_t cell_index(double coord, double resolution)
{
    return static_cast<int64_t>(std::floor(coord / resolution));
}

struct UnionFind
{
    std::vector<uint32_t> parent;

    explicit UnionFind(std::size_t n) : parent(n) { std::iota(parent.begin(), parent.end(), 0U); }

    uint32_t find(uint32_t i)
    {
        while (parent[i] != i)
        {
            parent[i] = parent[parent[i]];
            i         = parent[i];
        }
        return i;
    }
};

/** Conservative merge test of two segments: normals agree, the centroid of
 *  the smaller one lies close to the plane of the larger one (the plane of a
 *  small segment is not extrapolated to a far centroid), and the union is
 *  still flat. */
bool mergeable(
    const Moments& ma, const PlaneFit& fa, const Moments& mb, const PlaneFit& fb,
    const PlaneSegmentationParams& p, double* cost)
{
    if (!fa.valid || !fb.valid)
    {
        return false;
    }
    if (std::abs(fa.normal.dot(fb.normal)) < std::cos(mrpt::DEG2RAD(p.merge_normal_deg)))
    {
        return false;
    }
    const PlaneFit& big   = ma.n >= mb.n ? fa : fb;
    const PlaneFit& small = ma.n >= mb.n ? fb : fa;
    const double    d     = std::abs(big.normal.dot(small.point - big.point));
    if (d > p.merge_offset)
    {
        return false;
    }
    Moments u = ma;
    u.merge(mb);
    const auto fu = fit_plane(u);
    if (!fu.valid || fu.rms() > p.max_thickness)
    {
        return false;
    }
    if (cost)
    {
        *cost = d;
    }
    return true;
}

/** Greedy agglomeration over candidate pairs (a < b), cheapest first. Every
 *  merge is tested again against the clusters built so far, so a chain of
 *  pairwise-compatible segments cannot bend. Updates the moments and fits of
 *  the roots in place. */
UnionFind agglomerate(
    std::vector<Moments>& mom, std::vector<PlaneFit>& fit,
    const std::vector<std::pair<uint32_t, uint32_t>>& pairs, const PlaneSegmentationParams& p)
{
    std::vector<std::pair<double, std::pair<uint32_t, uint32_t>>> ranked;
    for (const auto& [a, b] : pairs)
    {
        double cost = 0;
        if (mergeable(mom[a], fit[a], mom[b], fit[b], p, &cost))
        {
            ranked.push_back({cost, {a, b}});
        }
    }
    std::sort(ranked.begin(), ranked.end());

    UnionFind uf(mom.size());
    for (bool changed = true; changed;)
    {
        changed = false;
        for (const auto& [cost, ab] : ranked)
        {
            const uint32_t ra = uf.find(ab.first);
            const uint32_t rb = uf.find(ab.second);
            if (ra == rb || !mergeable(mom[ra], fit[ra], mom[rb], fit[rb], p, nullptr))
            {
                continue;
            }
            const uint32_t keep = std::min(ra, rb);
            const uint32_t gone = std::max(ra, rb);
            mom[keep].merge(mom[gone]);
            mom[gone]       = Moments();
            fit[keep]       = fit_plane(mom[keep]);
            fit[gone]       = PlaneFit();
            uf.parent[gone] = keep;
            changed         = true;
        }
    }
    return uf;
}

/// The input points, by their position in the voxel-sorted order, relative
/// to the internal origin.
struct PointSource
{
    const std::vector<KeyIdx>*             sorted = nullptr;
    const mrpt::aligned_std_vector<float>* xs     = nullptr;
    const mrpt::aligned_std_vector<float>* ys     = nullptr;
    const mrpt::aligned_std_vector<float>* zs     = nullptr;
    Eigen::Vector3d                        origin = Eigen::Vector3d::Zero();

    Eigen::Vector3d at(uint32_t q) const
    {
        const uint32_t i = (*sorted)[q].idx;
        return {
            static_cast<double>((*xs)[i]) - origin.x(), static_cast<double>((*ys)[i]) - origin.y(),
            static_cast<double>((*zs)[i]) - origin.z()};
    }
};

/** Robust total least squares plane: IRLS with a Cauchy kernel whose scale
 *  follows the MAD of the residuals; points farther than `maxDist` get no
 *  weight. Refines (n, c) in place; returns false if degenerate. */
bool robust_plane_fit(
    const PointSource& src, const uint32_t* q, std::size_t nqAll, double maxDist,
    Eigen::Vector3d& n, Eigen::Vector3d& c)
{
    // A fixed-stride subsample bounds the cost on very large planes.
    const std::size_t            stride = (nqAll + kMaxFitPoints - 1) / kMaxFitPoints;
    const std::size_t            nq     = (nqAll + stride - 1) / stride;
    std::vector<Eigen::Vector3d> pts(nq);
    for (std::size_t i = 0; i < nq; i++)
    {
        pts[i] = src.at(q[i * stride]);
    }
    std::vector<double> res(nq);
    std::vector<double> absRes(nq);
    std::vector<double> w(nq);

    for (int it = 0; it < kRefineIterations; it++)
    {
        for (std::size_t i = 0; i < nq; i++)
        {
            res[i]    = n.dot(pts[i] - c);
            absRes[i] = std::abs(res[i]);
        }
        const auto mid = absRes.begin() + static_cast<std::ptrdiff_t>(nq / 2);
        std::nth_element(absRes.begin(), mid, absRes.end());
        const double scale = kCauchyScale * std::max(kMinSigma, 1.4826 * (*mid));

        double          sw = 0;
        Eigen::Vector3d sp = Eigen::Vector3d::Zero();
        for (std::size_t i = 0; i < nq; i++)
        {
            const double e = res[i] / scale;
            w[i]           = std::abs(res[i]) > maxDist ? 0.0 : 1.0 / (1.0 + e * e);
            sw += w[i];
            sp += w[i] * (pts[i] - c);
        }
        if (!(sw > 0))
        {
            return false;
        }
        const Eigen::Vector3d meanRel = sp / sw;
        Eigen::Matrix3d       C       = Eigen::Matrix3d::Zero();
        for (std::size_t i = 0; i < nq; i++)
        {
            const Eigen::Vector3d d = pts[i] - c - meanRel;
            C.noalias() += w[i] * d * d.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C / sw);
        const Eigen::Vector3d                          ev = es.eigenvalues();
        if (!(ev[0] < kMaxFlatness * ev[1]))
        {
            return false;
        }
        Eigen::Vector3d nNew = es.eigenvectors().col(0);
        if (nNew.dot(n) < 0)
        {
            nNew = -nNew;
        }
        const Eigen::Vector3d cNew = c + meanRel;

        const bool converged = (1.0 - nNew.dot(n)) < 1e-12 && std::abs(nNew.dot(cNew - c)) < 1e-9;
        n                    = nNew;
        c                    = cNew;
        if (converged)
        {
            break;
        }
    }
    return true;
}

/** Statistics of a plane over its assigned points, and the acceptance tests.
 *  Returns false if the plane is rejected. */
bool plane_statistics(
    const PointSource& src, const uint32_t* q, std::size_t nq, Eigen::Vector3d n,
    const Eigen::Vector3d& c, const PlaneSegmentationParams& p, SegmentedPlane& out)
{
    if (nq < p.min_points || nq < 3)
    {
        return false;
    }
    std::vector<Eigen::Vector3d> pts(nq);
    Eigen::Vector3d              sumP = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < nq; i++)
    {
        pts[i] = src.at(q[i]);
        sumP += pts[i] - c;
    }
    // Mean of the inliers, projected onto the plane:
    Eigen::Vector3d centroid = c + sumP / static_cast<double>(nq);
    centroid -= n * n.dot(centroid - c);

    // Residuals, and the in-plane scatter in an arbitrary basis (e1, e2):
    const Eigen::Vector3d e1    = n.unitOrthogonal();
    const Eigen::Vector3d e2    = n.cross(e1);
    Eigen::Matrix2d       S     = Eigen::Matrix2d::Zero();
    double                sumR2 = 0;
    std::vector<double>   absRes(nq);
    for (std::size_t i = 0; i < nq; i++)
    {
        const Eigen::Vector3d d = pts[i] - centroid;
        const double          r = n.dot(d);
        sumR2 += r * r;
        absRes[i] = std::abs(r);
        const Eigen::Vector2d uv(e1.dot(d), e2.dot(d));
        S.noalias() += uv * uv.transpose();
    }
    const double rms = std::sqrt(sumR2 / static_cast<double>(nq));
    if (rms > p.max_thickness)
    {
        return false;
    }
    const auto p95It = absRes.begin() + static_cast<std::ptrdiff_t>((nq - 1) * 95 / 100);
    std::nth_element(absRes.begin(), p95It, absRes.end());
    const double p95 = *p95It;

    // Orientation: the viewpoint must be on the positive side.
    const Eigen::Vector3d view(
        p.viewpoint.x - src.origin.x(), p.viewpoint.y - src.origin.y(),
        p.viewpoint.z - src.origin.z());
    const bool flip = n.dot(view - centroid) < 0;
    if (flip)
    {
        n = -n;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es2(S);
    const Eigen::Vector2d                          major = es2.eigenvectors().col(1);
    Eigen::Vector3d                                au    = major.x() * e1 + major.y() * e2;
    au.normalize();
    const Eigen::Vector3d av = n.cross(au);

    // Extent, sums of squares, and covered area:
    double uMin = std::numeric_limits<double>::max();
    double uMax = -std::numeric_limits<double>::max();
    double vMin = std::numeric_limits<double>::max();
    double vMax = -std::numeric_limits<double>::max();
    double su2  = 0;
    double sv2  = 0;
    // Area cells: points come grouped by voxel, and the few cells of each
    // voxel are deduplicated before the global sort.
    std::vector<uint64_t> cells;
    cells.reserve(nq / 2 + 1);
    std::size_t runStart = 0;
    for (std::size_t i = 0; i < nq; i++)
    {
        const Eigen::Vector3d d = pts[i] - centroid;
        const double          u = au.dot(d);
        const double          v = av.dot(d);
        uMin                    = std::min(uMin, u);
        uMax                    = std::max(uMax, u);
        vMin                    = std::min(vMin, v);
        vMax                    = std::max(vMax, v);
        su2 += u * u;
        sv2 += v * v;
        const auto cu = static_cast<uint64_t>(cell_index(u, p.voxel_size) + (int64_t{1} << 31));
        const auto cv = static_cast<uint64_t>(cell_index(v, p.voxel_size) + (int64_t{1} << 31));
        const bool newVoxel = i > 0 && (*src.sorted)[q[i]].key != (*src.sorted)[q[i - 1]].key;
        if (newVoxel)
        {
            std::sort(cells.begin() + static_cast<std::ptrdiff_t>(runStart), cells.end());
            cells.erase(
                std::unique(cells.begin() + static_cast<std::ptrdiff_t>(runStart), cells.end()),
                cells.end());
            runStart = cells.size();
        }
        cells.push_back((cu << 32) | (cv & 0xffffffffULL));
    }
    std::sort(cells.begin(), cells.end());
    const auto nCells =
        static_cast<std::size_t>(std::unique(cells.begin(), cells.end()) - cells.begin());
    const double area = static_cast<double>(nCells) * p.voxel_size * p.voxel_size;
    if (area < p.min_area)
    {
        return false;
    }

    // Output, in absolute coordinates:
    const Eigen::Vector3d cAbs = centroid + src.origin;
    out.normal                 = {n.x(), n.y(), n.z()};
    out.offset                 = n.dot(cAbs);
    out.centroid               = {cAbs.x(), cAbs.y(), cAbs.z()};
    out.area                   = area;
    out.thickness_rms          = rms;
    out.thickness_p95          = p95;
    out.axis_u                 = {au.x(), au.y(), au.z()};
    out.axis_v                 = {av.x(), av.y(), av.z()};
    out.u_min                  = uMin;
    out.u_max                  = uMax;
    out.v_min                  = vMin;
    out.v_max                  = vMax;

    const double s2          = std::max(rms * rms, kMinSigma * kMinSigma);
    out.tangent_variances[0] = su2 > 0 ? s2 / su2 : std::numeric_limits<double>::infinity();
    out.tangent_variances[1] = sv2 > 0 ? s2 / sv2 : std::numeric_limits<double>::infinity();
    out.tangent_variances[2] = s2 / static_cast<double>(nq);

    // First-order map from [alpha, beta, gamma] to [n, d], with d = n.c:
    Eigen::Matrix<double, 4, 3> J = Eigen::Matrix<double, 4, 3>::Zero();
    J.block<3, 1>(0, 0)           = au;
    J.block<3, 1>(0, 1)           = av;
    J(3, 0)                       = au.dot(cAbs);
    J(3, 1)                       = av.dot(cAbs);
    J(3, 2)                       = 1.0;
    const Eigen::Vector3d tv(
        out.tangent_variances[0], out.tangent_variances[1], out.tangent_variances[2]);
    const Eigen::Matrix4d cov4 = J * tv.asDiagonal() * J.transpose();
    for (int r = 0; r < 4; r++)
    {
        for (int k = 0; k < 4; k++)
        {
            out.covariance(r, k) = cov4(r, k);
        }
    }

    // The inlier indices are filled in by the caller.
    out.inliers.clear();
    return true;
}

}  // namespace

mrpt::math::TPlane SegmentedPlane::plane() const
{
    mrpt::math::TPlane pl;
    pl.coefs[0] = normal.x;
    pl.coefs[1] = normal.y;
    pl.coefs[2] = normal.z;
    pl.coefs[3] = -offset;
    return pl;
}

mp2p_icp::plane_patch_t SegmentedPlane::as_plane_patch() const
{
    return {plane(), centroid, area, static_cast<uint32_t>(inliers.size())};
}

PlaneSegmentationResult segment_planes(
    const mrpt::maps::CPointsMap& pc, const PlaneSegmentationParams& p)
{
    MRPT_START

    // Written so that NaN values also fail:
    ASSERT_(p.voxel_size > 0);
    ASSERT_GE_(p.normal_knn, static_cast<uint32_t>(kMinNeighbors));
    ASSERT_(p.normal_radius > 0);
    ASSERT_(p.normal_agreement_deg > 0 && p.normal_agreement_deg < 90);
    ASSERT_(p.distance_threshold > 0);
    ASSERT_(p.inlier_distance > 0);
    ASSERT_(p.max_thickness > 0);
    ASSERT_(p.min_area >= 0);
    ASSERT_(p.merge_normal_deg >= 0 && p.merge_normal_deg < 90);
    ASSERT_(p.merge_offset >= 0);
    ASSERT_(p.merge_max_gap > 0);
    ASSERT_GT_(p.parallelization_grain_size, 0UL);

    PlaneSegmentationResult result;

    const auto&       xs  = pc.getPointsBufferRef_x();
    const auto&       ys  = pc.getPointsBufferRef_y();
    const auto&       zs  = pc.getPointsBufferRef_z();
    const std::size_t nIn = xs.size();
    ASSERT_LT_(nIn, static_cast<std::size_t>(kNone));

    // 1) Bounding box of the finite points; all internal coordinates are
    // relative to its min corner, for precision with large map coordinates.
    Eigen::Vector3d bbMin   = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    Eigen::Vector3d bbMax   = -bbMin;
    std::size_t     nFinite = 0;
    for (std::size_t i = 0; i < nIn; i++)
    {
        if (!std::isfinite(xs[i]) || !std::isfinite(ys[i]) || !std::isfinite(zs[i]))
        {
            continue;
        }
        const Eigen::Vector3d q(xs[i], ys[i], zs[i]);
        bbMin = bbMin.cwiseMin(q);
        bbMax = bbMax.cwiseMax(q);
        nFinite++;
    }
    result.num_input_points = nFinite;
    if (nFinite < p.min_points || nFinite < kMinNeighbors)
    {
        return result;
    }
    const Eigen::Vector3d o   = bbMin;
    const double          vox = p.voxel_size;
    for (int k = 0; k < 3; k++)
    {
        // Checked as real numbers, before any conversion to integers:
        const double ext = bbMax[k] - bbMin[k];
        const double nc  = std::floor(ext / std::min(vox, p.merge_max_gap));
        ASSERTMSG_(
            nc < static_cast<double>(kMaxCellsPerAxis),
            mrpt::format(
                "segment_planes: the cloud spans %.1f m, too many cells of %g m", ext,
                std::min(vox, p.merge_max_gap)));
    }

    // 2) Voxels: sort the points by voxel (ties by index, so the order is
    // unique and independent of how the sort is parallelized).
    std::vector<KeyIdx> sorted;
    sorted.reserve(nFinite);
    for (std::size_t i = 0; i < nIn; i++)
    {
        if (!std::isfinite(xs[i]) || !std::isfinite(ys[i]) || !std::isfinite(zs[i]))
        {
            continue;
        }
        sorted.push_back(
            {pack_key(
                 cell_index(static_cast<double>(xs[i]) - o.x(), vox),
                 cell_index(static_cast<double>(ys[i]) - o.y(), vox),
                 cell_index(static_cast<double>(zs[i]) - o.z(), vox)),
             static_cast<uint32_t>(i)});
    }
    sort_range(sorted.begin(), sorted.end());

    std::vector<uint32_t> voxStart;
    for (std::size_t q = 0; q < sorted.size(); q++)
    {
        if (q == 0 || sorted[q].key != sorted[q - 1].key)
        {
            voxStart.push_back(static_cast<uint32_t>(q));
        }
    }
    const std::size_t M = voxStart.size();
    voxStart.push_back(static_cast<uint32_t>(sorted.size()));
    result.num_voxels = M;

    // Representatives: mean of each voxel. A voxel with enough points that lie
    // on a plane also gets its normal from them: at this small scale, two
    // close parallel surfaces do not mix. If the voxel is not planar as a
    // whole, trimmed refits drop the few points of a surface next to it (e.g.
    // the noise tail of a parallel wall), and the representative is then the
    // mean of the points kept.
    std::vector<Eigen::Vector3d> rep(M);
    std::vector<Eigen::Vector3d> nrm(M, Eigen::Vector3d::Zero());
    std::vector<double>          flat(M, std::numeric_limits<double>::infinity());
    // Variance of each representative along its normal (its noise):
    std::vector<double> noise2(M, 0.0);
    parallel_blocks(
        M, p.parallelization_grain_size,
        [&](std::size_t b, std::size_t e)
        {
            std::vector<Eigen::Vector3d> vp;
            std::vector<double>          absRes;
            std::vector<double>          sortedRes;
            for (std::size_t v = b; v < e; v++)
            {
                vp.clear();
                Moments m;
                for (uint32_t q = voxStart[v]; q < voxStart[v + 1]; q++)
                {
                    const uint32_t i = sorted[q].idx;
                    vp.emplace_back(
                        static_cast<double>(xs[i]) - o.x(), static_cast<double>(ys[i]) - o.y(),
                        static_cast<double>(zs[i]) - o.z());
                    m.add(vp.back());
                }
                rep[v] = m.mean;
                if (m.n < kMinVoxelPoints)
                {
                    continue;
                }
                PlaneFit f = fit_plane(m);
                if (!f.valid || !(f.eig[1] > kMinSpread * f.eig[2]))
                {
                    // Trimmed refits: each principal axis in turn as the
                    // normal, keeping the points closest to that plane.
                    const Eigen::Matrix3d cov = (0.5 / m.n) * (m.m2 + m.m2.transpose());
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
                    f.valid = false;
                    for (int h = 0; h < 3; h++)
                    {
                        const Eigen::Vector3d nh = es.eigenvectors().col(h);
                        absRes.resize(vp.size());
                        for (std::size_t k = 0; k < vp.size(); k++)
                        {
                            absRes[k] = std::abs(nh.dot(vp[k] - m.mean));
                        }
                        sortedRes      = absRes;
                        const auto q75 = sortedRes.begin() + static_cast<std::ptrdiff_t>(
                                                                 (sortedRes.size() * 3) / 4 - 1);
                        std::nth_element(sortedRes.begin(), q75, sortedRes.end());
                        Moments mt;
                        for (std::size_t k = 0; k < vp.size(); k++)
                        {
                            if (absRes[k] <= *q75)
                            {
                                mt.add(vp[k]);
                            }
                        }
                        const PlaneFit ft = fit_plane(mt);
                        if (ft.valid && ft.eig[1] > kMinSpread * ft.eig[2] &&
                            (!f.valid || ft.eig[0] / ft.eig[1] < f.eig[0] / f.eig[1]))
                        {
                            f      = ft;
                            rep[v] = mt.mean;
                        }
                    }
                    if (!f.valid)
                    {
                        continue;
                    }
                }
                nrm[v]    = f.normal;
                flat[v]   = f.eig[0] / f.eig[1];
                noise2[v] = f.eig[0] / m.n;
            }
        });

    // 3) Normals of the representatives, and the neighbor graph.
    const RepCloud repCloud{rep};
    RepTree        tree(3, repCloud, tree_params());
#if NANOFLANN_VERSION < 0x150
    tree.buildIndex();  // newer versions build it in the constructor
#endif

    const std::size_t     K = p.normal_knn;
    const std::size_t     G = K - 1;  // graph slots: neighbors except itself
    std::vector<uint32_t> graph(M * G, kNone);
    const double          maxD2 = p.normal_radius * p.normal_radius;

    parallel_blocks(
        M, p.parallelization_grain_size,
        [&](std::size_t b, std::size_t e)
        {
            std::vector<uint32_t> idx(K);
            std::vector<double>   d2(K);
            for (std::size_t i = b; i < e; i++)
            {
                const std::size_t nFound = tree.knnSearch(rep[i].data(), K, idx.data(), d2.data());
                // Sorted by distance: keep those within the radius.
                std::size_t nNear = 0;
                while (nNear < nFound && d2[nNear] <= maxD2)
                {
                    nNear++;
                }

                std::size_t slot = 0;
                for (std::size_t k = 0; k < nNear; k++)
                {
                    if (idx[k] != i && slot < G)
                    {
                        graph[i * G + slot] = idx[k];
                        slot++;
                    }
                }
                // Voxels with too few points get their normal from the
                // neighbor representatives; if those are not planar at this
                // scale (a noisy map), once more with a wider neighborhood.
                if (std::isfinite(flat[i]) || nNear < kMinNeighbors)
                {
                    continue;
                }
                const auto localFit = [&](std::size_t n)
                {
                    Moments m;
                    for (std::size_t k = 0; k < n; k++)
                    {
                        m.add(rep[idx[k]]);
                    }
                    const Eigen::Matrix3d cov = (0.5 / m.n) * (m.m2 + m.m2.transpose());
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
                    es.computeDirect(cov, Eigen::ComputeEigenvectors);
                    return es;
                };
                const auto planar = [](const Eigen::Vector3d& ev)
                { return ev[1] > kMinSpread * ev[2] && ev[0] < kMaxFlatness * ev[1]; };

                auto es = localFit(nNear);
                if (!planar(es.eigenvalues()) && nNear == K)
                {
                    idx.resize(kWideScale * K);
                    d2.resize(kWideScale * K);
                    const std::size_t nWide =
                        tree.knnSearch(rep[i].data(), kWideScale * K, idx.data(), d2.data());
                    std::size_t nWideIn = 0;
                    while (nWideIn < nWide && d2[nWideIn] <= maxD2)
                    {
                        nWideIn++;
                    }
                    es = localFit(nWideIn);
                    idx.resize(K);
                    d2.resize(K);
                }
                const Eigen::Vector3d ev = es.eigenvalues();
                if (!planar(ev))
                {
                    continue;
                }
                nrm[i]    = es.eigenvectors().col(0).normalized();
                flat[i]   = ev[0] / ev[1];
                noise2[i] = ev[0];
            }
        });

    std::vector<std::pair<double, uint32_t>> seeds;
    for (std::size_t i = 0; i < M; i++)
    {
        if (std::isfinite(flat[i]))
        {
            seeds.emplace_back(flat[i], static_cast<uint32_t>(i));
        }
    }
    result.num_normals = seeds.size();
    sort_range(seeds.begin(), seeds.end());

    // 4) Region growing, flattest seeds first.
    const double          cosGrow = std::cos(mrpt::DEG2RAD(p.normal_agreement_deg));
    std::vector<uint32_t> label(M, kNone);
    uint32_t              numRegions = 0;
    {
        std::vector<uint32_t> queue;
        for (const auto& [seedFlatness, s] : seeds)
        {
            if (label[s] != kNone)
            {
                continue;
            }
            Moments         mom;
            Eigen::Vector3d n = nrm[s];
            Eigen::Vector3d c = rep[s];
            mom.add(rep[s]);
            // The tolerance follows the noise of the representatives, which
            // are single noisy points in a sparse map but averages in a dense
            // one. It is measured at the scale of the normals, where curvature
            // does not show, so curved surfaces still split.
            double      sumNoise2 = noise2[s];
            std::size_t nNoise    = 1;
            const auto  tolerance = [&]()
            {
                const double sigma = std::sqrt(sumNoise2 / static_cast<double>(nNoise));
                return std::min(
                    p.inlier_distance, std::max(p.distance_threshold, kGrowSigmas * sigma));
            };
            double tol = tolerance();
            label[s]   = numRegions;
            queue.assign(1, s);
            std::size_t nextRefit = 8;
            for (std::size_t head = 0; head < queue.size(); head++)
            {
                const uint32_t i = queue[head];
                for (std::size_t slot = 0; slot < G; slot++)
                {
                    const uint32_t j = graph[i * G + slot];
                    if (j == kNone)
                    {
                        break;
                    }
                    if (label[j] != kNone)
                    {
                        continue;
                    }
                    // Representatives without a normal (a single ring of a
                    // sparse scan, edges) only need to lie on the plane:
                    if ((std::isfinite(flat[j]) && std::abs(nrm[j].dot(n)) < cosGrow) ||
                        std::abs(n.dot(rep[j] - c)) > tol)
                    {
                        continue;
                    }
                    label[j] = numRegions;
                    queue.push_back(j);
                    mom.add(rep[j]);
                    if (std::isfinite(flat[j]))
                    {
                        sumNoise2 += noise2[j];
                        nNoise++;
                    }
                    if (static_cast<std::size_t>(mom.n) >= nextRefit)
                    {
                        nextRefit = nextRefit * 3 / 2;
                        // A strip of collinear points does not define a plane yet:
                        // keep the normal until it does.
                        const auto f = fit_plane(mom);
                        if (f.valid)
                        {
                            n = f.normal.dot(n) < 0 ? Eigen::Vector3d(-f.normal) : f.normal;
                        }
                        c   = mom.mean;
                        tol = tolerance();
                    }
                }
            }
            if (queue.size() < kMinRegionSize)
            {
                for (const auto q : queue)
                {
                    label[q] = kNone;
                }
                continue;
            }
            numRegions++;
        }
    }
    result.num_regions = numRegions;
    if (numRegions == 0)
    {
        return result;
    }

    std::vector<Moments> regMom(numRegions);
    for (std::size_t i = 0; i < M; i++)
    {
        if (label[i] != kNone)
        {
            regMom[label[i]].add(rep[i]);
        }
    }
    std::vector<PlaneFit> regFit(numRegions);
    for (uint32_t r = 0; r < numRegions; r++)
    {
        regFit[r] = fit_plane(regMom[r]);
    }

    // 5) Conservative merging. Candidate pairs: regions sharing (or touching)
    // a cell of size `merge_max_gap`.
    std::vector<std::pair<uint32_t, uint32_t>> pairs;
    {
        const double                               g = p.merge_max_gap;
        std::vector<std::pair<uint64_t, uint32_t>> cellReg;  // (cell, region)
        for (std::size_t i = 0; i < M; i++)
        {
            const uint32_t r = label[i];
            if (r == kNone || regMom[r].n < kMinMergeSize || !regFit[r].valid)
            {
                continue;
            }
            cellReg.emplace_back(
                pack_key(
                    cell_index(rep[i].x(), g), cell_index(rep[i].y(), g),
                    cell_index(rep[i].z(), g)),
                r);
        }
        sort_range(cellReg.begin(), cellReg.end());
        cellReg.erase(std::unique(cellReg.begin(), cellReg.end()), cellReg.end());

        // Runs of equal cells:
        std::vector<std::size_t> cellStart;
        for (std::size_t k = 0; k < cellReg.size(); k++)
        {
            if (k == 0 || cellReg[k].first != cellReg[k - 1].first)
            {
                cellStart.push_back(k);
            }
        }
        const std::size_t nCells = cellStart.size();
        cellStart.push_back(cellReg.size());

        const auto findCell = [&](uint64_t key) -> std::size_t
        {
            std::size_t lo = 0;
            std::size_t hi = nCells;
            while (lo < hi)
            {
                const std::size_t mid = (lo + hi) / 2;
                if (cellReg[cellStart[mid]].first < key)
                {
                    lo = mid + 1;
                }
                else
                {
                    hi = mid;
                }
            }
            if (lo < nCells && cellReg[cellStart[lo]].first == key)
            {
                return lo;
            }
            return nCells;
        };

        const uint64_t mask = static_cast<uint64_t>(kMaxCellsPerAxis);
        for (std::size_t ci = 0; ci < nCells; ci++)
        {
            const uint64_t key = cellReg[cellStart[ci]].first;
            const auto     ix  = static_cast<int64_t>((key >> 42) & mask);
            const auto     iy  = static_cast<int64_t>((key >> 21) & mask);
            const auto     iz  = static_cast<int64_t>(key & mask);
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        const int64_t jx = ix + dx;
                        const int64_t jy = iy + dy;
                        const int64_t jz = iz + dz;
                        if (jx < 0 || jy < 0 || jz < 0 || jx > kMaxCellsPerAxis ||
                            jy > kMaxCellsPerAxis || jz > kMaxCellsPerAxis)
                        {
                            continue;
                        }
                        const std::size_t cj = findCell(pack_key(jx, jy, jz));
                        if (cj == nCells || cj < ci)
                        {
                            continue;
                        }
                        for (std::size_t a = cellStart[ci]; a < cellStart[ci + 1]; a++)
                        {
                            for (std::size_t b = cellStart[cj]; b < cellStart[cj + 1]; b++)
                            {
                                const uint32_t ra = cellReg[a].second;
                                const uint32_t rb = cellReg[b].second;
                                if (ra != rb)
                                {
                                    pairs.emplace_back(std::min(ra, rb), std::max(ra, rb));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    sort_range(pairs.begin(), pairs.end());
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());

    UnionFind uf = agglomerate(regMom, regFit, pairs, p);
    pairs.clear();

    // 6) Final clusters.
    std::vector<uint32_t> clusterId(numRegions, kNone);
    std::vector<uint32_t> clusterRoot;
    for (uint32_t r = 0; r < numRegions; r++)
    {
        if (uf.find(r) == r && regFit[r].valid)
        {
            clusterId[r] = static_cast<uint32_t>(clusterRoot.size());
            clusterRoot.push_back(r);
        }
    }
    std::size_t C = clusterRoot.size();

    std::vector<uint32_t> voxCluster(M, kNone);
    std::vector<uint32_t> voxPerCluster(C, 0);
    for (std::size_t v = 0; v < M; v++)
    {
        if (label[v] != kNone)
        {
            voxCluster[v] = clusterId[uf.find(label[v])];
            if (voxCluster[v] != kNone)
            {
                voxPerCluster[voxCluster[v]]++;
            }
        }
    }

    std::vector<Eigen::Vector3d> planeN(C);
    std::vector<Eigen::Vector3d> planeC(C);
    for (std::size_t k = 0; k < C; k++)
    {
        planeN[k] = regFit[clusterRoot[k]].normal;
        planeC[k] = regFit[clusterRoot[k]].point;
    }

    // Expansion: large clusters absorb, through the neighbor graph, the
    // representatives with no cluster, or with a small one, that lie within
    // `inlier_distance` of their plane. This recovers the borders of each
    // plane, where normals are biased by the surface next to it and grow small
    // fragments of their own, and the noisiest voxels. Which plane each point
    // belongs to is decided later, point by point.
    {
        std::vector<uint8_t> small(C, 0);
        for (std::size_t k = 0; k < C; k++)
        {
            small[k] = static_cast<double>(voxPerCluster[k]) * vox * vox < p.min_area ? 1 : 0;
        }
        std::vector<uint32_t> queue;
        for (std::size_t v = 0; v < M; v++)
        {
            if (voxCluster[v] != kNone && !small[voxCluster[v]])
            {
                queue.push_back(static_cast<uint32_t>(v));
            }
        }
        for (std::size_t head = 0; head < queue.size(); head++)
        {
            const uint32_t i = queue[head];
            const uint32_t k = voxCluster[i];
            for (std::size_t slot = 0; slot < G; slot++)
            {
                const uint32_t j = graph[i * G + slot];
                if (j == kNone)
                {
                    break;
                }
                if (voxCluster[j] != kNone && !small[voxCluster[j]])
                {
                    continue;
                }
                if (std::abs(planeN[k].dot(rep[j] - planeC[k])) > p.inlier_distance)
                {
                    continue;
                }
                voxCluster[j] = k;
                queue.push_back(j);
            }
        }
        voxPerCluster.assign(C, 0);
        for (std::size_t v = 0; v < M; v++)
        {
            if (voxCluster[v] != kNone)
            {
                voxPerCluster[voxCluster[v]]++;
            }
        }
    }

    // Frozen clusters keep competing for points with their current plane, but
    // are not refined in the rounds below (each voxel covers at most a few
    // cells of the area grid, so they are unlikely to reach `min_area`). They
    // are still measured at the end, on the points they got.
    std::vector<uint8_t> frozen(C, 0);
    for (std::size_t k = 0; k < C; k++)
    {
        frozen[k] = 9.0 * static_cast<double>(voxPerCluster[k]) * vox * vox < p.min_area ? 1 : 0;
    }

    // 7) Multi-plane refinement. Each point of a voxel that belongs to a
    // cluster goes to the closest plane among the clusters of the voxel and of
    // its neighbors, within `inlier_distance`, so the points of a voxel shared
    // by two close surfaces are split between them. Then each plane is refitted
    // robustly on its own points; twice, then a final assignment.
    PointSource src;
    src.sorted = &sorted;
    src.xs     = &xs;
    src.ys     = &ys;
    src.zs     = &zs;
    src.origin = o;

    const std::size_t     nSorted = sorted.size();
    std::vector<uint32_t> ptCluster(nSorted, kNone);
    std::vector<uint32_t> listStart(C + 1, 0);
    std::vector<uint32_t> list(nSorted);

    const auto assignPoints = [&]()
    {
        parallel_blocks(
            M, p.parallelization_grain_size,
            [&](std::size_t b, std::size_t e)
            {
                std::vector<uint32_t> cand;
                for (std::size_t v = b; v < e; v++)
                {
                    const uint32_t own = voxCluster[v];
                    cand.clear();
                    if (own != kNone)
                    {
                        cand.push_back(own);
                        for (std::size_t slot = 0; slot < G; slot++)
                        {
                            const uint32_t j = graph[v * G + slot];
                            if (j == kNone)
                            {
                                break;
                            }
                            if (voxCluster[j] != kNone)
                            {
                                cand.push_back(voxCluster[j]);
                            }
                        }
                        std::sort(cand.begin(), cand.end());
                        cand.erase(std::unique(cand.begin(), cand.end()), cand.end());
                    }
                    for (uint32_t q = voxStart[v]; q < voxStart[v + 1]; q++)
                    {
                        uint32_t best = kNone;
                        if (!cand.empty())
                        {
                            const Eigen::Vector3d pt    = src.at(q);
                            double                bestR = 0;
                            for (const uint32_t k : cand)
                            {
                                const double r = std::abs(planeN[k].dot(pt - planeC[k]));
                                if (r <= p.inlier_distance && (best == kNone || r < bestR))
                                {
                                    best  = k;
                                    bestR = r;
                                }
                            }
                        }
                        ptCluster[q] = best;
                    }
                }
            });

        // Point lists per cluster, in sorted order (counting sort):
        std::fill(listStart.begin(), listStart.end(), 0U);
        for (std::size_t q = 0; q < nSorted; q++)
        {
            if (ptCluster[q] != kNone)
            {
                listStart[ptCluster[q] + 1]++;
            }
        }
        std::partial_sum(listStart.begin(), listStart.end(), listStart.begin());
        std::vector<uint32_t> fill(listStart.begin(), listStart.end() - 1);
        for (std::size_t q = 0; q < nSorted; q++)
        {
            if (ptCluster[q] != kNone)
            {
                list[fill[ptCluster[q]]++] = static_cast<uint32_t>(q);
            }
        }
    };

    const auto refineAll = [&]()
    {
        parallel_blocks(
            C, 1,
            [&](std::size_t b, std::size_t e)
            {
                for (std::size_t k = b; k < e; k++)
                {
                    const std::size_t nk = listStart[k + 1] - listStart[k];
                    if (frozen[k] || nk < p.min_points || nk < kMinNeighbors)
                    {
                        frozen[k] = 1;
                        continue;
                    }
                    if (!robust_plane_fit(
                            src, list.data() + listStart[k], nk, p.inlier_distance, planeN[k],
                            planeC[k]))
                    {
                        frozen[k] = 1;
                    }
                }
            });
    };

    assignPoints();
    refineAll();

    // Merging again, now on the points: once refined, two segments of one
    // surface (e.g. a strip along an edge, pulled onto the face next to it)
    // share their plane, and would otherwise split its points between them.
    {
        std::vector<Moments>  cMom(C);
        std::vector<PlaneFit> cFit(C);
        parallel_blocks(
            C, 1,
            [&](std::size_t b, std::size_t e)
            {
                for (std::size_t k = b; k < e; k++)
                {
                    for (uint32_t i = listStart[k]; i < listStart[k + 1]; i++)
                    {
                        cMom[k].add(src.at(list[i]));
                    }
                    cFit[k] = fit_plane(cMom[k]);
                }
            });

        // Candidate pairs: clusters of neighbor voxels.
        std::vector<std::pair<uint32_t, uint32_t>> cPairs;
        for (std::size_t v = 0; v < M; v++)
        {
            const uint32_t a = voxCluster[v];
            if (a == kNone)
            {
                continue;
            }
            for (std::size_t slot = 0; slot < G; slot++)
            {
                const uint32_t j = graph[v * G + slot];
                if (j == kNone)
                {
                    break;
                }
                const uint32_t b = voxCluster[j];
                if (b != kNone && b != a && (rep[v] - rep[j]).norm() <= p.merge_max_gap)
                {
                    cPairs.emplace_back(std::min(a, b), std::max(a, b));
                }
            }
        }
        sort_range(cPairs.begin(), cPairs.end());
        cPairs.erase(std::unique(cPairs.begin(), cPairs.end()), cPairs.end());

        UnionFind             uf2 = agglomerate(cMom, cFit, cPairs, p);
        std::vector<uint32_t> members(C, 0);
        for (std::size_t k = 0; k < C; k++)
        {
            members[uf2.find(static_cast<uint32_t>(k))]++;
        }
        std::vector<uint32_t>        newId(C, kNone);
        std::vector<Eigen::Vector3d> newN;
        std::vector<Eigen::Vector3d> newC;
        for (std::size_t k = 0; k < C; k++)
        {
            if (uf2.find(static_cast<uint32_t>(k)) != k)
            {
                continue;
            }
            newId[k] = static_cast<uint32_t>(newN.size());
            if (members[k] > 1)
            {
                newN.push_back(cFit[k].normal);
                newC.push_back(cFit[k].point);
            }
            else
            {
                newN.push_back(planeN[k]);
                newC.push_back(planeC[k]);
            }
        }
        C = newN.size();
        planeN.swap(newN);
        planeC.swap(newC);
        voxPerCluster.assign(C, 0);
        for (std::size_t v = 0; v < M; v++)
        {
            if (voxCluster[v] != kNone)
            {
                voxCluster[v] = newId[uf2.find(voxCluster[v])];
                voxPerCluster[voxCluster[v]]++;
            }
        }
        frozen.assign(C, 0);
        for (std::size_t k = 0; k < C; k++)
        {
            frozen[k] =
                9.0 * static_cast<double>(voxPerCluster[k]) * vox * vox < p.min_area ? 1 : 0;
        }
        listStart.assign(C + 1, 0);
    }
    result.num_merged = C;

    assignPoints();
    refineAll();
    assignPoints();

    // Normal support: the fraction of the points of a plane whose voxel has a
    // normal of its own that agrees with the plane. A slab cut through clutter
    // (vegetation) can be thin enough, but has almost no support.
    std::vector<uint32_t> voxOf(nSorted);
    for (std::size_t v = 0; v < M; v++)
    {
        for (uint32_t q = voxStart[v]; q < voxStart[v + 1]; q++)
        {
            voxOf[q] = static_cast<uint32_t>(v);
        }
    }

    std::vector<SegmentedPlane> planes(C);
    std::vector<uint8_t>        accepted(C, 0);
    parallel_blocks(
        C, 1,
        [&](std::size_t b, std::size_t e)
        {
            for (std::size_t k = b; k < e; k++)
            {
                const std::size_t nk = listStart[k + 1] - listStart[k];
                if (nk < p.min_points || nk < kMinNeighbors)
                {
                    continue;
                }
                // A frozen cluster was not refitted, but the points it got from
                // its neighbors may still make a plane:
                if (frozen[k] && !robust_plane_fit(
                                     src, list.data() + listStart[k], nk, p.inlier_distance,
                                     planeN[k], planeC[k]))
                {
                    continue;
                }
                std::size_t supported = 0;
                for (uint32_t i = listStart[k]; i < listStart[k + 1]; i++)
                {
                    const uint32_t v = voxOf[list[i]];
                    if (std::isfinite(flat[v]) && std::abs(nrm[v].dot(planeN[k])) >= cosGrow)
                    {
                        supported++;
                    }
                }
                if (static_cast<double>(supported) < kMinNormalSupport * static_cast<double>(nk))
                {
                    continue;
                }
                accepted[k] =
                    plane_statistics(
                        src, list.data() + listStart[k], nk, planeN[k], planeC[k], p, planes[k])
                        ? 1
                        : 0;
            }
        });

    // Inlier indices, in ascending order, in one pass over the input order:
    {
        std::vector<uint32_t> clusterOfIdx(nIn, kNone);
        for (std::size_t q = 0; q < nSorted; q++)
        {
            clusterOfIdx[sorted[q].idx] = ptCluster[q];
        }
        for (std::size_t k = 0; k < C; k++)
        {
            if (accepted[k])
            {
                planes[k].inliers.reserve(listStart[k + 1] - listStart[k]);
            }
        }
        for (std::size_t i = 0; i < nIn; i++)
        {
            const uint32_t k = clusterOfIdx[i];
            if (k != kNone && accepted[k])
            {
                planes[k].inliers.push_back(static_cast<uint32_t>(i));
            }
        }
    }

    for (std::size_t k = 0; k < C; k++)
    {
        if (accepted[k])
        {
            result.planes.push_back(std::move(planes[k]));
        }
    }
    std::sort(
        result.planes.begin(), result.planes.end(),
        [](const SegmentedPlane& a, const SegmentedPlane& b)
        {
            if (a.inliers.size() != b.inliers.size())
            {
                return a.inliers.size() > b.inliers.size();
            }
            return a.inliers.front() < b.inliers.front();
        });

    return result;

    MRPT_END
}

}  // namespace mp2p_icp_filters
