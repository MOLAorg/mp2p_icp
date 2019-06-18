/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   optimal_tf_olae.cpp
 * @brief  OLAE algorithm to find the SE(3) optimal transformation
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */

#include <mp2p_icp/optimal_tf_olae.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/tfest/se3.h>
#include <Eigen/Dense>

using namespace mp2p_icp;

// Convert to quaternion by normalizing q=[1, optim_rot], then to rot. matrix:
static mrpt::poses::CPose3D gibbs2pose(const Eigen::Vector3d& v)
{
    auto       x = v[0], y = v[1], z = v[2];
    const auto r = 1.0 / std::sqrt(1.0 + x * x + y * y + z * z);
    x *= r;
    y *= r;
    z *= r;
    auto q = mrpt::math::CQuaternionDouble(r, -x, -y, -z);

    // Quaternion to 3x3 rot matrix:
    return mrpt::poses::CPose3D(q, .0, .0, .0);
}

/** The systems built by olae_build_linear_system.
 * The system is: "M g = v".
 *
 * However, if the solution is near the Gibbs vector singularity (|Phi|~= \pi)
 * we may need to use the alternative systems built by the
 * "sequential rotation method" [shuster1981attitude].
 *
 * (Refer to technical report for details)
 */
struct OLAE_LinearSystems
{
    Eigen::Matrix3d M, Mx, My, Mz;
    Eigen::Vector3d v, vx, vy, vz;

    /** Attitude profile matrix */
    Eigen::Matrix3d B;

    std::vector<std::size_t> outlierIndices;
};

/** Core of the OLAE algorithm  */
static OLAE_LinearSystems olae_build_linear_system(
    const Pairings_OLAE& in, const mrpt::math::TPoint3D& ct_other,
    const mrpt::math::TPoint3D& ct_this)
{
    MRPT_START

    using mrpt::math::TPoint3D;
    using mrpt::math::TVector3D;

    OLAE_LinearSystems res;

    // Build the linear system: M g = v
    res.M = Eigen::Matrix3d::Zero();
    res.v = Eigen::Vector3d::Zero();

    // Attitude profile matrix:
    res.B = Eigen::Matrix3d::Zero();

    const auto nPoints     = in.paired_points.size();
    const auto nLines      = in.paired_lines.size();
    const auto nPlanes     = in.paired_planes.size();
    const auto nAllMatches = nPoints + nLines + nPlanes;

    // weight of points, block by block:
    auto point_weights = in.point_weights;
    if (point_weights.empty())
    {
        // Default, equal weights:
        point_weights.emplace_back(nPoints, 1.0);
    }

    auto        cur_point_block_weights = point_weights.begin();
    std::size_t cur_point_block_start   = 0;

    // Normalized weights for attitude "waXX":
    double waPoints, waLines, waPlanes;
    {
        const auto wPt = in.attitude_weights.pt2pt,
                   wLi = in.attitude_weights.l2l,
                   wPl = in.attitude_weights.pl2pl;

        ASSERTMSG_(
            wPt + wLi + wPl > .0,
            "All, point, line, plane attidude weights, are <=0 (!)");

        const auto k = 1.0 / (wPt * nPoints + wLi * nLines + wPl * nPlanes);
        waPoints     = wPt * k;
        waLines      = wLi * k;
        waPlanes     = wPl * k;
    }

    // Accumulator of robust kernel terms (and other user-provided weights)
    // to normalize the final linear equation at the end:
    double w_sum = .0;

    // Terms contributed by points & vectors have now the uniform form of
    // unit vectors:

    for (std::size_t i = 0; i < nAllMatches; i++)
    {
        // Get "bi" (this/global) & "ri" (other/local) vectors:
        TVector3D bi, ri;
        double    wi = .0;

        // Points, lines, planes, are all stored in sequence:
        if (i < nPoints)
        {
            // point-to-point pairing:  normalize(point-centroid)
            const auto& p = in.paired_points[i];
            wi            = waPoints;

            if (i >= cur_point_block_start + cur_point_block_weights->first)
            {
                ASSERT_(cur_point_block_weights != point_weights.end());
                ++cur_point_block_weights;  // move to next block
                cur_point_block_start = i;
            }
            wi *= cur_point_block_weights->second;
            // (solution will be normalized via w_sum a the end)

            bi = TVector3D(p.this_x, p.this_y, p.this_z) - ct_this;
            ri = TVector3D(p.other_x, p.other_y, p.other_z) - ct_other;
            const auto bi_n = bi.norm(), ri_n = ri.norm();

            if (bi_n < 1e-4 || ri_n < 1e-4)
            {
                // In the rare event of a point (almost) exactly on the
                // centroid, just discard it:
                continue;
            }

            // Note: ideally, both norms should be equal if noiseless and a
            // real pairing. Let's use this property to detect outliers:
            bi *= 1.0 / bi_n;
            ri *= 1.0 / ri_n;

            if (in.use_scale_outlier_detector)
            {
                const double scale_mismatch =
                    std::max(bi_n, ri_n) / std::min(bi_n, ri_n);
                if (scale_mismatch > in.scale_outlier_threshold)
                {
                    // Discard this pairing:
                    res.outlierIndices.push_back(i);
                    continue;  // Skip (same effect than: wi = 0)
                }
            }
        }
        else if (i < nPoints + nLines)
        {
            // line-to-line pairing:
            wi = waLines;

            const auto idxLine = i - nPoints;
            MRPT_TODO("handle lines");
            THROW_EXCEPTION("handle lines");
        }
        else
        {
            // plane-to-plane pairing:
            wi = waPlanes;

            const auto idxPlane = i - (nPoints + nLines);
            bi = in.paired_planes[idxPlane].p_this.plane.getNormalVector();
            ri = in.paired_planes[idxPlane].p_other.plane.getNormalVector();

            ASSERTDEB_BELOW_(std::abs(bi.norm() - 1.0), 0.01);
            ASSERTDEB_BELOW_(std::abs(ri.norm() - 1.0), 0.01);
        }

        // If we are about to apply a robust kernel, we need a reference
        // attitude wrt which apply such kernel, i.e. the "current SE(3)
        // estimation" inside a caller ICP loop.
        if (in.use_robust_kernel)
        {
            const TVector3D ri2 =
                in.current_estimate_for_robust.composePoint(ri);

            // mismatch angle between the two vectors:
            const double ang =
                std::acos(ri2.x * bi.x + ri2.y * bi.y + ri2.z * bi.z);
            const double A = in.robust_kernel_param;
            const double B = in.robust_kernel_scale;
            if (ang > A)
            {
                const auto f = 1.0 / (1.0 + B * mrpt::square(ang - A));
                wi *= f;
            }
        }

        ASSERT_(wi > .0);
        w_sum += wi;

        // We will evaluate M from an alternative expression below from the
        // attitude profile matrix B instead, since it seems to be slightly more
        // stable, numerically. The original code for M is left here for
        // reference, though.
#if 0
        // M+=(1/2)* ([s_i]_{x})^2
        // with: s_i = b_i + r_i
        const double sx = bi.x + ri.x, sy = bi.y + ri.y, sz = bi.z + ri.z;

        /* ([s_i]_{x})^2 is:
         *
         *  ⎡    2     2                          ⎤
         *  ⎢- sy  - sz      sx⋅sy        sx⋅sz   ⎥
         *  ⎢                                     ⎥
         *  ⎢                 2     2             ⎥
         *  ⎢   sx⋅sy     - sx  - sz      sy⋅sz   ⎥
         *  ⎢                                     ⎥
         *  ⎢                              2     2⎥
         *  ⎣   sx⋅sz        sy⋅sz     - sx  - sy ⎦
         */
        const double c00 = -sy * sy - sz * sz;
        const double c11 = -sx * sx - sz * sz;
        const double c22 = -sx * sx - sy * sy;
        const double c01 = sx * sy;
        const double c02 = sx * sz;
        const double c12 = sy * sz;

        // clang-format off
        const auto dM = (Eigen::Matrix3d() <<
           c00, c01, c02,
           c01, c11, c12,
           c02, c12, c22 ).finished();
        // clang-format on

        // res.M += wi * dM;
#endif

        /* v-= weight *  [b_i]_{x}  r_i
         *  Each term is:
         *  ⎡by⋅rz - bz⋅ry ⎤   ⎡ B23 - B32 ⎤
         *  ⎢              ⎥   |           ⎥
         *  ⎢-bx⋅rz + bz⋅rx⎥ = | B31 - B13 ⎥
         *  ⎢              ⎥   |           ⎥
         *  ⎣bx⋅ry - by⋅rx ⎦   ⎣ B12 - B21 ⎦
         *
         * B (attitude profile matrix):
         *
         * B+= weight * (b_i * r_i')
         *
         */

        // clang-format off
        const auto dV = (Eigen::Vector3d() <<
           (bi.y * ri.z - bi.z * ri.y),
           (-bi.x * ri.z + bi.z * ri.x),
           (bi.x * ri.y - bi.y * ri.x) ).finished();
        // clang-format on

        res.v -= wi * dV;

        // clang-format off
        const auto dB = (Eigen::Matrix3d() <<
           bi.x * ri.x, bi.x * ri.y, bi.x * ri.z,
           bi.y * ri.x, bi.y * ri.y, bi.y * ri.z,
           bi.z * ri.x, bi.z * ri.y, bi.z * ri.z).finished();
        // clang-format on
        res.B += wi * dB;
    }

    // Normalize weights. OLAE assumes \sum(w_i) = 1.0
    if (w_sum > .0)
    {
        const auto f = (1.0 / w_sum);
        res.M *= f;
        res.v *= f;
        res.B *= f;
    }
    else
    {
        // We either had NO input correspondences, or ALL were detected as
        // inliers... What to do in this case?
    }

    // The missing (1/2) from the formulas above:
    res.M *= 0.5;

    // Now, compute the other three sets of linear systems, corresponding
    // to the "sequential rotation method" [shuster1981attitude], so we can
    // later keep the best one (i.e. the one with the largest |M|).
    {
        const Eigen::Matrix3d S = res.B + res.B.transpose();
        const double          p = res.B.trace() + 1;
        const double          m = res.B.trace() - 1;
        // Short cut:
        const auto& v = res.v;

        // Set #0: M g=v, without further rotations (the system built above).
        // clang-format off
        res.M = (Eigen::Matrix3d() <<
           S(0,0)-p,  S(0,1), S(0,2),
           S(0,1),   S(1,1)-p, S(1,2),
           S(0,2),   S(1,2),  S(2,2)-p ).finished();

        const auto &M0 = res.M; // shortcut
        const double z1 = v[0], z2 = v[1], z3 = v[2];

        // Set #1: rotating 180 deg around "x":
        // clang-format off
        res.Mx = (Eigen::Matrix3d() <<
           m     ,      -z3  ,     z2,
           -z3   ,  M0(2,2),     -S(1,2),
           z2    ,  -S(1,2),    M0(1,1)).finished();
        res.vx = (Eigen::Vector3d() <<
            -z1, S(0,2), -S(0,1)
            ).finished();
        // clang-format on

        // Set #2: rotating 180 deg around "y":
        // clang-format off
        res.My = (Eigen::Matrix3d() <<
           M0(2,2),     z3  ,     -S(0,2),
           z3     ,       m ,     -z1,
         -S(0,2)  ,     -z1 ,   M0(0,0)).finished();
        res.vy = (Eigen::Vector3d() <<
            -S(1,2), -z2, S(0,1)
            ).finished();
        // clang-format on

        // Set #3: rotating 180 deg around "z":
        // clang-format off
        res.Mz = (Eigen::Matrix3d() <<
           M0(1,1),  -S(0,1),     -z2,
          -S(0,1) ,  M0(0,0),      z1,
             -z2  ,      z1 ,      m).finished();
        res.vz = (Eigen::Vector3d() <<
            S(1,2), -S(0,2), -z3
            ).finished();
        // clang-format on
    }

    return res;

    MRPT_END
}

// Evaluates the centroids [ct_other, ct_this] for points and plane
// correspondences, taking into account the current guess for outliers:
static std::tuple<mrpt::math::TPoint3D, mrpt::math::TPoint3D>
    eval_centroids_robust(
        const Pairings_OLAE& in, const std::vector<std::size_t>& pt2pt_outliers)
{
    using mrpt::math::TPoint3D;

    const auto nPoints = in.paired_points.size();
    const auto nLines  = in.paired_lines.size();
    const auto nPlanes = in.paired_planes.size();

    // We need more points than outliers (!)
    ASSERT_ABOVE_(nPoints, pt2pt_outliers.size());

    // Normalized weights for centroids.
    // Discount outliers.
    const double wcPoints = 1.0 / (nPoints - pt2pt_outliers.size());

    // Add global coordinate of points for now, we'll convert them later to
    // unit vectors relative to the centroids:
    TPoint3D ct_other(0, 0, 0), ct_this(0, 0, 0);
    {
        std::size_t cnt             = 0;
        auto        it_next_outlier = pt2pt_outliers.begin();
        for (std::size_t i = 0; i < in.paired_points.size(); i++)
        {
            // Skip outlier?
            if (it_next_outlier != pt2pt_outliers.end() &&
                i == *it_next_outlier)
            {
                ++it_next_outlier;
                continue;
            }
            const auto& pair = in.paired_points[i];

            ct_this += TPoint3D(pair.this_x, pair.this_y, pair.this_z);
            ct_other += TPoint3D(pair.other_x, pair.other_y, pair.other_z);
            cnt++;
        }
        ASSERT_EQUAL_(cnt, nPoints - pt2pt_outliers.size());

        ct_other *= wcPoints;
        ct_this *= wcPoints;
    }

    return {ct_other, ct_this};
}

// See .h docs, and associated technical report.
void mp2p_icp::optimal_tf_olae(
    const Pairings_OLAE& in, OptimalTF_Result& result)
{
    MRPT_START

    using mrpt::math::TPoint3D;
    using mrpt::math::TVector3D;

    // Note on notation: we are search the relative transformation of
    // the "other" frame wrt to "this", i.e. "this"="global",
    // "other"="local":
    //   p_this = pose \oplus p_other
    //   p_A    = pose \oplus p_B      --> pB = p_A \ominus pose

    // Reset output to defaults:
    result = OptimalTF_Result();

    // Normalize weights for each feature type and for each target (attitude
    // / translation):
    ASSERT_(in.attitude_weights.pt2pt >= .0);
    ASSERT_(in.attitude_weights.l2l >= .0);
    ASSERT_(in.attitude_weights.pl2pl >= .0);

    // Compute the centroids:
    auto [ct_other, ct_this] =
        eval_centroids_robust(in, result.outliers /* empty for now  */);

    // Build the linear system: M g = v
    OLAE_LinearSystems linsys = olae_build_linear_system(in, ct_other, ct_this);

    MRPT_TODO("Refactor to avoid duplicated code? Is it possible?");

    // Re-evaluate the centroids, now that we have a guess on outliers.
    if (!linsys.outlierIndices.empty())
    {
        // Re-evaluate the centroids:
        const auto [new_ct_other, new_ct_this] =
            eval_centroids_robust(in, linsys.outlierIndices);

        ct_other = new_ct_other;
        ct_this  = new_ct_this;

        // And rebuild the linear system with the new values:
        linsys = olae_build_linear_system(in, ct_other, ct_this);
    }

    // We are finding the optimal rotation "g", as a Gibbs vector.
    // Solve linear system for optimal rotation: M g = v

    const double detM_orig = std::abs(linsys.M.determinant()),
                 detMx     = std::abs(linsys.Mx.determinant()),
                 detMy     = std::abs(linsys.My.determinant()),
                 detMz     = std::abs(linsys.Mz.determinant());

#if 0
    // clang-format off
    std::cout << " |M_orig|= " << detM_orig << "\n"
                 " |M_x|   = " << detMx << "\n"
                 " |M_t|   = " << detMy << "\n"
                 " |M_z|   = " << detMz << "\n";
    // clang-format on
#endif

    if (detM_orig > mrpt::max3(detMx, detMy, detMz))
    {
        // original rotation is the best numerically-determined problem:
        const auto sol0 =
            gibbs2pose(linsys.M.colPivHouseholderQr().solve(linsys.v));
        result.optimal_pose = sol0;
#if 0
        std::cout << "M   : |M|="
                  << mrpt::format("%16.07f", linsys.M.determinant())
                  << " sol: " << sol0.asString() << "\n";
#endif
    }
    else if (detMx > mrpt::max3(detM_orig, detMy, detMz))
    {
        // rotation wrt X is the best choice:
        auto sol1 =
            gibbs2pose(linsys.Mx.colPivHouseholderQr().solve(linsys.vx));
        sol1                = mrpt::poses::CPose3D(0, 0, 0, 0, 0, M_PI) + sol1;
        result.optimal_pose = sol1;
#if 0
        std::cout << "M_x : |M|="
                  << mrpt::format("%16.07f", linsys.Mx.determinant())
                  << " sol: " << sol1.asString() << "\n";
#endif
    }
    else if (detMy > mrpt::max3(detM_orig, detMx, detMz))
    {
        // rotation wrt Y is the best choice:
        auto sol2 =
            gibbs2pose(linsys.My.colPivHouseholderQr().solve(linsys.vy));
        sol2                = mrpt::poses::CPose3D(0, 0, 0, 0, M_PI, 0) + sol2;
        result.optimal_pose = sol2;
#if 0
        std::cout << "M_y : |M|="
                  << mrpt::format("%16.07f", linsys.My.determinant())
                  << " sol: " << sol2.asString() << "\n";
#endif
    }
    else
    {
        // rotation wrt Z is the best choice:
        auto sol3 =
            gibbs2pose(linsys.Mz.colPivHouseholderQr().solve(linsys.vz));
        sol3                = mrpt::poses::CPose3D(0, 0, 0, M_PI, 0, 0) + sol3;
        result.optimal_pose = sol3;
#if 0
        std::cout << "M_z : |M|="
                  << mrpt::format("%16.07f", linsys.Mz.determinant())
                  << " sol: " << sol3.asString() << "\n";
#endif
    }

    result.outliers = linsys.outlierIndices;

    // Use centroids to solve for optimal translation:
    mrpt::math::TPoint3D pp;
    result.optimal_pose.composePoint(
        ct_other.x, ct_other.y, ct_other.z, pp.x, pp.y, pp.z);

    result.optimal_pose.x(ct_this.x - pp.x);
    result.optimal_pose.y(ct_this.y - pp.y);
    result.optimal_pose.z(ct_this.z - pp.z);

    MRPT_END
}
