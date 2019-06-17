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

/** Rotates a vector along +Z 90 degrees CCW */
static auto vector_rot_Z_90d_CCW(const mrpt::math::TVector3D& v)
{
    return mrpt::math::TVector3D(-v.y, v.x, v.z);
}

/** Core of the OLAE algorithm.
 * Factored out here so it can be re-evaluated if the solution is near the Gibbs
 * vector singularity (|Phi|~= \pi)
 * (Refer to technical report for details) */
static std::tuple<Eigen::Matrix3d, Eigen::Vector3d, std::vector<std::size_t>>
    olae_build_linear_system(
        const Pairings_OLAE& in, const mrpt::math::TPoint3D& ct_other,
        const mrpt::math::TPoint3D& ct_this,
        const bool                  do_relinearize_singularity)
{
    MRPT_START

    using mrpt::math::TPoint3D;
    using mrpt::math::TVector3D;

    // Build the linear system: M g = v
    Eigen::Matrix3d          M = Eigen::Matrix3d::Zero();
    Eigen::Vector3d          v = Eigen::Vector3d::Zero();
    std::vector<std::size_t> outlierIndices;

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
        const auto wPt = in.weights.attitude.points,
                   wLi = in.weights.attitude.lines,
                   wPl = in.weights.attitude.planes;

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
            ASSERT_ABOVE_(bi_n, 1e-8);
            ASSERT_ABOVE_(ri_n, 1e-8);
            // (Note: ideally, both norms should be equal if noiseless and a
            // real pairing)
            bi *= 1.0 / bi_n;
            ri *= 1.0 / ri_n;

            if (in.use_scale_outlier_detector)
            {
                const double scale_mismatch =
                    std::max(bi_n, ri_n) / std::min(bi_n, ri_n);
                if (scale_mismatch > in.scale_outlier_threshold)
                {
                    // Discard this pairing:
                    outlierIndices.push_back(i);
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

        // If we are in a second stage, let's relinearize around a rotation
        // of +PI/2 along +Z, to avoid working near the Gibbs vector
        // singularity:
        if (do_relinearize_singularity)
        {
            // Rotate:
            ri = vector_rot_Z_90d_CCW(ri);
        }

        ASSERT_(wi > .0);
        w_sum += wi;

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

        /* v-= [b_i]_{x}  r_i
         *  Each term is:
         *  ⎡by⋅rz - bz⋅ry ⎤
         *  ⎢              ⎥
         *  ⎢-bx⋅rz + bz⋅rx⎥
         *  ⎢              ⎥
         *  ⎣bx⋅ry - by⋅rx ⎦
         */

        // clang-format off
        const auto dV = (Eigen::Vector3d() <<
           (bi.y * ri.z - bi.z * ri.y),
           (-bi.x * ri.z + bi.z * ri.x),
           (bi.x * ri.y - bi.y * ri.x) ).finished();
        // clang-format on

        M += wi * dM;
        v -= wi * dV;
    }

    if (w_sum > .0)
    {
        M *= (1.0 / w_sum);
        v *= (1.0 / w_sum);
    }
    else
    {
        // We either had NO input correspondences, or ALL were detected as
        // inliers... What to do in this case?
    }

    // The missing (1/2) from the formulas above:
    M *= 0.5;

    return {M, v, outlierIndices};

    MRPT_END
}

// "Markley, F. L., & Mortari, D. (1999). How to estimate attitude from
// vector observations."
static double olae_estimate_Phi(const double M_det, std::size_t n)
{
    return std::acos((M_det / (n == 2 ? -1.0 : -1.178)) - 1.);
}

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

// Evaluates the centroids [ct_other, ct_this] for points and plane
// correspondences, taking into account the current guess for outliers:
static std::tuple<mrpt::math::TPoint3D, mrpt::math::TPoint3D>
    eval_centroids_robust(
        const Pairings_OLAE& in, const std::vector<std::size_t>& pt2pt_outliers)
{
    using mrpt::math::TPoint3D;

    TPoint3D   ct_other(0, 0, 0), ct_this(0, 0, 0);
    const auto nPoints = in.paired_points.size();
    const auto nLines  = in.paired_lines.size();
    const auto nPlanes = in.paired_planes.size();

    // Normalized weights for centroid "wcXX":
    double wcPoints, wcPlanes;
    {
        const auto wPt = in.weights.translation.points,
                   wPl = in.weights.translation.planes;

        ASSERTMSG_(
            wPt + wPl > .0,
            "Both, point and plane translation weights, are <=0 (!)");

        // Discount outliers:
        const auto k =
            1.0 / (wPt * (nPoints - pt2pt_outliers.size()) + wPl * nPlanes);
        wcPoints = wPt * k;
        wcPlanes = wPl * k;
    }

    // Add global coordinate of points for now, we'll convert them later to
    // unit vectors relative to the centroids:
    {
        TPoint3D ct_other_pt(0, 0, 0), ct_this_pt(0, 0, 0);

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

            ct_this_pt += TPoint3D(pair.this_x, pair.this_y, pair.this_z);
            ct_other_pt += TPoint3D(pair.other_x, pair.other_y, pair.other_z);
            cnt++;
        }
        ASSERT_EQUAL_(cnt, nPoints - pt2pt_outliers.size());

        ct_other_pt *= wcPoints;
        ct_this_pt *= wcPoints;

        // Add plane centroids to the computation of centroids as well:
        TPoint3D ct_other_pl(0, 0, 0), ct_this_pl(0, 0, 0);
        if (wcPlanes > 0)
        {
            for (const auto& pair : in.paired_planes)
            {
                ct_this_pl += pair.p_this.centroid;
                ct_other_pl += pair.p_other.centroid;
            }
            ct_this_pl *= wcPlanes;
            ct_other_pl *= wcPlanes;
        }

        // Normalize sum of centroids:
        ct_other = ct_other_pt + ct_other_pl;
        ct_this  = ct_this_pt + ct_this_pl;

        // sanity check of weights:
        const double expected_sum_1 =
            wcPlanes * in.paired_planes.size() +
            wcPoints * (nPoints - pt2pt_outliers.size());
        ASSERT_BELOW_(std::abs(expected_sum_1 - 1.0), 0.01);
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

    const auto nPoints     = in.paired_points.size();
    const auto nLines      = in.paired_lines.size();
    const auto nPlanes     = in.paired_planes.size();
    const auto nAllMatches = nPoints + nLines + nPlanes;

    // Reset output to defaults:
    result = OptimalTF_Result();

    // Normalize weights for each feature type and for each target (attitude
    // / translation):
    ASSERT_(in.weights.attitude.points >= .0);
    ASSERT_(in.weights.attitude.lines >= .0);
    ASSERT_(in.weights.attitude.planes >= .0);
    ASSERT_(in.weights.translation.points >= .0);
    ASSERT_(in.weights.translation.planes >= .0);

    // Compute the centroids:
    auto [ct_other, ct_this] =
        eval_centroids_robust(in, result.outliers /* empty for now  */);

    // Build the linear system: M g = v
    auto [M, v, outliers] = olae_build_linear_system(
        in, ct_other, ct_this, false /*dont relinearize singularity*/);

    // Re-evaluate the centroids, now that we have a guess on outliers.
    if (!outliers.empty())
    {
        // Re-evaluate the centroids:
        const auto [new_ct_other, new_ct_this] =
            eval_centroids_robust(in, outliers);

        ct_other = new_ct_other;
        ct_this  = new_ct_this;

        // And rebuild the linear system with the new values:
        auto [new_M, new_v, new_outliers] = olae_build_linear_system(
            in, ct_other, ct_this, false /*dont relinearize singularity*/);

        M = new_M;
        v = new_v;
    }

    // We are finding this optimal rotation, as a Gibbs vector:
    Eigen::Vector3d optimal_rot;

    // Solve linear system for optimal rotation:
    const double Md = M.determinant();

    // Estimate |Phi|:
    const double estPhi1 = olae_estimate_Phi(Md, nAllMatches);

    // Threshold to decide whether to do a re-linearization:
    const bool do_relinearize = (estPhi1 > in.OLAE_relinearize_threshold);

    if (do_relinearize)
    {
        // relinearize on a different orientation:
        const auto [M2, v2, outliers2] = olae_build_linear_system(
            in, ct_other, ct_this, true /*DO relinearize singularity*/);

        // Find the optimal Gibbs vector:
        optimal_rot     = M2.colPivHouseholderQr().solve(v2);
        result.outliers = outliers2;
    }
    else
    {
        // Find the optimal Gibbs vector:
        optimal_rot     = M.colPivHouseholderQr().solve(v);
        result.outliers = outliers;
    }

    result.optimal_pose = gibbs2pose(optimal_rot);

    // Undo transformation above:
    if (do_relinearize)
    {
        result.optimal_pose = result.optimal_pose +
                              mrpt::poses::CPose3D(0, 0, 0, M_PI * 0.5, 0, 0);
    }

    // Use centroids to solve for optimal translation:
    mrpt::math::TPoint3D pp;
    result.optimal_pose.composePoint(
        ct_other.x, ct_other.y, ct_other.z, pp.x, pp.y, pp.z);

    result.optimal_pose.x(ct_this.x - pp.x);
    result.optimal_pose.y(ct_this.y - pp.y);
    result.optimal_pose.z(ct_this.z - pp.z);

    MRPT_END
}
