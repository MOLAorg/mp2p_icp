/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   visit_correspondences.h
 * @brief  Template that applies lambdas to unified vector forms of pairings.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 19, 2019
 */
#pragma once

#include <mp2p_icp/optimal_tf_common.h>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

/** Visit each correspondence */
template <class LAMBDA, class LAMBDA2>
void visit_correspondences(
    const Pairings_Common& in, const mrpt::math::TPoint3D& ct_other,
    const mrpt::math::TPoint3D& ct_this,OutlierIndices& in_out_outliers,
    LAMBDA lambda_each_pair, LAMBDA2 lambda_final)
{
    using mrpt::math::TPoint3D;
    using mrpt::math::TVector3D;

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

    OutlierIndices new_outliers;
    new_outliers.point2point.reserve(in_out_outliers.point2point.size());

    std::size_t cnt             = 0;
    auto        it_next_outlier = in_out_outliers.point2point.begin();

    // Terms contributed by points & vectors have now the uniform form of
    // unit vectors:
    for (std::size_t i = 0; i < nAllMatches; i++)
    {
        // Skip outlier?
        if (it_next_outlier != in_out_outliers.point2point.end() &&
            i == *it_next_outlier)
        {
            ++it_next_outlier;
            // also copy idx:
            new_outliers.point2point.push_back(i);
            continue;
        }

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
                    new_outliers.point2point.push_back(i);

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

        // Visit this pair:
        lambda_each_pair(bi, ri, wi);

    }  // for each match

    in_out_outliers = std::move(new_outliers);

    lambda_final(w_sum);

}  // end visit_correspondences()

/** @} */

}  // namespace mp2p_icp
