/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   visit_correspondences.h
 * @brief  Template that applies lambdas to unified vector forms of pairings.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 19, 2019
 */
#pragma once

#include <mp2p_icp/Pairings.h>
#include <mp2p_icp/WeightParameters.h>
#include <mrpt/core/optional_ref.h>
#include <mrpt/math/TPoint3D.h>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

struct VisitCorrespondencesStats
{
    uint32_t num_pairings_discarded_scale_outliers = 0;
};

/** Visit each correspondence */
template <class LAMBDA, class LAMBDA2>
void visit_correspondences(
    const Pairings& in, const WeightParameters& wp,
    const mrpt::math::TPoint3D& ct_local, const mrpt::math::TPoint3D& ct_global,
    OutlierIndices& in_out_outliers, LAMBDA lambda_each_pair,
    LAMBDA2 lambda_final, bool normalize_relative_point_vectors,
    const mrpt::optional_ref<VisitCorrespondencesStats>& outStats =
        std::nullopt)
{
    using mrpt::math::TPoint3D;
    using mrpt::math::TVector3D;

    const auto nPt2Pt = in.paired_pt2pt.size();
    const auto nPt2Ln = in.paired_pt2ln.size();
    const auto nPt2Pl = in.paired_pt2pl.size();
    const auto nLn2Ln = in.paired_ln2ln.size();
    const auto nPl2Pl = in.paired_pl2pl.size();

    ASSERTMSG_(
        nPt2Ln == 0, "This solver cannot handle point-to-line pairings.");
    ASSERTMSG_(
        nPt2Pl == 0, "This solver cannot handle point-to-plane pairings yet.");

    const auto nAllMatches = nPt2Pt + nLn2Ln + nPl2Pl;

    VisitCorrespondencesStats stats;

    // weight of points, block by block:
    auto point_weights = in.point_weights;
    if (point_weights.empty())
    {
        // Default, equal weights:
        point_weights.emplace_back(nPt2Pt, 1.0);
    }

    auto        cur_point_block_weights = point_weights.begin();
    std::size_t cur_point_block_start   = 0;

    // Normalized weights for attitude "waXX":
    double waPoints, waLines, waPlanes;
    {
        const auto wPt = wp.pair_weights.pt2pt, wLi = wp.pair_weights.ln2ln,
                   wPl = wp.pair_weights.pl2pl;

        ASSERTMSG_(
            wPt + wLi + wPl > .0,
            "All, point, line, plane attidude weights, are <=0 (!)");

        const auto k = 1.0 / (wPt * nPt2Pt + wLi * nLn2Ln + wPl * nPl2Pl);
        waPoints     = wPt * k;
        waLines      = wLi * k;
        waPlanes     = wPl * k;
    }

    // Accumulator of robust kernel terms (and other user-provided weights)
    // to normalize the final linear equation at the end:
    double w_sum = .0;

    OutlierIndices new_outliers;
    new_outliers.point2point.reserve(in_out_outliers.point2point.size());

    auto it_next_outlier = in_out_outliers.point2point.begin();

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
        if (i < nPt2Pt)
        {
            // point-to-point pairing:  normalize(point-centroid)
            const auto& p = in.paired_pt2pt[i];
            wi            = waPoints;

            if (i >= cur_point_block_start + cur_point_block_weights->first)
            {
                ASSERT_(cur_point_block_weights != point_weights.end());
                ++cur_point_block_weights;  // move to next block
                cur_point_block_start = i;
            }
            wi *= cur_point_block_weights->second;
            // (solution will be normalized via w_sum a the end)

            bi = p.global - ct_global;
            ri = p.local - ct_local;

            const auto bi_n = bi.norm(), ri_n = ri.norm();

            if (bi_n < 1e-4 || ri_n < 1e-4)
            {
                // In the rare event of a point (almost) exactly on the
                // centroid, just discard it:
                continue;
            }

            // Horn requires regular relative vectors.
            // OLAE requires unit vectors.
            if (normalize_relative_point_vectors)
            {
                bi *= 1.0 / bi_n;
                ri *= 1.0 / ri_n;
            }

            // Note: ideally, both norms should be equal if noiseless and a
            // real pairing. Let's use this property to detect outliers:
            if (wp.use_scale_outlier_detector)
            {
                const double scale_mismatch =
                    std::max(bi_n, ri_n) / std::min(bi_n, ri_n);
                if (scale_mismatch > wp.scale_outlier_threshold)
                {
                    // Discard this pairing:
                    new_outliers.point2point.push_back(i);
                    stats.num_pairings_discarded_scale_outliers++;

                    continue;  // Skip (same effect than: wi = 0)
                }
            }
        }
        else if (i < nPt2Pt + nLn2Ln)
        {
            // line-to-line pairing:
            wi = waLines;

            const auto idxLine = i - nPt2Pt;

            bi = in.paired_ln2ln[idxLine].ln_global.getDirectorVector();
            ri = in.paired_ln2ln[idxLine].ln_local.getDirectorVector();

            ASSERTDEB_LT_(std::abs(bi.norm() - 1.0), 0.01);
            ASSERTDEB_LT_(std::abs(ri.norm() - 1.0), 0.01);
        }
        else
        {
            // plane-to-plane pairing:
            wi = waPlanes;

            const auto idxPlane = i - (nPt2Pt + nLn2Ln);
            bi = in.paired_pl2pl[idxPlane].p_global.plane.getNormalVector();
            ri = in.paired_pl2pl[idxPlane].p_local.plane.getNormalVector();

            ASSERTDEB_LT_(std::abs(bi.norm() - 1.0), 0.01);
            ASSERTDEB_LT_(std::abs(ri.norm() - 1.0), 0.01);
        }

        // If we are about to apply a robust kernel, we need a reference
        // attitude wrt which apply such kernel, i.e. the "current SE(3)
        // estimation" inside a caller ICP loop.
        if (wp.use_robust_kernel)
        {
            ASSERT_(wp.currentEstimateForRobust.has_value());
            const TVector3D ri2 = wp.currentEstimateForRobust->composePoint(ri);

            // mismatch angle between the two vectors:
            const double ang =
                std::acos(ri2.x * bi.x + ri2.y * bi.y + ri2.z * bi.z);
            const double A = wp.robust_kernel_param;
            const double B = wp.robust_kernel_scale;
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

    // send out optional stats
    if (outStats.has_value()) outStats.value().get() = stats;

}  // end visit_correspondences()

/** @} */

}  // namespace mp2p_icp
