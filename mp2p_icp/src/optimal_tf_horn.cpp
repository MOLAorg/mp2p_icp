/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   optimal_tf_horn.cpp
 * @brief  Classic Horn's solution for optimal SE(3) transformation
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */

#include <mp2p_icp/optimal_tf_horn.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorFixed.h>

#include "visit_correspondences.h"

using namespace mp2p_icp;

// The next function is code ported from our former implementation in MRPT,
// from function: mrpt::tfest::se3_l2_internal()
// (BSD-3 Licence)

// "Closed-form solution of absolute orientation using unit quaternions", BKP
// Horn, Journal of the Optical Society of America, 1987.
// Algorithm:
// 0. Preliminary
//		pLi = { pLix, pLiy, pLiz }
//		pRi = { pRix, pRiy, pRiz }
// -------------------------------------------------------
// 1. Find the centroids of the two sets of measurements:
//		ct_locals = (1/n)*sum{i}( pLi )		ct_locals = { cLx, cLy, cLz }
//		ct_global = (1/n)*sum{i}( pRi )		ct_global = { cRx, cRy, cRz }
//
// 2. Substract centroids from the point coordinates:
//		pLi' = pLi - ct_locals					pLi' = { pLix', pLiy', pLiz' }
//		pRi' = pRi - ct_global					pRi' = { pRix', pRiy', pRiz' }
//
// 3. For each pair of coordinates (correspondences) compute the nine possible
// products:
//		pi1 = pLix'*pRix'		pi2 = pLix'*pRiy'		pi3 = pLix'*pRiz'
//		pi4 = pLiy'*pRix'		pi5 = pLiy'*pRiy'		pi6 = pLiy'*pRiz'
//		pi7 = pLiz'*pRix'		pi8 = pLiz'*pRiy'		pi9 = pLiz'*pRiz'
//
// 4. Compute S components:
//		Sxx = sum{i}( pi1 )		Sxy = sum{i}( pi2 )		Sxz = sum{i}( pi3 )
//		Syx = sum{i}( pi4 )		Syy = sum{i}( pi5 )		Syz = sum{i}( pi6 )
//		Szx = sum{i}( pi7 )		Szy = sum{i}( pi8 )		Szz = sum{i}( pi9 )
//
// 5. Compute N components:
//			[ Sxx+Syy+Szz	Syz-Szy			Szx-Sxz			Sxy-Syx		 ]
//			[ Syz-Szy		Sxx-Syy-Szz		Sxy+Syx			Szx+Sxz		 ]
//		N = [ Szx-Sxz		Sxy+Syx			-Sxx+Syy-Szz	Syz+Szy		 ]
//			[ Sxy-Syx		Szx+Sxz			Syz+Szy			-Sxx-Syy+Szz ]
//
// 6. Rotation represented by the quaternion eigenvector correspondent to the
// higher eigenvalue of N
//
// 7. Scale computation (symmetric expression)
//		s = sqrt( sum{i}( square(abs(pRi')) / sum{i}( square(abs(pLi')) ) )
//
// 8. Translation computation (distance between the Right centroid and the
// scaled and rotated Left centroid)
//		t = ct_global-sR(ct_locals)

// Returns false if the number of pairings is not >=3
static bool se3_l2_internal(
    const mp2p_icp::Pairings& in, const WeightParameters& wp,
    const mrpt::math::TPoint3D& ct_local, const mrpt::math::TPoint3D& ct_global,
    mrpt::math::CQuaternionDouble& out_attitude,
    OutlierIndices&                in_out_outliers)
{
    MRPT_START

    // Compute the centroids
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

    // Horn method needs at least 3 references
    if (nAllMatches < 3) return false;

    auto S = mrpt::math::CMatrixDouble33::Zero();

    // Lambda: process each pairing:
    auto lambda_each_pair = [&](const mrpt::math::TVector3D& bi,
                                const mrpt::math::TVector3D& ri,
                                const double                 wi)
    {
        // These vectors are already direction vectors, or the
        // centroids-centered relative positions of points. Compute the S matrix
        // of cross products.
        S(0, 0) += wi * ri.x * bi.x;
        S(0, 1) += wi * ri.x * bi.y;
        S(0, 2) += wi * ri.x * bi.z;

        S(1, 0) += wi * ri.y * bi.x;
        S(1, 1) += wi * ri.y * bi.y;
        S(1, 2) += wi * ri.y * bi.z;

        S(2, 0) += wi * ri.z * bi.x;
        S(2, 1) += wi * ri.z * bi.y;
        S(2, 2) += wi * ri.z * bi.z;
    };

    auto lambda_final = [&](const double w_sum)
    {
        // Normalize weights. OLAE assumes \sum(w_i) = 1.0
        if (w_sum > .0) S *= (1.0 / w_sum);
    };

    visit_correspondences(
        in, wp, ct_local, ct_global, in_out_outliers /*in/out*/,
        // Operations to run on pairs:
        lambda_each_pair, lambda_final,
        false /* do not make unit point vectors for Horn */);

    // Construct the N matrix
    auto N = mrpt::math::CMatrixDouble44::Zero();

    N(0, 0) = S(0, 0) + S(1, 1) + S(2, 2);
    N(0, 1) = S(1, 2) - S(2, 1);
    N(0, 2) = S(2, 0) - S(0, 2);
    N(0, 3) = S(0, 1) - S(1, 0);

    N(1, 0) = N(0, 1);
    N(1, 1) = S(0, 0) - S(1, 1) - S(2, 2);
    N(1, 2) = S(0, 1) + S(1, 0);
    N(1, 3) = S(2, 0) + S(0, 2);

    N(2, 0) = N(0, 2);
    N(2, 1) = N(1, 2);
    N(2, 2) = -S(0, 0) + S(1, 1) - S(2, 2);
    N(2, 3) = S(1, 2) + S(2, 1);

    N(3, 0) = N(0, 3);
    N(3, 1) = N(1, 3);
    N(3, 2) = N(2, 3);
    N(3, 3) = -S(0, 0) - S(1, 1) + S(2, 2);

    // q is the quaternion correspondent to the greatest eigenvector of the N
    // matrix (last column in Z)
    auto                Z = mrpt::math::CMatrixDouble44::Zero();
    std::vector<double> eigvals;
    N.eig_symmetric(Z, eigvals, true /*sorted*/);

    auto v = Z.blockCopy<4, 1>(0, 3);

    ASSERTDEB_(
        fabs(
            sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]) - 1.0) <
        0.1);

    // Make q_r > 0
    if (v[0] < 0)
    {
        v[0] *= -1;
        v[1] *= -1;
        v[2] *= -1;
        v[3] *= -1;
    }

    // out_transform: Create a pose rotation with the quaternion
    for (unsigned int i = 0; i < 4; i++) out_attitude[i] = v[i];

#if 0
    // Compute scale
    double s;
    if (forceScaleToUnity)
    {
        s = 1.0;  // Enforce scale to be 1
    }
    else
    {
        double num = 0.0;
        double den = 0.0;
        for (size_t i = 0; i < nMatches; i++)
        {
            num += ri[i].sqrNorm();
            den += bi[i].sqrNorm();
        }
        // The scale:
        s = std::sqrt(num / den);
    }
#endif

    return true;
    MRPT_END
}

bool mp2p_icp::optimal_tf_horn(
    const mp2p_icp::Pairings& in, const WeightParameters& wp,
    OptimalTF_Result& result)
{
    MRPT_START

    result = OptimalTF_Result();

    // Normalize weights for each feature type and for each target (attitude
    // / translation):
    ASSERT_(wp.pair_weights.pt2pt >= .0);
    ASSERT_(wp.pair_weights.ln2ln >= .0);
    ASSERT_(wp.pair_weights.pl2pl >= .0);

    // Compute the centroids:
    auto [ct_local, ct_global] =
        eval_centroids_robust(in, result.outliers /* in: empty for now  */);

    mrpt::math::CQuaternionDouble optimal_q;

    // Build the linear system & solves for optimal quaternion:
    if (!se3_l2_internal(
            in, wp, ct_local, ct_global, optimal_q,
            result.outliers /* in/out */))
        return false;

    // Re-evaluate the centroids, now that we have a guess on outliers.
    if (wp.use_scale_outlier_detector && !result.outliers.empty())
    {
        // Re-evaluate the centroids:
        const auto [new_ct_local, new_ct_global] =
            eval_centroids_robust(in, result.outliers);

        ct_local  = new_ct_local;
        ct_global = new_ct_global;

        // And rebuild the linear system with the new values:
        if (!se3_l2_internal(
                in, wp, ct_local, ct_global, optimal_q,
                result.outliers /* in/out */))
            return false;
    }

    // quaternion to rotation matrix:
    result.optimalPose = mrpt::poses::CPose3D(optimal_q, 0, 0, 0);

    // Use centroids to solve for optimal translation:
    mrpt::math::TPoint3D pp;
    result.optimalPose.composePoint(
        ct_local.x, ct_local.y, ct_local.z, pp.x, pp.y, pp.z);
    // Scale, if used, was: pp *= s;

    result.optimalPose.x(ct_global.x - pp.x);
    result.optimalPose.y(ct_global.y - pp.y);
    result.optimalPose.z(ct_global.z - pp.z);

    return true;

    MRPT_END
}
