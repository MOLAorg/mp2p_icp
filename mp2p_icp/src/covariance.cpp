/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   covariance.cpp
 * @brief  Covariance estimation methods for ICP results
 * @author Jose Luis Blanco Claraco
 * @date   Jun 9, 2020
 */

#include <mp2p_icp/covariance.h>
#include <mp2p_icp/errorTerms.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/num_jacobian.h>

#include <Eigen/Dense>

using namespace mp2p_icp;

mrpt::math::CMatrixDouble66 mp2p_icp::covariance(
    const Pairings& in, const mrpt::poses::CPose3D& finalAlignSolution,
    const CovarianceParameters& param)
{
    // If we don't have pairings, we can't provide an estimation:
    if (in.empty())
    {
        mrpt::math::CMatrixDouble66 cov;
        cov.setDiagonal(1e6);
        return cov;
    }

    mrpt::math::CMatrixDouble61 xInitial;
    xInitial[0] = finalAlignSolution.x();
    xInitial[1] = finalAlignSolution.y();
    xInitial[0] = finalAlignSolution.x();
    xInitial[3] = finalAlignSolution.yaw();
    xInitial[4] = finalAlignSolution.pitch();
    xInitial[5] = finalAlignSolution.roll();

    mrpt::math::CMatrixDouble61 xIncrs;
    for (int i = 0; i < 3; i++) xIncrs[i] = param.finDif_xyz;
    for (int i = 0; i < 3; i++) xIncrs[3 + i] = param.finDif_angles;

    struct LambdaParams
    {
    };

    LambdaParams lmbParams;

    auto errorLambda = [&](const mrpt::math::CMatrixDouble61& x,
                           const LambdaParams&,
                           mrpt::math::CVectorDouble& err) {
        mrpt::poses::CPose3D pose;
        pose.setFromValues(x[0], x[1], x[2], x[3], x[4], x[5]);

        const auto nPt2Pt = in.paired_pt2pt.size();
        const auto nPt2Ln = in.paired_pt2ln.size();
        const auto nPt2Pl = in.paired_pt2pl.size();
        const auto nPl2Pl = in.paired_pl2pl.size();
        const auto nLn2Ln = in.paired_ln2ln.size();

        const auto nErrorTerms =
            (nPt2Pt + nPl2Pl + nPt2Ln + nPt2Pl) * 3 + nLn2Ln * 4;
        ASSERT_(nErrorTerms > 0);
        err.resize(nErrorTerms);

        // Point-to-point:
        for (size_t idx_pt = 0; idx_pt < nPt2Pt; idx_pt++)
        {
            // Error:
            const auto&                       p = in.paired_pt2pt[idx_pt];
            mrpt::math::CVectorFixedDouble<3> ret =
                mp2p_icp::error_point2point(p, pose);
            err.block<3, 1>(idx_pt * 3, 0) = ret.asEigen();
        }
        auto base_idx = nPt2Pt * 3;

        // Point-to-line
        for (size_t idx_pt = 0; idx_pt < nPt2Ln; idx_pt++)
        {
            // Error
            const auto&                       p = in.paired_pt2ln[idx_pt];
            mrpt::math::CVectorFixedDouble<3> ret =
                mp2p_icp::error_point2line(p, pose);
            err.block<3, 1>(base_idx + idx_pt * 3, 0) = ret.asEigen();
        }
        base_idx += nPt2Ln * 3;

        // Line-to-Line
        // Minimum angle to approach zero
        for (size_t idx_ln = 0; idx_ln < nLn2Ln; idx_ln++)
        {
            const auto&                       p = in.paired_ln2ln[idx_ln];
            mrpt::math::CVectorFixedDouble<4> ret =
                mp2p_icp::error_line2line(p, pose);
            err.block<4, 1>(base_idx + idx_ln * 4, 0) = ret.asEigen();
        }
        base_idx += nLn2Ln;

        // Point-to-plane:
        for (size_t idx_pl = 0; idx_pl < nPt2Pl; idx_pl++)
        {
            // Error:
            const auto&                       p = in.paired_pt2pl[idx_pl];
            mrpt::math::CVectorFixedDouble<3> ret =
                mp2p_icp::error_point2plane(p, pose);
            err.block<3, 1>(idx_pl + base_idx, 0) = ret.asEigen();
        }
        base_idx += nPt2Pl * 3;

        // Plane-to-plane (only direction of normal vectors):
        for (size_t idx_pl = 0; idx_pl < nPl2Pl; idx_pl++)
        {
            // Error term:
            const auto&                       p = in.paired_pl2pl[idx_pl];
            mrpt::math::CVectorFixedDouble<3> ret =
                mp2p_icp::error_plane2plane(p, pose);
            err.block<3, 1>(idx_pl * 3 + base_idx, 0) = ret.asEigen();
        }
    };

    // Do NOT use "Eigen::MatrixXd", it may have different alignment
    // requirements than MRPT matrices:
    mrpt::math::CMatrixDouble jacob;
    mrpt::math::estimateJacobian(
        xInitial,
        std::function<void(
            const mrpt::math::CMatrixDouble61&, const LambdaParams&,
            mrpt::math::CVectorDouble&)>(errorLambda),
        xIncrs, lmbParams, jacob);

    const mrpt::math::CMatrixDouble66 hessian(
        jacob.asEigen().transpose() * jacob.asEigen());

    const mrpt::math::CMatrixDouble66 cov = hessian.inverse_LLt();

    return cov;
}

// other ideas?
// See: http://censi.mit.edu/pub/research/2007-icra-icpcov-slides.pdf
