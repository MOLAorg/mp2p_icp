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
 * @file   covariance.cpp
 * @brief  Covariance estimation methods for ICP results
 * @author Jose Luis Blanco Claraco
 * @date   Jun 9, 2020
 */

#include <mp2p_icp/covariance.h>
#include <mp2p_icp/errorTerms.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/num_jacobian.h>
#include <mrpt/poses/Lie/SE.h>

#include <Eigen/Dense>

using namespace mp2p_icp;

namespace
{
using M6  = Eigen::Matrix<double, 6, 6>;
using J36 = Eigen::Matrix<double, 3, 6>;

/** Apply the per-axis floor (in std-dev) to the diagonal of `cov`. */
void apply_floor(mrpt::math::CMatrixDouble66& cov, const CovarianceParameters& p)
{
    const double sx = p.floor_sigma_xyz * p.floor_sigma_xyz;
    const double sa = p.floor_sigma_angles * p.floor_sigma_angles;
    if (sx > 0)
    {
        for (int i = 0; i < 3; ++i)
        {
            cov(i, i) += sx;
        }
    }
    if (sa > 0)
    {
        for (int i = 3; i < 6; ++i)
        {
            cov(i, i) += sa;
        }
    }
}

mrpt::math::CMatrixDouble66 covariance_inverse_hessian(
    const Pairings& in, const mrpt::poses::CPose3D& finalAlignSolution,
    const CovarianceParameters& param)
{
    mrpt::math::CMatrixDouble61 xInitial;
    xInitial[0] = finalAlignSolution.x();
    xInitial[1] = finalAlignSolution.y();
    xInitial[2] = finalAlignSolution.z();
    xInitial[3] = finalAlignSolution.yaw();
    xInitial[4] = finalAlignSolution.pitch();
    xInitial[5] = finalAlignSolution.roll();

    mrpt::math::CMatrixDouble61 xIncrs;
    for (int i = 0; i < 3; i++)
    {
        xIncrs[i] = param.finDif_xyz;
    }
    for (int i = 0; i < 3; i++)
    {
        xIncrs[3 + i] = param.finDif_angles;
    }

    struct LambdaParams
    {
    };

    LambdaParams lmbParams;

    auto errorLambda = [&](const mrpt::math::CMatrixDouble61& x, const LambdaParams&,
                           mrpt::math::CVectorDouble&         err)
    {
        mrpt::poses::CPose3D pose;
        pose.setFromValues(x[0], x[1], x[2], x[3], x[4], x[5]);

        const auto nPt2Pt   = in.paired_pt2pt.size();
        const auto nPt2Ln   = in.paired_pt2ln.size();
        const auto nPt2Pl   = in.paired_pt2pl.size();
        const auto nPl2Pl   = in.paired_pl2pl.size();
        const auto nLn2Ln   = in.paired_ln2ln.size();
        const auto nCov2Cov = in.paired_cov2cov.size();

        const auto nErrorTerms = (nPt2Pt + nPl2Pl + nPt2Ln + nPt2Pl + nCov2Cov) * 3 + nLn2Ln * 4;
        ASSERT_(nErrorTerms > 0);
        err.resize(nErrorTerms);

        for (size_t idx_pt = 0; idx_pt < nPt2Pt; idx_pt++)
        {
            const auto&                       p = in.paired_pt2pt[idx_pt];
            mrpt::math::CVectorFixedDouble<3> ret =
                mp2p_icp::error_point2point(p.local, p.global, pose);
            err.block<3, 1>(static_cast<int>(idx_pt * 3), 0) = ret.asEigen();
        }
        auto base_idx = nPt2Pt * 3;

        for (size_t idx_pt = 0; idx_pt < nPt2Ln; idx_pt++)
        {
            const auto&                       p   = in.paired_pt2ln[idx_pt];
            mrpt::math::CVectorFixedDouble<3> ret = mp2p_icp::error_point2line(p, pose);
            err.block<3, 1>(static_cast<int>(base_idx + idx_pt * 3), 0) = ret.asEigen();
        }
        base_idx += nPt2Ln * 3;

        for (size_t idx_ln = 0; idx_ln < nLn2Ln; idx_ln++)
        {
            const auto&                       p   = in.paired_ln2ln[idx_ln];
            mrpt::math::CVectorFixedDouble<4> ret = mp2p_icp::error_line2line(p, pose);
            err.block<4, 1>(static_cast<int>(base_idx + idx_ln * 4), 0) = ret.asEigen();
        }
        base_idx += nLn2Ln * 4;

        for (size_t idx_pl = 0; idx_pl < nPt2Pl; idx_pl++)
        {
            const auto&                       p   = in.paired_pt2pl[idx_pl];
            mrpt::math::CVectorFixedDouble<3> ret = mp2p_icp::error_point2plane(p, pose);
            err.block<3, 1>(static_cast<int>(base_idx + idx_pl * 3), 0) = ret.asEigen();
        }
        base_idx += nPt2Pl * 3;

        for (size_t idx_pl = 0; idx_pl < nPl2Pl; idx_pl++)
        {
            const auto&                       p   = in.paired_pl2pl[idx_pl];
            mrpt::math::CVectorFixedDouble<3> ret = mp2p_icp::error_plane2plane(p, pose);
            err.block<3, 1>(static_cast<int>(base_idx + idx_pl * 3), 0) = ret.asEigen();
        }
        base_idx += nPl2Pl * 3;

        for (size_t idx_cov2cov = 0; idx_cov2cov < nCov2Cov; idx_cov2cov++)
        {
            const auto& p = in.paired_cov2cov[idx_cov2cov];

            const mrpt::math::CVectorFixedDouble<3> ret =
                mp2p_icp::error_point2point(p.local, p.global, pose);

            const Eigen::Matrix3d cov_inv = p.cov_inv.asEigen().cast<double>();

            // Whitening: residual = L^T * e, where L L^T = cov_inv (Cholesky).
            // This way Jacobian^T * Jacobian directly accumulates the proper
            // information matrix J^T * cov_inv * J, matching the GN solver.
            const Eigen::Matrix3d L_T = Eigen::LLT<Eigen::Matrix3d>(cov_inv).matrixU();
            err.block<3, 1>(static_cast<int>(base_idx + idx_cov2cov * 3), 0) = L_T * ret.asEigen();
        }
    };

    mrpt::math::CMatrixDouble jacob;
    mrpt::math::estimateJacobian(
        xInitial,
        std::function<void(
            const mrpt::math::CMatrixDouble61&, const LambdaParams&, mrpt::math::CVectorDouble&)>(
            errorLambda),
        xIncrs, lmbParams, jacob);

    const mrpt::math::CMatrixDouble66 hessian(jacob.asEigen().transpose() * jacob.asEigen());

    mrpt::math::CMatrixDouble66 cov = hessian.inverse_LLt();

    // A-posteriori residual-variance scaling: cov *= chi^2 / (m - n).
    {
        mrpt::math::CVectorDouble errAtOpt;
        errorLambda(xInitial, lmbParams, errAtOpt);
        const auto   m    = static_cast<int>(errAtOpt.size());
        const double chi2 = errAtOpt.asEigen().squaredNorm();
        const int    dof  = m - 6;
        if (dof > 0 && std::isfinite(chi2))
        {
            const double sigma2 = chi2 / static_cast<double>(dof);
            cov.asEigen() *= sigma2;
        }
    }

    return cov;
}

/** Censi-style sandwich H^{-1} M H^{-1}, computed analytically using the
 *  same SE(3) Jacobians as the GN solver.
 *
 *  Currently supports pt2pt and cov2cov pairings (the dominant cases for
 *  the cov2cov pipeline). pt2ln / pt2pl / pl2pl pairings, if present, are
 *  accumulated with isotropic Sigma_z = defaultPointSigma^2 * I as a
 *  reasonable fallback. ln2ln pairings are skipped.
 */
mrpt::math::CMatrixDouble66 covariance_censi3d(
    const Pairings& in, const mrpt::poses::CPose3D& pose, const CovarianceParameters& p)
{
    M6 H = M6::Zero();
    M6 M = M6::Zero();

    const auto dDexpe_de = mrpt::poses::Lie::SE<3>::jacob_dDexpe_de(pose);

    const double          s2              = p.defaultPointSigma * p.defaultPointSigma;
    const Eigen::Matrix3d Sigma_iso_pt2pt = s2 * Eigen::Matrix3d::Identity();

    auto accum_3x6 = [&](const J36& J, const Eigen::Matrix3d& Sigma_z)
    {
        H.noalias() += J.transpose() * J;
        M.noalias() += J.transpose() * Sigma_z * J;
    };

    // pt2pt: assume isotropic sensor noise.
    for (const auto& pr : in.paired_pt2pt)
    {
        mrpt::math::CMatrixFixed<double, 3, 12> J1;
        (void)mp2p_icp::error_point2point(pr.local, pr.global, pose, J1);
        const J36 Ji = J1.asEigen() * dDexpe_de.asEigen();
        accum_3x6(Ji, Sigma_iso_pt2pt);
    }

    // cov2cov: per-pair Sigma_z = inv(cov_inv).
    for (const auto& pr : in.paired_cov2cov)
    {
        mrpt::math::CMatrixFixed<double, 3, 12> J1;
        (void)mp2p_icp::error_point2point(pr.local, pr.global, pose, J1);
        const J36 Ji = J1.asEigen() * dDexpe_de.asEigen();

        const Eigen::Matrix3d cov_inv = pr.cov_inv.asEigen().cast<double>();
        // Use LDLT-based solve to avoid forming an explicit inverse on
        // ill-conditioned matrices.
        const Eigen::Matrix3d Sigma_z = cov_inv.ldlt().solve(Eigen::Matrix3d::Identity());
        accum_3x6(Ji, Sigma_z);
    }

    // Other 3-D residual pairings: fall back to isotropic noise.
    for (const auto& pr : in.paired_pt2ln)
    {
        mrpt::math::CMatrixFixed<double, 3, 12> J1;
        (void)mp2p_icp::error_point2line(pr, pose, J1);
        const J36 Ji = J1.asEigen() * dDexpe_de.asEigen();
        accum_3x6(Ji, Sigma_iso_pt2pt);
    }
    for (const auto& pr : in.paired_pt2pl)
    {
        mrpt::math::CMatrixFixed<double, 3, 12> J1;
        (void)mp2p_icp::error_point2plane(pr, pose, J1);
        const J36 Ji = J1.asEigen() * dDexpe_de.asEigen();
        accum_3x6(Ji, Sigma_iso_pt2pt);
    }
    for (const auto& pr : in.paired_pl2pl)
    {
        mrpt::math::CMatrixFixed<double, 3, 12> J1;
        (void)mp2p_icp::error_plane2plane(pr, pose, J1);
        const J36 Ji = J1.asEigen() * dDexpe_de.asEigen();
        // Plane-normal residual is dimensionless; treat with unit-noise
        // approximation. (ln2ln pairings are intentionally skipped here.)
        accum_3x6(Ji, Eigen::Matrix3d::Identity());
    }

    // Sandwich cov = H^{-1} M H^{-1}, computed via two LDLT solves on H.
    Eigen::LDLT<M6> Hldlt(H);
    const auto      d = Hldlt.vectorD();
    if ((d.array().abs() < 1e-12).any() || !d.allFinite())  // safety numerical check
    {
        mrpt::math::CMatrixDouble66 covFallback;
        covFallback.setDiagonal(1e6);
        return covFallback;
    }
    const M6 Hinv_M    = Hldlt.solve(M);
    const M6 cov_eigen = Hldlt.solve(Hinv_M.transpose()).transpose();

    mrpt::math::CMatrixDouble66 cov;
    cov.asEigen() = cov_eigen;
    return cov;
}

}  // namespace

mrpt::math::CMatrixDouble66 mp2p_icp::covariance(
    const Pairings& in, const mrpt::poses::CPose3D& finalAlignSolution,
    const CovarianceParameters& param)
{
    if (in.empty())
    {
        mrpt::math::CMatrixDouble66 cov;
        cov.setDiagonal(1e6);
        apply_floor(cov, param);
        return cov;
    }

    mrpt::math::CMatrixDouble66 cov;
    switch (param.method)
    {
        case CovarianceParameters::Method::InverseHessian:
            cov = covariance_inverse_hessian(in, finalAlignSolution, param);
            break;
        case CovarianceParameters::Method::Censi3D:
            cov = covariance_censi3d(in, finalAlignSolution, param);
            break;
    }

    apply_floor(cov, param);
    return cov;
}

void CovarianceParameters::load_from(const mrpt::containers::yaml& p)
{
    if (p.has("method"))
    {
        method = mrpt::typemeta::TEnumType<Method>::name2value(p["method"].as<std::string>());
    }
    MCP_LOAD_OPT(p, defaultPointSigma);
    MCP_LOAD_OPT(p, floor_sigma_xyz);
    MCP_LOAD_OPT(p, floor_sigma_angles);
    if (p.has("floor_sigma_angles_deg"))
    {
        floor_sigma_angles = mrpt::DEG2RAD(p["floor_sigma_angles_deg"].as<double>());
    }
    MCP_LOAD_OPT(p, finDif_xyz);
    MCP_LOAD_OPT(p, finDif_angles);
}

void CovarianceParameters::save_to(mrpt::containers::yaml& p) const
{
    p["method"] = mrpt::typemeta::TEnumType<Method>::value2name(method);
    MCP_SAVE(p, defaultPointSigma);
    MCP_SAVE(p, floor_sigma_xyz);
    MCP_SAVE(p, floor_sigma_angles);
    MCP_SAVE(p, finDif_xyz);
    MCP_SAVE(p, finDif_angles);
}

// other ideas?
// See: http://censi.mit.edu/pub/research/2007-icra-icpcov-slides.pdf
