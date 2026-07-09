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
 * @file   test-mp2p_censi3d_covariance.cpp
 * @brief  Unit tests for the Censi3D ICP covariance estimator
 * @author Jose Luis Blanco Claraco
 * @date   Apr 30, 2026
 */

#include <mp2p_icp/Pairings.h>
#include <mp2p_icp/covariance.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random.h>

#include <Eigen/Dense>
#include <iostream>

using namespace mrpt::literals;

namespace
{
/** Build a regular grid of cov2cov pairings spanning [-extent,+extent]^3 of
 *  step `step`. All pairs are perfect (local==global) so the ICP optimum is
 *  the identity; Sigma per pair is isotropic with std-dev `sigma`. */
mp2p_icp::Pairings make_volumetric_cov2cov_pairings(double extent, double step, double sigma)
{
    mp2p_icp::Pairings pairings;
    const float        info = static_cast<float>(1.0 / (sigma * sigma));
    for (double x = -extent; x <= extent + 1e-9; x += step)
    {
        for (double y = -extent; y <= extent + 1e-9; y += step)
        {
            for (double z = -extent; z <= extent + 1e-9; z += step)
            {
                auto& p  = pairings.paired_cov2cov.emplace_back();
                p.global = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
                p.local  = p.global;
                p.cov_inv.setDiagonal(std::vector<float>({info, info, info}));
            }
        }
    }
    return pairings;
}

/** Long corridor along +X with planar walls at y=+/-1. Each per-point
 *  information matrix is anisotropic, GICP-style: high info along the
 *  wall normal (Y) and a much smaller "in-plane" info along X and Z.
 *  This reflects the fact that a planar surface only observes its
 *  normal; the longitudinal X and vertical Z directions are weakly
 *  constrained, so the resulting covariance must show cov(X,X) >>
 *  cov(Y,Y) (corridor aperture problem). */
mp2p_icp::Pairings make_corridor_cov2cov_pairings(double sigma_normal, double sigma_in_plane)
{
    mp2p_icp::Pairings pairings;
    const float        info_n = static_cast<float>(1.0 / (sigma_normal * sigma_normal));
    const float        info_t = static_cast<float>(1.0 / (sigma_in_plane * sigma_in_plane));

    for (double x = -10.0; x <= 10.0 + 1e-9; x += 0.1)
    {
        for (double yWall : {-1.0, +1.0})
        {
            auto& p  = pairings.paired_cov2cov.emplace_back();
            p.global = {static_cast<float>(x), static_cast<float>(yWall), 0.0f};
            p.local  = p.global;
            // Wall normal is along Y; X and Z are tangent (planar surface).
            p.cov_inv.setDiagonal(std::vector<float>({info_t, info_n, info_t}));
        }
    }
    return pairings;
}

void expect_pd(const mrpt::math::CMatrixDouble66& cov)
{
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(cov.asEigen());
    ASSERT_(es.info() == Eigen::Success);
    ASSERT_GT_(es.eigenvalues().minCoeff(), -1e-12);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        const auto identityPose = mrpt::poses::CPose3D::Identity();

        // -------------------------------------------------------------------
        // Test 1 — well-conditioned cube of points
        // -------------------------------------------------------------------
        {
            const double sigma = 0.01;  // 1 cm per axis
            const auto   pairs =
                make_volumetric_cov2cov_pairings(/*extent=*/1.0, /*step=*/0.5, sigma);
            ASSERT_GT_(pairs.paired_cov2cov.size(), 20U);

            mp2p_icp::CovarianceParameters par;
            par.method = mp2p_icp::CovarianceParameters::Method::Censi3D;

            const auto cov = mp2p_icp::covariance(pairs, identityPose, par);
            std::cout << "[Test1] Censi3D cov diag: " << cov.asEigen().diagonal().transpose()
                      << "\n";

            expect_pd(cov);

            // Translation: with N independent isotropic-sigma points the
            // uncertainty in CoM is ~ sigma^2 / N. A loose upper bound is
            // sigma^2 (single-point), and a lower bound is 1e-3 * sigma^2.
            const double s2 = sigma * sigma;
            for (int i = 0; i < 3; ++i)
            {
                const double v = cov(i, i);
                ASSERT_GT_(v, 1e-4 * s2);
                ASSERT_LT_(v, s2);
            }

            // Rotation should be small but non-zero, roughly sigma^2 / sum r^2.
            for (int i = 3; i < 6; ++i)
            {
                ASSERT_GT_(cov(i, i), 0.0);
                ASSERT_LT_(cov(i, i), 1.0);  // < (1 rad)^2
            }

            // Sanity: Censi3D should give a *larger* trace than the
            // (chi^2-rescaled) inverse-Hessian on this perfect-data scene
            // (residuals are zero -> chi^2=0 -> InvHess cov = 0).
            mp2p_icp::CovarianceParameters par_invH;
            par_invH.method     = mp2p_icp::CovarianceParameters::Method::InverseHessian;
            const auto cov_invH = mp2p_icp::covariance(pairs, identityPose, par_invH);
            std::cout << "[Test1] InvHess cov diag: " << cov_invH.asEigen().diagonal().transpose()
                      << "\n";
            ASSERT_GT_(cov.asEigen().trace(), cov_invH.asEigen().trace());
        }

        // -------------------------------------------------------------------
        // Test 2 — corridor: covariance blows up in the longitudinal axis
        // -------------------------------------------------------------------
        {
            // 1 cm normal noise, 1 m in-plane noise (essentially unobserved
            // along the wall surface — typical of GICP on planar regions).
            const auto pairs = make_corridor_cov2cov_pairings(
                /*sigma_normal=*/0.01, /*sigma_in_plane=*/1.0);

            mp2p_icp::CovarianceParameters par;
            par.method = mp2p_icp::CovarianceParameters::Method::Censi3D;

            const auto cov = mp2p_icp::covariance(pairs, identityPose, par);
            std::cout << "[Test2] corridor cov diag: " << cov.asEigen().diagonal().transpose()
                      << "\n";

            expect_pd(cov);

            // Walls only observe their normal (Y), so X and Z translations
            // are weakly constrained while Y is tightly constrained. We
            // expect cov(X,X) and cov(Z,Z) to be orders of magnitude
            // larger than cov(Y,Y).
            ASSERT_GT_(cov(0, 0), 100.0 * cov(1, 1));
            ASSERT_GT_(cov(2, 2), 100.0 * cov(1, 1));
        }

        // -------------------------------------------------------------------
        // Test 3 — per-axis floor is applied to the diagonal
        // -------------------------------------------------------------------
        {
            const double sigma = 0.01;
            const auto   pairs =
                make_volumetric_cov2cov_pairings(/*extent=*/1.0, /*step=*/0.5, sigma);

            mp2p_icp::CovarianceParameters par;
            par.method             = mp2p_icp::CovarianceParameters::Method::Censi3D;
            par.floor_sigma_xyz    = 0.05;  // 5 cm
            par.floor_sigma_angles = 1.0_deg;  // ~0.0175 rad

            const auto cov = mp2p_icp::covariance(pairs, identityPose, par);
            std::cout << "[Test3] cov-w-floor diag: " << cov.asEigen().diagonal().transpose()
                      << "\n";

            const double minXYZ = par.floor_sigma_xyz * par.floor_sigma_xyz;
            const double minAng = par.floor_sigma_angles * par.floor_sigma_angles;
            for (int i = 0; i < 3; ++i)
            {
                ASSERT_GE_(cov(i, i), minXYZ);
            }
            for (int i = 3; i < 6; ++i)
            {
                ASSERT_GE_(cov(i, i), minAng);
            }
        }

        // -------------------------------------------------------------------
        // Test 4 — pt2pt path (no per-pair Sigma, falls back to default)
        // -------------------------------------------------------------------
        {
            mp2p_icp::Pairings pairs;
            // 8 cube corners
            for (double x : {-1.0, 1.0})
            {
                for (double y : {-1.0, 1.0})
                {
                    for (double z : {-1.0, 1.0})
                    {
                        auto& p  = pairs.paired_pt2pt.emplace_back();
                        p.global = {
                            static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
                        p.local = p.global;
                    }
                }
            }

            mp2p_icp::CovarianceParameters par;
            par.method            = mp2p_icp::CovarianceParameters::Method::Censi3D;
            par.defaultPointSigma = 0.01;

            const auto cov = mp2p_icp::covariance(pairs, identityPose, par);
            std::cout << "[Test4] pt2pt cov diag: " << cov.asEigen().diagonal().transpose() << "\n";
            expect_pd(cov);
            for (int i = 0; i < 6; ++i)
            {
                ASSERT_GT_(cov(i, i), 0.0);
            }
        }

        std::cout << "All Censi3D covariance tests passed.\n";
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
    return 0;
}
