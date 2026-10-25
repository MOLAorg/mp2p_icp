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
 * @file   test-mp2p_covariance_conventions.cpp
 * @brief  Every covariance method returns the CPose3DPDFGaussian layout
 *         [x y z yaw pitch roll]; checks of the tangent <-> ypr conversions.
 */

#include <mp2p_icp/Pairings.h>
#include <mp2p_icp/covariance.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/random.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

namespace
{
using mrpt::poses::CPose3D;

/** The `[x y z yaw pitch roll]` coordinates of a pose. */
Eigen::Matrix<double, 6, 1> ypr(const CPose3D& p)
{
    Eigen::Matrix<double, 6, 1> v;
    v << p.x(), p.y(), p.z(), p.yaw(), p.pitch(), p.roll();
    return v;
}

/** The analytic Jacobian matches central differences of the coordinates of
 *  `pose * exp(eps)`. */
void test_jacobian_against_finite_differences()
{
    auto& rng = mrpt::random::getRandomGenerator();

    for (int trial = 0; trial < 50; trial++)
    {
        const auto pose = CPose3D::FromXYZYawPitchRoll(
            rng.drawUniform(-10.0, 10.0), rng.drawUniform(-10.0, 10.0), rng.drawUniform(-2.0, 2.0),
            rng.drawUniform(-M_PI, M_PI), rng.drawUniform(-1.3, 1.3), rng.drawUniform(-M_PI, M_PI));

        const Eigen::Matrix<double, 6, 6> J = mp2p_icp::jacobian_ypr_wrt_tangent(pose).asEigen();

        constexpr double h = 1e-6;
        for (int k = 0; k < 6; k++)
        {
            mrpt::poses::Lie::SE<3>::tangent_vector eps;
            eps.setZero();
            eps[k]                               = h;
            const auto plusEps                   = ypr(pose + mrpt::poses::Lie::SE<3>::exp(eps));
            eps[k]                               = -h;
            const auto                  minusEps = ypr(pose + mrpt::poses::Lie::SE<3>::exp(eps));
            Eigen::Matrix<double, 6, 1> diff     = plusEps - minusEps;
            for (int a = 3; a < 6; a++)
            {
                diff[a] = mrpt::math::wrapToPi(diff[a]);
            }
            const Eigen::Matrix<double, 6, 1> numeric = diff / (2 * h);

            for (int r = 0; r < 6; r++)
            {
                ASSERT_NEAR_(J(r, k), numeric[r], 1e-5);
            }
        }
    }
    std::cout << "[covariance_conventions] Jacobian vs finite differences: OK\n";
}

/** The two conversions are consistent with each other: converting a covariance
 *  to ypr, inverting it, and converting the information back to the tangent
 *  gives the inverse of the original covariance. */
void test_conversions_round_trip()
{
    const auto pose = CPose3D::FromXYZYawPitchRoll(1.0, -2.0, 0.3, 2.5, -0.4, 1.1);

    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Random();
    mrpt::math::CMatrixDouble66 covTangent;
    covTangent.asEigen() = A * A.transpose() + 0.1 * Eigen::Matrix<double, 6, 6>::Identity();

    const auto                  covYpr = mp2p_icp::covariance_tangent_to_ypr(covTangent, pose);
    mrpt::math::CMatrixDouble66 infoYpr;
    infoYpr.asEigen()      = covYpr.asEigen().inverse();
    const auto infoTangent = mp2p_icp::information_ypr_to_tangent(infoYpr, pose);

    const Eigen::Matrix<double, 6, 6> expected = covTangent.asEigen().inverse();
    ASSERT_LT_((infoTangent.asEigen() - expected).norm(), 1e-9 * expected.norm());
    std::cout << "[covariance_conventions] conversions round trip: OK\n";
}

/** Censi3D and InverseHessian estimate the same thing on point-to-point
 *  pairings with isotropic noise of known sigma: sigma^2 * H^-1 for the former,
 *  H^-1 * chi^2 / dof for the latter, which only differ by a scale factor. So
 *  both must have the same structure in the same [x y z yaw pitch roll] layout.
 *
 *  The pose has distinct yaw, pitch and roll, and the cloud is long along x and
 *  thin along z, so roll is much less certain than yaw: returning the tangent
 *  covariance, as Censi3D did, swaps them, and leaves the translation in the
 *  body frame. */
void test_censi3d_matches_inverse_hessian_layout()
{
    auto& rng = mrpt::random::getRandomGenerator();

    const auto pose = CPose3D::FromXYZYawPitchRoll(2.0, -1.0, 0.5, 1.05, 0.17, -0.44);

    constexpr double sigma = 0.02;

    mp2p_icp::Pairings pairings;
    for (int i = 0; i < 3000; i++)
    {
        const mrpt::math::TPoint3D local(
            rng.drawUniform(-10.0, 10.0), rng.drawUniform(-2.0, 2.0), rng.drawUniform(-0.3, 0.3));
        const auto global = pose.composePoint(local) + mrpt::math::TPoint3D(
                                                           rng.drawGaussian1D(0, sigma),
                                                           rng.drawGaussian1D(0, sigma),
                                                           rng.drawGaussian1D(0, sigma));
        pairings.paired_pt2pt.emplace_back(
            static_cast<uint32_t>(i), static_cast<uint32_t>(i), global.cast<float>(),
            local.cast<float>());
    }

    mp2p_icp::CovarianceParameters par;
    par.method                              = mp2p_icp::CovarianceParameters::Method::Censi3D;
    par.defaultPointSigma                   = sigma;
    const Eigen::Matrix<double, 6, 6> censi = mp2p_icp::covariance(pairings, pose, par).asEigen();

    par.method = mp2p_icp::CovarianceParameters::Method::InverseHessian;
    const Eigen::Matrix<double, 6, 6> invHess = mp2p_icp::covariance(pairings, pose, par).asEigen();

    std::cout << "[covariance_conventions] Censi3D diag:        " << censi.diagonal().transpose()
              << "\n"
              << "[covariance_conventions] InverseHessian diag: " << invHess.diagonal().transpose()
              << "\n";

    // Both diagonals must agree entry by entry, roll and yaw included, up to
    // the chi^2/dof factor, which is close to 1 here (the noise is sigma):
    const double scale = invHess.trace() / censi.trace();
    ASSERT_GT_(scale, 0.8);
    ASSERT_LT_(scale, 1.25);
    ASSERT_LT_((invHess - scale * censi).norm(), 1e-2 * invHess.norm());

    // The scene really tells roll and yaw apart:
    ASSERT_GT_(invHess(5, 5), 5 * invHess(3, 3));
    std::cout << "[covariance_conventions] Censi3D in the ypr layout: OK\n";
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        mrpt::random::getRandomGenerator().randomize(1234);

        test_jacobian_against_finite_differences();
        test_conversions_round_trip();
        test_censi3d_matches_inverse_hessian_layout();
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
    return 0;
}
