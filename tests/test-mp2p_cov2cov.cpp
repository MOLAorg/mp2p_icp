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
 * @file   test-mp2p_cov2cov.cpp
 * @brief  Comprehensive unit tests for cov2cov pairings (Generalized ICP)
 * @author Jose Luis Blanco Claraco
 * @date   Jan 25, 2026
 */

#include <mp2p_icp/Pairings.h>
#include <mp2p_icp/Results.h>
#include <mp2p_icp/covariance.h>
#include <mp2p_icp/errorTerms.h>
#include <mp2p_icp/optimal_tf_gauss_newton.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/num_jacobian.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/random.h>

#include <Eigen/Dense>
#include <iostream>

using namespace mrpt::literals;  // for .0_deg suffix

auto& rnd = mrpt::random::getRandomGenerator();

// ===========================================================================
//  Helper functions
// ===========================================================================

namespace
{
double normal_d(const double sigma) { return rnd.drawGaussian1D_normalized() * sigma; }
float  normal_f(const float sigma)
{
    return static_cast<float>(rnd.drawGaussian1D_normalized()) * sigma;
}

/// Generates random cov2cov pairings with known ground truth transformation
mp2p_icp::Pairings generate_cov2cov_pairings(
    const mrpt::poses::CPose3D& gtPose, const size_t numPairings, const double xyz_noise_std = 0.0,
    const bool anisotropic_covariance = false)
{
    mp2p_icp::Pairings pairings;

    for (size_t i = 0; i < numPairings; i++)
    {
        auto& p = pairings.paired_cov2cov.emplace_back();

        // Generate random global point
        p.global = {normal_f(20.0f), normal_f(20.0f), normal_f(20.0f)};

        // Compute local point by inverse transform
        mrpt::math::TPoint3D globalPt(p.global.x, p.global.y, p.global.z);
        mrpt::math::TPoint3D localPt;
        gtPose.inverseComposePoint(globalPt, localPt);

        // Add optional noise
        p.local = {
            static_cast<float>(localPt.x + normal_d(xyz_noise_std)),
            static_cast<float>(localPt.y + normal_d(xyz_noise_std)),
            static_cast<float>(localPt.z + normal_d(xyz_noise_std))};

        // Set covariance inverse (information matrix)
        if (anisotropic_covariance)
        {
            // Anisotropic: different uncertainty in each axis
            const float var_x = static_cast<float>(rnd.drawUniform(0.01, 1.0));
            const float var_y = static_cast<float>(rnd.drawUniform(0.01, 1.0));
            const float var_z = static_cast<float>(rnd.drawUniform(0.01, 1.0));
            p.cov_inv.setDiagonal(std::vector<float>({1.0f / var_x, 1.0f / var_y, 1.0f / var_z}));
        }
        else
        {
            // Isotropic covariance
            p.cov_inv.setIdentity();
        }

        p.global_idx = static_cast<uint32_t>(i);
        p.local_idx  = static_cast<uint32_t>(i);
    }

    return pairings;
}

// ===========================================================================
//  Test 1: Basic cov2cov optimization convergence
// ===========================================================================

void test_cov2cov_optimization_basic()
{
    std::cout << "[test_cov2cov_optimization_basic] Running...\n";

    const auto gtPose =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(1.0, 0.0, 0.0, 0.0_deg, 0.0_deg, 0.0_deg);
    const auto initPose =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.0, 2.0, 3.0, 5.0_deg, -5.0_deg, -10.0_deg);

    // Pairings with explicit axis constraints (as in original test)
    mp2p_icp::Pairings pairings;
    {
        auto& p  = pairings.paired_cov2cov.emplace_back();
        p.global = {1.0f, 0.0f, 1.0f};
        p.local  = {0.0f, 0.0f, 1.0f};
        p.cov_inv.setDiagonal(std::vector<float>({1.0f, 1e6f, 1.0f}));
    }
    {
        auto& p  = pairings.paired_cov2cov.emplace_back();
        p.global = {2.0f, 5.0f, 0.0f};
        p.local  = {1.0f, 5.0f, 0.0f};
        p.cov_inv.setDiagonal(std::vector<float>({1.0f, 1.0f, 1e6f}));
    }
    {
        auto& p  = pairings.paired_cov2cov.emplace_back();
        p.global = {0.0f, 8.0f, 1.0f};
        p.local  = {-1.0f, 8.0f, 1.0f};
        p.cov_inv.setDiagonal(std::vector<float>({1e6f, 1.0f, 1.0f}));
    }

    // Run Gauss-Newton
    mp2p_icp::OptimalTF_GN_Parameters gnParams;
    gnParams.linearizationPoint = initPose;
    gnParams.verbose            = false;

    mp2p_icp::OptimalTF_Result result;

    const bool run_ok = mp2p_icp::optimal_tf_gauss_newton(pairings, result, gnParams);
    ASSERT_(run_ok);

    // Verify convergence
    const auto poseError = gtPose - result.optimalPose;
    ASSERT_LT_(poseError.translation().norm(), 0.1);
    ASSERT_LT_(mrpt::poses::Lie::SO<3>::log(poseError.getRotationMatrix()).norm(), 0.1);

    std::cout << "[test_cov2cov_optimization_basic] PASSED\n";
}

// ===========================================================================
//  Test 2: Cov2cov optimization with many random pairings
// ===========================================================================

void test_cov2cov_optimization_random()
{
    std::cout << "[test_cov2cov_optimization_random] Running...\n";

    for (int trial = 0; trial < 10; trial++)
    {
        // Random ground truth pose (small rotations for GN convergence)
        const auto gtPose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
            rnd.drawUniform(-2.0, 2.0), rnd.drawUniform(-2.0, 2.0), rnd.drawUniform(-2.0, 2.0),
            mrpt::DEG2RAD(rnd.drawUniform(-10.0, 10.0)),
            mrpt::DEG2RAD(rnd.drawUniform(-10.0, 10.0)),
            mrpt::DEG2RAD(rnd.drawUniform(-10.0, 10.0)));

        // Generate pairings
        auto pairings = generate_cov2cov_pairings(gtPose, 50, 0.0, true);

        // Initial guess: identity
        mp2p_icp::OptimalTF_GN_Parameters gnParams;
        gnParams.linearizationPoint     = mrpt::poses::CPose3D::Identity();
        gnParams.maxInnerLoopIterations = 20;
        gnParams.verbose                = false;

        mp2p_icp::OptimalTF_Result result;
        const bool run_ok = mp2p_icp::optimal_tf_gauss_newton(pairings, result, gnParams);
        ASSERT_(run_ok);

        // Check convergence
        const auto   poseError = gtPose - result.optimalPose;
        const double err_xyz   = poseError.translation().norm();
        const double err_rot   = mrpt::poses::Lie::SO<3>::log(poseError.getRotationMatrix()).norm();

        if (err_xyz > 0.15 || err_rot > 0.15)
        {
            std::cerr << "Trial " << trial << " failed:\n"
                      << "  GT pose: " << gtPose.asString() << "\n"
                      << "  Result:  " << result.optimalPose.asString() << "\n"
                      << "  XYZ error: " << err_xyz << ", Rot error: " << err_rot << "\n";
            THROW_EXCEPTION("Optimization failed to converge");
        }
    }

    std::cout << "[test_cov2cov_optimization_random] PASSED\n";
}

// ===========================================================================
//  Test 3: Covariance estimation produces valid (non-singular) output
// ===========================================================================

void test_cov2cov_covariance_output_valid()
{
    std::cout << "[test_cov2cov_covariance_output_valid] Running...\n";

    const auto gtPose =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(1.0, -0.5, 0.2, 5.0_deg, -3.0_deg, 2.0_deg);

    // Generate pairings with slight noise
    auto pairings = generate_cov2cov_pairings(gtPose, 30, 0.01, true);

    // Compute covariance at the ground truth pose
    mp2p_icp::CovarianceParameters covParams;
    const auto                     cov = mp2p_icp::covariance(pairings, gtPose, covParams);

    // Check that covariance is valid
    // 1. Should be symmetric
    const double symmetry_error = (cov.asEigen() - cov.asEigen().transpose()).norm();
    ASSERT_LT_(symmetry_error, 1e-10);

    // 2. Should be positive definite (all eigenvalues > 0)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigSolver(cov.asEigen());
    const auto eigenvalues = eigSolver.eigenvalues();
    for (int i = 0; i < 6; i++)
    {
        if (eigenvalues(i) <= 0)
        {
            std::cerr << "Covariance matrix is not positive definite!\n"
                      << "Eigenvalues: " << eigenvalues.transpose() << "\n"
                      << "Covariance:\n"
                      << cov.asEigen() << "\n";
            THROW_EXCEPTION("Covariance not positive definite");
        }
    }

    // 3. Should not have extremely large values (indicating near-singular Hessian)
    const double max_diag = cov.asEigen().diagonal().maxCoeff();
    ASSERT_LT_(max_diag, 1e4);  // Reasonable upper bound

    // 4. Should not be the "fallback" diagonal matrix (1e6)
    const double min_diag = cov.asEigen().diagonal().minCoeff();
    ASSERT_LT_(min_diag, 1e5);

    std::cout << "  Covariance diagonal: " << cov.asEigen().diagonal().transpose() << "\n";
    std::cout << "[test_cov2cov_covariance_output_valid] PASSED\n";
}

// ===========================================================================
//  Test 4: Covariance estimation with only cov2cov pairings (no pt2pt)
// ===========================================================================

void test_cov2cov_only_covariance()
{
    std::cout << "[test_cov2cov_only_covariance] Running...\n";

    const auto gtPose =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.5, 0.3, -0.2, 3.0_deg, -2.0_deg, 1.0_deg);

    // Create pairings with ONLY cov2cov (no other pairing types)
    mp2p_icp::Pairings pairings;
    for (int i = 0; i < 20; i++)
    {
        auto& p  = pairings.paired_cov2cov.emplace_back();
        p.global = {normal_f(10.0f), normal_f(10.0f), normal_f(10.0f)};

        mrpt::math::TPoint3D globalPt(p.global.x, p.global.y, p.global.z);
        mrpt::math::TPoint3D localPt;
        gtPose.inverseComposePoint(globalPt, localPt);
        p.local = {
            static_cast<float>(localPt.x), static_cast<float>(localPt.y),
            static_cast<float>(localPt.z)};

        // Anisotropic covariance
        p.cov_inv.setDiagonal(std::vector<float>(
            {static_cast<float>(rnd.drawUniform(1.0, 10.0)),
             static_cast<float>(rnd.drawUniform(1.0, 10.0)),
             static_cast<float>(rnd.drawUniform(1.0, 10.0))}));
    }

    // Verify pairings structure
    ASSERT_(pairings.paired_pt2pt.empty());
    ASSERT_(pairings.paired_pt2pl.empty());
    ASSERT_(pairings.paired_pl2pl.empty());
    ASSERT_(!pairings.paired_cov2cov.empty());
    ASSERT_(!pairings.empty());

    // Compute covariance
    mp2p_icp::CovarianceParameters covParams;
    const auto                     cov = mp2p_icp::covariance(pairings, gtPose, covParams);

    // Check validity
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigSolver(cov.asEigen());
    const auto   eigenvalues    = eigSolver.eigenvalues();
    const double min_eigenvalue = eigenvalues.minCoeff();

    ASSERT_GT_(min_eigenvalue, 0);
    ASSERT_LT_(cov.asEigen().diagonal().maxCoeff(), 1e4);

    std::cout << "  Covariance diagonal: " << cov.asEigen().diagonal().transpose() << "\n";
    std::cout << "  Min eigenvalue: " << min_eigenvalue << "\n";
    std::cout << "[test_cov2cov_only_covariance] PASSED\n";
}

// ===========================================================================
//  Test 5: Mixed pairings (cov2cov + pt2pt)
// ===========================================================================

void test_cov2cov_mixed_pairings()
{
    std::cout << "[test_cov2cov_mixed_pairings] Running...\n";

    const auto gtPose =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(1.0, -0.5, 0.3, 4.0_deg, -2.0_deg, 3.0_deg);

    mp2p_icp::Pairings pairings;

    // Add some pt2pt pairings
    for (int i = 0; i < 15; i++)
    {
        mrpt::tfest::TMatchingPair pair;
        pair.globalIdx = pair.localIdx = i;
        pair.global                    = {normal_f(15.0f), normal_f(15.0f), normal_f(15.0f)};

        mrpt::math::TPoint3D localPt;
        gtPose.inverseComposePoint(
            mrpt::math::TPoint3D(pair.global.x, pair.global.y, pair.global.z), localPt);
        pair.local = localPt.cast<float>();

        pairings.paired_pt2pt.push_back(pair);
    }

    // Add some cov2cov pairings
    for (int i = 0; i < 15; i++)
    {
        auto& p  = pairings.paired_cov2cov.emplace_back();
        p.global = {normal_f(15.0f), normal_f(15.0f), normal_f(15.0f)};

        mrpt::math::TPoint3D localPt;
        gtPose.inverseComposePoint(
            mrpt::math::TPoint3D(p.global.x, p.global.y, p.global.z), localPt);
        p.local = {
            static_cast<float>(localPt.x), static_cast<float>(localPt.y),
            static_cast<float>(localPt.z)};

        p.cov_inv.setDiagonal(std::vector<float>({5.0f, 5.0f, 5.0f}));
    }

    // Optimize using Gauss-Newton
    mp2p_icp::OptimalTF_GN_Parameters gnParams;
    gnParams.linearizationPoint     = mrpt::poses::CPose3D::Identity();
    gnParams.maxInnerLoopIterations = 20;
    gnParams.verbose                = false;

    mp2p_icp::OptimalTF_Result result;
    const bool run_ok = mp2p_icp::optimal_tf_gauss_newton(pairings, result, gnParams);
    ASSERT_(run_ok);

    // Check optimization result
    const auto poseError = gtPose - result.optimalPose;
    ASSERT_LT_(poseError.translation().norm(), 0.1);
    ASSERT_LT_(mrpt::poses::Lie::SO<3>::log(poseError.getRotationMatrix()).norm(), 0.1);

    // Compute and validate covariance
    mp2p_icp::CovarianceParameters covParams;
    const auto cov = mp2p_icp::covariance(pairings, result.optimalPose, covParams);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigSolver(cov.asEigen());
    ASSERT_GT_(eigSolver.eigenvalues().minCoeff(), 0);

    std::cout << "  Pose error: xyz=" << poseError.translation().norm()
              << " rot=" << mrpt::poses::Lie::SO<3>::log(poseError.getRotationMatrix()).norm()
              << "\n";
    std::cout << "[test_cov2cov_mixed_pairings] PASSED\n";
}

// ===========================================================================
//  Test 6: Anisotropic covariance affects solution
// ===========================================================================

void test_cov2cov_anisotropic_weighting()
{
    std::cout << "[test_cov2cov_anisotropic_weighting] Running...\n";

    // Create a scenario where anisotropic covariance should bias the solution
    // Point with high uncertainty in Y direction should have less influence on Y

    const auto gtPose =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(1.0, 0.0, 0.0, 0.0_deg, 0.0_deg, 0.0_deg);

    mp2p_icp::Pairings pairings;

    // Add multiple pairings with high certainty
    for (int i = 0; i < 10; i++)
    {
        auto& p  = pairings.paired_cov2cov.emplace_back();
        p.global = {static_cast<float>(i), static_cast<float>(i * 2), static_cast<float>(i * 0.5)};

        mrpt::math::TPoint3D localPt;
        gtPose.inverseComposePoint(
            mrpt::math::TPoint3D(p.global.x, p.global.y, p.global.z), localPt);
        p.local = {
            static_cast<float>(localPt.x), static_cast<float>(localPt.y),
            static_cast<float>(localPt.z)};

        // High certainty (large inverse covariance)
        p.cov_inv.setDiagonal(std::vector<float>({100.0f, 100.0f, 100.0f}));
    }

    // Run optimization
    mp2p_icp::OptimalTF_GN_Parameters gnParams;
    gnParams.linearizationPoint     = mrpt::poses::CPose3D::Identity();
    gnParams.maxInnerLoopIterations = 20;

    mp2p_icp::OptimalTF_Result result;
    ASSERT_(mp2p_icp::optimal_tf_gauss_newton(pairings, result, gnParams));

    const auto poseError = gtPose - result.optimalPose;
    ASSERT_LT_(poseError.translation().norm(), 0.05);

    std::cout << "[test_cov2cov_anisotropic_weighting] PASSED\n";
}

// ===========================================================================
//  Test 7: Robust kernel with cov2cov pairings
// ===========================================================================

void test_cov2cov_robust_kernel()
{
    std::cout << "[test_cov2cov_robust_kernel] Running...\n";

    const auto gtPose =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.8, -0.3, 0.2, 3.0_deg, -2.0_deg, 1.0_deg);

    // Generate good pairings
    auto pairings = generate_cov2cov_pairings(gtPose, 30, 0.0, false);

    // Add some outliers
    for (int i = 0; i < 5; i++)
    {
        auto& p  = pairings.paired_cov2cov.emplace_back();
        p.global = {normal_f(50.0f), normal_f(50.0f), normal_f(50.0f)};
        p.local  = {normal_f(50.0f), normal_f(50.0f), normal_f(50.0f)};  // Wrong correspondence!
        p.cov_inv.setIdentity();
    }

    // Run WITH robust kernel
    mp2p_icp::OptimalTF_GN_Parameters gnParams;
    gnParams.linearizationPoint     = mrpt::poses::CPose3D::Identity();
    gnParams.maxInnerLoopIterations = 30;
    gnParams.kernel                 = mp2p_icp::RobustKernel::GemanMcClure;
    gnParams.kernelParam            = 1.0;

    mp2p_icp::OptimalTF_Result result;
    ASSERT_(mp2p_icp::optimal_tf_gauss_newton(pairings, result, gnParams));

    const auto   poseError = gtPose - result.optimalPose;
    const double err_xyz   = poseError.translation().norm();
    const double err_rot   = mrpt::poses::Lie::SO<3>::log(poseError.getRotationMatrix()).norm();

    std::cout << "  With outliers + robust kernel: xyz_err=" << err_xyz << " rot_err=" << err_rot
              << "\n";

    // Should still converge reasonably well with robust kernel
    ASSERT_LT_(err_xyz, 0.3);
    ASSERT_LT_(err_rot, 0.3);

    std::cout << "[test_cov2cov_robust_kernel] PASSED\n";
}

// ===========================================================================
//  Test 8: Empty pairings returns fallback covariance
// ===========================================================================

void test_cov2cov_empty_pairings()
{
    std::cout << "[test_cov2cov_empty_pairings] Running...\n";

    mp2p_icp::Pairings emptyPairings;
    ASSERT_(emptyPairings.empty());

    mp2p_icp::CovarianceParameters covParams;
    const auto                     cov =
        mp2p_icp::covariance(emptyPairings, mrpt::poses::CPose3D::Identity(), covParams);

    // Should return the fallback diagonal matrix with large values
    for (int i = 0; i < 6; i++)
    {
        ASSERT_NEAR_(cov(i, i), 1e6, 1.0);  // Allow small tolerance
    }

    std::cout << "[test_cov2cov_empty_pairings] PASSED\n";
}

// ===========================================================================
//  Test 9: Covariance consistency between solver and estimator
// ===========================================================================

void test_cov2cov_solver_covariance_consistency()
{
    std::cout << "[test_cov2cov_solver_covariance_consistency] Running...\n";

    const auto gtPose =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.5, 0.3, -0.1, 2.0_deg, 1.0_deg, -1.0_deg);

    // Generate pairings with small noise
    auto pairings = generate_cov2cov_pairings(gtPose, 50, 0.005, true);

    // Optimize
    mp2p_icp::OptimalTF_GN_Parameters gnParams;
    gnParams.linearizationPoint     = mrpt::poses::CPose3D::Identity();
    gnParams.maxInnerLoopIterations = 30;

    mp2p_icp::OptimalTF_Result result;
    ASSERT_(mp2p_icp::optimal_tf_gauss_newton(pairings, result, gnParams));

    // Compute covariance at the optimized pose
    mp2p_icp::CovarianceParameters covParams;
    const auto cov = mp2p_icp::covariance(pairings, result.optimalPose, covParams);

    // The actual pose error should be within ~3 sigma of the estimated uncertainty
    const auto poseError    = gtPose - result.optimalPose;
    const auto err_se3      = mrpt::poses::Lie::SE<3>::log(poseError);
    const auto cov_diag_std = cov.asEigen().diagonal().array().sqrt();

    std::cout << "  Pose error (SE3 log): " << err_se3.asEigen().transpose() << "\n";
    std::cout << "  Estimated std dev:    " << cov_diag_std.transpose() << "\n";

    // Covariance should be reasonable (not too large, not too small)
    ASSERT_LT_(cov_diag_std.maxCoeff(), 1.0);  // Not too uncertain
    ASSERT_GT_(cov_diag_std.minCoeff(), 1e-6);  // Not unrealistically certain

    std::cout << "[test_cov2cov_solver_covariance_consistency] PASSED\n";
}

// ===========================================================================
//  Test 10: Large number of pairings
// ===========================================================================

void test_cov2cov_large_scale()
{
    std::cout << "[test_cov2cov_large_scale] Running...\n";

    const auto gtPose =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(1.2, -0.8, 0.5, 5.0_deg, -3.0_deg, 2.0_deg);

    // Generate many pairings
    auto pairings = generate_cov2cov_pairings(gtPose, 500, 0.001, true);

    // Optimize
    mp2p_icp::OptimalTF_GN_Parameters gnParams;
    gnParams.linearizationPoint     = mrpt::poses::CPose3D::Identity();
    gnParams.maxInnerLoopIterations = 20;

    mp2p_icp::OptimalTF_Result result;
    ASSERT_(mp2p_icp::optimal_tf_gauss_newton(pairings, result, gnParams));

    const auto poseError = gtPose - result.optimalPose;
    ASSERT_LT_(poseError.translation().norm(), 0.05);
    ASSERT_LT_(mrpt::poses::Lie::SO<3>::log(poseError.getRotationMatrix()).norm(), 0.05);

    // Covariance should be smaller with more pairings
    mp2p_icp::CovarianceParameters covParams;
    const auto cov = mp2p_icp::covariance(pairings, result.optimalPose, covParams);

    const double max_std = std::sqrt(cov.asEigen().diagonal().maxCoeff());
    ASSERT_LT_(max_std, 0.1);  // Should be quite certain with 500 pairings

    std::cout << "  Max std dev with 500 pairings: " << max_std << "\n";
    std::cout << "[test_cov2cov_large_scale] PASSED\n";
}

}  // namespace

// ===========================================================================
//  Main
// ===========================================================================

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        rnd.randomize(1234);  // For reproducible tests

        // Run all tests
        test_cov2cov_optimization_basic();
        test_cov2cov_optimization_random();
        test_cov2cov_covariance_output_valid();
        test_cov2cov_only_covariance();
        test_cov2cov_mixed_pairings();
        test_cov2cov_anisotropic_weighting();
        test_cov2cov_robust_kernel();
        test_cov2cov_empty_pairings();
        test_cov2cov_solver_covariance_consistency();
        test_cov2cov_large_scale();

        std::cout << "\n========================================\n";
        std::cout << "All cov2cov tests PASSED!\n";
        std::cout << "========================================\n";

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}