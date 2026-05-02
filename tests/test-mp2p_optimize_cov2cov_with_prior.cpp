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
 * @file   test-mp2p_optimize_cov2cov_with_prior.cpp
 * @brief  Verifies that, when many cov2cov pairings are combined with a
 *         pose prior, the prior is not drowned by the data block thanks to
 *         the generalized-Bayes alpha and the Birge-ratio auto-balance.
 * @author Jose Luis Blanco Claraco
 */

#include <mp2p_icp/Pairings.h>
#include <mp2p_icp/Results.h>
#include <mp2p_icp/optimal_tf_gauss_newton.h>
#include <mrpt/poses/Lie/SO.h>

#include <random>

namespace
{
// Build a set of cov2cov pairings consistent (up to noise) with `gtPose`,
// scattered around the origin. Each pairing has a slightly inflated, isotropic
// inverse covariance so that no single pairing fixes a degenerate direction.
mp2p_icp::Pairings makeCov2CovPairings(const mrpt::poses::CPose3D& gtPose, std::size_t N)
{
    std::mt19937                          rng(42);
    std::uniform_real_distribution<float> u(-5.0f, 5.0f);
    // Actual point-pair noise is large (σ ≈ 0.3 m), but the *modeled*
    // per-pair information below pretends σ ≈ 1 cm — i.e. the per-pair
    // covariances are heavily overconfident, mimicking the realistic case
    // where neighbouring cov2cov pairings see correlated surface noise that
    // their independent Gaussians cannot capture.
    std::normal_distribution<float> noise(0.0f, 0.3f);

    mp2p_icp::Pairings out;
    out.paired_cov2cov.reserve(N);
    for (std::size_t i = 0; i < N; i++)
    {
        auto& p       = out.paired_cov2cov.emplace_back();
        p.local       = {u(rng), u(rng), u(rng)};
        const auto pg = gtPose.composePoint(
            {p.local.x + noise(rng), p.local.y + noise(rng), p.local.z + noise(rng)});
        p.global = {static_cast<float>(pg.x), static_cast<float>(pg.y), static_cast<float>(pg.z)};
        // Overconfident modeled per-pair information: σ ≈ 1 cm isotropic.
        p.cov_inv.setDiagonal(std::vector<float>({1e4f, 1e4f, 1e4f}));
    }
    return out;
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    using mrpt::literals::operator""_deg;

    try
    {
        // Ground-truth pose, and a *biased* prior shifted from it. The data
        // covers the GT, so an unbalanced solver will ignore the prior; a
        // properly balanced one will pull the estimate towards the prior.
        const auto gtPose =
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(1.0, 0.0, 0.0, 0.0_deg, 0.0_deg, 0.0_deg);
        const auto priorMean =
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(1.5, 0.0, 0.0, 0.0_deg, 0.0_deg, 0.0_deg);

        // Many pairings, so the data block is O(N) larger than the prior.
        const std::size_t N        = 400;
        const auto        pairings = makeCov2CovPairings(gtPose, N);

        // Reasonably tight prior (info = 1e4 → σ ≈ 1 cm).
        mrpt::poses::CPose3DPDFGaussianInf prior;
        prior.mean = priorMean;
        prior.cov_inv.setZero();
        for (int i = 0; i < 6; i++) prior.cov_inv(i, i) = 1e4;

        const auto initPose = priorMean;

        // Case A: balancing DISABLED (alpha=1, auto-balance off) → data wins.
        mrpt::poses::CPose3D poseNoBalance;
        {
            mp2p_icp::OptimalTF_GN_Parameters gnParams;
            gnParams.linearizationPoint              = initPose;
            gnParams.prior                           = prior;
            gnParams.cov2cov_alpha                   = 1.0;
            gnParams.cov2cov_auto_balance_with_prior = false;
            gnParams.maxInnerLoopIterations          = 20;

            mp2p_icp::OptimalTF_Result result;
            ASSERT_(mp2p_icp::optimal_tf_gauss_newton(pairings, result, gnParams));
            poseNoBalance = result.optimalPose;
            std::cout << "[no-balance] pose: " << poseNoBalance << "\n";
        }

        // Case B: auto-balance ENABLED (default) → prior is respected.
        mrpt::poses::CPose3D poseBalanced;
        {
            mp2p_icp::OptimalTF_GN_Parameters gnParams;  // defaults: balance ON
            gnParams.linearizationPoint     = initPose;
            gnParams.prior                  = prior;
            gnParams.maxInnerLoopIterations = 20;

            mp2p_icp::OptimalTF_Result result;
            ASSERT_(mp2p_icp::optimal_tf_gauss_newton(pairings, result, gnParams));
            poseBalanced = result.optimalPose;
            std::cout << "[balanced ] pose: " << poseBalanced << "\n";
        }

        const double dxNoBalance = std::abs(poseNoBalance.x() - priorMean.x());
        const double dxBalanced  = std::abs(poseBalanced.x() - priorMean.x());
        std::cout << "|x - x_prior|  no-balance=" << dxNoBalance << "  balanced=" << dxBalanced
                  << "\n";

        // Without balancing, the solver should snap to the data (≈ gtPose).
        ASSERT_LT_(std::abs(poseNoBalance.x() - gtPose.x()), 0.05);

        // With balancing, the result must be measurably pulled back toward
        // the prior compared with the unbalanced case.
        ASSERT_LT_(dxBalanced, dxNoBalance - 0.05);

        // And it must remain a sensible compromise between prior and data,
        // i.e. located between them along x (within tolerances).
        ASSERT_GT_(poseBalanced.x(), gtPose.x() - 0.05);
        ASSERT_LT_(poseBalanced.x(), priorMean.x() + 0.05);

        // Sanity: with α=1 and balancing off, alpha=1/N should also recover a
        // prior-respecting result (manual generalized-Bayes path).
        {
            mp2p_icp::OptimalTF_GN_Parameters gnParams;
            gnParams.linearizationPoint              = initPose;
            gnParams.prior                           = prior;
            gnParams.cov2cov_alpha                   = 1.0 / static_cast<double>(N);
            gnParams.cov2cov_auto_balance_with_prior = false;
            gnParams.maxInnerLoopIterations          = 20;

            mp2p_icp::OptimalTF_Result result;
            ASSERT_(mp2p_icp::optimal_tf_gauss_newton(pairings, result, gnParams));
            std::cout << "[alpha=1/N] pose: " << result.optimalPose << "\n";
            ASSERT_LT_(std::abs(result.optimalPose.x() - priorMean.x()), dxNoBalance - 0.05);
        }

        // No-prior regression: with no prior, balancing must be inert (κ
        // multiplier disabled), so the original cov2cov-only test setup
        // still converges to GT.
        {
            const auto initBad = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                0.0, 0.2, 0.1, 2.0_deg, -2.0_deg, -3.0_deg);

            mp2p_icp::OptimalTF_GN_Parameters gnParams;  // default balance ON
            gnParams.linearizationPoint     = initBad;
            gnParams.maxInnerLoopIterations = 20;

            mp2p_icp::OptimalTF_Result result;
            ASSERT_(mp2p_icp::optimal_tf_gauss_newton(pairings, result, gnParams));
            const auto poseError = gtPose - result.optimalPose;
            ASSERT_LT_(poseError.translation().norm(), 0.05);
            ASSERT_LT_(mrpt::poses::Lie::SO<3>::log(poseError.getRotationMatrix()).norm(), 0.05);
        }
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
