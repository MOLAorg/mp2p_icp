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
 * @file   test-mp2p_optimize_prior_referenced_kernel.cpp
 * @brief  Unit tests for the prior-referenced robust kernel (Approach B):
 *         the robust kernel residual reference is blended toward the prior mean
 *         pose, so factors inconsistent with the prior are down-weighted even
 *         when the current linearization point is already corrupted.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 23, 2026
 */

#include <mp2p_icp/Solver_GaussNewton.h>
#include <mrpt/poses/Lie/SE.h>

#include <iostream>
#include <vector>

namespace
{
// A spread-out set of global points, enough to constrain SE(3).
const std::vector<mrpt::math::TPoint3D> kGlobals = {
    {3, 0, 0},  {0, 3, 0},  {0, 0, 3},  {2, 2, 0},   {2, 0, 2},   {0, 2, 2},
    {-3, 1, 1}, {1, -3, 1}, {1, 1, -3}, {-2, -2, 1}, {2, -1, -2}, {-1, 2, -2},
};

// Append point-to-point pairings consistent with a given pose:
// local = pose^{-1} * global, so the optimal pose mapping local->global is `pose`.
void appendPairings(mp2p_icp::Pairings& p, const mrpt::poses::CPose3D& pose)
{
    for (const auto& g : kGlobals)
    {
        auto& pp  = p.paired_pt2pt.emplace_back();
        pp.global = g;
        pp.local  = pose.inverseComposePoint(g);
    }
}

double poseDistance(const mrpt::poses::CPose3D& a, const mrpt::poses::CPose3D& b)
{
    return mrpt::poses::Lie::SE<3>::log(a - b).norm();
}

// Runs the GN solver with the given prior-reference blend, optionally providing
// a (zero-information) prior whose mean is used only as the kernel reference.
mrpt::poses::CPose3D runSolver(
    const mp2p_icp::Pairings& p, const mrpt::poses::CPose3D& guess, double blend, bool withPrior,
    const mrpt::poses::CPose3D& priorMean)
{
    mp2p_icp::Solver_GaussNewton solver;

    mrpt::containers::yaml solverParams;
    solverParams["maxIterations"]             = 50;
    solverParams["robustKernel"]              = "RobustKernel::Cauchy";
    solverParams["robustKernelParam"]         = 0.5;
    solverParams["robustKernelPriorRefBlend"] = blend;
    solver.initialize(solverParams);

    mp2p_icp::OptimalTF_Result result;
    mp2p_icp::SolverContext    sc;
    sc.guessRelativePose = guess;

    if (withPrior)
    {
        auto& prior = sc.prior.emplace();
        prior.mean  = priorMean;
        prior.cov_inv.fill(0);  // zero information: pure kernel reference, no spring
    }

    const bool ok = solver.optimal_pose(p, result, sc);
    ASSERT_(ok);
    return result.optimalPose;
}
}  // namespace

// Sanity: with clean data and no outliers, the prior-referenced kernel must
// still recover the ground truth (no regression).
static void test_clean_recovery()
{
    using namespace mrpt;  // _deg

    MRPT_START

    const auto gt =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.5, -0.3, 0.2, 8.0_deg, -5.0_deg, 3.0_deg);

    mp2p_icp::Pairings p;
    appendPairings(p, gt);

    const auto guess = mrpt::poses::CPose3D::Identity();

    // No prior at all:
    {
        const auto sol = runSolver(p, guess, 0.0, false, {});
        ASSERT_NEAR_(poseDistance(sol, gt), 0.0, 1e-4);
    }
    // Prior-referenced kernel active, prior mean == gt:
    {
        const auto sol = runSolver(p, guess, 1.0, true, gt);
        ASSERT_NEAR_(poseDistance(sol, gt), 0.0, 1e-4);
    }

    MRPT_END
}

// Headline test: a corrupted linearization point.
//
//  - Inliers are consistent with the ground-truth pose `gt`.
//  - Outliers are consistent with a very different `wrongPose`.
//  - The initial guess IS `wrongPose`, so at the linearization point the
//    OUTLIERS have ~zero residual and the inliers look bad. A classic
//    current-iterate robust kernel (blend=0) therefore locks onto the wrong
//    pose. The prior-referenced kernel (blend=1) evaluates each factor at the
//    prior mean (== gt), down-weighting the outliers and recovering `gt`.
static void test_corrupted_linearization_point()
{
    using namespace mrpt;  // _deg

    MRPT_START

    const auto gt = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto wrongPose =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(4.0, -3.0, 2.0, 40.0_deg, 20.0_deg, -25.0_deg);

    mp2p_icp::Pairings p;
    appendPairings(p, gt);  // inliers (12)
    // Coordinated outliers, outnumbering the inliers (3x) so that a plain
    // current-iterate robust kernel cannot escape the wrong basin:
    for (int i = 0; i < 3; i++)
    {
        appendPairings(p, wrongPose);
    }

    // The (corrupted) initial guess and linearization point:
    const auto guess = wrongPose;

    // 1) Baseline current-iterate kernel: should stay stuck near wrongPose.
    const auto solBaseline = runSolver(p, guess, 0.0, true, gt);
    std::cout << "[corrupted] baseline (blend=0): " << solBaseline << "\n";
    ASSERT_GT_(poseDistance(solBaseline, gt), 1.0);  // far from gt
    ASSERT_LT_(poseDistance(solBaseline, wrongPose), 0.5);  // near wrongPose

    // 2) Prior-referenced kernel: should recover gt.
    const auto solPriorRef = runSolver(p, guess, 1.0, true, gt);
    std::cout << "[corrupted] prior-ref (blend=1): " << solPriorRef << "\n";
    // Recovered gt: a small residual bias remains because the many coordinated
    // outliers keep tiny (nonzero) Cauchy weights, but it is an order of
    // magnitude closer to gt than the baseline (which sits ~5 units away).
    ASSERT_LT_(poseDistance(solPriorRef, gt), 0.25);  // recovered gt
    ASSERT_LT_(poseDistance(solPriorRef, gt), 0.2 * poseDistance(solBaseline, gt));

    // 3) Inertness: same blend=1 but NO prior -> must behave like the baseline
    //    (kernel falls back to the current iterate), i.e. stays near wrongPose.
    const auto solNoPrior = runSolver(p, guess, 1.0, false, {});
    std::cout << "[corrupted] blend=1 no prior: " << solNoPrior << "\n";
    ASSERT_GT_(poseDistance(solNoPrior, gt), 1.0);
    ASSERT_LT_(poseDistance(solNoPrior, solBaseline), 1e-4);  // identical to baseline

    MRPT_END
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_clean_recovery();
        test_corrupted_linearization_point();
    }
    catch (const std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
    return 0;
}
