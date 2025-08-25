/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/

/**
 * @file   test-mp2p_optimize_with_prior.cpp
 * @brief  Unit tests for point-to-point optimization with a prior belief
 * @author Jose Luis Blanco Claraco
 * @date   Jan 17, 2024
 */

#include <mp2p_icp/Solver_GaussNewton.h>
#include <mrpt/poses/Lie/SE.h>

static void test_opt_prior(const mrpt::poses::CPose3D& groundTruth)
{
    using namespace mrpt::poses::Lie;
    using namespace mrpt;  // _deg

    MRPT_START

    // Prepare test case pairings:
    // 3 point-to-line correspondences, such that the sought optimal
    // pose is the given one:

    mp2p_icp::Pairings p;

    {
        auto& pp  = p.paired_pt2pt.emplace_back();
        pp.global = {1, 0, 0};
        pp.local  = groundTruth.inverseComposePoint(pp.global);
    }
    {
        auto& pp  = p.paired_pt2pt.emplace_back();
        pp.global = {0, 1, 0};
        pp.local  = groundTruth.inverseComposePoint(pp.global);
    }
    {
        auto& pp  = p.paired_pt2pt.emplace_back();
        pp.global = {0, 0, 1};
        pp.local  = groundTruth.inverseComposePoint(pp.global);
    }

    std::cout << "Input pairings: " << p.contents_summary() << std::endl;

    // Init solver:
    mp2p_icp::Solver_GaussNewton solver;

    mrpt::containers::yaml solverParams;
    solverParams["maxIterations"]    = 25;
    solverParams["innerLoopVerbose"] = true;

    solver.initialize(solverParams);

    for (int Case = 0; Case < 3; Case++)
    {
        mp2p_icp::OptimalTF_Result result;
        mp2p_icp::SolverContext    sc;
        sc.guessRelativePose = mrpt::poses::CPose3D::Identity();

        auto& prior = sc.prior.emplace();
        prior.mean =
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(2.0, 3.0, 4.0, 10.0_deg, 10.0_deg, 10.0_deg);
        prior.cov_inv.fill(0);

        mrpt::poses::CPose3D      expected;
        std::function<void(void)> checkFn;

        switch (Case)
        {
            case 0:
                sc.prior.reset();  // no prior. Delete it
                expected = groundTruth;
                checkFn  = [&]()
                {
                    ASSERT_NEAR_(
                        mrpt::poses::Lie::SE<3>::log(result.optimalPose - expected).norm(), 0.0,
                        1e-3);
                };
                break;
            case 1:
                expected = prior.mean;
                for (int i = 0; i < 3; i++) prior.cov_inv(i, i) = 100.0;
                checkFn = [&]()
                {
                    for (int i = 0; i < 3; i++)
                        ASSERT_NEAR_(result.optimalPose[i], prior.mean[i], 0.05);
                };
                break;
            case 2:
                expected = prior.mean;
                for (int i = 3; i < 6; i++) prior.cov_inv(i, i) = 100.0;
                checkFn = [&]()
                {
                    for (int i = 3; i < 6; i++)
                        ASSERT_NEAR_(result.optimalPose[i], prior.mean[i], 0.05);
                };
                break;
        };

        bool solvedOk = solver.optimal_pose(p, result, sc);

        std::cout << "Case:" << Case << "\n"
                  << "Found    optimalPose: " << result.optimalPose << std::endl;
        std::cout << "Expected optimalPose: " << expected << std::endl;

        checkFn();

        // check results:
        ASSERT_(solvedOk);
    }

    MRPT_END
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    using mrpt::poses::CPose3D;
    using namespace mrpt;  // _deg

    try
    {
        test_opt_prior(CPose3D::FromXYZYawPitchRoll(1.0, 2.0, 3.0, 5.0_deg, 15.0_deg, 20.0_deg));
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
