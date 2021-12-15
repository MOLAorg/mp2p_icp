/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mp2p_optimize_pt2pl.cpp
 * @brief  Unit tests for point-to-plane optimization
 * @author Jose Luis Blanco Claraco
 * @date   Dec 4, 2021
 */

#include <mp2p_icp/Solver_GaussNewton.h>
#include <mp2p_icp/Solver_Horn.h>
#include <mrpt/poses/Lie/SE.h>

static void test_opt_pt2pl(
    const mrpt::poses::CPose3D& groundTruth, const mp2p_icp::Solver& solver)
{
    using namespace mrpt::poses::Lie;

    MRPT_START

    // Prepare test case pairings:
    // 3 point-to-plane correspondences, such that the sought optimal
    // pose is the given one:

    mp2p_icp::Pairings p;

    {
        auto& pp     = p.paired_pt2pl.emplace_back();
        pp.pl_global = {
            mrpt::math::TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1}),
            {0, 0, 0}};
        pp.pt_local = groundTruth.inverseComposePoint({0.5, 0, 0});
    }
    {
        auto& pp     = p.paired_pt2pl.emplace_back();
        pp.pl_global = {
            mrpt::math::TPlane::FromPointAndNormal({0, 0, 0}, {1, 0, 0}),
            {0, 0, 0}};
        pp.pt_local = groundTruth.inverseComposePoint({0, 0.8, 0});
    }
    {
        auto& pp     = p.paired_pt2pl.emplace_back();
        pp.pl_global = {
            mrpt::math::TPlane::FromPointAndNormal({0, 0, 0}, {0, 1, 0}),
            {0, 0, 0}};
        pp.pt_local = groundTruth.inverseComposePoint({0, 0, 0.3});
    }
    {
        auto& pp       = p.paired_pt2pt.emplace_back();
        pp.global      = {0, 0, 0};
        const auto loc = groundTruth.inverseComposePoint({0, 0, 0});
        pp.local       = loc;
    }

    std::cout << "Input pairings: " << p.contents_summary() << std::endl;

    // Init solver:

    mp2p_icp::OptimalTF_Result result;
    mp2p_icp::SolverContext    sc;
    sc.guessRelativePose = mrpt::poses::CPose3D::Identity();

    bool solvedOk = solver.optimal_pose(p, result, sc);

    std::cout << "Found    optimalPose: " << result.optimalPose << std::endl;
    std::cout << "Expected optimalPose: " << groundTruth << std::endl;

    // check results:
    ASSERT_(solvedOk);
    ASSERT_NEAR_(
        mrpt::poses::Lie::SE<3>::log(result.optimalPose - groundTruth).norm(),
        0.0, 1e-3);

    MRPT_END
}

static void test_mp2p_optimize_pt2pl()
{
    using mrpt::poses::CPose3D;
    using namespace mrpt;  // _deg

    // With different solvers:
    mp2p_icp::Solver_GaussNewton solverGN;
    {
        mrpt::containers::yaml solverParams;
        solverParams["maxIterations"] = 25;
        // solverParams["innerLoopVerbose"] = true;
        solverGN.initialize(solverParams);
    }
    // mp2p_icp::Solver_Horn solverHorn;

    const std::vector<const mp2p_icp::Solver*> solvers = {
        &solverGN,
        //&solverHorn
    };

    for (const auto solverPtr : solvers)
    {
        const auto& solver = *solverPtr;
        std::cout
            << "Using solver: " << solver.GetRuntimeClass()->className
            << "\n"
               "=========================================================\n";

        test_opt_pt2pl(CPose3D::FromTranslation(0, 0, 0), solver);

        test_opt_pt2pl(CPose3D::FromTranslation(1.0, 0, 0), solver);
        test_opt_pt2pl(CPose3D::FromTranslation(0, 1.0, 0), solver);
        test_opt_pt2pl(CPose3D::FromTranslation(0, 0, 1.0), solver);

        test_opt_pt2pl(CPose3D::FromTranslation(-2.0, 0, 0), solver);
        test_opt_pt2pl(CPose3D::FromTranslation(0, -3.0, 0), solver);
        test_opt_pt2pl(CPose3D::FromTranslation(0, 0, -4.0), solver);

        test_opt_pt2pl(
            CPose3D::FromYawPitchRoll(20.0_deg, 0.0_deg, 0.0_deg), solver);
        test_opt_pt2pl(
            CPose3D::FromYawPitchRoll(-20.0_deg, 0.0_deg, 0.0_deg), solver);

        test_opt_pt2pl(
            CPose3D::FromYawPitchRoll(0.0_deg, 10.0_deg, 0.0_deg), solver);
        test_opt_pt2pl(
            CPose3D::FromYawPitchRoll(0.0_deg, -10.0_deg, 0.0_deg), solver);

        test_opt_pt2pl(
            CPose3D::FromYawPitchRoll(0.0_deg, 0.0_deg, 15.0_deg), solver);
        test_opt_pt2pl(
            CPose3D::FromYawPitchRoll(0.0_deg, 0.0_deg, -15.0_deg), solver);

        test_opt_pt2pl(CPose3D::FromTranslation(1.0, 2.0, 3.0), solver);
        test_opt_pt2pl(
            CPose3D::FromXYZYawPitchRoll(
                1.0, 2.0, 3.0, -10.0_deg, 5.0_deg, 30.0_deg),
            solver);
    }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_mp2p_optimize_pt2pl();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
