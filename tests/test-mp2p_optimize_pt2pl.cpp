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
#include <mrpt/poses/Lie/SE.h>

static void test_opt_pt2pl(const mrpt::poses::CPose3D& groundTruth)
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
        auto& pp = p.paired_pt2pt.emplace_back();
        // this=global, other=local
        pp.this_x      = 0;
        pp.this_y      = 0;
        pp.this_z      = 0;
        const auto loc = groundTruth.inverseComposePoint({0, 0, 0});
        pp.other_x     = loc.x;
        pp.other_y     = loc.y;
        pp.other_z     = loc.z;
    }

    std::cout << "Input pairings: " << p.contents_summary() << std::endl;

    // Init solver:
    mp2p_icp::Solver_GaussNewton solver;

    mrpt::containers::yaml solverParams;
    solverParams["maxIterations"]    = 25;
    solverParams["innerLoopVerbose"] = true;

    solver.initialize(solverParams);

    mp2p_icp::OptimalTF_Result result;
    mp2p_icp::SolverContext    sc;
    sc.guessRelativePose = mrpt::poses::CPose3D::Identity();

    bool solvedOk = solver.optimal_pose(p, result, sc);

    std::cout << "Found    optimalPose: " << result.optimalPose << std::endl;
    std::cout << "Expected optimalPose: " << groundTruth << std::endl;

    ASSERT_NEAR_(
        mrpt::poses::Lie::SE<3>::log(result.optimalPose - groundTruth).norm(),
        0.0, 1e-3);

    // check results:
    ASSERT_(solvedOk);

    MRPT_END
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    using mrpt::poses::CPose3D;
    using namespace mrpt;  // _deg

    try
    {
        test_opt_pt2pl(CPose3D::FromTranslation(0, 0, 0));

        test_opt_pt2pl(CPose3D::FromTranslation(1.0, 0, 0));
        test_opt_pt2pl(CPose3D::FromTranslation(0, 1.0, 0));
        test_opt_pt2pl(CPose3D::FromTranslation(0, 0, 1.0));

        test_opt_pt2pl(CPose3D::FromTranslation(-2.0, 0, 0));
        test_opt_pt2pl(CPose3D::FromTranslation(0, -3.0, 0));
        test_opt_pt2pl(CPose3D::FromTranslation(0, 0, -4.0));

        test_opt_pt2pl(CPose3D::FromYawPitchRoll(20.0_deg, 0.0_deg, 0.0_deg));
        test_opt_pt2pl(CPose3D::FromYawPitchRoll(-20.0_deg, 0.0_deg, 0.0_deg));

        test_opt_pt2pl(CPose3D::FromYawPitchRoll(0.0_deg, 10.0_deg, 0.0_deg));
        test_opt_pt2pl(CPose3D::FromYawPitchRoll(0.0_deg, -10.0_deg, 0.0_deg));

        test_opt_pt2pl(CPose3D::FromYawPitchRoll(0.0_deg, 0.0_deg, 15.0_deg));
        test_opt_pt2pl(CPose3D::FromYawPitchRoll(0.0_deg, 0.0_deg, -15.0_deg));

        test_opt_pt2pl(CPose3D::FromTranslation(1.0, 2.0, 3.0));
        test_opt_pt2pl(CPose3D::FromXYZYawPitchRoll(
            1.0, 2.0, 3.0, -10.0_deg, 5.0_deg, 30.0_deg));
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
