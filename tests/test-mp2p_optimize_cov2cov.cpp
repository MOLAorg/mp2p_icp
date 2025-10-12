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
 * @file   test-mp2p_optimize_cov2cov.cpp
 * @brief  Unit tests for optimization of cov2cov pairings
 * @author Jose Luis Blanco Claraco
 * @date   Oct 10, 2025
 */

#include <mp2p_icp/Pairings.h>
#include <mp2p_icp/Results.h>
#include <mp2p_icp/optimal_tf_gauss_newton.h>
#include <mrpt/poses/Lie/SO.h>

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    using mrpt::literals::operator""_deg;

    try
    {
        const auto gtPose =
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(1.0, 0.0, 0.0, 0.0_deg, 0.0_deg, 0.0_deg);
        const auto initPose =
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.0, 2.0, 3.0, 5.0_deg, -5.0_deg, -10.0_deg);

        // Pairings:
        // -----------------------------
        mp2p_icp::Pairings pairings;
        // 3 good pairings, each in a different axis, to make the solution convex:
        {
            auto& p  = pairings.paired_cov2cov.emplace_back();
            p.global = {1.0f, 0.0f, 1.0f};
            p.local  = {0.0f, 0.0f, 1.0f};
            p.cov_inv.setDiagonal(std::vector<float>({1.0, 1e6, 1.0}));
        }
        {
            auto& p  = pairings.paired_cov2cov.emplace_back();
            p.global = {2.0f, 5.0f, 0.0f};
            p.local  = {1.0f, 5.0f, 0.0f};
            p.cov_inv.setDiagonal(std::vector<float>({1.0, 1.0, 1e6}));
        }
        {
            auto& p  = pairings.paired_cov2cov.emplace_back();
            p.global = {0.0f, 8.0f, 1.0f};
            p.local  = {-1.0f, 8.0f, 1.0f};
            p.cov_inv.setDiagonal(std::vector<float>({1e6, 1.0, 1.0}));
        }
        // plus a "bad" pairing:
        if (0)
        {
            auto& p  = pairings.paired_cov2cov.emplace_back();
            p.global = {0.0f, 8.0f, 1.0f};
            p.local  = {-1.0f, 7.0f, 0.8f};
            p.cov_inv.setDiagonal(std::vector<float>({1e6, 1.0, 1.0}));
        }

        // Run Gauss-Newton:
        // -----------------------------
        mp2p_icp::OptimalTF_GN_Parameters gnParams;
        gnParams.linearizationPoint = initPose;
        gnParams.verbose            = true;

        mp2p_icp::OptimalTF_Result result;

        const bool run_ok = mp2p_icp::optimal_tf_gauss_newton(pairings, result, gnParams);
        ASSERT_(run_ok);

        // Compare vs GT:
        // -----------------------------
        std::cout << "optimalPose: " << result.optimalPose << "\n";
        const auto poseError = gtPose - result.optimalPose;
        ASSERT_LT_(poseError.translation().norm(), 0.075);
        ASSERT_LT_(mrpt::poses::Lie::SO<3>::log(poseError.getRotationMatrix()).norm(), 0.05);
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
