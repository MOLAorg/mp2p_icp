/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   optimal_tf_gauss_newton.cpp
 * @brief  Simple non-linear optimizer to find the SE(3) optimal transformation
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */

#include <mp2p_icp/errorTerms.h>
#include <mp2p_icp/optimal_tf_gauss_newton.h>
#include <mrpt/poses/Lie/SE.h>
#include <Eigen/Dense>
#include <iostream>

using namespace mp2p_icp;

void mp2p_icp::optimal_tf_gauss_newton(
    const Pairings& in, const WeightParameters& wp, OptimalTF_Result& result,
    const OptimalTF_GN_Parameters& gnParams)
{
    using std::size_t;

    MRPT_START

    // Run Gauss-Newton steps, using SE(3) relinearization at the current
    // solution:
    ASSERTMSG_(
        gnParams.linearizationPoint.has_value(),
        "This method requires a linearization point");

    result.optimalPose = gnParams.linearizationPoint.value();

    const auto nPt2Pt = in.paired_pt2pt.size();
    const auto nPt2Ln = in.paired_pt2ln.size();
    const auto nPt2Pl = in.paired_pt2pl.size();
    const auto nPl2Pl = in.paired_pl2pl.size();
    const auto nLn2Ln = in.paired_ln2ln.size();

    const auto nErrorTerms = (nPt2Pt + nPl2Pl) * 3 + nPt2Pl + nPt2Ln + nLn2Ln * 4;

    Eigen::VectorXd                          err(nErrorTerms);
    Eigen::Matrix<double, Eigen::Dynamic, 6> J(nErrorTerms, 6);

    auto w = wp.pair_weights;

    const bool  has_per_pt_weight       = !in.point_weights.empty();
    auto        cur_point_block_weights = in.point_weights.begin();
    std::size_t cur_point_block_start   = 0;

    MRPT_TODO("Implement robust Kernel in this solver");

    for (size_t iter = 0; iter < gnParams.maxInnerLoopIterations; iter++)
    {
        // (12x6 Jacobian)
        const auto dDexpe_de =
            mrpt::poses::Lie::SE<3>::jacob_dDexpe_de(result.optimalPose);

        // Point-to-point:
        for (size_t idx_pt = 0; idx_pt < nPt2Pt; idx_pt++)
        {
            // Error:
            const auto& p = in.paired_pt2pt[idx_pt];
            mrpt::math::CMatrixFixed<double, 3, 12> J1;
            mrpt::math::CVectorFixedDouble<3> ret = mp2p_icp::error_point2point(p, result.optimalPose, J1);
            err.block<3, 1>(idx_pt * 3, 0) = ret.asEigen();

            // Get weight:
            if (has_per_pt_weight)
            {
                if (idx_pt >=
                    cur_point_block_start + cur_point_block_weights->first)
                {
                    ASSERT_(cur_point_block_weights != in.point_weights.end());
                    ++cur_point_block_weights;  // move to next block
                    cur_point_block_start = idx_pt;
                }
                w.pt2pt = cur_point_block_weights->second;
            }

            // Build Jacobian:
            J.block<3, 6>(idx_pt * 3, 0) = w.pt2pt * J1.asEigen() * dDexpe_de.asEigen();
        }
        auto base_idx = nPt2Pt * 3;

        // Point-to-line
        for (size_t idx_pt = 0; idx_pt < nPt2Ln; idx_pt++)
        {
            // Error
            const auto& p = in.paired_pt2ln[idx_pt];
            mrpt::math::CMatrixFixed<double, 1, 12> J1;
            mrpt::math::CVectorFixedDouble<1> ret = mp2p_icp::error_point2line(p, result.optimalPose, J1);
            err.block<1, 1>(base_idx + idx_pt, 0) = ret.asEigen();

            // Get weight
            // ...

            // Build Jacobian
            J.block<1, 6>(base_idx + idx_pt, 0) = w.pt2ln * J1.asEigen() * dDexpe_de.asEigen();
        }
        base_idx += nPt2Ln;

        // Line-to-Line
        // Minimum angle to approach zero
        for (size_t idx_ln = 0; idx_ln < nLn2Ln; idx_ln++)
        {
            const auto& p = in.paired_ln2ln[idx_ln];
            mrpt::math::CMatrixFixed<double, 4, 12> J1;
            mrpt::math::CVectorFixedDouble<4> ret = mp2p_icp::error_line2line(p,result.optimalPose, J1);
            err.block<4, 1>(base_idx + idx_ln * 4, 0) = ret.asEigen();

            // Build Jacobian
            J.block<4, 6>(base_idx + idx_ln, 0) = J1.asEigen() * dDexpe_de.asEigen();
        }
         base_idx += nLn2Ln;

        // Point-to-plane:
        for (size_t idx_pl = 0; idx_pl < nPt2Pl; idx_pl++)
        {
            // Error:
            const auto& p = in.paired_pt2pl[idx_pl];
            mrpt::math::CMatrixFixed<double, 1, 12> J1;
            mrpt::math::CVectorFixedDouble<1> ret = mp2p_icp::error_point2plane(p,result.optimalPose,J1);
            err.block<1, 1>(idx_pl + base_idx, 0) = ret.asEigen();

            J.block<1, 6>(idx_pl + base_idx, 0) =
                w.pt2pl * J1.asEigen() * dDexpe_de.asEigen();
        }
        base_idx += nPt2Pl * 1;

        // Plane-to-plane (only direction of normal vectors):
        for (size_t idx_pl = 0; idx_pl < nPl2Pl; idx_pl++)
        {
            // Error term:
            const auto& p = in.paired_pl2pl[idx_pl];
            mrpt::math::CMatrixFixed<double, 3, 12> J1;
            mrpt::math::CVectorFixedDouble<3> ret = mp2p_icp::error_plane2plane(p, result.optimalPose,J1);
            err.block<3, 1>(idx_pl * 3 + base_idx, 0) = ret.asEigen();

            J.block<3, 6>(3 * idx_pl + base_idx, 0) =
                w.pl2pl * J1.asEigen() * dDexpe_de.asEigen();
        }

        // 3) Solve Gauss-Newton:
        const Eigen::VectorXd             g = J.transpose() * err;
        const Eigen::Matrix<double, 6, 6> H = J.transpose() * J;
        const Eigen::Matrix<double, 6, 1> delta =
            -H.colPivHouseholderQr().solve(g);

        // 4) add SE(3) increment:
        const auto dE = mrpt::poses::Lie::SE<3>::exp(
            mrpt::math::CVectorFixed<double, 6>(delta));

        result.optimalPose = result.optimalPose + dE;

        if (gnParams.verbose)
        {
            std::cout << "[P2P GN] iter:" << iter << " err:" << err.norm()
                      << " delta:" << delta.transpose() << "\n";
        }

        // Simple convergence test:
        if (delta.norm() < gnParams.minDelta) break;

    }  // for each iteration
    MRPT_END
}
