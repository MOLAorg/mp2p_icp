/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
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
#include <mp2p_icp/robust_kernels.h>
#include <mrpt/poses/Lie/SE.h>

#include <Eigen/Dense>
#include <iostream>

#if defined(MP2P_HAS_TBB)
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#endif

using namespace mp2p_icp;

bool mp2p_icp::optimal_tf_gauss_newton(
    const Pairings& in, OptimalTF_Result& result, const OptimalTF_GN_Parameters& gnParams)
{
    using std::size_t;

    MRPT_START

    // Run Gauss-Newton steps, using SE(3) relinearization at the current
    // solution:
    ASSERTMSG_(
        gnParams.linearizationPoint.has_value(), "This method requires a linearization point");

    result.optimalPose = gnParams.linearizationPoint.value();

    const robust_sqrt_weight_func_t robustSqrtWeightFunc =
        mp2p_icp::create_robust_kernel(gnParams.kernel, gnParams.kernelParam);

    const auto nPt2Pt = in.paired_pt2pt.size();
    const auto nPt2Ln = in.paired_pt2ln.size();
    const auto nPt2Pl = in.paired_pt2pl.size();
    const auto nPl2Pl = in.paired_pl2pl.size();
    const auto nLn2Ln = in.paired_ln2ln.size();

    // Note: Using Matrix<N,1> instead of Vector<N> for compatibility
    //       with Eigen<=3.4 in ROS Noetic.
    Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();

    auto w = gnParams.pairWeights;

    const bool  has_per_pt_weight       = !in.point_weights.empty();
    auto        cur_point_block_weights = in.point_weights.begin();
    std::size_t cur_point_block_start   = 0;

    for (size_t iter = 0; iter < gnParams.maxInnerLoopIterations; iter++)
    {
        // (12x6 Jacobian)
        const auto dDexpe_de = mrpt::poses::Lie::SE<3>::jacob_dDexpe_de(result.optimalPose);

        double errNormSqr = 0;

#if defined(MP2P_HAS_TBB)
        // For the TBB lambdas:
        // TBB call structure based on the beautiful implementation in KISS-ICP.
        struct Result
        {
            Result()
            {
                H.setZero();
                g.setZero();
            }

            Result operator+(const Result& other)
            {
                H += other.H;
                g += other.g;
                return *this;
            }

            Eigen::Matrix<double, 6, 6> H;
            Eigen::Matrix<double, 6, 1> g;
        };

        const auto& [H_tbb_pt2pt, g_tbb_pt2pt] = tbb::parallel_reduce(
            // Range
            tbb::blocked_range<size_t>{0, nPt2Pt},
            // Identity
            Result(),
            // 1st lambda: Parallel computation
            [&](const tbb::blocked_range<size_t>& r, Result res) -> Result
            {
                auto& [H_local, g_local] = res;
                for (size_t idx_pt = r.begin(); idx_pt < r.end(); idx_pt++)
                {
                    // Error:
                    const auto&                             p = in.paired_pt2pt[idx_pt];
                    mrpt::math::CMatrixFixed<double, 3, 12> J1;
                    mrpt::math::CVectorFixedDouble<3>       ret =
                        mp2p_icp::error_point2point(p, result.optimalPose, J1);

                    // Get point weight:
                    if (has_per_pt_weight)
                    {
                        if (idx_pt >= cur_point_block_start + cur_point_block_weights->first)
                        {
                            ASSERT_(cur_point_block_weights != in.point_weights.end());
                            ++cur_point_block_weights;  // move to next block
                            cur_point_block_start = idx_pt;
                        }
                        w.pt2pt = cur_point_block_weights->second;
                    }

                    // Apply robust kernel?
                    double weight = w.pt2pt, retSqrNorm = ret.asEigen().squaredNorm();
                    if (robustSqrtWeightFunc) weight *= robustSqrtWeightFunc(retSqrNorm);

                    // Error and Jacobian:
                    const Eigen::Vector3d err_i = ret.asEigen();
                    errNormSqr += weight * retSqrNorm;

                    const Eigen::Matrix<double, 3, 6> Ji = J1.asEigen() * dDexpe_de.asEigen();
                    g_local.noalias() += weight * Ji.transpose() * err_i;
                    H_local.noalias() += weight * Ji.transpose() * Ji;
                }
                return res;
            },
            // 2nd lambda: Parallel reduction
            [](Result a, const Result& b) -> Result { return a + b; });

        H = std::move(H_tbb_pt2pt);
        g = std::move(g_tbb_pt2pt);
#else
        // Point-to-point:
        for (size_t idx_pt = 0; idx_pt < nPt2Pt; idx_pt++)
        {
            // Error:
            const auto&                             p = in.paired_pt2pt[idx_pt];
            mrpt::math::CMatrixFixed<double, 3, 12> J1;
            mrpt::math::CVectorFixedDouble<3>       ret =
                mp2p_icp::error_point2point(p, result.optimalPose, J1);

            // Get point weight:
            if (has_per_pt_weight)
            {
                if (idx_pt >= cur_point_block_start + cur_point_block_weights->first)
                {
                    ASSERT_(cur_point_block_weights != in.point_weights.end());
                    ++cur_point_block_weights;  // move to next block
                    cur_point_block_start = idx_pt;
                }
                w.pt2pt = cur_point_block_weights->second;
            }

            // Apply robust kernel?
            double weight = w.pt2pt, retSqrNorm = ret.asEigen().squaredNorm();
            if (robustSqrtWeightFunc) weight *= robustSqrtWeightFunc(retSqrNorm);

            // Error and Jacobian:
            const Eigen::Vector3d err_i = ret.asEigen();
            errNormSqr += weight * retSqrNorm;

            const Eigen::Matrix<double, 3, 6> Ji = J1.asEigen() * dDexpe_de.asEigen();
            g.noalias() += weight * Ji.transpose() * err_i;
            H.noalias() += weight * Ji.transpose() * Ji;
        }
#endif

        // Point-to-line
        for (size_t idx_pt = 0; idx_pt < nPt2Ln; idx_pt++)
        {
            // Error
            const auto&                             p = in.paired_pt2ln[idx_pt];
            mrpt::math::CMatrixFixed<double, 3, 12> J1;
            mrpt::math::CVectorFixedDouble<3>       ret =
                mp2p_icp::error_point2line(p, result.optimalPose, J1);

            // Apply robust kernel?
            double weight = w.pt2ln, retSqrNorm = ret.asEigen().squaredNorm();
            if (robustSqrtWeightFunc) weight *= robustSqrtWeightFunc(retSqrNorm);

            // Error and Jacobian:
            const Eigen::Vector3d err_i = ret.asEigen();
            errNormSqr += weight * weight * retSqrNorm;

            const Eigen::Matrix<double, 3, 6> Ji = J1.asEigen() * dDexpe_de.asEigen();
            g.noalias() += weight * Ji.transpose() * err_i;
            H.noalias() += weight * Ji.transpose() * Ji;
        }

        // Line-to-Line
        // Minimum angle to approach zero
        for (size_t idx_ln = 0; idx_ln < nLn2Ln; idx_ln++)
        {
            const auto&                             p = in.paired_ln2ln[idx_ln];
            mrpt::math::CMatrixFixed<double, 4, 12> J1;
            mrpt::math::CVectorFixedDouble<4>       ret =
                mp2p_icp::error_line2line(p, result.optimalPose, J1);

            // Apply robust kernel?
            double weight = w.ln2ln, retSqrNorm = ret.asEigen().squaredNorm();
            if (robustSqrtWeightFunc) weight *= robustSqrtWeightFunc(retSqrNorm);

            // Error and Jacobian:
            const Eigen::Vector4d err_i = ret.asEigen();
            errNormSqr += weight * weight * retSqrNorm;

            const Eigen::Matrix<double, 4, 6> Ji = J1.asEigen() * dDexpe_de.asEigen();
            g.noalias() += weight * Ji.transpose() * err_i;
            H.noalias() += weight * Ji.transpose() * Ji;
        }

#if defined(MP2P_HAS_TBB)
        // Point-to-plane:
        const auto& [H_tbb_pt2pl, g_tbb_pt2pl] = tbb::parallel_reduce(
            // Range
            tbb::blocked_range<size_t>{0, nPt2Pl},
            // Identity
            Result(),
            // 1st lambda: Parallel computation
            [&](const tbb::blocked_range<size_t>& r, Result res) -> Result
            {
                auto& [H_local, g_local] = res;
                for (size_t idx_pl = r.begin(); idx_pl < r.end(); idx_pl++)
                {
                    // Error:
                    const auto&                             p = in.paired_pt2pl[idx_pl];
                    mrpt::math::CMatrixFixed<double, 3, 12> J1;
                    mrpt::math::CVectorFixedDouble<3>       ret =
                        mp2p_icp::error_point2plane(p, result.optimalPose, J1);

                    // Apply robust kernel?
                    double weight = w.pt2pl, retSqrNorm = ret.asEigen().squaredNorm();
                    if (robustSqrtWeightFunc) weight *= robustSqrtWeightFunc(retSqrNorm);

                    // Error and Jacobian:
                    const Eigen::Vector3d err_i = ret.asEigen();
                    errNormSqr += weight * retSqrNorm;

                    const Eigen::Matrix<double, 3, 6> Ji = J1.asEigen() * dDexpe_de.asEigen();
                    g_local.noalias() += weight * Ji.transpose() * err_i;
                    H_local.noalias() += weight * Ji.transpose() * Ji;
                }
                return res;
            },
            // 2nd lambda: Parallel reduction
            [](Result a, const Result& b) -> Result { return a + b; });

        H += H_tbb_pt2pl;
        g += g_tbb_pt2pl;
#else
        // Point-to-plane:
        for (size_t idx_pl = 0; idx_pl < nPt2Pl; idx_pl++)
        {
            // Error:
            const auto&                             p = in.paired_pt2pl[idx_pl];
            mrpt::math::CMatrixFixed<double, 3, 12> J1;
            mrpt::math::CVectorFixedDouble<3>       ret =
                mp2p_icp::error_point2plane(p, result.optimalPose, J1);

            // Apply robust kernel?
            double weight = w.pt2pl, retSqrNorm = ret.asEigen().squaredNorm();
            if (robustSqrtWeightFunc) weight *= robustSqrtWeightFunc(retSqrNorm);

            // Error and Jacobian:
            const Eigen::Vector3d err_i = ret.asEigen();
            errNormSqr += weight * weight * retSqrNorm;

            const Eigen::Matrix<double, 3, 6> Ji = J1.asEigen() * dDexpe_de.asEigen();
            g.noalias() += weight * Ji.transpose() * err_i;
            H.noalias() += weight * Ji.transpose() * Ji;
        }
#endif

        // Plane-to-plane (only direction of normal vectors):
        for (size_t idx_pl = 0; idx_pl < nPl2Pl; idx_pl++)
        {
            // Error term:
            const auto&                             p = in.paired_pl2pl[idx_pl];
            mrpt::math::CMatrixFixed<double, 3, 12> J1;
            mrpt::math::CVectorFixedDouble<3>       ret =
                mp2p_icp::error_plane2plane(p, result.optimalPose, J1);

            // Apply robust kernel?
            double weight = w.pl2pl, retSqrNorm = ret.asEigen().squaredNorm();
            if (robustSqrtWeightFunc) weight *= robustSqrtWeightFunc(retSqrNorm);

            const Eigen::Vector3d err_i = ret.asEigen();
            errNormSqr += weight * weight * retSqrNorm;

            const Eigen::Matrix<double, 3, 6> Ji = J1.asEigen() * dDexpe_de.asEigen();
            g.noalias() += weight * Ji.transpose() * err_i;
            H.noalias() += weight * Ji.transpose() * Ji;
        }

        // Prior guess term:
        if (gnParams.prior.has_value())
        {
            const auto& priorMean = gnParams.prior->mean;
            const auto& priorInf  = gnParams.prior->cov_inv;

            // Compute the residual pose error of these pair of nodes + its
            // constraint:
            // SE(3) error = inv(P_prior) * P_current
            //             = (P_current \ominus P_prior)

            const mrpt::poses::CPose3D P1invP2 = result.optimalPose - priorMean;
            const auto                 err_i   = mrpt::poses::Lie::SE<3>::log(P1invP2);

            mrpt::math::CMatrixDouble66 df_de2;

            mrpt::poses::Lie::SE<3>::jacob_dDinvP1invP2_de1e2(
                // edge between the two poses:in this case, both should coincide
                mrpt::poses::CPose3D::Identity(),
                // P1:
                priorMean,
                // P2:
                result.optimalPose,
                // df_de1
                std::nullopt,
                // df_de2
                df_de2);

            g.noalias() += (df_de2.transpose() * priorInf.asEigen()) * err_i.asEigen();

            H.noalias() += (df_de2.transpose() * priorInf.asEigen()) * df_de2.asEigen();
        }

        // Target error?
        const double errNorm = std::sqrt(errNormSqr);

        if (errNorm <= gnParams.maxCost) break;

        // 3) Solve Gauss-Newton:
        // g = J.transpose() * err;
        // H = J.transpose() * J;
        const Eigen::Matrix<double, 6, 1> delta = -H.ldlt().solve(g);

        // 4) add SE(3) increment:
        const auto dE = mrpt::poses::Lie::SE<3>::exp(mrpt::math::CVectorFixed<double, 6>(delta));

        result.optimalPose = result.optimalPose + dE;

        if (gnParams.verbose)
        {
            std::cout << "[P2P GN] iter:" << iter << " err:" << errNorm
                      << " delta:" << delta.transpose() << "\n";
        }

        // Simple convergence test:
        if (delta.norm() < gnParams.minDelta) break;

    }  // for each iteration

    return true;

    MRPT_END
}
