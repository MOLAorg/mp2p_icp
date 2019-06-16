/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   optimal_tf_gauss_newton.cpp
 * @brief  Simple non-linear optimizer to find the SE(3) optimal transformation
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */

#include <mp2p_icp/optimal_tf_gauss_newton.h>
#include <mrpt/poses/Lie/SE.h>
#include <Eigen/Dense>

using namespace mp2p_icp;

void mp2p_icp::optimal_tf_gauss_newton(
    const Pairings_GaussNewton& in, OptimalTF_Result& result)
{
    using std::size_t;

    MRPT_START

    // Run Gauss-Newton steps, using SE(3) relinearization at the current
    // solution:
    result.optimal_pose = in.initial_guess;

    const auto nPt2Pt = in.paired_points.size();
    const auto nPt2Pl = in.paired_pt2pl.size();

    const auto nErrorTerms = nPt2Pt * 3 + nPt2Pl;

    Eigen::VectorXd err(nErrorTerms);
    Eigen::MatrixXd J(nErrorTerms, 6);

    double w_pt = 1.0, w_pl = 50.0;

    for (size_t iter = 0; iter < in.max_iterations; iter++)
    {
        const auto dDexpe_de =
            mrpt::poses::Lie::SE<3>::jacob_dDexpe_de(result.optimal_pose);

        // Point-to-point:
        for (size_t idx_pt = 0; idx_pt < nPt2Pt; idx_pt++)
        {
            // Error:
            const auto&  p  = in.paired_points[idx_pt];
            const double lx = p.other_x, ly = p.other_y, lz = p.other_z;
            double       gx, gy, gz;
            result.optimal_pose.composePoint(lx, ly, lz, gx, gy, gz);
            err[idx_pt * 3 + 0] = gx - p.this_x;
            err[idx_pt * 3 + 1] = gy - p.this_y;
            err[idx_pt * 3 + 2] = gz - p.this_z;

            // Eval Jacobian:
            // clang-format off
            const Eigen::Matrix<double, 3, 12> J1 =
                (Eigen::Matrix<double, 3, 12>() <<
                   lx,  0,  0,  ly,  0,  0, lz,  0,  0,  1,  0,  0,
                    0, lx,  0,  0,  ly,  0,  0, lz,  0,  0,  1,  0,
                    0,  0, lx,  0,  0,  ly,  0,  0, lz,  0,  0,  1
                 ).finished();
            // clang-format on

            J.block<3, 6>(idx_pt * 3, 0) = w_pt * J1 * dDexpe_de.asEigen();
        }

        // Point-to-plane:
        for (size_t idx_pl = 0; idx_pl < nPt2Pl; idx_pl++)
        {
            // Error:
            const auto& p = in.paired_pt2pl[idx_pl];

            const double lx = p.pt_other.x, ly = p.pt_other.y,
                         lz = p.pt_other.z;
            mrpt::math::TPoint3D g;
            result.optimal_pose.composePoint(lx, ly, lz, g.x, g.y, g.z);

            err(idx_pl + nPt2Pt * 3) = p.pl_this.plane.evaluatePoint(g);

            // Eval Jacobian:
            // clang-format off
            const Eigen::Matrix<double, 3, 12> J1 =
                (Eigen::Matrix<double, 3, 12>() <<
                   lx,  0,  0,  ly,  0,  0, lz,  0,  0,  1,  0,  0,
                    0, lx,  0,  0,  ly,  0,  0, lz,  0,  0,  1,  0,
                    0,  0, lx,  0,  0,  ly,  0,  0, lz,  0,  0,  1
                 ).finished();
            // clang-format on

            const Eigen::Matrix<double, 1, 3> Jpl =
                (Eigen::Matrix<double, 1, 3>() << p.pl_this.plane.coefs[0],
                 p.pl_this.plane.coefs[1], p.pl_this.plane.coefs[2])
                    .finished();

            const Eigen::Matrix<double, 1, 6> Jb =
                Jpl * J1 * dDexpe_de.asEigen();

            J.block<1, 6>(idx_pl + nPt2Pt * 3, 0) = w_pl * Jb;
        }

        // 3) Solve Gauss-Newton:
        const Eigen::VectorXd             g = J.transpose() * err;
        const Eigen::Matrix<double, 6, 6> H = J.transpose() * J;
        const Eigen::Matrix<double, 6, 1> delta =
            -H.colPivHouseholderQr().solve(g);

        // 4) add SE(3) increment:
        const auto dE = mrpt::poses::Lie::SE<3>::exp(
            mrpt::math::CVectorFixed<double, 6>(delta));

        result.optimal_pose = result.optimal_pose + dE;

        //        std::cout << "[P2P GN] iter:" << iter << " err:" << err.norm()
        //                  << " delta:" << delta.transpose() << "\n";

        // Simple convergence test:
        if (delta.norm() < in.min_delta) break;

    }  // for each iteration
    MRPT_END
}
