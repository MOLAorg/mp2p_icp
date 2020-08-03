/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
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

    const auto nErrorTerms = (nPt2Pt + nPl2Pl) * 3 + nPt2Pl + nPt2Ln;

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
            const auto&  p  = in.paired_pt2pt[idx_pt];
            const double lx = p.other_x, ly = p.other_y, lz = p.other_z;
            double       gx, gy, gz;
            result.optimalPose.composePoint(lx, ly, lz, gx, gy, gz);
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
            J.block<3, 6>(idx_pt * 3, 0) = w.pt2pt * J1 * dDexpe_de.asEigen();
        }

        // Point-to-line
        auto base_idx = nPt2Pt * 3;
        for (size_t idx_pt = 0; idx_pt < nPt2Ln; idx_pt++)
        {
            // Error
            const auto&  p  = in.paired_pt2ln[idx_pt];
            const double lx = p.pt_other.x, ly = p.pt_other.y,
                         lz = p.pt_other.z;
            double gx, gy, gz;
            result.optimalPose.composePoint(lx, ly, lz, gx, gy, gz);
            const auto g           = mrpt::math::TPoint3D(gx, gy, gz);
            err(base_idx + idx_pt) = pow(p.ln_this.distance(g), 2);

            // Eval Jacobian:
            // "A tutorial on SE(3) transformation parameterizations and
            // on-manifold optimization"
            // d(T_{A}·p)/dT_{A}. Ec.7.16
            // clang-format off
            // Doc auxiliar: Section 4.1.2.
            // p_r0 = (p-r_{0,r}). Ec.9
            const Eigen::Matrix<double, 1, 3> p_r0 =
           (Eigen::Matrix<double, 1, 3>() << g.x-p.ln_this.pBase.x, g.y-p.ln_this.pBase.y, g.z-p.ln_this.pBase.z
            ).finished();
            // Module of vector director of line
            const Eigen::Matrix<double, 1, 3> ru =
           (Eigen::Matrix<double, 1, 3>() << p.ln_this.director[0], p.ln_this.director[1], p.ln_this.director[2]
            ).finished();
            double mod_ru = ru*ru.transpose();

            // J1
            Eigen::Matrix<double, 1, 3> J1 = 2*p_r0-(2/mod_ru)*(p_r0*ru.transpose())*ru;
            // J2
            const Eigen::Matrix<double, 3, 12> J2 =
                (Eigen::Matrix<double, 3, 12>() <<
                   lx,  0,  0,  ly,  0,  0, lz,  0,  0,  1,  0,  0,
                    0, lx,  0,  0,  ly,  0,  0, lz,  0,  0,  1,  0,
                    0,  0, lx,  0,  0,  ly,  0,  0, lz,  0,  0,  1
                 ).finished();
            // clang-format on

            // Get weight
            // ...

            // Build Jacobian
            J.block<1, 6>(base_idx + idx_pt, 0) =
                w.pt2ln * J1 * J2 * dDexpe_de.asEigen();
        }

        // Line-to-Line
        base_idx = base_idx + nPt2Ln;
        // Minimum angle to approach zero
        const double tolerance = 0.01;
        for (size_t idx_ln = 0; idx_ln < nLn2Ln; idx_ln++)
        {
            const auto&         p = in.paired_ln2ln[idx_ln];
            mrpt::math::TLine3D ln_aux;
            double              gx, gy, gz;
            result.optimalPose.composePoint(
                p.ln_other.pBase.x, p.ln_other.pBase.y, p.ln_other.pBase.z, gx,
                gy, gz);
            ln_aux.pBase = mrpt::math::TPoint3D(gx, gy, gz);
            // Homogeneous matrix calculation
            mrpt::math::CMatrixDouble44 aux;
            result.optimalPose.getHomogeneousMatrix(aux);
            const Eigen::Matrix<double, 4, 4> T = aux.asEigen();
            // Projection of the director vector for the new pose
            const Eigen::Matrix<double, 1, 4> U =
                (Eigen::Matrix<double, 1, 4>() << p.ln_other.director[0],
                 p.ln_other.director[1], p.ln_other.director[2], 1)
                    .finished();
            const Eigen::Matrix<double, 1, 4> U_T = U * T;
            ln_aux.director = {U_T(1, 1), U_T(1, 2), U_T(1, 3)};
            // Angle formed between the lines
            double alfa = getAngle(p.ln_this, ln_aux);
            // p_r0 = (p-r_{0,r}). Ec.20
            const Eigen::Matrix<double, 1, 3> p_r2 =
                (Eigen::Matrix<double, 1, 3>()
                     << ln_aux.pBase.x - p.ln_this.pBase.x,
                 ln_aux.pBase.y - p.ln_this.pBase.y,
                 ln_aux.pBase.z - p.ln_this.pBase.z)
                    .finished();
            const Eigen::Matrix<double, 1, 3> rv =
                (Eigen::Matrix<double, 1, 3>() << p.ln_this.director[0],
                 p.ln_this.director[1], p.ln_this.director[2])
                    .finished();

            // Relationship between lines
            if (abs(alfa) < tolerance)
            {  // Parallel
                // Error: Ec.20
                err(base_idx + idx_ln) =
                    pow(p.ln_this.distance(ln_aux.pBase), 2);

                // Module of vector director of line
                double mod_rv = rv * rv.transpose();

                // J1: Ec.22
                Eigen::Matrix<double, 1, 3> J1 =
                    2 * p_r2 - (2 / mod_rv) * (p_r2 * rv.transpose()) * rv;
                // J2: Ec.23
                const Eigen::Matrix<double, 3, 12> J2 =
                    (Eigen::Matrix<double, 3, 12>() << p.ln_other.pBase.x, 0, 0,
                     p.ln_other.pBase.y, 0, 0, p.ln_other.pBase.z, 0, 0, 1, 0,
                     0, 0, p.ln_other.pBase.x, 0, 0, p.ln_other.pBase.y, 0, 0,
                     p.ln_other.pBase.z, 0, 0, 1, 0, 0, 0, p.ln_other.pBase.x,
                     0, 0, p.ln_other.pBase.y, 0, 0, p.ln_other.pBase.z, 0, 0,
                     1)
                        .finished();
                // Build Jacobian
                J.block<1, 6>(base_idx + idx_ln, 0) =
                    w.ln2ln * J1 * J2 * dDexpe_de.asEigen();
            }
            else
            {  // Rest
                // Error:
                // Cross product (r_u x r_2,v)
                const double rw_x = U_T[1] * p.ln_this.director[2] -
                                    U_T[2] * p.ln_this.director[1];
                const double rw_y =
                    -(U_T[0] * p.ln_this.director[2] -
                      U_T[2] * p.ln_this.director[0]);
                const double rw_z = U_T[0] * p.ln_this.director[1] -
                                    U_T[1] * p.ln_this.director[0];
                const Eigen::Matrix<double, 1, 3> r_w =
                    (Eigen::Matrix<double, 1, 3>() << rw_x, rw_y, rw_z)
                        .finished();
                double aux_rw = r_w * r_w.transpose();
                // Error 1. Ec.26
                err(base_idx + idx_ln) = p_r2.dot(r_w) / sqrt(aux_rw);
                // Error 2. Ec.27
                err(base_idx + idx_ln + 1) = U_T[0] - p.ln_this.director[0];
                err(base_idx + idx_ln + 2) = U_T[1] - p.ln_this.director[1];
                err(base_idx + idx_ln + 3) = U_T[2] - p.ln_this.director[2];
                // Desplazamiento del indicador del vector de error para los
                // casos en el que hay 4 errores en lugar de 1. Espero que esto
                // se pueda hacer.
                base_idx = base_idx + 3;

                // Ec.35
                const Eigen::Matrix<double, 1, 3> I =
                    (Eigen::Matrix<double, 1, 3>() << 1, 1, 1).finished();
                Eigen::Matrix<double, 1, 3> C = I.cross(rv);
                // J1.1: Ec.32
                Eigen::Matrix<double, 1, 3> J1_1 = r_w / sqrt(aux_rw);
                // J1.2: Ec.36
                Eigen::Matrix<double, 1, 3> J1_2 =
                    (p_r2.cross(rv) * sqrt(aux_rw) -
                     p_r2 * r_w.transpose() * C) /
                    aux_rw;
                // J1.3: Ec.37-38
                Eigen::Matrix<double, 3, 6> J1_3 =
                    (Eigen::Matrix<double, 3, 6>() << 0, 0, 0, 1, 0, 0, 0, 0, 0,
                     0, 1, 0, 0, 0, 0, 0, 0, 1)
                        .finished();
                // J1: Ec.29
                Eigen::Matrix<double, 4, 6> J1;
                J1.block<1, 3>(0, 0) = J1_1;
                J1.block<1, 3>(3, 5) = J1_2;
                J1.block<3, 6>(0, 1) = J1_3;

                // J2: Ec.39-41
                const Eigen::Matrix<double, 6, 12> J2 =
                    (Eigen::Matrix<double, 6, 12>() << p.ln_other.pBase.x, 0, 0,
                     p.ln_other.pBase.y, 0, 0, p.ln_other.pBase.z, 0, 0, 1, 0,
                     0, 0, p.ln_other.pBase.x, 0, 0, p.ln_other.pBase.y, 0, 0,
                     p.ln_other.pBase.z, 0, 0, 1, 0, 0, 0, p.ln_other.pBase.x,
                     0, 0, p.ln_other.pBase.y, 0, 0, p.ln_other.pBase.z, 0, 0,
                     1, p.ln_other.director[0], 0, 0, p.ln_other.director[1], 0,
                     0, p.ln_other.director[2], 0, 0, 1, 0, 0, 0,
                     p.ln_other.director[0], 0, 0, p.ln_other.director[1], 0, 0,
                     p.ln_other.director[2], 0, 0, 1, 0, 0, 0,
                     p.ln_other.director[0], 0, 0, p.ln_other.director[1], 0, 0,
                     p.ln_other.director[2], 0, 0, 1)
                        .finished();
                // Build Jacobian
                J.block<4, 6>(base_idx + idx_ln, 0) =
                    w.ln2ln * J1 * J2 * dDexpe_de.asEigen();
            }
        }
        // Point-to-plane:
        base_idx = base_idx + nLn2Ln;
        for (size_t idx_pl = 0; idx_pl < nPt2Pl; idx_pl++)
        {
            // Error:
            const auto& p = in.paired_pt2pl[idx_pl];

            const double lx = p.pt_other.x, ly = p.pt_other.y,
                         lz = p.pt_other.z;
            mrpt::math::TPoint3D g;
            result.optimalPose.composePoint(lx, ly, lz, g.x, g.y, g.z);

            err(idx_pl + base_idx) = p.pl_this.plane.evaluatePoint(g);

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

            J.block<1, 6>(idx_pl + base_idx, 0) = w.pt2pl * Jb;
        }

        // Plane-to-plane (only direction of normal vectors):
        base_idx += nPt2Pl * 1;
        for (size_t idx_pl = 0; idx_pl < nPl2Pl; idx_pl++)
        {
            // Error term:
            const auto& p = in.paired_pl2pl[idx_pl];

            const auto nl = p.p_other.plane.getNormalVector();
            const auto ng = p.p_this.plane.getNormalVector();

            const auto p_oplus_nl = result.optimalPose.rotateVector(nl);

            for (int i = 0; i < 3; i++)
                err(i + idx_pl * 3 + base_idx) = ng[i] - p_oplus_nl[i];

            // Eval Jacobian:

            // df_oplus(A,p)/d_A. Section 7.3.2 tech. report:
            // "A tutorial on SE(3) transformation parameterizations and
            // on-manifold optimization"
            // Modified, to discard the last I_3 block, since this particular
            // cost function is insensible to translations.

            // clang-format off
            const Eigen::Matrix<double, 3, 12> J1 =
                (Eigen::Matrix<double, 3, 12>() <<
                   nl.x,  0,  0,  nl.y,  0,  0, nl.z,  0,  0,  0,  0,  0,
                    0, nl.x,  0,  0,  nl.y,  0,  0, nl.z,  0,  0,  0,  0,
                    0,  0, nl.x,  0,  0,  nl.y,  0,  0, nl.z,  0,  0,  0
                 ).finished();
            // clang-format on

            J.block<3, 6>(3 * idx_pl + base_idx, 0) =
                w.pl2pl * J1 * dDexpe_de.asEigen();
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
