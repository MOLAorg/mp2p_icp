/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   errorTerms.cpp
 * @brief
 * @author Francisco José Mañas Álvarez, Jose Luis Blanco Claraco
 * @date   Aug 4, 2020
 */

#include <mp2p_icp/errorTerms.h>
#include <mp2p_icp/optimal_tf_gauss_newton.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_containers.h>  // dotProduct()
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SE.h>

#include <Eigen/Dense>
#include <iostream>

using namespace mp2p_icp;
using namespace mrpt::math;

mrpt::math::CVectorFixedDouble<3> mp2p_icp::error_point2point(
    const mrpt::tfest::TMatchingPair& pairing, const mrpt::poses::CPose3D& relativePose,
    mrpt::optional_ref<mrpt::math::CMatrixFixed<double, 3, 12>> jacobian)
{
    MRPT_START
    mrpt::math::CVectorFixedDouble<3> error;
    const mrpt::math::TPoint3D&       l = pairing.local;

    const mrpt::math::TPoint3D g = relativePose.composePoint(l);

    error[0] = g.x - pairing.global.x;
    error[1] = g.y - pairing.global.y;
    error[2] = g.z - pairing.global.z;

    // It's possible change the error to scalar with the function
    // g.DistanceTo(l) Eval Jacobian:
    if (jacobian)
    {
        mrpt::math::CMatrixFixed<double, 3, 12>& J_aux = jacobian.value().get();
        // clang-format off
        J_aux = (Eigen::Matrix<double, 3, 12>() <<
                 l.x,   0,   0, l.y,   0,    0, l.z,   0,   0,  1,  0,  0,
                   0, l.x,   0,   0, l.y,    0,   0, l.z,   0,  0,  1,  0,
                   0,   0, l.x,   0,   0,  l.y,   0,   0, l.z,  0,  0,  1
                 ).finished();
        // clang-format on
    }

    return error;
    MRPT_END
}

mrpt::math::CVectorFixedDouble<3> mp2p_icp::error_point2line(
    const mp2p_icp::point_line_pair_t& pairing, const mrpt::poses::CPose3D& relativePose,
    mrpt::optional_ref<mrpt::math::CMatrixFixed<double, 3, 12>> jacobian)
{
    MRPT_START
    mrpt::math::CVectorFixedDouble<3> error;
    const auto&                       lnGlob = pairing.ln_global;

    // local and global point:
    const mrpt::math::TPoint3D l = pairing.pt_local;
    const mrpt::math::TPoint3D g = relativePose.composePoint(l);

    // Module of vector director of line
    const auto& u = lnGlob.director;
    const auto  q = (g - lnGlob.pBase);

    const auto uq = mrpt::math::dotProduct<3, double>(u, q);

    error[0] = q[0] - u.x * uq;
    error[1] = q[1] - u.y * uq;
    error[2] = q[2] - u.z * uq;

    if (jacobian)
    {
        // J1
        const Eigen::Matrix<double, 3, 3> J1 =
            (Eigen::Matrix<double, 3, 3>() << 1 - mrpt::square(u.x), -u.x * u.y, -u.x * u.z,
             -u.y * u.x, 1 - mrpt::square(u.y), -u.y * u.z, -u.z * u.x, -u.z * u.y,
             1 - mrpt::square(u.z))
                .finished();
        // J2
        // clang-format off
        Eigen::Matrix<double, 3, 12> J2 =
            (Eigen::Matrix<double, 3, 12>() <<
             l.x,   0,   0, l.y,   0,    0, l.z,   0,   0,  1,  0,  0,
               0, l.x,   0,   0, l.y,    0,   0, l.z,   0,  0,  1,  0,
               0,   0, l.x,   0,   0,  l.y,   0,   0, l.z,  0,  0,  1
             ).finished();
        // clang-format on
        mrpt::math::CMatrixFixed<double, 3, 12>& J_aux = jacobian.value().get();

        J_aux = J1 * J2;
    }
    return error;
    MRPT_END
}

mrpt::math::CVectorFixedDouble<3> mp2p_icp::error_point2plane(
    const mp2p_icp::point_plane_pair_t& pairing, const mrpt::poses::CPose3D& relativePose,
    mrpt::optional_ref<mrpt::math::CMatrixFixed<double, 3, 12>> jacobian)
{
    MRPT_START
    mrpt::math::CVectorFixedDouble<3> error;
    const auto&                       p      = pairing.pt_local;
    const auto&                       pl_aux = pairing.pl_global.plane;
    const mrpt::math::TPoint3D        l      = TPoint3D(p.x, p.y, p.z);
    mrpt::math::TPoint3D              g;
    relativePose.composePoint(l, g);
    double mod_n = mrpt::square(pl_aux.coefs[0]) + mrpt::square(pl_aux.coefs[1]) +
                   mrpt::square(pl_aux.coefs[2]);

    error[0] = -(pl_aux.coefs[0] / mod_n) * (pl_aux.coefs[0] * g.x + pl_aux.coefs[1] * g.y +
                                             pl_aux.coefs[2] * g.z + pl_aux.coefs[3]);
    error[1] = -(pl_aux.coefs[1] / mod_n) * (pl_aux.coefs[0] * g.x + pl_aux.coefs[1] * g.y +
                                             pl_aux.coefs[2] * g.z + pl_aux.coefs[3]);
    error[2] = -(pl_aux.coefs[2] / mod_n) * (pl_aux.coefs[0] * g.x + pl_aux.coefs[1] * g.y +
                                             pl_aux.coefs[2] * g.z + pl_aux.coefs[3]);
    if (jacobian)
    {
        // Eval Jacobian:
        // J1
        const Eigen::Matrix<double, 3, 3> J1 =
            (Eigen::Matrix<double, 3, 3>() << -mrpt::square(pl_aux.coefs[0]) / mod_n,
             -pl_aux.coefs[0] * pl_aux.coefs[1] / mod_n, -pl_aux.coefs[0] * pl_aux.coefs[2] / mod_n,
             -pl_aux.coefs[1] * pl_aux.coefs[0] / mod_n, -mrpt::square(pl_aux.coefs[1]) / mod_n,
             -pl_aux.coefs[1] * pl_aux.coefs[2] / mod_n, -pl_aux.coefs[2] * pl_aux.coefs[0] / mod_n,
             -pl_aux.coefs[2] * pl_aux.coefs[1] / mod_n, -mrpt::square(pl_aux.coefs[2]) / mod_n)
                .finished();
        // J2
        // clang-format off
        const Eigen::Matrix<double, 3, 12> J2 =
            (Eigen::Matrix<double, 3, 12>() <<
             l.x,   0,   0, l.y,   0,    0, l.z,   0,   0,  1,  0,  0,
               0, l.x,   0,   0, l.y,    0,   0, l.z,   0,  0,  1,  0,
               0,   0, l.x,   0,   0,  l.y,   0,   0, l.z,  0,  0,  1
             ).finished();
        // clang-format on

        mrpt::math::CMatrixFixed<double, 3, 12>& J_aux = jacobian.value().get();
        J_aux                                          = J1 * J2;
    }
    return error;
    MRPT_END
}

mrpt::math::CVectorFixedDouble<4> mp2p_icp::error_line2line(
    const mp2p_icp::matched_line_t& pairing, const mrpt::poses::CPose3D& relativePose,
    mrpt::optional_ref<mrpt::math::CMatrixFixed<double, 4, 12>> jacobian)
{
    MRPT_START
    mrpt::math::CVectorFixedDouble<4> error;
    mrpt::math::TLine3D               ln_aux;
    mrpt::math::TPoint3D              g;

    const auto& p0 = pairing.ln_local.pBase;
    const auto& u0 = pairing.ln_local.director;
    const auto& p1 = pairing.ln_global.pBase;
    const auto& u1 = pairing.ln_global.director;

    relativePose.composePoint(p0, g);
    ln_aux.pBase = mrpt::math::TPoint3D(g);

    // Homogeneous matrix calculation
    mrpt::math::CMatrixDouble44 aux;
    relativePose.getHomogeneousMatrix(aux);
    const Eigen::Matrix<double, 4, 4> T = aux.asEigen();

    // Projection of the director vector for the new pose
    const Eigen::Matrix<double, 1, 4> U =
        (Eigen::Matrix<double, 1, 4>() << u0[0], u0[1], u0[2], 1).finished();
    const Eigen::Matrix<double, 1, 4> U_T = U * T;
    ln_aux.director                       = {U_T[0], U_T[1], U_T[2]};

    // Angle formed between the lines
    double alfa = getAngle(pairing.ln_global, ln_aux) * 180 / (2 * 3.14159265);
    /*
        std::cout << "\nLine 1:\n"
                  <<  pairing.ln_this << "\nLine 2:\n"
                   << pairing.ln_other << "\nLine 2':\n"
                   << ln_aux << "\nAngle:\n"
                   << alfa << "\nT:\n"
                   <<  T << "\n";
    */

    // p_r0 = (p-r_{0,r}). Ec.20
    const Eigen::Matrix<double, 1, 3> p_r2 =
        (Eigen::Matrix<double, 1, 3>() << ln_aux.pBase.x - p1.x, ln_aux.pBase.y - p1.y,
         ln_aux.pBase.z - p1.z)
            .finished();

    const Eigen::Matrix<double, 1, 3> rv =
        (Eigen::Matrix<double, 1, 3>() << u1[0], u1[1], u1[2]).finished();

    // Relationship between lines
    const double tolerance = 0.01;
    if (abs(alfa) < tolerance)
    {  // Parallel
        // Error: Ec.20
        error[0] = mrpt::square(pairing.ln_global.distance(ln_aux.pBase));
        if (jacobian)
        {
            // Module of vector director of line
            double mod_rv = rv * rv.transpose();

            // J1: Ec.22
            Eigen::Matrix<double, 1, 3> J1 = 2 * p_r2 - (2 / mod_rv) * (p_r2 * rv.transpose()) * rv;
            // J2: Ec.23
            // clang-format off
            const Eigen::Matrix<double, 3, 12> J2 =
                (Eigen::Matrix<double, 3, 12>() <<
                 p0.x,    0,    0, p0.y,    0,    0, p0.z,    0,    0, 1, 0, 0,
                    0, p0.x,    0,    0, p0.y,    0,    0, p0.z,    0, 0, 1, 0,
                    0,    0, p0.x,    0,    0, p0.y,    0,    0, p0.z, 0, 0, 1
                 ).finished();
            // clang-format on
            // Build Jacobian
            mrpt::math::CMatrixFixed<double, 4, 12>& J_auxp = jacobian.value().get();
            J_auxp.block<1, 12>(0, 0)                       = J1 * J2;
        }
    }
    else
    {  // Rest
        // Error:
        // Cross product (r_u x r_2,v)
        const double rw_x = U_T[1] * u1[2] - U_T[2] * u1[1];
        const double rw_y = -(U_T[0] * u1[2] - U_T[2] * u1[0]);
        const double rw_z = U_T[0] * u1[1] - U_T[1] * u1[0];

        const Eigen::Matrix<double, 1, 3> r_w =
            (Eigen::Matrix<double, 1, 3>() << rw_x, rw_y, rw_z).finished();
        double aux_rw = r_w * r_w.transpose();
        // Error 1. Ec.26
        error[0] = p_r2.dot(r_w) / sqrt(aux_rw);
        // Error 2. Ec.27
        error[1] = U_T[0] - u1[0];
        error[2] = U_T[1] - u1[1];
        error[3] = U_T[2] - u1[2];
        if (jacobian)
        {
            // J1.1: Ec.32
            Eigen::Matrix<double, 1, 3> J1_1 = r_w / sqrt(aux_rw);

            // J1.2:
            // A
            const double A  = p_r2[0] * r_w[0] + p_r2[1] * r_w[1] + p_r2[2] * r_w[2];
            const double Ax = -u1[2] * p_r2[1] + u1[1] * p_r2[2];
            const double Ay = u1[2] * p_r2[0] - u1[0] * p_r2[2];
            const double Az = -u1[1] * p_r2[0] + u1[0] * p_r2[1];
            // B
            const double B  = sqrt(aux_rw);
            const double Bx = (-u1[2] * r_w[1] + u1[1] * r_w[2]) / B;
            const double By = (u1[2] * r_w[0] + u1[0] * r_w[2]) / B;
            const double Bz = (-u1[1] * r_w[0] + u1[0] * r_w[1]) / B;

#if 0
            std::cout << "\nA: " << A << "\nAx: " << Ax << "\nAy: " << Ay
                      << "\nAz: " << Az << "\nB: " << B << "\nBx: " << Bx
                      << "\nBy: " << By << "\nBz: " << Bz << "\n";
#endif

            // Ec.36
            // clang-format off
            Eigen::Matrix<double, 1, 3> J1_2 =
                (Eigen::Matrix<double, 1, 3>() <<
                 (Ax * B - A * Bx) / mrpt::square(B),
                 (Ay * B - A * By) / mrpt::square(B),
                 (Az * B - A * Bz) / mrpt::square(B)
                 ).finished();
            // clang-format on

            // J1.3: Ec.37-38
            // clang-format off
            Eigen::Matrix<double, 3, 6> J1_3 =
                (Eigen::Matrix<double, 3, 6>() <<
                 0, 0, 0, 1, 0, 0,
                 0, 0, 0, 0, 1, 0,
                 0, 0, 0, 0, 0, 1
                 ).finished();
            // clang-format on

            // J1: Ec.29
            Eigen::Matrix<double, 4, 6> J1;
            J1.block<1, 3>(0, 0) = J1_1;
            J1.block<1, 3>(0, 3) = J1_2;
            J1.block<3, 6>(1, 0) = J1_3;

            // J2: Ec.39-41
            // clang-format off
            const Eigen::Matrix<double, 6, 12> J2 =
                (Eigen::Matrix<double, 6, 12>() <<
                 p0.x,     0,     0,  p0.y,     0,     0,  p0.z,     0,     0, 1, 0, 0,
                    0,  p0.x,     0,     0,  p0.y,     0,     0,  p0.z,     0, 0, 1, 0,
                    0,     0,  p0.x,     0,     0,  p0.y,     0,     0,  p0.z, 0, 0, 1,
                -u0[0],     0,     0, -u0[1],     0,     0, -u0[2],     0,     0, 0, 0, 0,
                    0, -u0[0],     0,     0, -u0[1],     0,     0, -u0[2],     0, 0, 0, 0,
                    0,     0, -u0[0],     0,     0, -u0[1],     0,     0, -u0[2], 0, 0, 0
                ).finished();
            // clang-format on
            // Build Jacobian
            mrpt::math::CMatrixFixed<double, 4, 12>& J_aux = jacobian.value().get();
            J_aux.block<4, 12>(0, 0)                       = J1 * J2;

            std::cout << "\nJ1:\n" << J1 << "\nJ2:\n" << J2 << "\n";
        }
    }
    //    std::cout<<"\nError:\n"<<error;
    return error;
    MRPT_END
}

mrpt::math::CVectorFixedDouble<3> mp2p_icp::error_plane2plane(
    const mp2p_icp::matched_plane_t& pairing, const mrpt::poses::CPose3D& relativePose,
    mrpt::optional_ref<mrpt::math::CMatrixFixed<double, 3, 12>> jacobian)
{
    MRPT_START
    mrpt::math::CVectorFixedDouble<3> error;

    const auto nl = pairing.p_local.plane.getNormalVector();
    const auto ng = pairing.p_global.plane.getNormalVector();

    const auto p_oplus_nl = relativePose.rotateVector(nl);

    for (int i = 0; i < 3; i++) error[i] = p_oplus_nl[i] - ng[i];

    if (jacobian)
    {
        // Eval Jacobian:

        // df_oplus(A,p)/d_A. Section 7.3.2 tech. report:
        // "A tutorial on SE(3) transformation parameterizations and
        // on-manifold optimization"
        // Modified, to discard the last I_3 block, since this particular
        // cost function is insensible to translations.

        // clang-format off
        mrpt::math::CMatrixFixed<double, 3, 12>& J_aux = jacobian.value().get();
        J_aux = (Eigen::Matrix<double, 3, 12>() <<
                 nl.x,    0,    0, nl.y,    0,    0, nl.z,    0,    0,  0,  0,  0,
                    0, nl.x,    0,    0, nl.y,    0,    0, nl.z,    0,  0,  0,  0,
                    0,    0, nl.x,    0,    0, nl.y,    0,    0, nl.z,  0,  0,  0
                ).finished();
        // clang-format on
    }
    return error;
    MRPT_END
}
