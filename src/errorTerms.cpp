/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
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
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/TPoint3D.h>
#include <Eigen/Dense>
#include <iostream>

using namespace mp2p_icp;
using namespace mrpt::math;

mrpt::math::CVectorFixedDouble<3> error_point2point(
    mrpt::tfest::TMatchingPair& pairing, const mrpt::poses::CPose3D &relativePose,
    mrpt::optional_ref<mrpt::math::CMatrixFixed<double, 3, 12>> jacobian)
{
    mrpt::math::CVectorFixedDouble<3> error;
    const mrpt::math::TPoint3D l = TPoint3D(pairing.other_x,pairing.other_y,pairing.other_z);
    mrpt::math::TPoint3D g;

    relativePose.composePoint(l,g);

    error[0] = g.x - pairing.this_x;
    error[1] = g.y - pairing.this_y;
    error[2] = g.z - pairing.this_z;

    // It's possible change the error to scalar with the function g.DistanceTo(l)
    // Eval Jacobian:
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
}

mrpt::math::CVectorFixedDouble<1> error_point2line(
    const mp2p_icp::point_line_pair_t& pairing, const mrpt::poses::CPose3D &relativePose,
    Eigen::Matrix<double, 1, 12> jacobian)
{
    mrpt::math::CVectorFixedDouble<1> error;
    const mrpt::math::TPoint3D l = TPoint3D(pairing.pt_other.x,pairing.pt_other.y,pairing.pt_other.z);
    mrpt::math::TPoint3D g;
    relativePose.composePoint(l,g);

    error[1] = pow(pairing.ln_this.distance(g), 2);

    // Eval Jacobian:
    // "A tutorial on SE(3) transformation parameterizations and
    // on-manifold optimization"
    // d(T_{A}·p)/dT_{A}. Ec.7.16
    // Doc auxiliar: Section 4.1.2.
    // p_r0 = (p-r_{0,r}). Ec.9
    const Eigen::Matrix<double, 1, 3> p_r0 =
        (Eigen::Matrix<double, 1, 3>() << g.x - pairing.ln_this.pBase.x,
         g.y - pairing.ln_this.pBase.y, g.z - pairing.ln_this.pBase.z)
            .finished();
    // Module of vector director of line
    const Eigen::Matrix<double, 1, 3> ru =
        (Eigen::Matrix<double, 1, 3>() << pairing.ln_this.director[0],
         pairing.ln_this.director[1], pairing.ln_this.director[2])
            .finished();
    double mod_ru = ru * ru.transpose();

    // J1
    Eigen::Matrix<double, 1, 3> J1 =
        2 * p_r0 - (2 / mod_ru) * (p_r0 * ru.transpose()) * ru;
    // J2
    // clang-format off
    Eigen::Matrix<double, 3, 12> J2 =
        (Eigen::Matrix<double, 3, 12>() <<
         l.x,   0,   0, l.y,   0,    0, l.z,   0,   0,  1,  0,  0,
           0, l.x,   0,   0, l.y,    0,   0, l.z,   0,  0,  1,  0,
           0,   0, l.x,   0,   0,  l.y,   0,   0, l.z,  0,  0,  1
         ).finished();
    // clang-format on
    jacobian = J1 * J2;

    return error;
}

mrpt::math::CVectorFixedDouble<1> error_point2plane(
    const mp2p_icp::point_plane_pair_t& pairing, const mrpt::poses::CPose3D &relativePose,
    Eigen::Matrix<double, 1, 12> jacobian)
{
    mrpt::math::CVectorFixedDouble<1> error;
    const mrpt::math::TPoint3D l = TPoint3D(pairing.pt_other.x,pairing.pt_other.y,pairing.pt_other.z);
    mrpt::math::TPoint3D g;
    relativePose.composePoint(l,g);

    error[0] = pairing.pl_this.plane.evaluatePoint(g);

    // Eval Jacobian:
    // clang-format off
    const Eigen::Matrix<double, 3, 12> J1 =
        (Eigen::Matrix<double, 3, 12>() <<
         l.x,   0,   0, l.y,   0,    0, l.z,   0,   0,  1,  0,  0,
           0, l.x,   0,   0, l.y,    0,   0, l.z,   0,  0,  1,  0,
           0,   0, l.x,   0,   0,  l.y,   0,   0, l.z,  0,  0,  1
         ).finished();
    // clang-format on

    const Eigen::Matrix<double, 1, 3> Jpl =
        (Eigen::Matrix<double, 1, 3>() << pairing.pl_this.plane.coefs[0],
         pairing.pl_this.plane.coefs[1], pairing.pl_this.plane.coefs[2])
            .finished();

    jacobian = Jpl * J1;

    return error;
}

mrpt::math::CVectorFixedDouble<4> error_line2line(
    const mp2p_icp::matched_line_t& pairing, const mrpt::poses::CPose3D &relativePose,
    Eigen::Matrix<double, Eigen::Dynamic, 12>& jacobian, bool jump = false)
{
    mrpt::math::CVectorFixedDouble<4> error;
    mrpt::math::TLine3D               ln_aux;
    double                            gx, gy, gz;
    relativePose.composePoint(
        pairing.ln_other.pBase.x, pairing.ln_other.pBase.y,
        pairing.ln_other.pBase.z, gx, gy, gz);
    ln_aux.pBase = mrpt::math::TPoint3D(gx, gy, gz);
    // Homogeneous matrix calculation
    mrpt::math::CMatrixDouble44 aux;
    relativePose.getHomogeneousMatrix(aux);
    const Eigen::Matrix<double, 4, 4> T = aux.asEigen();
    // Projection of the director vector for the new pose
    const Eigen::Matrix<double, 1, 4> U =
        (Eigen::Matrix<double, 1, 4>() << pairing.ln_other.director[0],
         pairing.ln_other.director[1], pairing.ln_other.director[2], 1)
            .finished();
    const Eigen::Matrix<double, 1, 4> U_T = U * T;
    ln_aux.director                       = {U_T(1, 1), U_T(1, 2), U_T(1, 3)};
    // Angle formed between the lines
    double alfa = getAngle(pairing.ln_this, ln_aux);
    // p_r0 = (p-r_{0,r}). Ec.20
    const Eigen::Matrix<double, 1, 3> p_r2 =
        (Eigen::Matrix<double, 1, 3>()
             << ln_aux.pBase.x - pairing.ln_this.pBase.x,
         ln_aux.pBase.y - pairing.ln_this.pBase.y,
         ln_aux.pBase.z - pairing.ln_this.pBase.z)
            .finished();
    const Eigen::Matrix<double, 1, 3> rv =
        (Eigen::Matrix<double, 1, 3>() << pairing.ln_this.director[0],
         pairing.ln_this.director[1], pairing.ln_this.director[2])
            .finished();

    // Relationship between lines
    const double tolerance = 0.01;
    if (abs(alfa) < tolerance)
    {  // Parallel
        // Error: Ec.20
        error[0] = pow(pairing.ln_this.distance(ln_aux.pBase), 2);

        // Module of vector director of line
        double mod_rv = rv * rv.transpose();

        // J1: Ec.22
        Eigen::Matrix<double, 1, 3> J1 =
            2 * p_r2 - (2 / mod_rv) * (p_r2 * rv.transpose()) * rv;
        // J2: Ec.23
        const Eigen::Matrix<double, 3, 12> J2 =
            (Eigen::Matrix<double, 3, 12>() << pairing.ln_other.pBase.x, 0, 0,
             pairing.ln_other.pBase.y, 0, 0, pairing.ln_other.pBase.z, 0, 0, 1,
             0, 0, 0, pairing.ln_other.pBase.x, 0, 0, pairing.ln_other.pBase.y,
             0, 0, pairing.ln_other.pBase.z, 0, 0, 1, 0, 0, 0,
             pairing.ln_other.pBase.x, 0, 0, pairing.ln_other.pBase.y, 0, 0,
             pairing.ln_other.pBase.z, 0, 0, 1)
                .finished();
        // Build Jacobian
        jacobian = J1 * J2;
    }
    else
    {  // Rest
        // Error:
        // Cross product (r_u x r_2,v)
        const double rw_x = U_T[1] * pairing.ln_this.director[2] -
                            U_T[2] * pairing.ln_this.director[1];
        const double rw_y =
            -(U_T[0] * pairing.ln_this.director[2] -
              U_T[2] * pairing.ln_this.director[0]);
        const double rw_z = U_T[0] * pairing.ln_this.director[1] -
                            U_T[1] * pairing.ln_this.director[0];
        const Eigen::Matrix<double, 1, 3> r_w =
            (Eigen::Matrix<double, 1, 3>() << rw_x, rw_y, rw_z).finished();
        double aux_rw = r_w * r_w.transpose();
        // Error 1. Ec.26
        error[0] = p_r2.dot(r_w) / sqrt(aux_rw);
        // Error 2. Ec.27
        error[1] = U_T[0] - pairing.ln_this.director[0];
        error[2] = U_T[1] - pairing.ln_this.director[1];
        error[3] = U_T[2] - pairing.ln_this.director[2];
        // Desplazamiento del indicador del vector de error para los
        // casos en el que hay 4 errores en lugar de 1. Espero que esto
        // se pueda hacer.
        jump = true;

        // Ec.35
        const Eigen::Matrix<double, 1, 3> I =
            (Eigen::Matrix<double, 1, 3>() << 1, 1, 1).finished();
        Eigen::Matrix<double, 1, 3> C = I.cross(rv);
        // J1.1: Ec.32
        Eigen::Matrix<double, 1, 3> J1_1 = r_w / sqrt(aux_rw);
        // J1.2: Ec.36
        Eigen::Matrix<double, 1, 3> J1_2 =
            (p_r2.cross(rv) * sqrt(aux_rw) - p_r2 * r_w.transpose() * C) /
            aux_rw;
        // J1.3: Ec.37-38
        Eigen::Matrix<double, 3, 6> J1_3 =
            (Eigen::Matrix<double, 3, 6>() << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
             0, 0, 0, 0, 0, 0, 1)
                .finished();
        // J1: Ec.29
        Eigen::Matrix<double, 4, 6> J1;
        J1.block<1, 3>(0, 0) = J1_1;
        J1.block<1, 3>(3, 5) = J1_2;
        J1.block<3, 6>(0, 1) = J1_3;

        // J2: Ec.39-41
        const Eigen::Matrix<double, 6, 12> J2 =
            (Eigen::Matrix<double, 6, 12>() << pairing.ln_other.pBase.x, 0, 0,
             pairing.ln_other.pBase.y, 0, 0, pairing.ln_other.pBase.z, 0, 0, 1,
             0, 0, 0, pairing.ln_other.pBase.x, 0, 0, pairing.ln_other.pBase.y,
             0, 0, pairing.ln_other.pBase.z, 0, 0, 1, 0, 0, 0,
             pairing.ln_other.pBase.x, 0, 0, pairing.ln_other.pBase.y, 0, 0,
             pairing.ln_other.pBase.z, 0, 0, 1, pairing.ln_other.director[0], 0,
             0, pairing.ln_other.director[1], 0, 0,
             pairing.ln_other.director[2], 0, 0, 1, 0, 0, 0,
             pairing.ln_other.director[0], 0, 0, pairing.ln_other.director[1],
             0, 0, pairing.ln_other.director[2], 0, 0, 1, 0, 0, 0,
             pairing.ln_other.director[0], 0, 0, pairing.ln_other.director[1],
             0, 0, pairing.ln_other.director[2], 0, 0, 1)
                .finished();
        // Build Jacobian
        jacobian = J1 * J2;
    }
    return error;
}

mrpt::math::CVectorFixedDouble<3> error_plane2plane(
    const mp2p_icp::matched_plane_t& pairing, const mrpt::poses::CPose3D &relativePose,
    Eigen::Matrix<double, 3, 12> jacobian)
{
    mrpt::math::CVectorFixedDouble<3> error;

    const auto nl = pairing.p_other.plane.getNormalVector();
    const auto ng = pairing.p_this.plane.getNormalVector();

    const auto p_oplus_nl = relativePose.rotateVector(nl);

    for (int i = 0; i < 3; i++) error[i] = ng[i] - p_oplus_nl[i];

    // Eval Jacobian:

    // df_oplus(A,p)/d_A. Section 7.3.2 tech. report:
    // "A tutorial on SE(3) transformation parameterizations and
    // on-manifold optimization"
    // Modified, to discard the last I_3 block, since this particular
    // cost function is insensible to translations.

    // clang-format off
    jacobian =
        (Eigen::Matrix<double, 3, 12>() <<
           nl.x,  0,  0,  nl.y,  0,  0, nl.z,  0,  0,  0,  0,  0,
            0, nl.x,  0,  0,  nl.y,  0,  0, nl.z,  0,  0,  0,  0,
            0,  0, nl.x,  0,  0,  nl.y,  0,  0, nl.z,  0,  0,  0
         ).finished();
    // clang-format on

    return error;
}
