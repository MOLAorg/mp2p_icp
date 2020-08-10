/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2020 University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mp2p_error_terms_jacobians.cpp
 * @brief  Unit tests for Jacobians of error terms
 * @author Francisco Jose Mañas Alvarez, Jose Luis Blanco Claraco
 * @date   Apr 10, 2020
 */

#include <mp2p_icp/errorTerms.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/num_jacobian.h>  // finite difference method
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/random.h>
#include <Eigen/Dense>
#include <iostream>  // cerr

using namespace mrpt;  // for the "_deg" suffix
using namespace mrpt::math;
using namespace mrpt::poses;

auto& rnd = mrpt::random::getRandomGenerator();

static double normald(const double sigma)
{
    return rnd.drawGaussian1D_normalized() * sigma;
}
static float normalf(const float sigma)
{
    return rnd.drawGaussian1D_normalized() * sigma;
}

// ===========================================================================
//  Test: error_point2point
// ===========================================================================

void test_Jacob_error_point2point()
{
    const CPose3D p = CPose3D(
        // x y z
        normald(10), normald(10), normald(10),
        // Yaw pitch roll
        rnd.drawUniform(-M_PI, M_PI), rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5),
        rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5));

    mrpt::tfest::TMatchingPair pair;

    pair.this_x  = normalf(20);
    pair.this_y  = normalf(20);
    pair.this_z  = normalf(20);
    pair.other_x = normalf(10);
    pair.other_y = normalf(10);
    pair.other_z = normalf(10);

    // Implemented values:
    mrpt::math::CMatrixFixed<double, 3, 12> J1;
    // const mrpt::math::CVectorFixed<double, 3> error = // (Ignored here)
    mp2p_icp::error_point2point(pair, p, J1);

    // (12x6 Jacobian)
    const auto dDexpe_de = mrpt::poses::Lie::SE<3>::jacob_dDexpe_de(p);

    const mrpt::math::CMatrixFixed<double, 3, 6> jacob(J1 * dDexpe_de);

    // Numerical Jacobian:
    CMatrixDouble numJacob;
    {
        CVectorFixedDouble<6> x_mean;
        x_mean.setZero();

        CVectorFixedDouble<6> x_incrs;
        x_incrs.fill(1e-6);
        mrpt::math::estimateJacobian(
            x_mean,
            /* Error function to evaluate */
            std::function<void(
                const CVectorFixedDouble<6>& eps, const CPose3D& D,
                CVectorFixedDouble<3>& err)>(
                /* Lambda, capturing the pair data */
                [pair](
                    const CVectorFixedDouble<6>& eps, const CPose3D& D,
                    CVectorFixedDouble<3>& err) {
                    // SE(3) pose increment on the manifold:
                    const CPose3D incr         = Lie::SE<3>::exp(eps);
                    const CPose3D D_expEpsilon = D + incr;
                    err = mp2p_icp::error_point2point(pair, D_expEpsilon);
                }),
            x_incrs, p, numJacob);
    }

    if ((numJacob.asEigen() - jacob.asEigen()).array().abs().maxCoeff() > 1e-5)
    {
        std::cerr << "numJacob:\n"
                  << numJacob.asEigen() << "\njacob:\n"
                  << jacob.asEigen() << "\nDiff:\n"
                  << (numJacob - jacob) << "\nJ1:\n"
                  << J1.asEigen() << "\n";
        THROW_EXCEPTION("Jacobian mismatch, see above.");
    }
}

// ===========================================================================
//  Test: error_point2line
// ===========================================================================

void test_Jacob_error_point2line()
{
    const CPose3D p = CPose3D(
        // x y z
        normald(10), normald(10), normald(10),
        // Yaw pitch roll
        rnd.drawUniform(-M_PI, M_PI), rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5),
        rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5));

    mp2p_icp::point_line_pair_t pair;

    pair.ln_this.pBase.x = normalf(20);
    pair.ln_this.pBase.y = normalf(20);
    pair.ln_this.pBase.z = normalf(20);
    pair.ln_this.director[0] = normald(20);
    pair.ln_this.director[1] = normald(20);
    pair.ln_this.director[2] = normald(20);

    pair.pt_other.x = normalf(10);
    pair.pt_other.y = normalf(10);
    pair.pt_other.z = normalf(10);

    // Implemented values:
    mrpt::math::CMatrixFixed<double, 1, 12> J1;

    mp2p_icp::error_point2line(pair, p, J1);

    // (12x6 Jacobian)
    const auto dDexpe_de = mrpt::poses::Lie::SE<3>::jacob_dDexpe_de(p);

    const mrpt::math::CMatrixFixed<double, 1, 6> jacob(J1 * dDexpe_de);

    // Numerical Jacobian:
    CMatrixDouble numJacob;
    {
        CVectorFixedDouble<6> x_mean;
        x_mean.setZero();

        CVectorFixedDouble<6> x_incrs;
        x_incrs.fill(1e-6);
        mrpt::math::estimateJacobian(
            x_mean,
            /* Error function to evaluate */
            std::function<void(
                const CVectorFixedDouble<6>& eps, const CPose3D& D,
                CVectorFixedDouble<1>& err)>(
                /* Lambda, capturing the pair data */
                [pair](
                    const CVectorFixedDouble<6>& eps, const CPose3D& D,
                    CVectorFixedDouble<1>& err) {
                    // SE(3) pose increment on the manifold:
                    const CPose3D incr         = Lie::SE<3>::exp(eps);
                    const CPose3D D_expEpsilon = D + incr;
                    err = mp2p_icp::error_point2line(pair, D_expEpsilon);
                }),
            x_incrs, p, numJacob);
    }

    if ((numJacob.asEigen() - jacob.asEigen()).array().abs().maxCoeff() > 1e-5)
    {
        std::cerr << "numJacob:\n"
                  << numJacob.asEigen() << "\njacob:\n"
                  << jacob.asEigen() << "\nDiff:\n"
                  << (numJacob - jacob) << "\nJ1:\n"
                  << J1.asEigen() << "\n";
        THROW_EXCEPTION("Jacobian mismatch, see above.");
    }
}

// ===========================================================================
//  Test: error_point2plane
// ===========================================================================

void test_Jacob_error_point2plane()
{
    const CPose3D p = CPose3D(
        // x y z
        normald(10), normald(10), normald(10),
        // Yaw pitch roll
        rnd.drawUniform(-M_PI, M_PI), rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5),
        rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5));

    mp2p_icp::point_plane_pair_t pair;

    pair.pl_this.centroid.x = normalf(20);
    pair.pl_this.centroid.y = normalf(20);
    pair.pl_this.centroid.z = normalf(20);
    pair.pl_this.plane.coefs[0] = normald(20);
    pair.pl_this.plane.coefs[1] = normald(20);
    pair.pl_this.plane.coefs[2] = normald(20);

    pair.pt_other.x = normalf(10);
    pair.pt_other.y = normalf(10);
    pair.pt_other.z = normalf(10);

    // Implemented values:
    mrpt::math::CMatrixFixed<double, 1, 12> J1;

    mp2p_icp::error_point2plane(pair, p, J1);

    // (12x6 Jacobian)
    const auto dDexpe_de = mrpt::poses::Lie::SE<3>::jacob_dDexpe_de(p);

    const mrpt::math::CMatrixFixed<double, 1, 6> jacob(J1 * dDexpe_de);

    // Numerical Jacobian:
    CMatrixDouble numJacob;
    {
        CVectorFixedDouble<6> x_mean;
        x_mean.setZero();

        CVectorFixedDouble<6> x_incrs;
        x_incrs.fill(1e-6);
        mrpt::math::estimateJacobian(
            x_mean,
            /* Error function to evaluate */
            std::function<void(
                const CVectorFixedDouble<6>& eps, const CPose3D& D,
                CVectorFixedDouble<1>& err)>(
                /* Lambda, capturing the pair data */
                [pair](
                    const CVectorFixedDouble<6>& eps, const CPose3D& D,
                    CVectorFixedDouble<1>& err) {
                    // SE(3) pose increment on the manifold:
                    const CPose3D incr         = Lie::SE<3>::exp(eps);
                    const CPose3D D_expEpsilon = D + incr;
                    err = mp2p_icp::error_point2plane(pair, D_expEpsilon);
                }),
            x_incrs, p, numJacob);
    }

    if ((numJacob.asEigen() - jacob.asEigen()).array().abs().maxCoeff() > 1e-5)
    {
        std::cerr << "numJacob:\n"
                  << numJacob.asEigen() << "\njacob:\n"
                  << jacob.asEigen() << "\nDiff:\n"
                  << (numJacob - jacob) << "\nJ1:\n"
                  << J1.asEigen() << "\n";
        THROW_EXCEPTION("Jacobian mismatch, see above.");
    }
}

// ===========================================================================
//  Test: error_line2line
// ===========================================================================

void test_Jacob_error_line2line()
{
    const CPose3D p = CPose3D(
        // x y z
        normald(10), normald(10), normald(10),
        // Yaw pitch roll
        rnd.drawUniform(-M_PI, M_PI), rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5),
        rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5));

    mp2p_icp::matched_line_t pair;

    pair.ln_this.pBase.x = normalf(20);
    pair.ln_this.pBase.y = normalf(20);
    pair.ln_this.pBase.z = normalf(20);
    pair.ln_this.director[0] = normald(20);
    pair.ln_this.director[1] = normald(20);
    pair.ln_this.director[2] = normald(20);

    pair.ln_other.pBase.x = normalf(10);
    pair.ln_other.pBase.y = normalf(10);
    pair.ln_other.pBase.z = normalf(10);
    pair.ln_other.director[0] = normald(10);
    pair.ln_other.director[1] = normald(10);
    pair.ln_other.director[2] = normald(10);

    // Implemented values:
    mrpt::math::CMatrixFixed<double, 4, 12> J1;

    mp2p_icp::error_line2line(pair, p, J1);

    // (12x6 Jacobian)
    const auto dDexpe_de = mrpt::poses::Lie::SE<3>::jacob_dDexpe_de(p);

    const mrpt::math::CMatrixFixed<double, 4, 6> jacob(J1 * dDexpe_de);

    // Numerical Jacobian:
    CMatrixDouble numJacob;
    {
        CVectorFixedDouble<6> x_mean;
        x_mean.setZero();

        CVectorFixedDouble<6> x_incrs;
        x_incrs.fill(1e-6);
        mrpt::math::estimateJacobian(
            x_mean,
            /* Error function to evaluate */
            std::function<void(
                const CVectorFixedDouble<6>& eps, const CPose3D& D,
                CVectorFixedDouble<4>& err)>(
                /* Lambda, capturing the pair data */
                [pair](
                    const CVectorFixedDouble<6>& eps, const CPose3D& D,
                    CVectorFixedDouble<4>& err) {
                    // SE(3) pose increment on the manifold:
                    const CPose3D incr         = Lie::SE<3>::exp(eps);
                    const CPose3D D_expEpsilon = D + incr;
                    err = mp2p_icp::error_line2line(pair, D_expEpsilon);
                }),
            x_incrs, p, numJacob);
    }

    if ((numJacob.asEigen() - jacob.asEigen()).array().abs().maxCoeff() > 1e-5)
    {
        std::cerr << "numJacob:\n"
                  << numJacob.asEigen() << "\njacob:\n"
                  << jacob.asEigen() << "\nDiff:\n"
                  << (numJacob - jacob) << "\nJ1:\n"
                  << J1.asEigen() << "\n";
        THROW_EXCEPTION("Jacobian mismatch, see above.");
    }
}

// ===========================================================================
//  Test: error_plane2plane
// ===========================================================================

void test_Jacob_error_plane2plane()
{
    const CPose3D p = CPose3D(
        // x y z
        normald(10), normald(10), normald(10),
        // Yaw pitch roll
        rnd.drawUniform(-M_PI, M_PI), rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5),
        rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5));

    mp2p_icp::matched_plane_t pair;

    pair.p_this.centroid.x = normalf(20);
    pair.p_this.centroid.y = normalf(20);
    pair.p_this.centroid.z = normalf(20);
    pair.p_this.plane.coefs[0] = normald(20);
    pair.p_this.plane.coefs[1] = normald(20);
    pair.p_this.plane.coefs[2] = normald(20);

    pair.p_other.centroid.x = normalf(10);
    pair.p_other.centroid.y = normalf(10);
    pair.p_other.centroid.z = normalf(10);
    pair.p_other.plane.coefs[0] = normald(10);
    pair.p_other.plane.coefs[1] = normald(10);
    pair.p_other.plane.coefs[2] = normald(10);

    // Implemented values:
    mrpt::math::CMatrixFixed<double, 3, 12> J1;

    mp2p_icp::error_plane2plane(pair, p, J1);

    // (12x6 Jacobian)
    const auto dDexpe_de = mrpt::poses::Lie::SE<3>::jacob_dDexpe_de(p);

    const mrpt::math::CMatrixFixed<double, 3, 6> jacob(J1 * dDexpe_de);

    // Numerical Jacobian:
    CMatrixDouble numJacob;
    {
        CVectorFixedDouble<6> x_mean;
        x_mean.setZero();

        CVectorFixedDouble<6> x_incrs;
        x_incrs.fill(1e-6);
        mrpt::math::estimateJacobian(
            x_mean,
            /* Error function to evaluate */
            std::function<void(
                const CVectorFixedDouble<6>& eps, const CPose3D& D,
                CVectorFixedDouble<3>& err)>(
                /* Lambda, capturing the pair data */
                [pair](
                    const CVectorFixedDouble<6>& eps, const CPose3D& D,
                    CVectorFixedDouble<3>& err) {
                    // SE(3) pose increment on the manifold:
                    const CPose3D incr         = Lie::SE<3>::exp(eps);
                    const CPose3D D_expEpsilon = D + incr;
                    err = mp2p_icp::error_plane2plane(pair, D_expEpsilon);
                }),
            x_incrs, p, numJacob);
    }

    if ((numJacob.asEigen() - jacob.asEigen()).array().abs().maxCoeff() > 1e-5)
    {
        std::cerr << "numJacob:\n"
                  << numJacob.asEigen() << "\njacob:\n"
                  << jacob.asEigen() << "\nDiff:\n"
                  << (numJacob - jacob) << "\nJ1:\n"
                  << J1.asEigen() << "\n";
        THROW_EXCEPTION("Jacobian mismatch, see above.");
    }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        rnd.randomize(1234);  // for reproducible tests

        test_Jacob_error_point2point();
        test_Jacob_error_point2line();
        test_Jacob_error_point2plane();
    //    test_Jacob_error_line2line();
        test_Jacob_error_plane2plane();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
