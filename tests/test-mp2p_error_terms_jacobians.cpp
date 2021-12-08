/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2021 University of Almeria
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

static void test_Jacob_error_point2point()
{
    const CPose3D p = CPose3D(
        // x y z
        normald(10), normald(10), normald(10),
        // Yaw pitch roll
        rnd.drawUniform(-M_PI, M_PI), rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5),
        rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5));

    mrpt::tfest::TMatchingPair pair;
    pair.global = {normalf(20), normalf(20), normalf(20)};
    pair.local  = {normalf(20), normalf(20), normalf(20)};

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

static void test_Jacob_error_point2line()
{
    const CPose3D p = CPose3D(
        // x y z
        normald(10), normald(10), normald(10),
        // Yaw pitch roll
        rnd.drawUniform(-M_PI, M_PI), rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5),
        rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5));

    mp2p_icp::point_line_pair_t pair;

    pair.ln_global.pBase.x = normalf(20);
    pair.ln_global.pBase.y = normalf(20);
    pair.ln_global.pBase.z = normalf(20);
    pair.ln_global.director =
        mrpt::math::TPoint3D(normald(1), normald(1), normald(1)).unitarize();

    pair.pt_local.x = normalf(10);
    pair.pt_local.y = normalf(10);
    pair.pt_local.z = normalf(10);

    // Implemented values:
    mrpt::math::CMatrixFixed<double, 3, 12> J1;

    mp2p_icp::error_point2line(pair, p, J1);

    // (12x6 Jacobian)
    const auto dDexpe_de = mrpt::poses::Lie::SE<3>::jacob_dDexpe_de(p);

    const mrpt::math::CMatrixFixed<double, 3, 6> jacob(J1 * dDexpe_de);

    // Numerical Jacobian:
    CMatrixDouble numJacob;
    {
        CVectorFixedDouble<6> x_mean;
        x_mean.setZero();

        CVectorFixedDouble<6> x_incrs;
        x_incrs.fill(1e-5);
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
                    err = mp2p_icp::error_point2line(pair, D_expEpsilon);
                }),
            x_incrs, p, numJacob);
    }

    if ((numJacob.asEigen() - jacob.asEigen()).array().abs().maxCoeff() > 1e-5)
    {
        std::cerr << "relativePose: " << p
                  << "\n"
                     "numJacob:"
                  << numJacob.asEigen()
                  << "\n"
                     "jacob   :"
                  << jacob.asEigen()
                  << "\n"
                     "diff    :"
                  << (numJacob - jacob)
                  << "\n"
                     "J1      :"
                  << J1.asEigen() << "\n";
        THROW_EXCEPTION("Jacobian mismatch, see above.");
    }
}

// ===========================================================================
//  Test: error_point2plane
// ===========================================================================

static void test_Jacob_error_point2plane()
{
    const CPose3D p = CPose3D(
        // x y z
        normald(10), normald(10), normald(10),
        // Yaw pitch roll
        rnd.drawUniform(-M_PI, M_PI), rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5),
        rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5));

    mp2p_icp::point_plane_pair_t pair;

    pair.pl_global.centroid.x     = normalf(20);
    pair.pl_global.centroid.y     = normalf(20);
    pair.pl_global.centroid.z     = normalf(20);
    pair.pl_global.plane.coefs[0] = normald(20);
    pair.pl_global.plane.coefs[1] = normald(20);
    pair.pl_global.plane.coefs[2] = normald(20);

    pair.pt_local.x = normalf(10);
    pair.pt_local.y = normalf(10);
    pair.pt_local.z = normalf(10);

    // Implemented values:
    mrpt::math::CMatrixFixed<double, 3, 12> J1;

    mp2p_icp::error_point2plane(pair, p, J1);

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

static void test_Jacob_error_line2line()
{
    const CPose3D p = CPose3D(
        // x y z
        normald(10), normald(10), normald(10),
        // Yaw pitch roll
        rnd.drawUniform(-M_PI, M_PI), rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5),
        rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5));

    mp2p_icp::matched_line_t pair;

    pair.ln_global.pBase.x     = normalf(10);
    pair.ln_global.pBase.y     = normalf(10);
    pair.ln_global.pBase.z     = normalf(10);
    pair.ln_global.director[0] = normald(10);
    pair.ln_global.director[1] = normald(10);
    pair.ln_global.director[2] = normald(10);

    pair.ln_local.pBase.x     = normalf(10);
    pair.ln_local.pBase.y     = normalf(10);
    pair.ln_local.pBase.z     = normalf(10);
    pair.ln_local.director[0] = normald(10);
    pair.ln_local.director[1] = normald(10);
    pair.ln_local.director[2] = normald(10);

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

static void test_Jacob_error_plane2plane()
{
    const CPose3D p = CPose3D(
        // x y z
        normald(10), normald(10), normald(10),
        // Yaw pitch roll
        rnd.drawUniform(-M_PI, M_PI), rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5),
        rnd.drawUniform(-M_PI * 0.5, M_PI * 0.5));

    mp2p_icp::matched_plane_t pair;

    pair.p_global.centroid.x     = normalf(20);
    pair.p_global.centroid.y     = normalf(20);
    pair.p_global.centroid.z     = normalf(20);
    pair.p_global.plane.coefs[0] = normald(20);
    pair.p_global.plane.coefs[1] = normald(20);
    pair.p_global.plane.coefs[2] = normald(20);

    pair.p_local.centroid.x     = normalf(10);
    pair.p_local.centroid.y     = normalf(10);
    pair.p_local.centroid.z     = normalf(10);
    pair.p_local.plane.coefs[0] = normald(10);
    pair.p_local.plane.coefs[1] = normald(10);
    pair.p_local.plane.coefs[2] = normald(10);

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

// ===========================================================================
//  Test: error_line2line
// ===========================================================================

static void test_error_line2line()
{
    const CPose3D p = CPose3D(
        // x y z
        1, 0.5, 0.1,
        // Yaw pitch roll
        0, 0, 0);

    mp2p_icp::matched_line_t pair;

    pair.ln_global.pBase.x     = 0;
    pair.ln_global.pBase.y     = 1;
    pair.ln_global.pBase.z     = -4;
    pair.ln_global.director[0] = 0.4364;
    pair.ln_global.director[1] = 0.8729;
    pair.ln_global.director[2] = -0.2182;

    pair.ln_local.pBase.x     = 2;
    pair.ln_local.pBase.y     = 1;
    pair.ln_local.pBase.z     = -0.5;
    pair.ln_local.director[0] = 0.2357;
    pair.ln_local.director[1] = 0.2357;
    pair.ln_local.director[2] = 0.9428;

    // Implemented values:
    mrpt::math::CMatrixFixed<double, 4, 12> J1;

    mrpt::math::CVectorFixedDouble<4> ref_error;
    ref_error[0] = 0.0517;
    ref_error[1] = 0.2007;
    ref_error[2] = 0.6372;
    ref_error[3] = -1.1610;

#if 0
    mrpt::math::CVectorFixedDouble<4> error =
        mp2p_icp::error_line2line(pair, p, J1);

    std::cout << "\nResultado: \n"
              << error << "\nRecta A:\n"
              << pair.ln_global << "\nRecta B:\n"
              << pair.ln_local << "\n";
#endif

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
#if 0
    std::cout << "numJacob:\n"
              << numJacob.asEigen() << "\njacob:\n"
              << jacob.asEigen() << "\nDiff:\n"
              << (numJacob - jacob) << "\nJ1:\n"
              << J1.asEigen() << "\ndDexp_de:\n"
              << dDexpe_de.asEigen() << "\n";
#endif
}

static void test_against_ground_truth_error_point2line()
{
    const CPose3D p = CPose3D::Identity();

    mp2p_icp::point_line_pair_t pair;

    pair.ln_global.pBase.x  = 10;
    pair.ln_global.pBase.y  = 11;
    pair.ln_global.pBase.z  = 12;
    pair.ln_global.director = mrpt::math::TPoint3D(0, 0, 1).unitarize();

    pair.pt_local.x = 10;
    pair.pt_local.y = 11;
    pair.pt_local.z = -1.0;

    // Implemented values:
    mrpt::math::CMatrixFixed<double, 3, 12> J1;

    CVectorFixedDouble<3> err = mp2p_icp::error_point2line(pair, p, J1);
    ASSERT_NEAR_(err[0], 0.0, 1e-6);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        // test Jacobians:
        // ----------------------------------------------
        rnd.randomize(1234);  // for reproducible tests

        // Run for many different random conditions:
        for (int i = 0; i < 1000; i++)
        {
            test_Jacob_error_point2point();
            test_Jacob_error_point2line();
            test_Jacob_error_point2plane();
            test_Jacob_error_plane2plane();
            // TODO: Fix this one:
            // test_Jacob_error_line2line();

            test_error_line2line();
        }

        // Test for known fixed conditions:
        // ----------------------------------------------
        test_against_ground_truth_error_point2line();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
