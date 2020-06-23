/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2020 University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   test-mp2p_jacobians.cpp
 * @brief  Unit tests for Jacobians
 * @author Francisco Jose Mañas Alvarez, Jose Luis Blanco Claraco
 * @date   Apr 10, 2020
 */

#include <mp2p_icp/optimal_tf_common.h>
#include <mp2p_icp/optimal_tf_gauss_newton.h>
#include <mp2p_icp/optimal_tf_horn.h>
#include <mp2p_icp/optimal_tf_olae.h>
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

static void func_jacob_D_expe(
    const CVectorFixedDouble<6>& eps, const CPose3D& D,
    CVectorFixedDouble<12>& Y)
{
    const CPose3D incr   = Lie::SE<3>::exp(eps);
    const CPose3D expe_D = D + incr;
    expe_D.getAs12Vector(Y);
}

// Test Jacobian: d D*exp(e) / d e
// 10.3.4 in tech report
void test_Jacob_dDexpe_de()
{
    const CPose3D p = CPose3D(1, 2, 3, 10.0_deg, 20.0_deg, 30.0_deg);

    const auto theor_jacob = Lie::SE<3>::jacob_dDexpe_de(p);

    CMatrixDouble numJacobs;
    {
        CVectorFixedDouble<6> x_mean;
        x_mean.setZero();

        CVectorFixedDouble<6> x_incrs;
        x_incrs.fill(1e-6);
        mrpt::math::estimateJacobian(
            x_mean,
            std::function<void(
                const CVectorFixedDouble<6>& eps, const CPose3D& D,
                CVectorFixedDouble<12>& Y)>(&func_jacob_D_expe),
            x_incrs, p, numJacobs);
    }

    ASSERT_BELOW_(
        (numJacobs.asEigen() - theor_jacob.asEigen()).array().abs().maxCoeff(),
        1e-3);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        auto& rnd = mrpt::random::getRandomGenerator();
        rnd.randomize(1234);  // for reproducible tests

        test_Jacob_dDexpe_de();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
