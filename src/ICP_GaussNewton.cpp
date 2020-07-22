/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP_GaussNewton.cpp
 * @brief  ICP registration for points and planes
 * @author Jose Luis Blanco Claraco
 * @date   May 11, 2019
 */

#include <mp2p_icp/ICP_GaussNewton.h>
#include <mp2p_icp/optimal_tf_gauss_newton.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/poses/Lie/SE.h>
#include <Eigen/Dense>

IMPLEMENTS_MRPT_OBJECT(ICP_GaussNewton, mp2p_icp::ICP_Base, mp2p_icp)

using namespace mp2p_icp;

void ICP_GaussNewton::impl_ICP_iteration(
    ICP_State& s, const Parameters& p, ICP_iteration_result& out)
{
    using namespace std::string_literals;

    MRPT_START

    // the global list of pairings:
    s.currentPairings = ICP_Base::runMatchers(s);

    // Compute the optimal pose:
    OptimalTF_Result res;

    OptimalTF_GN_Parameters gnParams;
    gnParams.maxInnerLoopIterations = p.maxInnerLoopIterations;
    gnParams.linearizationPoint     = s.current_solution;

    try
    {
        optimal_tf_gauss_newton(
            s.currentPairings, p.pairingsWeightParameters, res, gnParams);
    }
    catch (const std::exception& e)
    {
        // Skip ill-defined problems if the no. of points is too small.
        // Nothing we can do:
        out.success = false;
        return;
    }

    out.success      = true;
    out.new_solution = mrpt::poses::CPose3D(res.optimal_pose);

    MRPT_END
}
