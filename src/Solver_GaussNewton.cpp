/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Solver_GaussNewton.cpp
 * @brief  ICP registration for points and planes
 * @author Jose Luis Blanco Claraco
 * @date   May 11, 2019
 */

#include <mp2p_icp/Solver_GaussNewton.h>
#include <mp2p_icp/optimal_tf_gauss_newton.h>
#include <mrpt/core/exceptions.h>

IMPLEMENTS_MRPT_OBJECT(Solver_GaussNewton, mp2p_icp::Solver, mp2p_icp)

using namespace mp2p_icp;

void Solver_GaussNewton::initialize(const mrpt::containers::Parameters& params)
{
    Solver::initialize(params);

    MCP_LOAD_REQ(params, maxIterations);
}

bool Solver_GaussNewton::impl_optimal_pose(
    const Pairings& pairings, OptimalTF_Result& out, const WeightParameters& wp,
    [[maybe_unused]] const SolverContext& sc) const
{
    MRPT_START

    out = OptimalTF_Result();

    OptimalTF_GN_Parameters gnParams;
    gnParams.maxInnerLoopIterations = maxIterations;

    ASSERT_(sc.guessRelativePose.has_value());
    gnParams.linearizationPoint =
        mrpt::poses::CPose3D(sc.guessRelativePose.value());

    // Compute the optimal pose:
    try
    {
        optimal_tf_gauss_newton(pairings, wp, out, gnParams);
    }
    catch (const std::exception& e)
    {
        // Skip ill-defined problems if the no. of points is too small.
        // Nothing we can do:
        return false;
    }

    return true;

    MRPT_END
}
