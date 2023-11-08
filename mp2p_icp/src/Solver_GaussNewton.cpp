/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2023 Jose Luis Blanco, University of Almeria
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

void Solver_GaussNewton::initialize(const mrpt::containers::yaml& params)
{
    Solver::initialize(params);

    MCP_LOAD_REQ(params, maxIterations);
    MCP_LOAD_OPT(params, innerLoopVerbose);
    MCP_LOAD_OPT(params, robustKernel);
    MCP_LOAD_OPT(params, robustKernelParam);

    if (params.has("pair_weights"))
        pairWeights.load_from(params["pair_weights"]);
}

bool Solver_GaussNewton::impl_optimal_pose(
    const Pairings& pairings, OptimalTF_Result& out,
    [[maybe_unused]] const SolverContext& sc) const
{
    MRPT_START

    out = OptimalTF_Result();

    OptimalTF_GN_Parameters gnParams;
    gnParams.maxInnerLoopIterations = maxIterations;
    gnParams.pairWeights            = pairWeights;

    gnParams.kernel      = robustKernel;
    gnParams.kernelParam = robustKernelParam;

    ASSERT_(sc.guessRelativePose.has_value());
    gnParams.linearizationPoint =
        mrpt::poses::CPose3D(sc.guessRelativePose.value());

    gnParams.verbose = innerLoopVerbose;

    // Compute the optimal pose, and return its validity:
    return optimal_tf_gauss_newton(pairings, out, gnParams);

    MRPT_END
}
