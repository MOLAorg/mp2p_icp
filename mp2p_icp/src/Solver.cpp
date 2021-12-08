/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Solver.cpp
 * @brief  Virtual base class for optimal alignment solvers (one step in ICP).
 * @author Jose Luis Blanco Claraco
 * @date   August 3, 2020
 */

#include <mp2p_icp/Solver.h>
#include <mrpt/core/exceptions.h>

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(Solver, mrpt::rtti::CObject, mp2p_icp)

using namespace mp2p_icp;

void Solver::initialize(const mrpt::containers::yaml& params)
{
    MCP_LOAD_OPT(params, runFromIteration);
    MCP_LOAD_OPT(params, runUpToIteration);
}

bool Solver::optimal_pose(
    const Pairings& pairings, OptimalTF_Result& out,
    const SolverContext& sc) const
{
    const auto iter = sc.icpIteration;
    if (iter.has_value() && *iter < runFromIteration) return false;
    if (iter.has_value() && runUpToIteration > 0 && *iter > runUpToIteration)
        return false;

    return impl_optimal_pose(pairings, out, sc);
}
