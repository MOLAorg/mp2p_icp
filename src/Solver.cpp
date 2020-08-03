/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
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

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(Solver, mrpt::rtti::CObject, mp2p_icp);

using namespace mp2p_icp;

void Solver::initialize(const mrpt::containers::Parameters& params)
{
    if (params.has("runFromIteration"))
        runFromIteration = params["runFromIteration"].as<uint32_t>();
    if (params.has("runUpToIteration"))
        runUpToIteration = params["runUpToIteration"].as<uint32_t>();
}

bool Solver::optimal_pose(
    const Pairings& pairings, OptimalTF_Result& out, const WeightParameters& wp,
    const SolverContext& sc) const
{
    const auto iter = sc.icpIteration;
    if (iter < runFromIteration) return false;
    if (runUpToIteration > 0 && iter > runUpToIteration) return false;

    return impl_optimal_pose(pairings, out, wp, sc);
}
