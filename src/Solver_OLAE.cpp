/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Solver_OLAE.cpp
 * @brief  ICP registration for points and planes
 * @author Jose Luis Blanco Claraco
 * @date   May 11, 2019
 */

#include <mp2p_icp/Solver_OLAE.h>
#include <mp2p_icp/optimal_tf_olae.h>

IMPLEMENTS_MRPT_OBJECT(Solver_OLAE, mp2p_icp::Solver, mp2p_icp)

using namespace mp2p_icp;

bool Solver_OLAE::impl_optimal_pose(
    const Pairings& pairings, OptimalTF_Result& out, const WeightParameters& wp,
    [[maybe_unused]] const SolverContext& sc) const
{
    MRPT_START

    out = OptimalTF_Result();

    // Compute the optimal pose:
    try
    {
        optimal_tf_olae(pairings, wp, out);
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
