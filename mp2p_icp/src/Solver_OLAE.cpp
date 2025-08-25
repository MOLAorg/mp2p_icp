/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
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
#include <mp2p_icp/pt2ln_pl_to_pt2pt.h>

#include <iostream>

IMPLEMENTS_MRPT_OBJECT(Solver_OLAE, mp2p_icp::Solver, mp2p_icp)

using namespace mp2p_icp;

void Solver_OLAE::initialize(const mrpt::containers::yaml& p)
{
    Solver::initialize(p);

    if (p.has("pairingsWeightParameters"))
        pairingsWeightParameters.load_from(p["pairingsWeightParameters"]);
}

/* Save params:
     mrpt::containers::yaml pp = mrpt::containers::yaml::Map();
    pairingsWeightParameters.save_to(pp);
    p["pairingsWeightParameters"] = std::move(pp);
*/

bool Solver_OLAE::impl_optimal_pose(
    const Pairings& pairings, OptimalTF_Result& out, [[maybe_unused]] const SolverContext& sc) const
{
    MRPT_START

    out = OptimalTF_Result();

    const Pairings*         effectivePairings = &pairings;
    std::optional<Pairings> altPairings;

    if (!pairings.paired_pt2ln.empty() || !pairings.paired_pt2pl.empty())
    {
        altPairings       = pt2ln_pl_to_pt2pt(pairings, sc);
        effectivePairings = &altPairings.value();
    }

    // Compute the optimal pose, and return its validity:
    return optimal_tf_olae(*effectivePairings, pairingsWeightParameters, out);

    MRPT_END
}
