/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   pt2ln_pl_to_pt2pt.h
 * @brief  Converter of pairings.
 * @author Jose Luis Blanco Claraco
 * @date   Dec 15, 2021
 */
#pragma once

#include <mp2p_icp/Pairings.h>
#include <mp2p_icp/Solver.h>

namespace mp2p_icp
{
/** Returns a copy of the input pairings, adding new pt2pt pairings for those
 *  pt2ln and pt2pl pairings.
 */
Pairings pt2ln_pl_to_pt2pt(const Pairings& in, const SolverContext& sc);

}  // namespace mp2p_icp
