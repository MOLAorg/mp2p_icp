/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: BSD-3-Clause
*/
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
