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
 * @file   Solver_Horn.h
 * @brief  ICP solver for pointclouds split in different "layers"
 * @author Jose Luis Blanco Claraco
 * @date   Jan 20, 2019
 */
#pragma once

#include <mp2p_icp/Solver.h>

namespace mp2p_icp
{
/** ICP registration for pointclouds split in different "layers"
 *
 * \ingroup mp2p_icp_grp
 */
class Solver_Horn : public Solver
{
    DEFINE_MRPT_OBJECT(Solver_Horn, mp2p_icp)
   public:
    /** Weight and robust kernel parameters associated with the low-level
     * optimal pose estimation algorithms */
    WeightParameters pairingsWeightParameters;

    void initialize(const mrpt::containers::yaml& params) override;

   protected:
    // See base class docs
    bool impl_optimal_pose(
        const Pairings& pairings, OptimalTF_Result& out, const SolverContext& sc) const override;
};

}  // namespace mp2p_icp
