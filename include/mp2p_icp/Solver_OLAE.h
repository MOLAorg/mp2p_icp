/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Solver_OLAE.h
 * @brief  ICP registration for points and planes
 * @author Jose Luis Blanco Claraco
 * @date   May 11, 2019
 */
#pragma once

#include <mp2p_icp/Solver.h>

namespace mp2p_icp
{
/** ICP registration for points, planes, and lines.
 * Refer to technical report: XXX
 *
 * \ingroup mp2p_icp_grp
 */
class Solver_OLAE : public Solver
{
    DEFINE_MRPT_OBJECT(Solver_OLAE, mp2p_icp)

   protected:
    // See base class docs
    bool impl_optimal_pose(
        const Pairings& pairings, OptimalTF_Result& out,
        const WeightParameters& wp, const SolverContext& sc) const override;
};

}  // namespace mp2p_icp
