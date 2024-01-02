/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   Solver_GaussNewton.h
 * @brief  ICP registration for points and planes
 * @author Jose Luis Blanco Claraco
 * @date   May 11, 2019
 */
#pragma once

#include <mp2p_icp/PairWeights.h>
#include <mp2p_icp/Solver.h>
#include <mp2p_icp/robust_kernels.h>

namespace mp2p_icp
{
/** ICP registration for points, planes, and lines, using an iterative
 * Gauss-Newton numerical solver.
 *
 * \ingroup mp2p_icp_grp
 */
class Solver_GaussNewton : public Solver
{
    DEFINE_MRPT_OBJECT(Solver_GaussNewton, mp2p_icp)

   public:
    uint32_t    maxIterations = 5;
    PairWeights pairWeights;

    RobustKernel robustKernel      = RobustKernel::None;
    double       robustKernelParam = 1.0;
    bool         innerLoopVerbose  = false;  //!< Prints GN inner loop details

    void initialize(const mrpt::containers::yaml& params) override;

   protected:
    // See base class docs
    bool impl_optimal_pose(
        const Pairings& pairings, OptimalTF_Result& out,
        const SolverContext& sc) const override;
};

}  // namespace mp2p_icp
