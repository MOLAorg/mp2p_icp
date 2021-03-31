/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2021 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   optimal_tf_gauss_newton.h
 * @brief  Simple non-linear optimizer to find the SE(3) optimal transformation
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */
#pragma once

#include <mp2p_icp/OptimalTF_Result.h>
#include <mp2p_icp/Pairings.h>
#include <mp2p_icp/WeightParameters.h>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

struct OptimalTF_GN_Parameters
{
    bool verbose = false;

    /** Maximum number of iterations trying to solve for the optimal pose */
    uint32_t maxInnerLoopIterations = 6;

    /** Minimum SE(3) change to stop iterating. */
    double minDelta = 1e-7;

    /** The linerization point (the current relative pose guess) */
    std::optional<mrpt::poses::CPose3D> linearizationPoint;
};

/** Gauss-Newton non-linear, iterative optimizer to find the SE(3) optimal
 * transformation between a set of correspondences.
 *
 * This method requires a linearization point in
 * `OptimalTF_GN_Parameters::linearizationPoint`.
 */
void optimal_tf_gauss_newton(
    const Pairings& in, const WeightParameters& wp, OptimalTF_Result& result,
    const OptimalTF_GN_Parameters& gnParams = OptimalTF_GN_Parameters());

/** @} */

}  // namespace mp2p_icp
