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
 * @file   optimal_tf_gauss_newton.h
 * @brief  Simple non-linear optimizer to find the SE(3) optimal transformation
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */
#pragma once

#include <mp2p_icp/OptimalTF_Result.h>
#include <mp2p_icp/PairWeights.h>
#include <mp2p_icp/Pairings.h>
#include <mp2p_icp/robust_kernels.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */
struct OptimalTF_GN_Parameters
{
    OptimalTF_GN_Parameters() = default;

    /** The linerization point (the current relative pose guess) */
    std::optional<mrpt::poses::CPose3D> linearizationPoint;

    /** Optional prior guess of the SE(3) solution, including a mean value
     *  and an inverse covariance (information) matrix, i.e. zeros in the
     * diagonal mean that those prior coordinates should be ignored, a large
     * value means the solution must be close to those coordinates.
     */
    std::optional<mrpt::poses::CPose3DPDFGaussianInf> prior;

    /** Minimum SE(3) change to stop iterating. */
    double minDelta = 1e-7;

    /** Maximum cost function; when reached, stop iterating. */
    double maxCost = 0;

    PairWeights pairWeights;

    /** Maximum number of iterations trying to solve for the optimal pose */
    uint32_t maxInnerLoopIterations = 6;

    RobustKernel kernel      = RobustKernel::None;
    double       kernelParam = 1.0;

    bool verbose = false;
};

/** Gauss-Newton non-linear, iterative optimizer to find the SE(3) optimal
 * transformation between a set of correspondences.
 *
 * This method requires a linearization point in
 * `OptimalTF_GN_Parameters::linearizationPoint`.
 *
 * \return false If the number of pairings is too small for a unique
 * solution, true on success.
 */
[[nodiscard]] bool optimal_tf_gauss_newton(
    const Pairings& in, OptimalTF_Result& result,
    const OptimalTF_GN_Parameters& gnParams = OptimalTF_GN_Parameters());

/** @} */

}  // namespace mp2p_icp
