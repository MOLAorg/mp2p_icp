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
 * @file   covariance.h
 * @brief  Covariance estimation methods for ICP results
 * @author Jose Luis Blanco Claraco
 * @date   Jun 9, 2020
 */
#pragma once

#include <mp2p_icp/Pairings.h>
#include <mrpt/math/CMatrixFixed.h>

namespace mp2p_icp
{
struct CovarianceParameters
{
    // Finite difference deltas:
    double finDif_xyz    = 1e-7;
    double finDif_angles = 1e-7;
};

/** Covariance estimation methods for an ICP result.
 *
 * \ingroup mp2p_icp_grp
 */
mrpt::math::CMatrixDouble66 covariance(
    const Pairings& finalPairings, const mrpt::poses::CPose3D& finalAlignSolution,
    const CovarianceParameters& p);

}  // namespace mp2p_icp
