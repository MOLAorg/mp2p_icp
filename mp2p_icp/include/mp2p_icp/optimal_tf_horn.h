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
 * @file   optimal_tf_horn.h
 * @brief  Classic Horn's solution for optimal SE(3) transformation
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */
#pragma once

#include <mp2p_icp/OptimalTF_Result.h>
#include <mp2p_icp/Pairings.h>
#include <mp2p_icp/WeightParameters.h>

namespace mp2p_icp
{
/** Classic Horn's solution for optimal SE(3) transformation, modified to
 * accept point-to-point, line-to-line, plane-to-plane pairings.
 * If you need point-to-line or point-to-plane pairings, use the wrapper
 * mp2p_icp::Solver_Horn.
 *
 * \note On MRPT naming convention: "this"=global; "other"=local.
 * \return false If the number of pairings is too small for a unique
 * solution, true on success.
 */
[[nodiscard]] bool optimal_tf_horn(
    const mp2p_icp::Pairings& in, const WeightParameters& wp, OptimalTF_Result& result);

/** @} */

}  // namespace mp2p_icp
