/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
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
    const mp2p_icp::Pairings& in, const WeightParameters& wp,
    OptimalTF_Result& result);

/** @} */

}  // namespace mp2p_icp
