/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   optimal_tf_olae.h
 * @brief  OLAE algorithm to find the SE(3) optimal transformation
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

/** OLAE algorithm to find the SE(3) optimal transformation given a set of
 * correspondences between points-points, lines-lines, planes-planes. Refer to
 * technical report: Jose-Luis Blanco-Claraco. OLAE-ICP: Robust and fast
 * alignment of geometric features with the optimal linear attitude estimator,
 * Arxiv 2019.
 *
 * \return false If the number of pairings is too small for a unique
 * solution, true on success.
 */
[[nodiscard]] bool optimal_tf_olae(
    const Pairings& in, const WeightParameters& wp, OptimalTF_Result& result);

/** @} */

}  // namespace mp2p_icp
