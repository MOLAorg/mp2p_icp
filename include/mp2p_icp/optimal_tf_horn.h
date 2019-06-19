/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   optimal_tf_horn.h
 * @brief  Classic Horn's solution for optimal SE(3) transformation
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */
#pragma once

#include <mp2p_icp/optimal_tf_common.h>

namespace mp2p_icp
{
/** Classic Horn's solution for optimal SE(3) transformation, modified to
 * accept point-to-point, line-to-line, plane-to-plane pairings.
 *
 *
 * \note On MRPT naming convention: "this"=global; "other"=local.
 */
void optimal_tf_horn(
    const mp2p_icp::WeightedPairings& in, OptimalTF_Result& result);

/** @} */

}  // namespace mp2p_icp
