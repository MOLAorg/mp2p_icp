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
/** \addtogroup  mp2p_icp_grp
 * @{ */

struct Pairings_Horn : public Pairings_Common
{
};

/** Classic Horn's solution for optimal SE(3) transformation.
 * Modified accepts point-to-point pairings. No robust estimation is attempted.
 *
 * Just a wrapper on MRPT implementation mrpt::tfest::se3_l2()
 *
 * \note On MRPT naming convention: "this"=global; "other"=local.
 */
void optimal_tf_horn(
    const mrpt::tfest::TMatchingPairList& in, OptimalTF_Result& result);

/** @} */

}  // namespace mp2p_icp
