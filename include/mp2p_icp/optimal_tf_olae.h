/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   optimal_tf_olae.h
 * @brief  OLAE algorithm to find the SE(3) optimal transformation
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */
#pragma once

#include <mp2p_icp/optimal_tf_common.h>
#include <cstdlib>
#include <map>
#include <string>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

/** OLAE algorithm to find the SE(3) optimal transformation given a set of
 * correspondences between points-points, lines-lines, planes-planes. Refer to
 * technical report: Jose-Luis Blanco-Claraco. OLAE-ICP: Robust and fast
 * alignment of geometric features with the optimal linear attitude estimator,
 * Arxiv 2019.
 */
void optimal_tf_olae(const Pairings_Common& in, OptimalTF_Result& result);

/** @} */

}  // namespace mp2p_icp
