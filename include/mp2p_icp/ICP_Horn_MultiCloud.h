/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2) ICP algorithms in C++
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   ICP_Horn_MultiCloud.h
 * @brief  ICP registration for pointclouds split in different "layers"
 * @author Jose Luis Blanco Claraco
 * @date   Jan 20, 2019
 */
#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <vector>
#include "IterTermReason.h"
#include "Parameters.h"
#include "Results.h"

/** ICP registration for pointclouds split in different "layers"
 *
 * \ingroup mp2p_icp_grp */
namespace mp2p_icp::ICP_Horn_MultiCloud
{
using pointcloud_t = std::vector<mrpt::maps::CPointsMap::Ptr>;

/** Compute the displacement (relative pose) between
 *   two maps: the relative pose of pcs2 with respect to pcs1.
 */
void align(
    const pointcloud_t& pcs1, const pointcloud_t& pcs2,
    const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
    Results& result);

}  // namespace mp2p_icp::ICP_Horn_MultiCloud
