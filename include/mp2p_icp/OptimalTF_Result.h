/* -------------------------------------------------------------------------
 *  A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
 * Copyright (C) 2018-2020 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   OptimalTF_Result.h
 * @brief  Common types for all SE(3) optimal transformation methods.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 16, 2019
 */
#pragma once

#include <mp2p_icp/Pairings.h>
#include <mrpt/poses/CPose3D.h>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

/** The is the output structure for all optimal transformation methods.
 */
struct OptimalTF_Result
{
    mrpt::poses::CPose3D optimalPose;
    double               optimalScale = {1.0};

    /** Correspondence that were detected as outliers. */
    OutlierIndices outliers;
};

/** @} */

}  // namespace mp2p_icp
