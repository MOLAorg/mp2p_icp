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
    double               optimalScale = 1.0;

    /** Correspondence that were detected as outliers. */
    OutlierIndices outliers;
};

/** @} */

}  // namespace mp2p_icp
