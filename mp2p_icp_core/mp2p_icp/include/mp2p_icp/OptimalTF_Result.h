/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
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

    /** Share of the final rotational information that came from the gravity
     *  prior, in [0,1], on the last Gauss-Newton iteration; <0 when no prior
     *  was given.
     *
     *  A verticality prior competes against the point pairs for the same two
     *  tilt DOFs, but it is expressed in radians while they are in metres, so
     *  its influence is set by the pair count and their lever arms and is not
     *  readable from `sigma_rad` alone. This reports what it actually was.
     */
    double gravity_information_share = -1.0;
};

/** @} */

}  // namespace mp2p_icp
