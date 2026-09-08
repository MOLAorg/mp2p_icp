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

#include <cstdint>

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

    /** Share of the final total information contributed by the photometric
     *  patch term, in [0,1], on the last Gauss-Newton iteration; <0 when no
     *  such term was given.
     *
     *  Read it the same way as gravity_information_share: the balance between
     *  gray levels and metres is set by the patch count, the image gradient
     *  and the lever arms, none of which are readable from sigma alone.
     */
    double visual_information_share = -1.0;

    /** Patches that contributed to the last iteration, and those discarded
     *  there as out of frame, behind the camera, or too dissimilar. */
    uint32_t visual_patches_used     = 0;
    uint32_t visual_patches_rejected = 0;

    /** Weighted photometric chi-square per degree of freedom on the last
     *  iteration; <0 when no such term was given.
     *
     *  This is the quantity that says whether `sigma_intensity` is honest: a
     *  value far above 1 means the declared photometric noise does not cover
     *  what the model actually fails to predict (warp linearization, gain
     *  changes, the anchor's own 3D error), and the block is therefore
     *  over-weighted no matter what sigma was chosen. */
    double visual_chi2_per_dof = -1.0;

    /** The scale actually applied to the photometric block on the last
     *  iteration: either `VisualPatchTerm::weight`, or the automatic
     *  calibration ratio when `auto_balance` is on. <0 when no term. */
    double visual_auto_scale = -1.0;

    /** The calibration ratio computed from THIS solve's own residuals, before
     *  any slow estimate replaced it. This is what a caller should feed its
     *  running estimate of the scale; `visual_auto_scale` is what was used. */
    double visual_auto_scale_instant = -1.0;
};

/** @} */

}  // namespace mp2p_icp
