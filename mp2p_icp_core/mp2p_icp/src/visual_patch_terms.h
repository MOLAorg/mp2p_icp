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
 * @file   visual_patch_terms.h
 * @brief  Accumulation of the photometric patch block into the GN normal eqs.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 8, 2026
 *
 * Internal header: not part of the public API.
 */
#pragma once

#include <mp2p_icp/VisualPatches.h>
#include <mrpt/poses/CPose3D.h>

#include <Eigen/Dense>
#include <cstddef>

namespace mp2p_icp
{
/** Outcome of one accumulation pass over the visual patches. */
struct VisualPatchAccumStats
{
    /// Patches that contributed to H and g.
    std::size_t used = 0;
    /// Patches discarded: behind the camera, out of frame, or too dissimilar.
    std::size_t rejected = 0;
    /// Sum of weighted, normalized squared residuals (dimensionless).
    double chi2 = 0;
    /// The frame-wide photometric gain that was solved for and applied.
    double gain = 1.0;
};

/** Adds the photometric ("virtual patch") block of the normal equations,
 *  linearized at `pose`, to `H` and `g`.
 *
 * `pose` is the same SE(3) unknown the pairings use: it maps LOCAL to GLOBAL,
 * and increments are applied on the right, P <- P * Exp(eps), with eps ordered
 * as [translation; rotation-vector], matching mrpt::poses::Lie::SE<3>.
 */
VisualPatchAccumStats accumulate_visual_patches(
    const VisualPatchTerm& term, const mrpt::poses::CPose3D& pose, Eigen::Matrix<double, 6, 6>& H,
    Eigen::Matrix<double, 6, 1>& g, unsigned int level = 0);

}  // namespace mp2p_icp
