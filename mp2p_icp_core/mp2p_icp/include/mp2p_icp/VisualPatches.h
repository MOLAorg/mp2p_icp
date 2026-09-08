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
 * @file   VisualPatches.h
 * @brief  Photometric patches anchored to map points ("virtual patches").
 * @author Jose Luis Blanco Claraco
 * @date   Sep 8, 2026
 */
#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/typemeta/TTypeName.h>

#include <cstdint>
#include <optional>
#include <vector>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_grp
 * @{ */

/** One "virtual patch": a small photometric patch anchored to a map point.
 *
 * The patch is captured once, from the image in which its anchor point was
 * seen, and afterwards scored against every new image by projecting the anchor
 * with the pose under estimation. The 3D position belongs to the geometric map
 * and is never optimized here, so, unlike a self-triangulated visual landmark,
 * the term is an extra observation OF THE MAP and contributes no trajectory of
 * its own to be averaged against the geometric one.
 *
 * \note The reference and the current image are assumed to come from the same
 *       camera, i.e. to share the intrinsics in VisualPatchTerm::camera.
 */
struct visual_patch_t
{
    /** Anchor point, in the GLOBAL (map) frame. */
    mrpt::math::TPoint3D pt_global;

    /** Surface normal at the anchor, in the GLOBAL frame, used to warp the
     *  reference patch into the current view. A null vector (the default) means
     *  "unknown", and the patch is then taken as fronto-parallel to the
     *  reference camera. Need not be normalized. */
    mrpt::math::TVector3D normal_global{0, 0, 0};

    /** Reference intensities in [0,255], row-major, exactly
     *  (2*half_size+1)^2 entries, sampled on the reference image pixel grid
     *  centered at the anchor's projection. */
    std::vector<float> ref_patch;

    /** Camera pose, in the GLOBAL frame, when `ref_patch` was captured. */
    mrpt::poses::CPose3D ref_camera_pose;

    DECLARE_TTYPENAME_CLASSNAME(mp2p_icp::visual_patch_t)
};

/** A direct photometric observation for the optimal-transformation solvers:
 *  a set of visual_patch_t scored against one current image.
 *
 * The residual of each patch pixel is the mean-normalized intensity difference
 * between the current image, sampled where the pose under estimation projects
 * the anchor, and the stored reference patch, warped by the affine map induced
 * by the two viewpoints and the patch plane. Mean normalization removes the
 * constant part of any exposure change; what it cannot remove (a gain change)
 * is left to the robust weight.
 *
 * The term enters the same normal equations as the geometric pairings, and it
 * constrains all 6 DOF wherever the image has gradient, so it is not a prior:
 * it is counted as measurement information by the solver diagnostics.
 *
 * \ingroup mp2p_icp_grp
 */
struct VisualPatchTerm
{
    VisualPatchTerm() = default;

    /** The current image. Must be 8-bit grayscale. */
    mrpt::img::CImage image;

    /** Intrinsics and distortion of `image`. Distortion is applied in the
     *  forward projection, so the image needs no rectification. */
    mrpt::img::TCamera camera;

    /** Camera pose with respect to the LOCAL frame, that is, the frame whose
     *  global pose is being estimated: the same frame the pairings' `local`
     *  points live in. */
    mrpt::poses::CPose3D camera_pose_on_local;

    /** The patches to score against `image`. */
    std::vector<visual_patch_t> patches;

    /** Patch half-size in pixels: each patch is (2*half_size+1)^2. */
    uint32_t half_size = 3;

    /** Standard deviation of the photometric residual [gray levels]. Sets the
     *  weight 1/sigma^2 of every patch pixel against the metric pairings. */
    double sigma_intensity = 12.0;

    /** Huber threshold on a patch's RMS residual, in units of
     *  `sigma_intensity`. Applied per patch, not per pixel, so a patch is
     *  down-weighted as a whole. */
    double huber_delta = 2.0;

    /** Patches whose RMS residual exceeds this many `sigma_intensity` are
     *  discarded outright as mismatches or occlusions. */
    double max_rms_sigmas = 6.0;

    /** Minimum depth [m] along the camera Z axis for a patch to be used. */
    double min_depth = 0.5;

    /** Margin [px] from the image border required by the patch center. */
    double border_margin = 4.0;

    /** Global multiplier on the whole term, to trade it against the pairings
     *  without touching the noise model. */
    double weight = 1.0;

    DECLARE_TTYPENAME_CLASSNAME(mp2p_icp::VisualPatchTerm)
};

/** Projects a point given in the CAMERA frame into pixel coordinates, applying
 *  the distortion model declared in `cam` (none, plumb_bob or kannala_brandt).
 *
 * \return The pixel, or std::nullopt if the point is behind the camera or
 *         closer than `minDepth`.
 * \ingroup mp2p_icp_grp
 */
std::optional<mrpt::img::TPixelCoordf> projectToPixel(
    const mrpt::img::TCamera& cam, const mrpt::math::TPoint3D& ptCamera, double minDepth = 0.05);

/** @} */

}  // namespace mp2p_icp
