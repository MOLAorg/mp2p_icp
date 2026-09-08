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
#include <memory>
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

    /** Reference intensities in [0,255], row-major, (2*half_size+1)^2 entries
     *  per pyramid level, sampled on the reference image's grid at that level,
     *  centered at the anchor's projection. Index 0 is full resolution.
     *
     *  Levels are stored rather than the reference image itself: a patch costs
     *  a few hundred bytes this way, where keeping every reference frame alive
     *  for as long as its patches would cost hundreds of megabytes. */
    std::vector<std::vector<float>> ref_patches;

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

    /** The current image at full resolution. Must be 8-bit grayscale. */
    mrpt::img::CImage image;

    /** `image` and its successive half-resolution reductions, index 0 being
     *  `image` itself. Fill with buildPyramid(); a single level reproduces the
     *  behavior of no pyramid at all. */
    std::vector<mrpt::img::CImage> image_pyramid;

    /** Fills image_pyramid with `levels` entries from `image`. */
    void buildPyramid(uint32_t levels);

    /** Number of levels the solve walks, coarsest first. Bounded by what
     *  `image_pyramid` and the patches actually carry.
     *
     *  A single level is one Gauss-Newton linearization of a raw image, whose
     *  valid range is about the correlation length of its texture, a pixel or
     *  two. Starting coarse both widens that basin and, because a reduced
     *  image is smoother, lowers the linearization error that the photometric
     *  noise would otherwise have to absorb. */
    uint32_t pyramid_levels = 1;

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

    /** Estimate one photometric GAIN for the whole frame, in closed form from
     *  all patches, and apply it before the residual.
     *
     *  Mean normalization already removes an exposure OFFSET; it does not
     *  remove a gain, and an uncorrected gain is a systematic residual
     *  proportional to each patch's own contrast. It inflates the chi-square
     *  without carrying any pose information, which under `auto_balance`
     *  directly costs the term its weight. This is the same quantity
     *  FAST-LIVO2 carries in its state as an inverse exposure time; solving it
     *  in closed form keeps the SE(3) solve six-dimensional.
     *
     *  DEFAULT OFF, on measurement rather than on principle: it is what the
     *  leader does and it is unit-tested to recover a synthetic exposure
     *  change exactly, but on GrandTour it left the real chi-square untouched
     *  and cost accuracy on both missions. A nuisance parameter estimated from
     *  the same residuals it corrects can absorb pose signal as readily as
     *  exposure, which is the standard hazard of a profile likelihood and the
     *  leading suspect. */
    bool estimate_gain = false;

    /** Bound on that gain, applied both ways (g and 1/g). A frame needing more
     *  than this is a scene change, not an exposure change. */
    double max_gain = 3.0;

    /** Global multiplier on the whole term. With `auto_balance` on this is a
     *  nudge factor on top of the measured scale, not the scale itself. */
    double weight = 1.0;

    /** Set the term's scale from the DATA instead of from `weight`, by making
     *  its chi-square per degree of freedom match the LiDAR block's:
     *
     *      scale = (chi2_cov2cov / dof_cov2cov) / (chi2_visual / dof_visual)
     *
     *  Both blocks are then equally (mis)calibrated, which is the only sense
     *  in which their relative weight is meaningful, and it is re-evaluated
     *  at every Gauss-Newton iteration like the cov2cov Birge ratio.
     *
     *  Note this makes the WEIGHTING independent of `sigma_intensity`: the
     *  scale goes as sigma^2 and the block goes as 1/sigma^2, so the noise
     *  cancels exactly. That removes the knob rather than hiding it. What
     *  sigma still sets, and should set, is the operating point of the robust
     *  kernel and of `max_rms_sigmas`.
     *
     *  Requires cov-to-cov pairings, since those are the only ones carrying a
     *  modeled covariance to compare against; falls back to `weight` alone
     *  when there are too few of them.
     */
    bool auto_balance = true;

    /** Effective number of INDEPENDENT residuals a patch supplies, which is
     *  not its pixel count: a patch is a small window of a smooth image and
     *  its residuals are strongly correlated. Measured at ~8 of 49 for a 7x7
     *  patch on GrandTour (correlation +0.74 at one pixel, +0.33 at two, zero
     *  by three). Using the pixel count here understates the chi-square per
     *  degree of freedom by the same factor and over-weights the term.
     *
     *  Measure it with MP2P_ICP_VISUAL_RESIDUAL_FILE if the patch size,
     *  interpolation or camera changes. */
    double effective_pixels_per_patch = 8.0;

    /** Safety rail on `auto_balance`, expressed as the largest share of the
     *  total information the photometric block may take on one iteration.
     *
     *  It is a rail, not a target: measured shares are 0.005 to 0.18, so it
     *  binds only on a frame whose photometric fit is anomalously good, where
     *  the ratio above would hand the camera the whole solve on the strength
     *  of one lucky frame. It is deliberately expressed as an information
     *  share and not as a scale, because a scale limit would depend on
     *  `sigma_intensity` and would reintroduce exactly the knob that
     *  `auto_balance` removes. */
    double max_information_share = 0.5;

    /** A slowly-estimated value of the calibration ratio, supplied by the
     *  caller. When positive it is used INSTEAD of this scan's own ratio.
     *
     *  Recomputing the ratio from one scan's residuals makes the weight react
     *  to every frame, and on a confined mission that feedback was measured to
     *  be actively harmful (arc-3 went from 0.135 m to 0.483 m). The quantity
     *  being estimated is a property of the sensor pair and the residual
     *  models, not of the frame, so it should be estimated slowly and then
     *  held. This is how FAST-LIVO2's constant `img_point_cov` behaves, except
     *  that here the constant is measured rather than configured.
     *
     *  Leave <=0 to fall back to this scan's own ratio, which is what bootstraps
     *  the estimate. */
    double scale_hint = -1.0;

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
