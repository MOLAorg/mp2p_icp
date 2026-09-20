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
 * @file   PointWeightByIncidence.h
 * @brief  Per-point weighting of correspondences by beam incidence angle
 * @author Jose Luis Blanco Claraco
 * @date   Sep 20, 2026
 */
#pragma once

#include <algorithm>
#include <cmath>

/** Feature-detection macro for downstream packages, since an older release
 * does not ship this header at all. Guard with
 * `__has_include(<mp2p_icp/PointWeightByIncidence.h>)`. */
#define MP2P_ICP_HAS_POINT_WEIGHT_BY_INCIDENCE 1

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_map_grp
 * @{ */

/** How much a correspondence counts, as a function of how obliquely its beam
 *  struck the surface.
 *
 * Disabled by default (`alpha = 0`, every point weighs the same), which is the
 * behavior of every release before this existed.
 *
 * \f[
 *   w(c) = \mathrm{clamp}\left(
 *            \left(\frac{c}{c_{ref}}\right)^{\alpha}, w_{min}, w_{max}\right)
 * \f]
 *
 * where \f$c = |\cos\theta|\f$ and \f$\theta\f$ is the angle between the beam
 * and the surface normal: \f$c = 1\f$ is a head-on hit and \f$c = 0\f$ a
 * perfectly grazing one.
 *
 * A positive `alpha` with the default `maxWeight = 1` is a knee that trusts
 * head-on returns fully and discounts grazing ones. The motivation is that a
 * range measurement is biased, not merely noisier, at a shallow incidence:
 * the footprint is stretched along the surface, and the returned range mixes
 * contributions across it. On a ground-hugging sensor the far ground is
 * measured almost entirely this way, so a bias that varies slowly with
 * geometry enters as a slow apparent change in surface height rather than as
 * noise, which no amount of averaging removes.
 *
 * \warning As with range weighting, `minWeight` is the safety parameter, not
 * `maxWeight`. A large `alpha` with `minWeight = 0` discards the grazing
 * returns outright, and on a sensor mounted low those returns ARE the ground:
 * removing them leaves the vertical unconstrained, which is the opposite of
 * the intent. Keep `minWeight` well above zero unless a sweep says otherwise.
 *
 * \note The consumer multiplies a correspondence's information matrix by this
 *  weight, so it acts as an inverse variance. Only the SHAPE of the curve
 *  matters when the solver rescales the whole data block.
 */
struct PointWeightByIncidence
{
    PointWeightByIncidence() = default;

    /** Decay exponent. 0 disables this entirely (the default). */
    double alpha = 0.0;

    /** |cos| of the incidence angle below which the weight starts to fall.
     *  The default, 0.5, is 60 degrees away from the surface normal. */
    double refCos = 0.5;

    /** Floor, so a grazing point is never dropped outright. */
    double minWeight = 0.05;

    /** Ceiling. Keep at 1 for a plain knee. */
    double maxWeight = 1.0;

    [[nodiscard]] inline bool enabled() const { return alpha != 0.0; }

    /** Weight of a correspondence whose beam met the surface at |cos| = c. */
    [[nodiscard]] inline double operator()(double cosIncidence) const
    {
        if (!enabled())
        {
            return 1.0;  // fast path: no pow() call
        }
        const double c = std::clamp(cosIncidence, 1e-3, 1.0);
        const double r = std::max(refCos, 1e-3);
        return std::clamp(std::pow(c / r, alpha), minWeight, maxWeight);
    }
};

/** @} */

}  // namespace mp2p_icp
