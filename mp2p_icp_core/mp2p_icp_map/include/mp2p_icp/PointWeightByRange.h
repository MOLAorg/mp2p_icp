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
 * @file   PointWeightByRange.h
 * @brief  Per-point weighting of correspondences as a function of range
 * @author Jose Luis Blanco Claraco
 * @date   Sep 20, 2026
 */
#pragma once

#include <algorithm>
#include <cmath>

/** Feature-detection macro for downstream packages, since an older release
 * does not ship this header at all. Guard with
 * `__has_include(<mp2p_icp/PointWeightByRange.h>)`. */
#define MP2P_ICP_HAS_POINT_WEIGHT_BY_RANGE 1

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_map_grp
 * @{ */

/** How much a correspondence counts, as a function of the range at which its
 *  point was measured.
 *
 * Disabled by default (`alpha = 0`, every point weighs the same), which is the
 * behavior of every release before this existed.
 *
 * \f[
 *   w(r) = \mathrm{clamp}\left(
 *            \left(\frac{r_{ref}}{\max(r,\epsilon)}\right)^{\alpha},
 *            w_{min}, w_{max}\right)
 * \f]
 *
 * With the default `maxWeight = 1` and a positive `alpha`, this is a knee:
 * everything closer than `refRange` counts fully, and beyond it the weight
 * decays as a power of the range. Two exponents have a physical reading:
 *
 * - `alpha = 1`: the lateral footprint of a beam grows linearly with range
 *   (beam divergence), so the position uncertainty of a point does too.
 * - `alpha = 2`: additionally, a surface is sampled at a density falling as
 *   \f$1/r^2\f$, so a far point also stands for more surface than a near one.
 *
 * A negative `alpha` up-weights the far field instead. That direction is the
 * useful one on a spinning LiDAR, where a fixed-size decimation voxel cannot
 * thin the far field and the near field ends up over-represented in the
 * correspondence set relative to its information content.
 *
 * \warning With a negative `alpha`, `minWeight` is what keeps this safe. A
 * steep exponent drives the NEAR field to zero — at `alpha = -2` a return at
 * a tenth of `refRange` is weighted 0.01 — and a scene that needs its near
 * returns then loses them. Measured on one 127 m scene, holding the exponent
 * at -2 and changing only the floor:
 *
 *   - `minWeight = 0`: ATE 0.011 m -> 0.383 m (and 0.712 m with the ceiling
 *     also removed), i.e. divergence.
 *   - `minWeight = 1`: ATE 0.011 m -> 0.011 m, entirely benign.
 *
 * So either keep `|alpha|` at 1.5 or below, or set `minWeight = 1` so no
 * point can count for less than it does today and the weighting can only add
 * emphasis to the far field. The latter is also the conservative choice for a
 * scene smaller than `refRange`, where it reduces to the identity.
 *
 * `maxWeight` bounds the opposite end and matters much less: at a fixed
 * `alpha = -1`, ceilings of 2, 5 and 20 span about 0.6 mm.
 *
 * \note The consumer multiplies a correspondence's information matrix by this
 *  weight, so it acts as an inverse variance. Only the SHAPE of the curve
 *  matters when the solver rescales the whole data block (for example
 *  `Solver_GaussNewton`'s Birge-ratio balancing against the prior), since a
 *  constant factor is absorbed there.
 */
struct PointWeightByRange
{
    PointWeightByRange() = default;

    /** Decay exponent. 0 disables this entirely (the default). */
    double alpha = 0.0;

    /** Range [m] below which the weight saturates at `maxWeight`. */
    double refRange = 20.0;

    /** Floor, so a far point is never dropped outright. */
    double minWeight = 0.01;

    /** Ceiling. Keep at 1 for a plain knee. */
    double maxWeight = 1.0;

    [[nodiscard]] inline bool enabled() const { return alpha != 0.0; }

    /** Weight of a correspondence measured at `range` meters. */
    [[nodiscard]] inline double operator()(double range) const
    {
        if (!enabled())
        {
            return 1.0;  // fast path: no pow() call
        }
        const double r = std::max(range, 1e-3);
        return std::clamp(std::pow(refRange / r, alpha), minWeight, maxWeight);
    }
};

/** @} */

}  // namespace mp2p_icp
