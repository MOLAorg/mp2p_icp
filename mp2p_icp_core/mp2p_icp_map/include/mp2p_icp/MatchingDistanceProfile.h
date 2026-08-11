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
 * @file   MatchingDistanceProfile.h
 * @brief  Matching distance for nn_search_cov2cov(), optionally range-adaptive
 * @author Jose Luis Blanco Claraco
 * @date   Aug 10, 2026
 */
#pragma once

#include <algorithm>
#include <cmath>

/** Feature-detection macro for downstream packages: defined only by mp2p_icp
 * releases that have MatchingDistanceProfile and the corresponding
 * `NearestPointWithCovCapable::nn_search_cov2cov()` overload. Guard with
 * `__has_include(<mp2p_icp/MatchingDistanceProfile.h>)` first, since an older
 * release does not ship this header at all. */
#define MP2P_ICP_HAS_MATCHING_DISTANCE_PROFILE 1

namespace mp2p_icp
{
/** The correspondence-acceptance criteria used by
 * `NearestPointWithCovCapable::nn_search_cov2cov()`.
 *
 * Implicitly constructible from a plain `float`, so every existing caller
 * passing a flat threshold keeps compiling and behaving exactly as before.
 *
 * Opt-in refinement over a single flat distance: when `far != near`, the
 * acceptance distance is a logistic function of the query point's range from
 * the sensor, transitioning from `near` to `far` around `kneeRange`, over
 * `width` meters. Motivation: map point density falls off with range, so a
 * single flat threshold is loose near the sensor (dense map, many close
 * alternatives) and tight far away (sparse map).
 */
struct MatchingDistanceProfile
{
    MatchingDistanceProfile() = default;

    /** Implicit on purpose: `nn_search_cov2cov(map, pose, 0.4f, out)` must
     * keep compiling and behave as a flat threshold. */
    MatchingDistanceProfile(float flatThreshold) : near(flatThreshold), far(flatThreshold) {}

    MatchingDistanceProfile(float nearDist, float farDist, float kneeRangeM, float widthM)
        : near(nearDist), far(farDist), kneeRange(kneeRangeM), width(widthM)
    {
    }

    /** Distance at range=0. Also the flat value when far==near. */
    float near = 0.40f;
    /** Distance as range -> infinity. Equal to `near` means "flat" (the
     * common case, and the fast path below). */
    float far = 0.40f;
    /** Range [m] at which the logistic transition is centered. */
    float kneeRange = 15.0f;
    /** Logistic transition width [m]. Smaller = closer to a hard knee. */
    float width = 5.0f;

    [[nodiscard]] inline bool isFlat() const { return far == near; }

    /** Whether a query point's range must be computed at all: only if the
     * acceptance distance depends on it. */
    [[nodiscard]] inline bool needsRange() const { return !isFlat(); }

    /** Matching distance at the given range [m] from the sensor. */
    [[nodiscard]] inline float operator()(float range) const
    {
        if (isFlat())
        {
            return near;  // fast path: no exp() call, matches the old flat behavior exactly
        }
        const float s = 1.0f / (1.0f + std::exp(-(range - kneeRange) / std::max(width, 1e-3f)));
        return near + (far - near) * s;
    }
};

}  // namespace mp2p_icp
