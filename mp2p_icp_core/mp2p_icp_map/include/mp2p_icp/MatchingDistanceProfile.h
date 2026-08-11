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

namespace mp2p_icp
{
/** The correspondence-acceptance criteria used by
 * `NearestPointWithCovCapable::nn_search_cov2cov()`.
 *
 * Implicitly constructible from a plain `float`, so every existing caller
 * passing a flat threshold keeps compiling and behaving exactly as before.
 *
 * Two independent, opt-in refinements over a single flat distance:
 *
 * - When `far != near`, the acceptance distance is a logistic function of the
 *   query point's range from the sensor, transitioning from `near` to `far`
 *   around `kneeRange`, over `width` meters. Motivation: map point density
 *   falls off with range, so a single flat threshold is loose near the sensor
 *   (dense map, many close alternatives) and tight far away (sparse map).
 *
 * - When `firstToSecondDistanceMin > 1`, a correspondence is additionally
 *   required to be unambiguous: the second-nearest map point must be at least
 *   that many times farther than the nearest one. Range is only a proxy for
 *   "this map region is sparse and distinctive"; this measures it directly.
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

    /** Minimum ratio of the second-nearest to the nearest map point distance
     * for a correspondence to be accepted, i.e. `d2 >= d1 * this`. Anything
     * <= 1 disables the test (the default), since `d2 >= d1` always holds.
     *
     * Same quantity and same naming convention as `Matcher_Adaptive`'s
     * `firstToSecondDistanceMax`, which bounds `d2/d1` from the other side
     * (there, to decide whether a runner-up is close enough to *also* be
     * worth pairing; here, to decide the winner is too close to call).
     */
    float firstToSecondDistanceMin = 0.0f;

    /** Minimum query-point range [m] at which the ambiguity test above is
     * applied; closer points are always accepted on distance alone. Default 0,
     * i.e. applied everywhere.
     *
     * Near the sensor the map is a dense accumulation of many observations of
     * the same surface, so the runner-up is that same surface seen again and
     * the ratio sits just above 1 no matter how good the match is. Restricting
     * the test to long range aims it at the regime it was meant for: the wide
     * far-range acceptance window, where a distant repeated structure can be
     * matched to the wrong instance of itself. */
    float firstToSecondMinRange = 0.0f;

    [[nodiscard]] inline bool isFlat() const { return far == near; }

    /** Whether the first-to-second-nearest ambiguity test is enabled at all. */
    [[nodiscard]] inline bool hasAmbiguityGate() const { return firstToSecondDistanceMin > 1.0f; }

    /** Whether a query point's range must be computed at all: only if the
     * acceptance distance or the ambiguity test depends on it. */
    [[nodiscard]] inline bool needsRange() const
    {
        return !isFlat() || (hasAmbiguityGate() && firstToSecondMinRange > 0);
    }

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
