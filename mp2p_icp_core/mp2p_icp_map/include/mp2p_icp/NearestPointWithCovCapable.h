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
 * @file   NearestPointWithCovCapable.h
 * @brief  Virtual interface for "nearest point with covariance" search algorithms
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2025
 */
#pragma once

#include <mp2p_icp/MatchingDistanceProfile.h>
#include <mp2p_icp/point_with_cov_pair_t.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/poses/CPose3D.h>

namespace mp2p_icp
{
/** \addtogroup  mp2p_icp_map_grp
 * @{ */

/** Virtual interface for "nearest point with covariance" search algorithms */
class NearestPointWithCovCapable
{
   public:
    NearestPointWithCovCapable() = default;
    virtual ~NearestPointWithCovCapable();

    NearestPointWithCovCapable(const NearestPointWithCovCapable&)            = default;
    NearestPointWithCovCapable& operator=(const NearestPointWithCovCapable&) = default;
    NearestPointWithCovCapable(NearestPointWithCovCapable&&)                 = default;
    NearestPointWithCovCapable& operator=(NearestPointWithCovCapable&&)      = default;

    /** Implements search for pairings between me ("global") and another ("local") map
     * outPairings may not be empty, so implementations must add values here, never clear it.
     *
     * This flat-threshold form remains the pure virtual one on purpose: map classes
     * written against mp2p_icp releases that predate MatchingDistanceProfile keep
     * overriding it verbatim, so they still compile and behave identically. New
     * implementations should override the MatchingDistanceProfile form below and
     * implement this one as a one-line forwarder.
     *
     * \note Declaring only one of the two overloads in a derived class hides the
     *       other for calls made on that derived type (ordinary C++ name hiding).
     *       This is harmless for the matchers, which always dispatch through a
     *       `NearestPointWithCovCapable` reference, but a derived class that wants
     *       both visible on its own type should add
     *       `using NearestPointWithCovCapable::nn_search_cov2cov;`.
     */
    virtual void nn_search_cov2cov(
        const NearestPointWithCovCapable& localMap, const mrpt::poses::CPose3D& localMapPose,
        const float max_search_distance, MatchedPointWithCovList& outPairings) const = 0;

    /** As above, but with the full acceptance criteria: an optionally range-adaptive
     * matching distance.
     *
     * Not pure: the default implementation forwards to the flat overload above, so an
     * implementation predating this API keeps working. Since such an implementation
     * cannot honor anything beyond a flat distance, the default throws rather than
     * silently ignoring a profile it cannot apply.
     */
    virtual void nn_search_cov2cov(
        const NearestPointWithCovCapable& localMap, const mrpt::poses::CPose3D& localMapPose,
        const MatchingDistanceProfile& matchingDistance, MatchedPointWithCovList& outPairings) const
    {
        ASSERTMSG_(
            matchingDistance.isFlat(),
            "This map class only implements the flat-threshold nn_search_cov2cov() "
            "overload, so it cannot honor a range-adaptive matching distance. Rebuild "
            "against a map implementation that overrides the MatchingDistanceProfile "
            "overload, or keep the matcher's thresholdFar at its default.");

        nn_search_cov2cov(localMap, localMapPose, matchingDistance.near, outPairings);
    }

    [[nodiscard]] virtual std::size_t point_count() const = 0;
};

/** @} */

}  // namespace mp2p_icp
