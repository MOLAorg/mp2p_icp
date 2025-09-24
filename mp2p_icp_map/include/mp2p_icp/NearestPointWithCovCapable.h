/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 A repertory of multi primitive-to-primitive (MP2P) ICP algorithms
 and map building tools. mp2p_icp is part of MOLA.

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
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

#include <mp2p_icp/point_with_cov_pair_t.h>
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
     */
    virtual void nn_search_cov2cov(
        const NearestPointWithCovCapable& localMap, const mrpt::poses::CPose3D& localMapPose,
        const float max_search_distance, MatchedPointWithCovList& outPairings) const = 0;

    [[nodiscard]] virtual std::size_t point_count() const = 0;
};

/** @} */

}  // namespace mp2p_icp
