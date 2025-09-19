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
 * @brief  Defines a virtual interface for maps capable of finding pt-plane
 * pairings.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 8, 2024
 */
#pragma once

#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>

#include <optional>

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

    struct NearestPointCovResult
    {
        NearestPointCovResult() = default;

        /// Found pairing and its covariance (both in the "global" frame of reference):
        mrpt::math::TPoint3Df       point;
        mrpt::math::CMatrixDouble33 cov;

        /// Absolute value of plane-point distance, if a pairing is found:
        float distance = 0;
    };

    virtual std::optional<NearestPointCovResult> nn_search_pt2pl(
        const mrpt::math::TPoint3Df& point, const float max_search_distance) const = 0;
};

/** @} */

}  // namespace mp2p_icp
